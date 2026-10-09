#include "background_remeshing.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tet = std::array<size_t, 4>;

bool contains(const Tet& tet, size_t v)
{
    return std::find(tet.begin(), tet.end(), v) != tet.end();
}

// Geometry is shared with input; topology is private until the pass is synchronized.
// Core WMTK swaps roll back connectivity when invariants() rejects a proposal.
class BackgroundMesh : public TetMesh
{
public:
    BackgroundMesh(PrismaticMeshInput& input, const BackgroundRemeshingOptions& options)
        : data(input)
        , options(options)
    {
        std::vector<Tet> tets(input.tetrahedra.rows());
        for (size_t i = 0; i < tets.size(); ++i)
            for (int j = 0; j < 4; ++j) tets[i][j] = input.tetrahedra(i, j);
        init_with_isolated_vertices(input.vertices.rows(), tets);
        pinned.assign(input.vertices.rows(), false);
        for (size_t v = 0; v < pinned.size(); ++v)
            pinned[v] = input.vertex_tags[v] != -1 || (v < input.fixed_background_vertices.size() &&
                                                       input.fixed_background_vertices[v]);
        for (const auto& t : get_tets())
            if (!background(t.tid(*this)))
                for (size_t v : oriented_tet_vids(t)) pinned[v] = true;
        for (const auto& face : get_faces())
            if (face.is_boundary_face(*this))
                for (const auto& v : get_face_vertices(face)) pinned[v.vid(*this)] = true;
    }

    bool background(size_t tid) const
    {
        // New slots are allocated only by accepted pure-background swaps; original
        // slots are reused only inside their pure-background cavity.
        return tid >= data.offset_tet_tags.size() ||
               (data.offset_tet_tags[tid] != 1 && data.input_cells[tid] != 1);
    }

    size_t operations() const { return flips23 + flips32 + moves; }
    bool exhausted() const
    {
        return operations() >= static_cast<size_t>(options.max_operations) ||
               attempts >= static_cast<size_t>(options.max_attempts);
    }

    double quality(size_t tid) const
    {
        return background_tet_mean_ratio(data.vertices, oriented_tet_vids(tid));
    }

    bool blocks(Tet tet, const BackgroundCollapseGoal& target) const
    {
        if (!contains(tet, target.removed) || contains(tet, target.survivor)) return false;
        for (auto& v : tet)
            if (v == target.removed) v = target.survivor;
        return !tet_volume_above_threshold(data.vertices, tet, 0);
    }

    std::vector<size_t> blockers(const BackgroundCollapseGoal& target) const
    {
        std::vector<size_t> result;
        for (size_t tid : get_one_ring_tids_for_vertex(target.removed))
            if (background(tid) && blocks(oriented_tet_vids(tid), target)) result.push_back(tid);
        return result;
    }

    // One improving operation per local patch; subsequent passes revisit changed regions.
    bool improve_patch(const std::vector<size_t>& tids, const BackgroundCollapseGoal* target)
    {
        goal = target;
        std::set<std::array<size_t, 3>> faces;
        std::set<std::array<size_t, 2>> edges;
        std::set<size_t> vertices;
        for (size_t tid : tids) {
            const auto tet = oriented_tet_vids(tid);
            vertices.insert(tet.begin(), tet.end());
            for (int a = 0; a < 4; ++a) {
                std::array<size_t, 3> face;
                int k = 0;
                for (int b = 0; b < 4; ++b)
                    if (a != b) face[k++] = tet[b];
                std::sort(face.begin(), face.end());
                faces.insert(face);
                for (int b = a + 1; b < 4; ++b) {
                    std::array<size_t, 2> edge = {tet[a], tet[b]};
                    std::sort(edge.begin(), edge.end());
                    edges.insert(edge);
                }
            }
        }
        // Try the cheaper coarsening operation before increasing local cell count.
        for (const auto& edge : edges) {
            if (exhausted()) return false;
            ++attempts;
            std::vector<Tuple> created;
            if (swap_edge(tuple_from_edge(edge), created)) {
                ++flips32;
                return true;
            }
        }
        for (const auto& face : faces) {
            if (exhausted()) return false;
            ++attempts;
            std::vector<Tuple> created;
            const auto [t, fid] = tuple_from_face(face);
            if (swap_face(t, created)) {
                ++flips23;
                return true;
            }
        }
        for (size_t v : vertices) {
            if (exhausted()) return false;
            if (pinned[v]) continue;
            ++attempts;
            if (relax(v)) {
                ++moves;
                return true;
            }
        }
        return false;
    }

    void synchronize()
    {
        const auto live = get_tets();
        MatrixXi cells(live.size(), 4);
        std::vector<int> inputs, offsets;
        std::vector<Tet> tets;
        for (size_t i = 0; i < live.size(); ++i) {
            const size_t tid = live[i].tid(*this);
            const auto tet = oriented_tet_vids(live[i]);
            tets.push_back(tet);
            for (int j = 0; j < 4; ++j) cells(i, j) = tet[j];
            inputs.push_back(tid < data.input_cells.size() ? data.input_cells[tid] : 0);
            offsets.push_back(tid < data.offset_tet_tags.size() ? data.offset_tet_tags[tid] : -1);
        }
        auto mesh = std::make_unique<TetMesh>();
        mesh->init_with_isolated_vertices(data.vertices.rows(), tets);
        data.mesh = std::move(mesh);
        data.tetrahedra = std::move(cells);
        data.input_cells = std::move(inputs);
        data.offset_tet_tags = std::move(offsets);
        data.offset_face_tags.clear();
    }

    size_t flips23 = 0, flips32 = 0, moves = 0, attempts = 0;

private:
    bool prepare(const std::vector<size_t>& tids)
    {
        old_quality = 1;
        old_blockers = 0;
        for (size_t tid : tids) {
            if (!background(tid)) return false;
            old_quality = std::min(old_quality, quality(tid));
            if (goal && blocks(oriented_tet_vids(tid), *goal)) ++old_blockers;
        }
        return true;
    }

    bool swap_edge_before(const Tuple& t) override
    {
        if (t.is_boundary_edge(*this)) return false;
        const auto incident = get_incident_tets_for_edge(t);
        if (incident.size() != 3) return false;
        std::vector<size_t> tids;
        for (const auto& tet : incident) tids.push_back(tet.tid(*this));
        return prepare(tids);
    }

    bool swap_face_before(const Tuple& t) override
    {
        const auto other = t.switch_tetrahedron(*this);
        return other && prepare({t.tid(*this), other->tid(*this)});
    }

    bool acceptable(const std::vector<Tet>& tets) const
    {
        double minimum = 1;
        size_t bad = 0;
        for (const auto& tet : tets) {
            if (!tet_volume_above_threshold(data.vertices, tet, 0)) return false;
            minimum = std::min(minimum, background_tet_mean_ratio(data.vertices, tet));
            if (goal && blocks(tet, *goal)) ++bad;
        }
        if (goal) {
            // Repair must strictly reduce collapse blockers. Do not trade that for
            // an arbitrarily worse current mesh; retain at least half its worst quality.
            return bad < old_blockers && minimum >= 0.5 * old_quality;
        }
        return minimum > old_quality + std::max(1e-14, 1e-6 * old_quality);
    }

    bool invariants(const std::vector<Tuple>& cells) override
    {
        std::vector<Tet> tets;
        for (const auto& cell : cells) tets.push_back(oriented_tet_vids(cell));
        return acceptable(tets);
    }

    bool relax(size_t v)
    {
        const auto tids = get_one_ring_tids_for_vertex(v);
        if (tids.empty() || !prepare(tids)) return false;
        std::set<size_t> neighbors;
        std::vector<Tet> tets;
        for (size_t tid : tids) {
            tets.push_back(oriented_tet_vids(tid));
            neighbors.insert(tets.back().begin(), tets.back().end());
        }
        neighbors.erase(v);
        const Vector3d start = data.vertices.row(v);
        Vector3d direction = Vector3d::Zero();
        double length = 0;
        for (size_t w : neighbors) {
            const Vector3d delta = data.vertices.row(w).transpose() - start;
            direction += delta;
            length += delta.norm();
        }
        direction /= neighbors.size();
        length /= neighbors.size();
        if (!direction.allFinite() || direction.norm() == 0) return false;
        if (direction.norm() > 0.25 * length) direction *= 0.25 * length / direction.norm();
        double alpha = 1;
        for (int i = 0; i <= 12; ++i, alpha *= 0.5) {
            data.vertices.row(v) = (start + alpha * direction).transpose();
            if (acceptable(tets)) return true;
        }
        data.vertices.row(v) = start.transpose();
        return false;
    }

    PrismaticMeshInput& data;
    const BackgroundRemeshingOptions& options;
    std::vector<bool> pinned;
    const BackgroundCollapseGoal* goal = nullptr;
    double old_quality = 0;
    size_t old_blockers = 0;
};

nlohmann::json quality_summary(const PrismaticMeshInput& input, double threshold)
{
    size_t count = 0, low = 0;
    double minimum = 1, sum = 0;
    for (const auto& tuple : input.mesh->get_tets()) {
        const size_t tid = tuple.tid(*input.mesh);
        if (input.offset_tet_tags[tid] == 1 || input.input_cells[tid] == 1) continue;
        const double q =
            background_tet_mean_ratio(input.vertices, input.mesh->oriented_tet_vids(tuple));
        ++count;
        low += q < threshold;
        sum += q;
        minimum = std::min(minimum, q);
    }
    return {
        {"tetrahedra", count},
        {"below_threshold", low},
        {"minimum_mean_ratio", count ? nlohmann::json(minimum) : nlohmann::json(nullptr)},
        {"mean_ratio_average", count ? nlohmann::json(sum / count) : nlohmann::json(nullptr)}};
}
} // namespace

double background_tet_mean_ratio(const MatrixXd& vertices, const Tet& tet)
{
    std::array<Vector3d, 4> p;
    p[0] = Vector3d::Zero();
    double scale = 0;
    for (int i = 1; i < 4; ++i) {
        p[i] = vertices.row(tet[i]).transpose() - vertices.row(tet[0]).transpose();
        scale = std::max(scale, p[i].norm());
    }
    if (!(scale > 0) || !std::isfinite(scale)) return 0;
    for (auto& v : p) v /= scale;
    const double determinant = p[1].dot(p[2].cross(p[3]));
    if (!(determinant > 0) || !std::isfinite(determinant)) return 0;
    double squared_edges = 0;
    for (int i = 0; i < 4; ++i)
        for (int j = i + 1; j < 4; ++j) squared_edges += (p[i] - p[j]).squaredNorm();
    return std::min(1., 12 * std::pow(determinant / 2, 2. / 3.) / squared_edges);
}

void validate_background_remeshing_options(const BackgroundRemeshingOptions& options)
{
    if (options.passes < 1 || options.max_operations < 1 || options.max_attempts < 1 ||
        !std::isfinite(options.quality_threshold) || options.quality_threshold <= 0 ||
        options.quality_threshold > 1)
        log_and_throw_error(
            "Background remeshing requires positive passes/budgets and quality_threshold in "
            "(0,1].");
}

nlohmann::json remesh_background(
    PrismaticMeshInput& input,
    double min_tet_volume,
    const BackgroundRemeshingOptions& options,
    std::vector<BackgroundCollapseGoal>* goals_before)
{
    if (goals_before) goals_before->clear();
    validate_background_remeshing_options(options);
    auto report = nlohmann::json{{"enabled", options.enabled}};
    if (!options.enabled) return report;
    const auto goals = background_blocked_collapses(input, min_tet_volume);
    if (goals_before) *goals_before = goals;
    report["quality_before"] = quality_summary(input, options.quality_threshold);
    report["background_blocked_directions_before"] = goals.size();
    BackgroundMesh mesh(input, options);
    size_t passes = 0;
    for (int pass = 0; pass < options.passes && !mesh.exhausted(); ++pass) {
        ++passes;
        const size_t before = mesh.operations();
        // Reserve half the remaining budget for collapse repair when goals exist.
        const size_t quality_limit =
            goals.empty() ? options.max_operations
                          : mesh.operations() + (options.max_operations - mesh.operations()) / 2;
        const size_t attempt_limit =
            goals.empty() ? options.max_attempts
                          : mesh.attempts + (options.max_attempts - mesh.attempts) / 2;
        std::vector<std::pair<double, size_t>> low;
        for (const auto& tet : mesh.get_tets()) {
            const size_t tid = tet.tid(mesh);
            if (mesh.background(tid) && mesh.quality(tid) < options.quality_threshold)
                low.emplace_back(mesh.quality(tid), tid);
        }
        std::sort(low.begin(), low.end());
        for (const auto& [q, tid] : low) {
            if (mesh.exhausted() || mesh.operations() >= quality_limit ||
                mesh.attempts >= attempt_limit)
                break;
            if (!mesh.tuple_from_tet(tid).is_valid(mesh) ||
                mesh.quality(tid) >= options.quality_threshold)
                continue;
            mesh.improve_patch({tid}, nullptr);
        }
        for (const auto& goal : goals) {
            if (mesh.exhausted()) break;
            const auto blockers = mesh.blockers(goal);
            if (!blockers.empty()) mesh.improve_patch(blockers, &goal);
        }
        if (mesh.operations() == before) break;
    }
    if (mesh.operations() > 0) mesh.synchronize();
    report["passes"] = passes;
    report["attempts"] = mesh.attempts;
    report["face_swaps_2_to_3"] = mesh.flips23;
    report["edge_swaps_3_to_2"] = mesh.flips32;
    report["vertex_moves"] = mesh.moves;
    report["accepted_operations"] = mesh.operations();
    report["budget_exhausted"] = mesh.exhausted();
    report["quality_after"] = quality_summary(input, options.quality_threshold);
    report["background_blocked_directions_after"] =
        background_blocked_collapses(input, min_tet_volume).size();
    return report;
}
} // namespace wmtk::components::prismatic_mesh
