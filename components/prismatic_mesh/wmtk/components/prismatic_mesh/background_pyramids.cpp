#include "background_pyramids.hpp"
#include "junction_protection.hpp"
#include "rod_mesh.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::components::prismatic_mesh::detail {
namespace {
using Edge = std::array<size_t, 2>;
using Tri = std::array<size_t, 3>;
using Tet = std::array<size_t, 4>;
using Quad = std::array<size_t, 4>;
template <typename T>
T sorted(T x)
{
    std::sort(x.begin(), x.end());
    return x;
}
Tet tet(const PrismaticMeshInput& input, size_t tid)
{
    return {
        size_t(input.tetrahedra(tid, 0)),
        size_t(input.tetrahedra(tid, 1)),
        size_t(input.tetrahedra(tid, 2)),
        size_t(input.tetrahedra(tid, 3))};
}
std::array<Tri, 4> faces(const Tet& t)
{
    return {{{t[1], t[2], t[3]}, {t[0], t[3], t[2]}, {t[0], t[1], t[3]}, {t[0], t[2], t[1]}}};
}
using FaceOwners = std::map<Tri, std::vector<std::pair<size_t, Tri>>>;
std::array<Tri, 4> pyramid_corners(const Quad& q)
{
    return {{{q[0], q[1], q[2]}, {q[0], q[2], q[3]}, {q[0], q[1], q[3]}, {q[1], q[2], q[3]}}};
}
Rational determinant(const MatrixXd& points, const Tet& t)
{
    std::array<std::array<Rational, 3>, 3> e;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            e[i][j] = Rational(points(t[i + 1], j)) - Rational(points(t[0], j));
    return e[0][0] * (e[1][1] * e[2][2] - e[1][2] * e[2][1]) -
           e[0][1] * (e[1][0] * e[2][2] - e[1][2] * e[2][0]) +
           e[0][2] * (e[1][0] * e[2][1] - e[1][1] * e[2][0]);
}

struct Plane
{
    Eigen::Vector4d row;
    double bound;
};

// Four-variable max-clearance LP: n.x + r <= b. The closed cavity bounds x at
// every nonnegative r. Enumerating all rank-four active sets therefore finds an
// optimal vertex, including degenerate optima. Normalize coordinates first. Limit
// work on unusually large stars; report a search limit separately from infeasibility.
std::optional<Eigen::Vector4d> kernel(const std::vector<Plane>& planes)
{
    std::optional<Eigen::Vector4d> best;
    const size_t n = planes.size();
    for (size_t a = 0; a < n; ++a)
        for (size_t b = a + 1; b < n; ++b)
            for (size_t c = b + 1; c < n; ++c)
                for (size_t d = c + 1; d < n; ++d) {
                    Eigen::Matrix4d A;
                    Eigen::Vector4d rhs;
                    const std::array<size_t, 4> ids = {a, b, c, d};
                    for (int j = 0; j < 4; ++j) {
                        A.row(j) = planes[ids[j]].row.transpose();
                        rhs[j] = planes[ids[j]].bound;
                    }
                    Eigen::FullPivLU<Eigen::Matrix4d> lu(A);
                    if (lu.rank() != 4) continue;
                    const Eigen::Vector4d x = lu.solve(rhs);
                    if (!x.allFinite() || x[3] <= 1e-10 || (best && x[3] <= (*best)[3])) continue;
                    if (std::all_of(planes.begin(), planes.end(), [&](const Plane& p) {
                            return p.row.dot(x) <= p.bound + 2e-12;
                        }))
                        best = x;
                }
    return best;
}

std::string make_plan(
    const PrismaticMeshInput& input,
    const BackgroundQuad& quad,
    const FaceOwners& all_faces,
    const std::map<Edge, std::vector<size_t>>& edge_stars,
    const std::set<size_t>& claimed,
    double floor,
    int growth,
    BackgroundPyramidPlan& plan)
{
    std::array<size_t, 2> targets;
    for (int k = 0; k < 2; ++k) {
        const auto it = all_faces.find(sorted(quad.triangles[k]));
        if (it == all_faces.end() || it->second.size() != 2) return "not_background_interface";
        size_t band = 0, background = 0;
        for (const auto& [tid, f] : it->second) {
            if (input.offset_tet_tags[tid] == 1)
                ++band;
            else if (input.input_cells[tid] != 1) {
                ++background;
                targets[k] = tid;
            }
        }
        if (band != 1 || background != 1) return "not_background_interface";
    }
    std::vector<size_t> diagonal;
    for (size_t v : quad.triangles[0])
        if (std::find(quad.triangles[1].begin(), quad.triangles[1].end(), v) !=
            quad.triangles[1].end())
            diagonal.push_back(v);
    if (diagonal.size() != 2) return "invalid_quad_diagonal";
    const Edge edge = sorted(Edge{diagonal[0], diagonal[1]});
    const auto star = edge_stars.find(edge);
    if (star == edge_stars.end()) return "missing_background_star";
    const std::set<size_t> sector(star->second.begin(), star->second.end());
    std::set<size_t> cavity{targets[0]};
    std::vector<size_t> queue{targets[0]};
    for (size_t k = 0; k < queue.size(); ++k)
        for (const auto& f : faces(tet(input, queue[k])))
            for (const auto& [tid, oriented] : all_faces.at(sorted(f)))
                if (sector.count(tid) && cavity.insert(tid).second) queue.push_back(tid);
    if (!cavity.count(targets[1])) return "disconnected_background_sector";
    for (size_t tid : cavity) {
        if (claimed.count(tid)) return "overlapping_cavity";
        if (input.input_cells[tid] != input.input_cells[targets[0]] ||
            input.offset_tet_tags[tid] != input.offset_tet_tags[targets[0]])
            return "mixed_background_regions";
    }
    std::map<size_t, size_t> link;
    for (size_t tid : cavity)
        for (size_t v : tet(input, tid))
            if (v != edge[0] && v != edge[1]) ++link[v];
    std::set<size_t> endpoints, expected(quad.vertices.begin(), quad.vertices.end());
    expected.erase(edge[0]);
    expected.erase(edge[1]);
    for (const auto& [v, count] : link) {
        if (count == 1)
            endpoints.insert(v);
        else if (count != 2)
            return "non_path_edge_link";
    }
    if (endpoints != expected) return "non_path_edge_link";
    // A rod front exchange may need to expand past the original diagonal star.
    // Grow only across background faces, never through a protected connection,
    // material band, input volume, claimed cavity or region boundary.
    for (int layer = 0; layer < growth; ++layer) {
        auto expanded = cavity;
        for (size_t tid : cavity)
            for (const auto& f : faces(tet(input, tid)))
                for (const auto& [neighbor, orientation] : all_faces.at(sorted(f))) {
                    if (claimed.count(neighbor) || input.input_cells[neighbor] == 1 ||
                        input.offset_tet_tags[neighbor] == 1 ||
                        input.input_cells[neighbor] != input.input_cells[targets[0]] ||
                        input.offset_tet_tags[neighbor] != input.offset_tet_tags[targets[0]] ||
                        junction_tet_protected(input, tet(input, neighbor)))
                        continue;
                    expanded.insert(neighbor);
                }
        cavity = std::move(expanded);
    }
    std::vector<Tri> boundary;
    std::set<size_t> vertices;
    std::map<Edge, size_t> boundary_edges;
    std::set<Tri> boundary_keys;
    for (size_t tid : cavity)
        for (const auto& f : faces(tet(input, tid))) {
            const auto& owners = all_faces.at(sorted(f));
            if (std::count_if(owners.begin(), owners.end(), [&](const auto& x) {
                    return cavity.count(x.first);
                }) != 1)
                continue;
            boundary.push_back(f);
            boundary_keys.insert(sorted(f));
            vertices.insert(f.begin(), f.end());
            for (int j = 0; j < 3; ++j) ++boundary_edges[sorted(Edge{f[j], f[(j + 1) % 3]})];
        }
    if (vertices.size() + boundary.size() != boundary_edges.size() + 2 ||
        !std::all_of(boundary_edges.begin(), boundary_edges.end(), [](const auto& x) {
            return x.second == 2;
        }))
        return "non_spherical_cavity";
    for (const auto& f : quad.triangles)
        if (!boundary_keys.count(sorted(f))) return "missing_cavity_base";
    if (boundary.size() < cavity.size()) return "cavity_partition_requires_compaction";
    if (boundary.size() + 4 > 64) return "kernel_search_limit";

    Vector3d lo = input.vertices.row(*vertices.begin());
    Vector3d hi = lo;
    for (size_t v : vertices) {
        const Vector3d p = input.vertices.row(v);
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    const double scale = (hi - lo).norm();
    if (!(scale > 0) || !std::isfinite(scale)) return "invalid_cavity_scale";
    const Vector3d origin = lo + .5 * (hi - lo);
    std::vector<Plane> planes;
    auto add_plane = [&](const Tri& f, bool reverse) {
        const Vector3d a = (Vector3d(input.vertices.row(f[0])) - origin) / scale;
        const Vector3d b = (Vector3d(input.vertices.row(f[1])) - origin) / scale;
        const Vector3d c = (Vector3d(input.vertices.row(f[2])) - origin) / scale;
        Vector3d n = (b - a).cross(c - a);
        const double area = n.norm();
        if (!(area > 0) || !std::isfinite(area)) return false;
        n /= reverse ? -area : area;
        const double bound = n.dot(a) - 6 * (floor / scale / scale / scale) / area;
        if (!std::isfinite(bound)) return false;
        planes.push_back({{n[0], n[1], n[2], 1}, bound});
        return true;
    };
    for (const auto& f : boundary)
        if (!add_plane(f, false)) return "invalid_constraint";
    for (const auto& f : pyramid_corners(quad.vertices))
        if (!add_plane(f, true)) return "invalid_constraint";
    const auto solution = kernel(planes);
    if (!solution) return "no_positive_common_kernel";
    const Vector3d p = origin + scale * solution->head<3>();

    // Recheck on the actual stored doubles, using exact rational determinants.
    // The LP tolerance is never used to accept an inverted or below-floor cell.
    MatrixXd points(5, 3);
    points.row(0) = p.transpose();
    auto positive = [&](const Tri& f, bool reverse) {
        for (int j = 0; j < 3; ++j) points.row(j + 1) = input.vertices.row(f[j]);
        return tet_volume_above_threshold(
            points,
            reverse ? Tet{1, 2, 3, 0} : Tet{0, 1, 2, 3},
            floor);
    };
    Rational old_volume(0), new_volume(0);
    for (size_t tid : cavity)
        old_volume = old_volume + determinant(input.vertices, tet(input, tid));
    for (const auto& f : boundary) {
        if (!positive(f, false)) return "exact_positivity_rejected";
        new_volume = new_volume + determinant(points, {0, 1, 2, 3});
    }
    for (const auto& f : pyramid_corners(quad.vertices))
        if (!positive(f, true)) return "exact_pyramid_positivity_rejected";
    if (old_volume != new_volume) return "exact_volume_mismatch";
    plan = {quad, {cavity.begin(), cavity.end()}, std::move(boundary), p, scale * (*solution)[3]};
    return {};
}

struct BackgroundIndex
{
    FaceOwners all_faces;
    std::map<Edge, std::vector<size_t>> edge_stars;

    void add(const PrismaticMeshInput& input, size_t tid, bool locked)
    {
        const auto t = tet(input, tid);
        for (const auto& f : faces(t)) all_faces[sorted(f)].push_back({tid, f});
        if (locked || input.offset_tet_tags[tid] == 1 || input.input_cells[tid] == 1) return;
        for (int a = 0; a < 4; ++a)
            for (int b = a + 1; b < 4; ++b) edge_stars[sorted(Edge{t[a], t[b]})].push_back(tid);
    }

    void remove(size_t tid, const Tet& t)
    {
        for (const auto& f : faces(t)) {
            auto it = all_faces.find(sorted(f));
            if (it == all_faces.end())
                throw std::runtime_error("sequential pyramids: missing indexed face");
            auto& owners = it->second;
            owners.erase(
                std::remove_if(
                    owners.begin(),
                    owners.end(),
                    [&](const auto& x) { return x.first == tid; }),
                owners.end());
            if (owners.empty()) all_faces.erase(it);
        }
        for (int a = 0; a < 4; ++a)
            for (int b = a + 1; b < 4; ++b) {
                auto it = edge_stars.find(sorted(Edge{t[a], t[b]}));
                if (it == edge_stars.end()) continue; // Pyramid reference tets are locked.
                auto& rows = it->second;
                rows.erase(std::remove(rows.begin(), rows.end(), tid), rows.end());
                if (rows.empty()) edge_stars.erase(it);
            }
    }
};

struct ScratchEdit
{
    size_t vertices_before, tets_before;
    std::vector<std::pair<size_t, Tet>> old_tets;
    std::vector<size_t> new_rows, locked_rows;
};

// Materialize only on the planner's private mesh. The two cones below the quad
// are the pyramid body; all other new cones remain reusable background tets.
ScratchEdit materialize_scratch(
    PrismaticMeshInput& work,
    const BackgroundPyramidPlan& plan,
    std::set<size_t>& locked,
    BackgroundIndex* index)
{
    if (plan.boundary.size() < plan.cavity.size())
        throw std::runtime_error("sequential pyramids: invalid cavity partition");
    ScratchEdit edit{size_t(work.vertices.rows()), size_t(work.tetrahedra.rows()), {}, {}, {}};
    for (size_t tid : plan.cavity) {
        if (locked.count(tid))
            throw std::runtime_error("sequential pyramids: consuming an existing pyramid");
        edit.old_tets.push_back({tid, tet(work, tid)});
        if (index) index->remove(tid, edit.old_tets.back().second);
    }
    const int region = work.input_cells.at(plan.cavity.front());
    const int band = work.offset_tet_tags.at(plan.cavity.front());
    const size_t p = work.vertices.rows();
    work.vertices.conservativeResize(p + 1, 3);
    work.vertices.row(p) = plan.point.transpose();
    const size_t count = edit.tets_before + plan.boundary.size() - plan.cavity.size();
    work.tetrahedra.conservativeResize(count, 4);
    work.input_cells.resize(count, region);
    work.offset_tet_tags.resize(count, band);
    for (size_t k = 0; k < plan.boundary.size(); ++k) {
        const auto& f = plan.boundary[k];
        const size_t tid =
            k < plan.cavity.size() ? plan.cavity[k] : edit.tets_before + k - plan.cavity.size();
        work.tetrahedra.row(tid) << p, f[0], f[1], f[2];
        edit.new_rows.push_back(tid);
        if (sorted(f) == sorted(plan.quad.triangles[0]) ||
            sorted(f) == sorted(plan.quad.triangles[1])) {
            locked.insert(tid);
            edit.locked_rows.push_back(tid);
        }
    }
    if (edit.locked_rows.size() != 2)
        throw std::runtime_error("sequential pyramids: invalid pyramid reference split");
    if (index)
        for (size_t tid : edit.new_rows) index->add(work, tid, locked.count(tid));
    return edit;
}

void undo_scratch(
    PrismaticMeshInput& work,
    const ScratchEdit& edit,
    std::set<size_t>& locked,
    BackgroundIndex& index)
{
    for (size_t tid : edit.new_rows) index.remove(tid, tet(work, tid));
    for (size_t tid : edit.locked_rows) locked.erase(tid);
    for (const auto& [tid, t] : edit.old_tets) {
        for (int j = 0; j < 4; ++j) work.tetrahedra(tid, j) = t[j];
        index.add(work, tid, locked.count(tid));
    }
    work.vertices.conservativeResize(edit.vertices_before, 3);
    work.tetrahedra.conservativeResize(edit.tets_before, 4);
    work.input_cells.resize(edit.tets_before);
    work.offset_tet_tags.resize(edit.tets_before);
}

size_t append_background_point(PrismaticMeshInput& reference, const Vector3d& point, int64_t id)
{
    const size_t p = reference.vertices.rows();
    reference.vertices.conservativeResize(p + 1, 3);
    reference.vertices.row(p) = point.transpose();
    reference.vertex_tags.push_back(-1);
    reference.source_vertex_ids.push_back(id);
    reference.corr_input_vid.push_back(-1);
    reference.corr_input_vertex.push_back(-1);
    reference.input_to_offset_vertices.resize(p + 1);
    if (!reference.fixed_background_vertices.empty())
        reference.fixed_background_vertices.push_back(false);
    if (!reference.input_to_components.empty()) reference.input_to_components.resize(p + 1);
    if (!reference.vertex_component_ids.empty()) reference.vertex_component_ids.push_back(-1);
    if (!reference.singular_vertex_tags.empty()) reference.singular_vertex_tags.push_back(-1);
    if (reference.optimal_normals.rows()) {
        reference.optimal_normals.conservativeResize(p + 1, 3);
        reference.optimal_normals.row(p).setZero();
    }
    if (reference.target_positions.rows()) {
        reference.target_positions.conservativeResize(p + 1, 3);
        reference.target_positions.row(p) = point.transpose();
    }
    return p;
}
} // namespace

BackgroundPyramidPlans plan_background_pyramids(
    const PrismaticMeshInput& input,
    const std::vector<BackgroundQuad>& quads,
    double min_volume,
    int max_cavity_growth,
    bool sequential_retry)
{
    if (!std::isfinite(min_volume) || min_volume < 0)
        throw std::runtime_error("background pyramids: invalid volume floor");
    if (max_cavity_growth < 0 || max_cavity_growth > 2)
        throw std::runtime_error("background pyramids: invalid cavity growth limit");
    if (sequential_retry) {
        std::set<Quad> seen;
        std::vector<std::vector<BackgroundQuad>> groups;
        for (const auto& quad : quads)
            if (seen.insert(sorted(quad.vertices)).second) groups.push_back({quad});
        std::vector<std::string> reasons;
        auto result =
            plan_atomic_rod_layer_pyramids(input, groups, min_volume, reasons, max_cavity_growth);
        result.report["method"] = "shell_quad_batch_then_sequential_overlap_retry";
        result.report["rejections"].erase("not_background_interface");
        result.report["initial_rejections"].erase("not_background_interface");
        return result;
    }
    FaceOwners all_faces;
    std::map<Edge, std::vector<size_t>> edge_stars;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        const auto t = tet(input, tid);
        for (const auto& f : faces(t)) all_faces[sorted(f)].push_back({tid, f});
        if (input.offset_tet_tags[tid] == 1 || input.input_cells[tid] == 1) continue;
        for (int a = 0; a < 4; ++a)
            for (int b = a + 1; b < 4; ++b) edge_stars[sorted(Edge{t[a], t[b]})].push_back(tid);
    }
    BackgroundPyramidPlans result;
    std::set<Quad> seen;
    std::set<size_t> claimed;
    std::map<std::string, size_t> rejections;
    size_t grown_cavities = 0;
    for (const auto& quad : quads) {
        const auto key = sorted(quad.vertices);
        if (!seen.insert(key).second) continue;
        BackgroundPyramidPlan plan;
        std::string reason;
        int growth = 0;
        for (; growth <= max_cavity_growth; ++growth) {
            reason =
                make_plan(input, quad, all_faces, edge_stars, claimed, min_volume, growth, plan);
            if (reason != "no_positive_common_kernel") break;
        }
        if (!reason.empty()) {
            if (reason != "not_background_interface") ++rejections[reason];
            continue;
        }
        claimed.insert(plan.cavity.begin(), plan.cavity.end());
        grown_cavities += growth > 0;
        result.accepted.emplace(key, std::move(plan));
    }
    result.report = {
        {"feasible_disjoint_cavities", result.accepted.size()},
        {"rejections", rejections},
        {"applied", 0},
        {"solver", "normalized four-variable max-clearance LP; active-set enumeration"},
        {"max_constraints", 64},
        {"max_cavity_growth", max_cavity_growth},
        {"grown_cavities", grown_cavities},
        {"min_tet_volume", min_volume},
        {"regions", nlohmann::json::array()}};
    return result;
}

void apply_background_pyramids(
    PrismaticMeshInput& reference,
    PrismDominantMesh& hybrid,
    BackgroundPyramidPlans& plans)
{
    std::set<Quad> active;
    for (const auto& c : hybrid.cells) {
        if (reference.offset_tet_tags[c.source_tets[0]] != 1) continue;
        const auto& v = c.vertices;
        if (c.type == HybridCellType::Prism)
            for (int s = 0; s < 3; ++s)
                active.insert(sorted(Quad{v[s], v[(s + 1) % 3], v[(s + 1) % 3 + 3], v[s + 3]}));
        else if (c.type == HybridCellType::Pyramid)
            active.insert(sorted(Quad{v[0], v[1], v[2], v[3]}));
    }
    std::set<size_t> replaced;
    std::vector<const BackgroundPyramidPlan*> selected;
    for (const auto& [key, plan] : plans.accepted)
        if (active.count(key)) {
            selected.push_back(&plan);
            replaced.insert(plan.cavity.begin(), plan.cavity.end());
        }
    if (selected.empty() && plans.sequential.empty()) return;
    if (!plans.sequential.empty() && selected.size() != plans.accepted.size())
        throw std::runtime_error("sequential pyramids: missing prerequisite exterior quad");
    for (const auto& step : plans.sequential)
        if (!active.count(sorted(step.plan.quad.vertices)))
            throw std::runtime_error("sequential pyramids: missing retry exterior quad");
    int64_t next_id =
        *std::max_element(reference.source_vertex_ids.begin(), reference.source_vertex_ids.end());
    if (next_id >
        std::numeric_limits<int64_t>::max() - int64_t(selected.size() + plans.sequential.size()))
        throw std::runtime_error("background pyramids: vertex source ID overflow");
    std::vector<HybridCell> cells;
    for (auto& c : hybrid.cells) {
        if (!replaced.count(c.source_tets[0]))
            cells.push_back(std::move(c));
        else if (c.type != HybridCellType::Tetrahedron || c.source_tets.size() != 1)
            throw std::runtime_error("background pyramids: cavity contains a merged cell");
    }
    std::vector<Tet> tets;
    for (size_t tid = 0; tid < size_t(reference.tetrahedra.rows()); ++tid)
        tets.push_back(tet(reference, tid));
    double minimum_margin = std::numeric_limits<double>::infinity();
    for (const auto* ptr : selected) {
        const auto& plan = *ptr;
        const size_t p = append_background_point(reference, plan.point, ++next_id);
        HybridCell pyramid;
        pyramid.type = HybridCellType::Pyramid;
        pyramid.vertices.assign(plan.quad.vertices.begin(), plan.quad.vertices.end());
        pyramid.vertices.push_back(p);
        pyramid.background_transition = true;
        const int input_tag = reference.input_cells[plan.cavity[0]];
        const int offset_tag = reference.offset_tet_tags[plan.cavity[0]];
        size_t k = 0;
        std::vector<size_t> new_rows;
        for (const auto& f : plan.boundary) {
            const Tet t = {p, f[0], f[1], f[2]};
            size_t tid;
            if (k < plan.cavity.size()) {
                tid = plan.cavity[k];
                tets[tid] = t;
            } else {
                tid = tets.size();
                tets.push_back(t);
                reference.input_cells.push_back(input_tag);
                reference.offset_tet_tags.push_back(offset_tag);
            }
            ++k;
            new_rows.push_back(tid);
            const auto key = sorted(f);
            if (key == sorted(plan.quad.triangles[0]) || key == sorted(plan.quad.triangles[1]))
                pyramid.source_tets.push_back(tid);
            else
                cells.push_back(
                    {HybridCellType::Tetrahedron, {t.begin(), t.end()}, {tid}, -1, true});
        }
        if (pyramid.source_tets.size() != 2 || k < plan.cavity.size())
            throw std::runtime_error("background pyramids: invalid cavity partition");
        cells.push_back(std::move(pyramid));
        minimum_margin = std::min(minimum_margin, plan.margin);
        plans.report["regions"].push_back(
            {{"quad_vertex_rows", plan.quad.vertices},
             {"original_reference_tet_rows", plan.cavity},
             {"new_reference_tet_rows", new_rows},
             {"point_vertex_row", p},
             {"point_source_vid", next_id},
             {"margin", plan.margin}});
    }
    // Replay only the successful private transactions. Existing pyramid bodies
    // have merged owners and may never be consumed by a later cavity.
    std::vector<size_t> owner(tets.size(), std::numeric_limits<size_t>::max());
    std::vector<bool> live(cells.size(), true);
    for (size_t ci = 0; ci < cells.size(); ++ci)
        for (size_t tid : cells[ci].source_tets) owner.at(tid) = ci;
    for (const auto& step : plans.sequential) {
        const auto& plan = step.plan;
        if (reference.vertices.rows() != step.point_vertex_row)
            throw std::runtime_error("sequential pyramids: point replay order changed");
        const int region = reference.input_cells.at(plan.cavity.front());
        const int band = reference.offset_tet_tags.at(plan.cavity.front());
        for (size_t tid : plan.cavity) {
            const size_t ci = owner.at(tid);
            if (ci >= cells.size() || !live[ci] || cells[ci].type != HybridCellType::Tetrahedron ||
                cells[ci].source_tets.size() != 1 || reference.input_cells[tid] == 1 ||
                reference.offset_tet_tags[tid] == 1 || reference.input_cells[tid] != region ||
                reference.offset_tet_tags[tid] != band)
                throw std::runtime_error("sequential pyramids: retry consumes a fixed cell");
            live[ci] = false;
        }
        const size_t p = append_background_point(reference, plan.point, ++next_id);
        HybridCell pyramid;
        pyramid.type = HybridCellType::Pyramid;
        pyramid.vertices.assign(plan.quad.vertices.begin(), plan.quad.vertices.end());
        pyramid.vertices.push_back(p);
        pyramid.background_transition = true;
        std::vector<size_t> new_rows;
        for (size_t k = 0; k < plan.boundary.size(); ++k) {
            const auto& f = plan.boundary[k];
            const Tet t = {p, f[0], f[1], f[2]};
            size_t tid;
            if (k < plan.cavity.size()) {
                tid = plan.cavity[k];
                tets.at(tid) = t;
            } else {
                tid = tets.size();
                tets.push_back(t);
                owner.push_back(std::numeric_limits<size_t>::max());
                reference.input_cells.push_back(region);
                reference.offset_tet_tags.push_back(band);
            }
            new_rows.push_back(tid);
            if (sorted(f) == sorted(plan.quad.triangles[0]) ||
                sorted(f) == sorted(plan.quad.triangles[1]))
                pyramid.source_tets.push_back(tid);
            else {
                owner[tid] = cells.size();
                cells.push_back(
                    {HybridCellType::Tetrahedron, {t.begin(), t.end()}, {tid}, -1, true});
                live.push_back(true);
            }
        }
        if (pyramid.source_tets.size() != 2)
            throw std::runtime_error("sequential pyramids: invalid replayed pyramid");
        for (size_t tid : pyramid.source_tets) owner[tid] = cells.size();
        cells.push_back(std::move(pyramid));
        live.push_back(true);
        minimum_margin = std::min(minimum_margin, plan.margin);
        plans.report["regions"].push_back(
            {{"quad_vertex_rows", plan.quad.vertices},
             {"original_reference_tet_rows", plan.cavity},
             {"new_reference_tet_rows", new_rows},
             {"point_vertex_row", p},
             {"point_source_vid", next_id},
             {"margin", plan.margin},
             {"sequential_retry_layer", step.layer}});
    }
    std::vector<HybridCell> retained;
    retained.reserve(cells.size());
    for (size_t ci = 0; ci < cells.size(); ++ci)
        if (live[ci]) retained.push_back(std::move(cells[ci]));
    cells = std::move(retained);
    reference.tetrahedra.resize(tets.size(), 4);
    for (size_t tid = 0; tid < tets.size(); ++tid)
        for (int j = 0; j < 4; ++j) reference.tetrahedra(tid, j) = tets[tid][j];
    hybrid.cells = std::move(cells);
    hybrid.source_tet_to_cell.assign(tets.size(), std::numeric_limits<size_t>::max());
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci)
        for (size_t tid : hybrid.cells[ci].source_tets) {
            if (hybrid.source_tet_to_cell[tid] != std::numeric_limits<size_t>::max())
                throw std::runtime_error("background pyramids: overlapping source ownership");
            hybrid.source_tet_to_cell[tid] = ci;
        }
    plans.report["applied"] = selected.size() + plans.sequential.size();
    plans.report["sequential_applied"] = plans.sequential.size();
    plans.report["new_background_vertices"] = selected.size() + plans.sequential.size();
    plans.report["minimum_margin"] = minimum_margin;
    plans.report["exact_positive_tets_and_pyramid_corners"] = true;
    plans.report["exact_cavity_volume_preserved"] = true;
}
BackgroundPyramidPlans plan_atomic_rod_layer_pyramids(
    const PrismaticMeshInput& input,
    const std::vector<std::vector<BackgroundQuad>>& layers,
    double floor,
    std::vector<std::string>& reasons,
    int max_cavity_growth)
{
    FaceOwners all_faces;
    std::map<Edge, std::vector<size_t>> edge_stars;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid) {
        const auto t = tet(input, tid);
        for (const auto& f : faces(t)) all_faces[sorted(f)].push_back({tid, f});
        if (input.offset_tet_tags[tid] == 1 || input.input_cells[tid] == 1) continue;
        for (int a = 0; a < 4; ++a)
            for (int b = a + 1; b < 4; ++b) edge_stars[sorted(Edge{t[a], t[b]})].push_back(tid);
    }
    BackgroundPyramidPlans result;
    std::set<size_t> claimed;
    for (size_t tid = 0; tid < size_t(input.tetrahedra.rows()); ++tid)
        if (is_fixed_shell_tet(input, tid)) claimed.insert(tid);
    std::map<std::string, size_t> rejected;
    reasons.assign(layers.size(), "");
    for (size_t i = 0; i < layers.size(); ++i) {
        auto transaction = claimed;
        std::map<Quad, BackgroundPyramidPlan> plans;
        for (const auto& quad : layers[i]) {
            BackgroundPyramidPlan plan;
            std::string reason;
            for (int growth = 0; growth <= max_cavity_growth; ++growth) {
                reason =
                    make_plan(input, quad, all_faces, edge_stars, transaction, floor, growth, plan);
                if (reason != "no_positive_common_kernel") break;
            }
            if (!reason.empty()) {
                reasons[i] = reason;
                ++rejected[reason];
                break;
            }
            transaction.insert(plan.cavity.begin(), plan.cavity.end());
            plans.emplace(sorted(quad.vertices), std::move(plan));
        }
        if (!reasons[i].empty()) continue; // Roll back every cavity in a failed layer.
        claimed = std::move(transaction);
        for (auto& [q, plan] : plans) result.accepted.emplace(q, std::move(plan));
    }
    result.planning_tet_count = input.tetrahedra.rows();
    const auto initial_rejections = rejected;
    nlohmann::json attempts = nlohmann::json::array();
    size_t recovered = 0;
    if (std::find(reasons.begin(), reasons.end(), "overlapping_cavity") != reasons.end()) {
        PrismaticMeshInput work;
        work.vertices = input.vertices;
        work.tetrahedra = input.tetrahedra;
        work.input_cells = input.input_cells;
        work.offset_tet_tags = input.offset_tet_tags;
        work.junction_protection = input.junction_protection;
        work.rod_state = input.rod_state;
        std::set<size_t> locked;
        for (size_t tid = 0; tid < size_t(work.tetrahedra.rows()); ++tid)
            if (is_fixed_shell_tet(work, tid)) locked.insert(tid);
        std::set<Quad> emitted;
        for (const auto& [q, plan] : result.accepted) {
            materialize_scratch(work, plan, locked, nullptr);
            emitted.insert(q);
        }
        BackgroundIndex index;
        for (size_t tid = 0; tid < size_t(work.tetrahedra.rows()); ++tid)
            index.add(work, tid, locked.count(tid));
        for (size_t i = 0; i < layers.size(); ++i) {
            if (reasons[i] != "overlapping_cavity") continue;
            const size_t checkpoint = result.sequential.size();
            std::vector<ScratchEdit> edits;
            std::vector<Quad> new_quads;
            std::string failure;
            for (const auto& quad : layers[i]) {
                const auto q = sorted(quad.vertices);
                if (emitted.count(q)) {
                    failure = "quad_already_emitted";
                    break;
                }
                BackgroundPyramidPlan plan;
                for (int growth = 0; growth <= max_cavity_growth; ++growth) {
                    failure = make_plan(
                        work,
                        quad,
                        index.all_faces,
                        index.edge_stars,
                        locked,
                        floor,
                        growth,
                        plan);
                    if (failure != "no_positive_common_kernel") break;
                }
                if (!failure.empty()) break;
                const size_t point_row = work.vertices.rows();
                edits.push_back(materialize_scratch(work, plan, locked, &index));
                emitted.insert(q);
                new_quads.push_back(q);
                result.sequential.push_back({std::move(plan), point_row, i});
            }
            const size_t completed = edits.size();
            if (!failure.empty()) {
                for (auto it = edits.rbegin(); it != edits.rend(); ++it)
                    undo_scratch(work, *it, locked, index);
                for (const auto& q : new_quads) emitted.erase(q);
                result.sequential.resize(checkpoint);
            } else {
                reasons[i].clear();
                ++recovered;
            }
            attempts.push_back(
                {{"layer", i},
                 {"accepted", failure.empty()},
                 {"reason", failure},
                 {"quads_completed_before_failure_or_commit", completed}});
        }
    }
    rejected.clear();
    for (const auto& reason : reasons)
        if (!reason.empty()) ++rejected[reason];
    result.report = {
        {"method", "atomic_per_rod_layer"},
        {"feasible_disjoint_cavities", result.accepted.size()},
        {"rejections", rejected},
        {"applied", 0},
        {"max_cavity_growth", max_cavity_growth},
        {"initial_rejections", initial_rejections},
        {"sequential_overlap_retry",
         {{"attempted_layers", attempts.size()},
          {"recovered_layers", recovered},
          {"pyramids", result.sequential.size()},
          {"protects_existing_pyramid_bodies", true},
          {"rolls_back_failed_layers", true},
          {"attempts", attempts}}},
        {"min_tet_volume", floor},
        {"regions", nlohmann::json::array()}};
    return result;
}

} // namespace wmtk::components::prismatic_mesh::detail
