#include "background_pyramids.hpp"

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
} // namespace

BackgroundPyramidPlans plan_background_pyramids(
    const PrismaticMeshInput& input,
    const std::vector<BackgroundQuad>& quads,
    double min_volume)
{
    if (!std::isfinite(min_volume) || min_volume < 0)
        throw std::runtime_error("background pyramids: invalid volume floor");
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
    for (const auto& quad : quads) {
        const auto key = sorted(quad.vertices);
        if (!seen.insert(key).second) continue;
        BackgroundPyramidPlan plan;
        const auto reason =
            make_plan(input, quad, all_faces, edge_stars, claimed, min_volume, plan);
        if (!reason.empty()) {
            if (reason != "not_background_interface") ++rejections[reason];
            continue;
        }
        claimed.insert(plan.cavity.begin(), plan.cavity.end());
        result.accepted.emplace(key, std::move(plan));
    }
    result.report = {
        {"feasible_disjoint_cavities", result.accepted.size()},
        {"rejections", rejections},
        {"applied", 0},
        {"solver", "normalized four-variable max-clearance LP; active-set enumeration"},
        {"max_constraints", 64},
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
    if (selected.empty()) return;
    int64_t next_id =
        *std::max_element(reference.source_vertex_ids.begin(), reference.source_vertex_ids.end());
    if (next_id > std::numeric_limits<int64_t>::max() - int64_t(selected.size()))
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
        const size_t p = reference.vertices.rows();
        reference.vertices.conservativeResize(p + 1, 3);
        reference.vertices.row(p) = plan.point.transpose();
        reference.vertex_tags.push_back(-1);
        reference.source_vertex_ids.push_back(++next_id);
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
            reference.target_positions.row(p) = plan.point.transpose();
        }
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
    plans.report["applied"] = selected.size();
    plans.report["new_background_vertices"] = selected.size();
    plans.report["minimum_margin"] = minimum_margin;
    plans.report["exact_positive_tets_and_pyramid_corners"] = true;
    plans.report["exact_cavity_volume_preserved"] = true;
}
} // namespace wmtk::components::prismatic_mesh::detail
