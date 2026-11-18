#include "bad_quality_relocation.hpp"
#include "junction_protection.hpp"
#include "prism_jacobian.hpp"
#include "relocation_projection.hpp"
#include "rod_mesh.hpp"
#include "surface_domain.hpp"

#include <wmtk/utils/AMIPS.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <queue>
#include <set>
#include <stdexcept>
#include <unordered_map>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tet = std::array<size_t, 4>;
using Points = std::array<Vector3d, 4>;
constexpr double infinity = std::numeric_limits<double>::infinity();

Points points(const MatrixXd& p, const Tet& t)
{
    Points out;
    for (int j = 0; j < 4; ++j) out[j] = p.row(t[j]);
    return out;
}
Tet tet(const PrismaticMeshInput& input, size_t tid)
{
    Tet out;
    for (int j = 0; j < 4; ++j) out[j] = input.tetrahedra(tid, j);
    return out;
}
std::array<double, 12> packed(const Points& p)
{
    std::array<double, 12> out;
    for (int j = 0; j < 4; ++j)
        for (int k = 0; k < 3; ++k) out[3 * j + k] = p[j][k];
    return out;
}
double quality(const Points& p)
{
    return std::cbrt(AMIPS_energy_stable_p3<Rational>(packed(p)));
}
double quality(const PrismaticMeshInput& input, size_t tid)
{
    return quality(points(input.vertices, tet(input, tid)));
}
double determinant(const Points& p)
{
    using V = Eigen::Matrix<long double, 3, 1>;
    const V a = p[1].cast<long double>() - p[0].cast<long double>();
    const V b = p[2].cast<long double>() - p[0].cast<long double>();
    const V c = p[3].cast<long double>() - p[0].cast<long double>();
    return static_cast<double>(a.dot(b.cross(c)));
}

// Coupled moves need more than positive endpoints. Certify the determinant's
// cubic Bernstein controls exactly on the stored double endpoints. Single-corner
// moves have affine determinants and are already certified by their endpoints.
bool positive_segment(const Points& start, const Points& end, double floor)
{
    int changed = 0;
    for (int j = 0; j < 4; ++j) changed += start[j] != end[j];
    if (changed < 2) return true;
    using RVec = std::array<Rational, 3>;
    std::array<RVec, 3> e, d;
    for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) {
            e[j][k] = Rational(start[j + 1][k]) - Rational(start[0][k]);
            d[j][k] = Rational(end[j + 1][k]) - Rational(end[0][k]) - e[j][k];
        }
    const auto det = [](const RVec& a, const RVec& b, const RVec& c) -> Rational {
        return a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
               a[2] * (b[0] * c[1] - b[1] * c[0]);
    };
    const Rational c0 = det(e[0], e[1], e[2]);
    const Rational c1 = det(d[0], e[1], e[2]) + det(e[0], d[1], e[2]) + det(e[0], e[1], d[2]);
    const Rational c2 = det(d[0], d[1], e[2]) + det(d[0], e[1], d[2]) + det(e[0], d[1], d[2]);
    const Rational c3 = det(d[0], d[1], d[2]);
    const Rational b = Rational(6) * Rational(floor);
    return c0 > b && Rational(3) * c0 + c1 > Rational(3) * b &&
           Rational(3) * c0 + Rational(2) * c1 + c2 > Rational(3) * b && c0 + c1 + c2 + c3 > b;
}

bool same_component(const PrismaticMeshInput& input, size_t a, size_t b)
{
    return input.vertex_tags[a] == 2 && input.vertex_tags[b] == 2 &&
           input.vertex_component_ids.size() == static_cast<size_t>(input.vertices.rows()) &&
           input.vertex_component_ids[a] >= 0 &&
           input.vertex_component_ids[a] == input.vertex_component_ids[b] &&
           input.corr_input_vid[a] >= 0 && input.corr_input_vid[a] == input.corr_input_vid[b] &&
           input.corr_input_vertex[a] == input.corr_input_vertex[b];
}

nlohmann::json
statistics(const PrismaticMeshInput& input, const std::vector<size_t>& tids, double bar)
{
    std::vector<double> q;
    for (size_t tid : tids) q.push_back(quality(input, tid));
    if (q.empty()) return {{"count", 0}, {"over_threshold", 0}, {"over_1000", 0}};
    std::sort(q.begin(), q.end());
    return {
        {"count", q.size()},
        {"maximum", q.back()},
        {"p95", q[static_cast<size_t>(.95 * (q.size() - 1))]},
        {"over_threshold", std::count_if(q.begin(), q.end(), [&](double x) { return x > bar; })},
        {"over_1000", std::count_if(q.begin(), q.end(), [](double x) { return x > 1000; })}};
}

std::array<Tet, 4> pyramid_tets(const HybridCell& c)
{
    const auto& v = c.vertices;
    return {
        {{v[0], v[1], v[2], v[4]},
         {v[0], v[2], v[3], v[4]},
         {v[0], v[1], v[3], v[4]},
         {v[1], v[2], v[3], v[4]}}};
}

// Conservative native-prism AMIPS bound, and exact maximum of the four native
// pyramid corner paths. Reference-tet caps alone do not protect native cells.
double native_bound(const MatrixXd& p, const HybridCell& c)
{
    const auto at = [&](size_t j) -> Vector3d { return p.row(c.vertices[j]); };
    if (c.type == HybridCellType::Prism) {
        const double det = 2 / std::sqrt(3.) * minimum_prism_jacobian(p, c).determinant;
        if (!(det > 0)) return infinity;
        double norm = 0;
        for (size_t side = 0; side < 2; ++side)
            for (size_t j = 0; j < 3; ++j) {
                const Vector3d dr = at(3 * side + 1) - at(3 * side);
                const Vector3d ds = (2 * (at(3 * side + 2) - at(3 * side)) - dr) / std::sqrt(3.);
                const Vector3d dt = at(j + 3) - at(j);
                norm = std::max(norm, dr.squaredNorm() + ds.squaredNorm() + dt.squaredNorm());
            }
        return norm / std::cbrt(det * det);
    }
    double worst = 0;
    for (const auto& uv : std::array<std::array<double, 2>, 4>{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}}) {
        const double u = uv[0], v = uv[1];
        const Vector3d q =
            (1 - u) * (1 - v) * at(0) + u * (1 - v) * at(1) + u * v * at(2) + (1 - u) * v * at(3);
        const Vector3d du = (1 - v) * (at(1) - at(0)) + v * (at(2) - at(3));
        const Vector3d dv = (1 - u) * (at(3) - at(0)) + u * (at(2) - at(1));
        const Vector3d dz = std::sqrt(2.) * (at(4) - q + (u - .5) * du + (v - .5) * dv);
        const double det = du.dot(dv.cross(dz));
        if (!(det > 0)) return infinity;
        worst = std::max(
            worst,
            (du.squaredNorm() + dv.squaredNorm() + dz.squaredNorm()) / std::cbrt(det * det));
    }
    return worst;
}

struct Patch
{
    std::vector<size_t> vertices, tids, native_cells;
};

struct Guard
{
    PrismaticMeshInput& input;
    const PrismDominantMesh& hybrid;
    const BadQualityRelocationOptions& options;
    const MatrixXd entry;
    const bool enforce_quality_limits;
    double h, min_volume;
    std::vector<bool> movable;
    std::vector<std::vector<size_t>> incident_native;
    std::unordered_map<size_t, double> tet_limits, native_limits;

    Guard(
        PrismaticMeshInput& data,
        const PrismDominantMesh& cells,
        double floor,
        const BadQualityRelocationOptions& opt,
        bool quality_limits = true)
        : input(data)
        , hybrid(cells)
        , options(opt)
        , entry(data.vertices)
        , enforce_quality_limits(quality_limits)
        , h(data.target_thickness)
        , min_volume(floor)
        , movable(data.vertices.rows(), false)
        , incident_native(data.vertices.rows())
    {
        for (size_t v : input.offset_vertices) {
            const auto& star = input.mesh->get_one_ring_tids_for_vertex(v);
            // Junction protection forbids topology changes, not relocation.
            // These post-reconstruction passes preserve connectivity and check
            // the complete incident reference/native cells for every move.
            movable[v] = !star.empty() && input.vertex_tags[v] == 2 &&
                         input.corr_input_vertex[v] >= 0 && !is_rod_offset_vertex(input, v) &&
                         (!input.rod_state || !input.rod_state->frozen_offset.count(v)) &&
                         (v >= input.fixed_background_vertices.size() ||
                          !input.fixed_background_vertices[v]);
            if (!movable[v]) continue;
            for (size_t tid : star)
                if (input.input_cells[tid] == 1 ||
                    (input.offset_tet_tags[tid] == 1 && min_volume > 0 &&
                     !tet_volume_above_threshold(input.vertices, tet(input, tid), min_volume))) {
                    movable[v] = false;
                    break;
                }
        }
        for (size_t ci = 0; ci < hybrid.cells.size(); ++ci)
            if (hybrid.cells[ci].type != HybridCellType::Tetrahedron)
                for (size_t v : hybrid.cells[ci].vertices) incident_native[v].push_back(ci);
    }

    Patch patch(std::vector<size_t> vertices)
    {
        Patch out;
        out.vertices = std::move(vertices);
        std::set<size_t> tids, cells;
        for (size_t v : out.vertices) {
            const auto& star = input.mesh->get_one_ring_tids_for_vertex(v);
            tids.insert(star.begin(), star.end());
            cells.insert(incident_native[v].begin(), incident_native[v].end());
        }
        out.tids.assign(tids.begin(), tids.end());
        out.native_cells.assign(cells.begin(), cells.end());
        if (!enforce_quality_limits) return out;
        for (size_t tid : out.tids)
            if (!tet_limits.count(tid))
                tet_limits[tid] =
                    std::max(options.amips_threshold, quality(points(entry, tet(input, tid))));
        for (size_t ci : out.native_cells)
            if (!native_limits.count(ci))
                native_limits[ci] =
                    std::max(options.amips_threshold, native_bound(entry, hybrid.cells[ci]));
        return out;
    }

    bool valid(const Patch& patch, const std::vector<Points>& before, bool bounded = true) const
    {
        for (size_t v : patch.vertices)
            if (!movable[v] || !input.vertices.row(v).allFinite() ||
                (bounded && (input.vertices.row(v) - entry.row(v)).norm() >
                                options.max_displacement_ratio * h * (1 + 1e-12)))
                return false;
        for (size_t i = 0; i < patch.tids.size(); ++i) {
            const size_t tid = patch.tids[i];
            const auto t = tet(input, tid);
            const auto p = points(input.vertices, t);
            if (enforce_quality_limits) {
                const double q = quality(p);
                if (!std::isfinite(q) || q > tet_limits.at(tid)) return false;
            }
            const double floor = std::max(
                input.offset_tet_tags[tid] == 1 ? min_volume : 0.,
                input.relative_tet_volume_floor * determinant(before[i]) / 6);
            if (!tet_volume_above_threshold(input.vertices, t, floor) ||
                !positive_segment(before[i], p, floor))
                return false;
        }
        for (size_t ci : patch.native_cells) {
            const auto& c = hybrid.cells[ci];
            const double q = native_bound(input.vertices, c);
            if (!std::isfinite(q) || (enforce_quality_limits && q > native_limits.at(ci)))
                return false;
            if (c.type == HybridCellType::Pyramid)
                for (const auto& t : pyramid_tets(c))
                    if (!tet_volume_above_threshold(input.vertices, t, min_volume)) return false;
        }
        return true;
    }
    std::vector<Points> snapshot(const Patch& patch) const
    {
        std::vector<Points> out;
        for (size_t tid : patch.tids) out.push_back(points(input.vertices, tet(input, tid)));
        return out;
    }
    void accept(const Patch& patch)
    {
        if (!enforce_quality_limits) return;
        for (size_t tid : patch.tids) {
            tet_limits.at(tid) = std::min(
                tet_limits.at(tid),
                std::max(options.amips_threshold, quality(input, tid)));
        }
        for (size_t ci : patch.native_cells)
            native_limits.at(ci) = std::min(
                native_limits.at(ci),
                std::max(options.amips_threshold, native_bound(input.vertices, hybrid.cells[ci])));
    }
};

std::pair<double, double> seed_quality(
    const PrismaticMeshInput& input,
    const std::vector<size_t>& seeds)
{
    double sum = 0, maximum = 0;
    for (size_t tid : seeds) {
        const double q = quality(input, tid);
        sum += q;
        maximum = std::max(maximum, q);
    }
    return {sum, maximum};
}

using Triangle = internal::SurfaceDomain::Triangle;

struct OffsetSurface
{
    std::vector<Triangle> faces;
    std::vector<std::vector<size_t>> incident;

    explicit OffsetSurface(const PrismaticMeshInput& input)
        : incident(input.vertices.rows())
    {
        std::set<Triangle> unique;
        for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid) {
            const auto t = tet(input, tid);
            for (size_t omit = 0; omit < 4; ++omit) {
                Triangle face;
                size_t count = 0;
                for (size_t j = 0; j < 4; ++j)
                    if (j != omit && input.vertex_tags[t[j]] == 2) face[count++] = t[j];
                if (count != 3) continue;
                std::sort(face.begin(), face.end());
                unique.insert(face);
            }
        }
        faces.assign(unique.begin(), unique.end());
        for (size_t j = 0; j < faces.size(); ++j)
            for (size_t v : faces[j]) incident[v].push_back(j);
    }

    std::vector<Triangle> patch(const std::array<size_t, 2>& edge) const
    {
        std::set<size_t> selected;
        for (size_t v : edge) selected.insert(incident[v].begin(), incident[v].end());
        std::map<std::array<size_t, 2>, std::vector<size_t>> adjacent;
        std::queue<size_t> queue;
        std::set<size_t> visited;
        for (size_t f : selected) {
            const auto& t = faces[f];
            if (std::find(t.begin(), t.end(), edge[0]) != t.end() &&
                std::find(t.begin(), t.end(), edge[1]) != t.end()) {
                queue.push(f);
                visited.insert(f);
            }
            for (size_t j = 0; j < 3; ++j) {
                std::array<size_t, 2> e{t[j], t[(j + 1) % 3]};
                if (e[0] > e[1]) std::swap(e[0], e[1]);
                adjacent[e].push_back(f);
            }
        }
        // A cut/open offset surface can have additional fans meeting only at
        // an endpoint. Use the edge-connected fan containing the selected edge.
        while (!queue.empty()) {
            const size_t f = queue.front();
            queue.pop();
            const auto& t = faces[f];
            for (size_t j = 0; j < 3; ++j) {
                std::array<size_t, 2> e{t[j], t[(j + 1) % 3]};
                if (e[0] > e[1]) std::swap(e[0], e[1]);
                for (size_t next : adjacent.at(e))
                    if (visited.insert(next).second) queue.push(next);
            }
        }
        std::vector<Triangle> out;
        for (size_t f : visited) out.push_back(faces[f]);
        return out;
    }
};

struct DomainEvaluation
{
    double energy = 0;
    std::array<Vector3d, 2> gradients{Vector3d::Zero(), Vector3d::Zero()};
};

DomainEvaluation evaluate_domain(
    const Guard& guard,
    const Patch& patch,
    const std::array<size_t, 2>& edge,
    bool derivatives)
{
    DomainEvaluation out;
    for (size_t tid : patch.tids) {
        const auto t = tet(guard.input, tid);
        const auto p = points(guard.input.vertices, t);
        const double q = quality(p);
        out.energy += q / 3;
        if (!std::isfinite(q) || !derivatives) continue;
        using V = Eigen::Matrix<long double, 3, 1>;
        std::array<V, 4> x, det_gradient;
        V sum = V::Zero();
        for (size_t j = 0; j < 4; ++j) {
            x[j] = p[j].cast<long double>() - p[0].cast<long double>();
            sum += x[j];
        }
        const long double det = x[1].dot(x[2].cross(x[3]));
        if (!(det > 0)) {
            out.energy = infinity;
            return out;
        }
        long double squared_edges = 0;
        for (size_t j = 0; j < 4; ++j)
            for (size_t k = j + 1; k < 4; ++k) squared_edges += (x[j] - x[k]).squaredNorm();
        det_gradient[1] = x[2].cross(x[3]);
        det_gradient[2] = x[3].cross(x[1]);
        det_gradient[3] = x[1].cross(x[2]);
        det_gradient[0] = -det_gradient[1] - det_gradient[2] - det_gradient[3];
        for (size_t j = 0; j < 4; ++j)
            for (size_t end = 0; end < 2; ++end)
                if (t[j] == edge[end])
                    out.gradients[end] += ((q / 3) * ((8 * x[j] - 2 * sum) / squared_edges -
                                                      (2.L / 3) * det_gradient[j] / det))
                                              .cast<double>();
    }
    return out;
}

void relocate_collapse_domains(
    Guard& guard,
    const BadQualityRegions& regions,
    nlohmann::json& report)
{
    using Domain = internal::SurfaceDomain;
    using UV = Domain::UV;
    if (guard.options.collapse_edge_iterations == 0) {
        report["collapse_edge_domain"] = {
            {"requested_iterations", 0},
            {"accepted_moves", 0},
            {"uses_target_positions", false},
            {"uses_thickness_energy", false}};
        return;
    }
    struct Item
    {
        std::array<size_t, 2> edge;
        Patch patch;
        std::vector<Triangle> triangles;
    };
    std::vector<Item> items;
    std::set<size_t> energy_tets;
    auto rejected = nlohmann::json::array();
    size_t fixed = 0;
    const OffsetSurface surface(guard.input);
    for (const auto& edge : regions.collapse_edges) {
        if (!guard.movable[edge[0]] || !guard.movable[edge[1]]) {
            ++fixed;
            continue;
        }
        auto triangles = surface.patch(edge);
        std::string failure;
        if (!Domain::build(guard.input.vertices, triangles, edge, failure)) {
            rejected.push_back(
                {{"edge", edge},
                 {"source_vids",
                  {guard.input.source_vertex_ids[edge[0]], guard.input.source_vertex_ids[edge[1]]}},
                 {"reason", failure}});
            continue;
        }
        auto patch = guard.patch({edge[0], edge[1]});
        energy_tets.insert(patch.tids.begin(), patch.tids.end());
        items.push_back({edge, std::move(patch), std::move(triangles)});
    }
    const auto energy = [&]() {
        double total = 0;
        for (size_t tid : energy_tets) total += quality(guard.input, tid) / 3;
        return total;
    };
    auto result = nlohmann::json{
        {"method", "shared offset triangle domain; pulled-back incident tet AMIPS gradient"},
        {"requested_iterations", guard.options.collapse_edge_iterations},
        {"movable_edges", items.size()},
        {"fixed_edges", fixed},
        {"rejected_domains", std::move(rejected)},
        {"energy_before", energy()},
        {"uses_target_positions", false},
        {"uses_thickness_energy", false},
        {"per_cell_quality_limits", false},
        {"energy_tet_count", energy_tets.size()},
        {"acceptance", "strict decrease of summed AMIPS/3 over all incident tets"},
        {"domain_bound", "first outer-boundary hit or UV triangle collapse"}};
    size_t total = 0, blocked = 0;
    auto history = nlohmann::json::array();
    for (int pass = 0; pass < guard.options.collapse_edge_iterations; ++pass) {
        size_t accepted = 0;
        for (const auto& item : items) {
            std::string failure;
            auto domain = Domain::build(guard.input.vertices, item.triangles, item.edge, failure);
            if (!domain) {
                ++blocked;
                continue;
            }
            const auto evaluation = evaluate_domain(guard, item.patch, item.edge, true);
            const auto& g = evaluation.gradients;
            const double scale = std::max(g[0].stableNorm(), g[1].stableNorm());
            if (!std::isfinite(evaluation.energy) || !g[0].allFinite() || !g[1].allFinite() ||
                !(scale > 0)) {
                ++blocked;
                continue;
            }
            const std::array<UV, 2> origin{
                domain->coordinate(item.edge[0]),
                domain->coordinate(item.edge[1])};
            std::array<UV, 2> direction{
                domain->descent(origin[0], g[0] / scale),
                domain->descent(origin[1], g[1] / scale)};
            const double norm = std::max(direction[0].stableNorm(), direction[1].stableNorm());
            if (!(norm > 0) || !std::isfinite(norm)) {
                ++blocked;
                continue;
            }
            for (auto& d : direction) d /= norm;
            const double limit = domain->step_limit(direction);
            if (!(limit > 0) || !std::isfinite(limit)) {
                ++blocked;
                continue;
            }
            const std::array<Vector3d, 2> before{
                guard.input.vertices.row(item.edge[0]),
                guard.input.vertices.row(item.edge[1])};
            const auto snapshot = guard.snapshot(item.patch);
            bool success = false;
            double alpha = .99 * limit;
            for (int bt = 0; bt <= guard.options.max_backtracks; ++bt, alpha *= .5) {
                const std::array<UV, 2> next{
                    origin[0] + alpha * direction[0],
                    origin[1] + alpha * direction[1]};
                if (!domain->valid(next)) continue;
                const std::array<std::optional<Vector3d>, 2> p{
                    domain->position(next[0]),
                    domain->position(next[1])};
                if (!p[0] || !p[1]) continue;
                if (*p[0] == before[0] && *p[1] == before[1]) break;
                for (size_t end = 0; end < 2; ++end)
                    guard.input.vertices.row(item.edge[end]) = p[end]->transpose();
                if (!guard.valid(item.patch, snapshot, false)) continue;
                const double trial = evaluate_domain(guard, item.patch, item.edge, false).energy;
                if (trial < evaluation.energy * (1 - 1e-12)) {
                    guard.accept(item.patch);
                    success = true;
                    ++accepted;
                    ++total;
                    break;
                }
            }
            if (!success) {
                for (size_t end = 0; end < 2; ++end)
                    guard.input.vertices.row(item.edge[end]) = before[end].transpose();
                ++blocked;
            }
        }
        history.push_back({{"iteration", pass + 1}, {"accepted", accepted}, {"energy", energy()}});
        if (accepted == 0) break;
    }
    result["accepted_moves"] = total;
    result["blocked_moves"] = blocked;
    result["energy_after"] = energy();
    result["history"] = std::move(history);
    report["collapse_edge_domain"] = std::move(result);
    logger().info(
        "Offset-domain AMIPS relocation: {} edges, {} accepted moves",
        items.size(),
        total);
}

struct ThicknessTarget
{
    Vector3d normal;
    double initial_residual, limit;
};
struct Evaluation
{
    double energy = 0;
    Vector3d gradient = Vector3d::Zero();
    Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
};
Evaluation evaluate_vertex(
    const Guard& guard,
    const Patch& patch,
    size_t vertex,
    const ThicknessTarget& target,
    bool derivatives)
{
    Evaluation out;
    for (size_t tid : patch.tids) {
        const auto t = tet(guard.input, tid);
        auto p = points(guard.input.vertices, t);
        const double q = quality(p);
        out.energy += q / 3;
        if (!std::isfinite(q)) return out;
        if (!derivatives) continue;
        const int corner = static_cast<int>(std::find(t.begin(), t.end(), vertex) - t.begin());
        const Vector3d reference = p[0];
        for (auto& point : p) point = (point - reference) / guard.h;
        if (corner != 0) {
            std::swap(p[0], p[corner]);
            std::swap(p[1], p[2]);
        }
        Vector3d g;
        Eigen::Matrix3d h;
        const auto c = packed(p);
        AMIPS_jacobian(c, g);
        AMIPS_hessian(c, h);
        out.gradient += g / 3;
        out.hessian += h / 3;
    }
    const size_t parent = guard.input.corr_input_vertex[vertex];
    const Vector3d column = guard.input.vertices.row(vertex) - guard.input.vertices.row(parent);
    const double residual = column.dot(target.normal) / guard.h - 1;
    out.energy += guard.options.thickness_weight * residual * residual;
    if (derivatives) {
        out.gradient += 2 * guard.options.thickness_weight * residual * target.normal;
        out.hessian +=
            2 * guard.options.thickness_weight * target.normal * target.normal.transpose();
    }
    return out;
}
Vector3d newton_step(const Evaluation& e)
{
    if (!e.hessian.allFinite()) return Vector3d::Zero();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(e.hessian);
    if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) return Vector3d::Zero();
    const double floor = std::max(1e-8, 1e-8 * eig.eigenvalues().cwiseAbs().maxCoeff());
    const Vector3d values = eig.eigenvalues().cwiseMax(floor);
    const Vector3d force = eig.eigenvectors().transpose() * e.gradient;
    double damping = 0;
    if ((force.array() / values.array()).matrix().norm() > .2) {
        double lo = 0, hi = force.norm() / .2;
        for (int i = 0; i < 40; ++i) {
            const double mid = (lo + hi) / 2;
            if ((force.array() / (values.array() + mid)).matrix().norm() > .2)
                lo = mid;
            else
                hi = mid;
        }
        damping = hi;
    }
    return -eig.eigenvectors() * (force.array() / (values.array() + damping)).matrix();
}

// Project a normalized displacement onto the intersection of a ball and a
// normal-thickness slab. The entry position is feasible by construction.
Vector3d project(const Vector3d& x, const ThicknessTarget& target, double radius)
{
    return internal::project_relocation_displacement(
        x,
        target.normal,
        std::max(-radius, -target.limit - target.initial_residual),
        std::min(radius, target.limit - target.initial_residual),
        radius);
}

nlohmann::json relocate_thickness_vertices(
    PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    double min_volume,
    const BadQualityRelocationOptions& options,
    const std::vector<size_t>& seed_tets,
    int iterations,
    bool tet_faces)
{
    Guard guard(input, hybrid, min_volume, options, !tet_faces);

    std::vector<bool> on_surface(input.vertices.rows(), false);
    if (tet_faces && iterations > 0) {
        const OffsetSurface surface(input);
        for (size_t v = 0; v < on_surface.size(); ++v) on_surface[v] = !surface.incident[v].empty();
    }
    std::set<size_t> selected;
    std::set<size_t> non_tet_neighbors;
    for (size_t tid : seed_tets)
        if (iterations > 0 && (tet_faces || quality(input, tid) > options.amips_threshold))
            for (size_t v : tet(input, tid))
                if (guard.movable[v]) {
                    if (tet_faces && !options.tet_face_allow_mixed_cells &&
                        !guard.incident_native[v].empty()) {
                        non_tet_neighbors.insert(v);
                        continue;
                    }
                    if (!tet_faces || on_surface[v]) selected.insert(v);
                }
    std::map<size_t, Patch> vertex_patches;
    std::map<size_t, ThicknessTarget> targets;
    std::map<size_t, std::vector<size_t>> vertex_seeds;
    size_t fallback_normals = 0;
    for (size_t v : selected) {
        Vector3d n = Vector3d::Zero();
        if (input.optimal_normals.rows() == input.vertices.rows()) n = input.optimal_normals.row(v);
        const Vector3d column =
            input.vertices.row(v) - input.vertices.row(input.corr_input_vertex[v]);
        if ((v < input.singular_vertex_tags.size() && input.singular_vertex_tags[v] == 1) ||
            !n.allFinite() || !(n.stableNorm() > 0)) {
            n = column;
            ++fallback_normals;
        }
        if (!n.allFinite() || !(n.stableNorm() > 0)) continue;
        n.normalize();
        const Vector3d original_column =
            guard.entry.row(v) - guard.entry.row(input.corr_input_vertex[v]);
        const double original_residual = original_column.dot(n) / guard.h - 1;
        const double current_residual = column.dot(n) / guard.h - 1;
        targets[v] = {
            n,
            original_residual,
            std::max(options.thickness_tolerance, std::abs(current_residual))};
        vertex_patches.emplace(v, guard.patch({v}));
        if (!tet_faces)
            for (size_t tid : seed_tets) {
                const auto t = tet(input, tid);
                if (std::find(t.begin(), t.end(), v) != t.end()) vertex_seeds[v].push_back(tid);
            }
    }
    const auto thickness_stats = [&]() {
        double squared = 0, maximum = 0;
        for (const auto& [v, target] : targets) {
            const Vector3d column =
                input.vertices.row(v) - input.vertices.row(input.corr_input_vertex[v]);
            const double error = std::abs(column.dot(target.normal) / guard.h - 1);
            squared += error * error;
            maximum = std::max(maximum, error);
        }
        return nlohmann::json{
            {"count", targets.size()},
            {"rms_relative_error", targets.empty() ? 0 : std::sqrt(squared / targets.size())},
            {"maximum_relative_error", maximum}};
    };
    std::set<size_t> energy_tets;
    for (const auto& [v, patch] : vertex_patches)
        energy_tets.insert(patch.tids.begin(), patch.tids.end());
    const auto energy = [&]() {
        double e = 0;
        for (size_t tid : energy_tets) e += quality(input, tid) / 3;
        for (const auto& [v, target] : targets) {
            const Vector3d column =
                input.vertices.row(v) - input.vertices.row(input.corr_input_vertex[v]);
            const double r = column.dot(target.normal) / guard.h - 1;
            e += options.thickness_weight * r * r;
        }
        return e;
    };
    auto result = nlohmann::json{
        {"requested_iterations", iterations},
        {"per_cell_quality_limits", !tet_faces},
        {"tet_only_vertex_stars", tet_faces && !options.tet_face_allow_mixed_cells},
        {"allows_mixed_native_cells", tet_faces && options.tet_face_allow_mixed_cells},
        {"excluded_non_tet_vertices", non_tet_neighbors.size()},
        {"energy_tet_count", energy_tets.size()},
        {"offset_vertices", targets.size()},
        {"fallback_column_normals", fallback_normals},
        {"thickness_weight", options.thickness_weight},
        {"thickness_tolerance", options.thickness_tolerance},
        {"energy_before", energy()},
        {"thickness_before", thickness_stats()}};
    if (tet_faces) {
        std::vector<int64_t> selected_ids;
        size_t mixed_vertices = 0;
        for (const auto& [v, target] : targets) selected_ids.push_back(input.source_vertex_ids[v]);
        for (const auto& [v, target] : targets) mixed_vertices += !guard.incident_native[v].empty();
        result["selected_source_vids"] = selected_ids;
        result["selected_mixed_native_vertices"] = mixed_vertices;
        result["shell_tets_before"] = statistics(input, seed_tets, options.amips_threshold);
        result["selection"] = options.tet_face_allow_mixed_cells
                                  ? "all movable offset-surface vertices incident to actual shell "
                                    "tets; mixed native one-rings allowed"
                                  : "all offset-surface vertices incident to actual shell tets; "
                                    "native one-ring contains only tetrahedra";
        result["acceptance"] = "strict decrease of total incident AMIPS/3 plus projected-thickness "
                               "energy; no individual quality caps";
    }
    auto history = nlohmann::json::array();
    size_t relocations = 0, newton_moves = 0, gradient_moves = 0, gradient_attempts = 0;
    size_t newton_stagnations = 0, tiny_newton_moves = 0;
    for (int pass = 0; pass < iterations; ++pass) {
        size_t accepted = 0, attempted = 0, accepted_newton = 0, accepted_gradient = 0;
        for (const auto& [v, patch] : vertex_patches) {
            std::pair<double, double> seeds{0, 0};
            if (!tet_faces) {
                seeds = seed_quality(input, vertex_seeds.at(v));
                if (seeds.second <= options.amips_threshold) continue;
            }
            ++attempted;
            const auto& target = targets.at(v);
            const auto evaluation = evaluate_vertex(guard, patch, v, target, true);
            if (!std::isfinite(evaluation.energy) || !evaluation.gradient.allFinite()) continue;
            const Vector3d before = input.vertices.row(v), origin = guard.entry.row(v);
            const Vector3d x = (before - origin) / guard.h;
            const auto snapshot = guard.snapshot(patch);
            struct Trial
            {
                Vector3d position;
                double energy;
            };
            const auto search = [&](const Vector3d& direction,
                                    bool is_newton) -> std::optional<Trial> {
                if (!direction.allFinite() || !(evaluation.gradient.dot(direction) < 0))
                    return std::nullopt;
                double alpha = 1;
                for (int bt = 0; bt <= options.max_backtracks; ++bt, alpha *= .5) {
                    const Vector3d next =
                        origin +
                        guard.h *
                            project(x + alpha * direction, target, options.max_displacement_ratio);
                    if (next == before) {
                        if (is_newton) ++newton_stagnations;
                        break;
                    }
                    const double slope = evaluation.gradient.dot((next - before) / guard.h);
                    if (!tet_faces && !(slope < 0)) continue;
                    input.vertices.row(v) = next.transpose();
                    bool improves = false;
                    double trial_energy = infinity;
                    const double residual =
                        (next - Vector3d(input.vertices.row(input.corr_input_vertex[v])))
                                .dot(target.normal) /
                            guard.h -
                        1;
                    if (std::abs(residual) <= target.limit + 1e-12 &&
                        guard.valid(patch, snapshot)) {
                        trial_energy = evaluate_vertex(guard, patch, v, target, false).energy;
                        improves = trial_energy < evaluation.energy * (1 - 1e-12);
                        if (!tet_faces) {
                            const auto next_seeds = seed_quality(input, vertex_seeds.at(v));
                            improves = trial_energy <= evaluation.energy + 1e-4 * slope &&
                                       next_seeds.first < seeds.first * (1 - 1e-12) &&
                                       next_seeds.second <= seeds.second;
                        }
                    }
                    input.vertices.row(v) = before.transpose();
                    if (improves) return Trial{next, trial_energy};
                }
                return std::nullopt;
            };
            auto trial = search(newton_step(evaluation), true);
            const bool tiny = trial && (trial->position - before).stableNorm() / guard.h <=
                                           std::sqrt(std::numeric_limits<double>::epsilon());
            bool used_gradient = false;
            if (!trial || tiny) {
                if (tiny) ++tiny_newton_moves;
                const double scale = evaluation.gradient.cwiseAbs().maxCoeff();
                if (scale > 0) {
                    ++gradient_attempts;
                    const Vector3d scaled = evaluation.gradient / scale;
                    const auto gradient = search(-.2 * scaled / scaled.stableNorm(), false);
                    if (gradient && (!trial || gradient->energy < trial->energy)) {
                        trial = gradient;
                        used_gradient = true;
                    }
                }
            }
            if (!trial) continue;
            input.vertices.row(v) = trial->position.transpose();
            guard.accept(patch);
            ++accepted;
            ++relocations;
            if (used_gradient) {
                ++gradient_moves;
                ++accepted_gradient;
            } else {
                ++newton_moves;
                ++accepted_newton;
            }
        }
        history.push_back(
            {{"iteration", pass + 1},
             {"attempted", attempted},
             {"accepted", accepted},
             {"accepted_newton", accepted_newton},
             {"accepted_gradient", accepted_gradient},
             {"energy", energy()}});
        if (accepted == 0) break;
    }
    result["accepted_moves"] = relocations;
    result["solver"] = "damped Newton with projected gradient fallback";
    result["projection"] =
        "Euclidean projection onto displacement ball intersected with thickness slab";
    result["accepted_newton_moves"] = newton_moves;
    result["gradient_fallback_attempts"] = gradient_attempts;
    result["accepted_gradient_moves"] = gradient_moves;
    result["newton_stagnations"] = newton_stagnations;
    result["tiny_newton_candidates"] = tiny_newton_moves;
    result["history"] = std::move(history);
    result["energy_after"] = energy();
    result["thickness_after"] = thickness_stats();
    result["maximum_displacement"] = (input.vertices - guard.entry).rowwise().norm().maxCoeff();
    if (tet_faces)
        result["shell_tets_after"] = statistics(input, seed_tets, options.amips_threshold);
    return result;
}

void validate_options(const BadQualityRelocationOptions& o, double floor)
{
    if (o.collapse_edge_iterations < 0 || o.collapse_edge_iterations > 200 ||
        o.prism_tet_iterations < 0 || o.prism_tet_iterations > 200 || o.tet_face_iterations < 0 ||
        o.tet_face_iterations > 200 || !std::isfinite(o.amips_threshold) || o.amips_threshold < 3 ||
        !std::isfinite(o.thickness_weight) || o.thickness_weight <= 0 ||
        !std::isfinite(o.thickness_tolerance) || o.thickness_tolerance < 0 ||
        o.thickness_tolerance >= 1 || !std::isfinite(o.max_displacement_ratio) ||
        o.max_displacement_ratio <= 0 || o.max_backtracks < 0 || o.max_backtracks > 60 ||
        !std::isfinite(floor) || floor < 0)
        throw std::invalid_argument("Invalid bad-quality relocation options");
}
} // namespace

BadQualityRegions classify_bad_quality_regions(
    const PrismaticMeshInput& input,
    const PrismDominantMesh& hybrid,
    double threshold)
{
    if (!std::isfinite(threshold) || threshold < 3)
        throw std::invalid_argument("Relocation AMIPS threshold must be finite and at least 3");
    BadQualityRegions out;
    std::set<std::array<size_t, 2>> edges;
    std::set<int64_t> valid_prisms;
    if (hybrid.report.contains("transition_and_tet_regions"))
        for (const auto& region : hybrid.report["transition_and_tet_regions"])
            if (region.contains("minimum_global_prism_jacobian") &&
                region["minimum_global_prism_jacobian"].is_number() &&
                region["minimum_global_prism_jacobian"].get<double>() > 0)
                valid_prisms.insert(region["candidate"].get<int64_t>());
    for (const auto& c : hybrid.cells) {
        if (c.type != HybridCellType::Tetrahedron) continue;
        const size_t tid = c.source_tets.at(0);
        if (input.offset_tet_tags[tid] != 1 || !(quality(input, tid) > threshold)) continue;
        bool repeated = false;
        for (size_t j = 0; j < 4; ++j)
            for (size_t k = j + 1; k < 4; ++k) {
                size_t a = c.vertices[j], b = c.vertices[k];
                if (!same_component(input, a, b)) continue;
                if (a > b) std::swap(a, b);
                edges.insert({a, b});
                repeated = true;
            }
        if (repeated)
            out.collapse_edge_tets.push_back(tid);
        else if (valid_prisms.count(c.prism_candidate))
            out.prism_tets.push_back(tid);
        else
            out.other_tets.push_back(tid);
    }
    out.collapse_edges.assign(edges.begin(), edges.end());
    return out;
}

void relocate_bad_quality_regions(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_volume,
    const BadQualityRelocationOptions& options)
{
    validate_options(options, min_volume);
    if (options.collapse_edge_iterations == 0 && options.prism_tet_iterations == 0 &&
        options.tet_face_iterations == 0)
        return;
    validate_prism_dominant_mesh(input, hybrid, min_volume);
    if ((options.prism_tet_iterations > 0 || options.tet_face_iterations > 0) &&
        (!(input.target_thickness > 0) || !std::isfinite(input.target_thickness)))
        throw std::invalid_argument("Bad-quality relocation requires a positive target thickness");
    const auto regions = classify_bad_quality_regions(input, hybrid, options.amips_threshold);
    Guard first_guard(input, hybrid, min_volume, options, false);
    auto& report = hybrid.report["bad_quality_relocation"];
    report = {
        {"amips_threshold", options.amips_threshold},
        {"quality_guard",
         {{"collapse_edge", "total incident AMIPS decreases; no per-cell quality limits"},
          {"prism_tet",
           "per affected tet: max(threshold, entry AMIPS), tightened after improvement"},
          {"tet_face", "total AMIPS plus thickness energy decreases; no per-cell quality limits"}}},
        {"collapse_edge_tet_ids", regions.collapse_edge_tets},
        {"prism_tet_ids", regions.prism_tets},
        {"other_bad_tet_ids", regions.other_tets},
        {"collapse_edge_before",
         statistics(input, regions.collapse_edge_tets, options.amips_threshold)},
        {"prism_tet_before", statistics(input, regions.prism_tets, options.amips_threshold)},
        {"first_class_displacement_bound", "shared local offset surface domain"},
        {"prism_maximum_displacement_ratio", options.max_displacement_ratio}};
    logger().info(
        "Bad-quality relocation: {} residual-edge tets ({} edges), {} prism-derived tets, {} other "
        "bad tets",
        regions.collapse_edge_tets.size(),
        regions.collapse_edges.size(),
        regions.prism_tets.size(),
        regions.other_tets.size());

    relocate_collapse_domains(first_guard, regions, report);
    report["collapse_edge_after_domain"] =
        statistics(input, regions.collapse_edge_tets, options.amips_threshold);
    report["prism_tet_relocation"] = relocate_thickness_vertices(
        input,
        hybrid,
        min_volume,
        options,
        regions.prism_tets,
        options.prism_tet_iterations,
        false);
    std::vector<size_t> surface_tet_ids;
    if (options.tet_face_iterations > 0)
        for (const auto& c : hybrid.cells)
            if (c.type == HybridCellType::Tetrahedron &&
                input.offset_tet_tags[c.source_tets.at(0)] == 1)
                surface_tet_ids.push_back(c.source_tets[0]);
    report["tet_face_relocation"] = relocate_thickness_vertices(
        input,
        hybrid,
        min_volume,
        options,
        surface_tet_ids,
        options.tet_face_iterations,
        true);
    report["collapse_edge_after"] =
        statistics(input, regions.collapse_edge_tets, options.amips_threshold);
    report["prism_tet_after"] = statistics(input, regions.prism_tets, options.amips_threshold);
    report["maximum_displacement"] =
        (input.vertices - first_guard.entry).rowwise().norm().maxCoeff();
    report["connectivity_unchanged"] = true;
    validate_prism_dominant_mesh(input, hybrid, min_volume);
    report["reference_and_native_cells_positive"] = true;
    std::vector<int64_t> relocated_no_collapse_vertices;
    bool protected_positions_fixed = true;
    if (input.junction_protection) {
        for (size_t v : input.junction_protection->offset_vertices)
            if (input.vertices.row(v) != first_guard.entry.row(v))
                relocated_no_collapse_vertices.push_back(input.source_vertex_ids[v]);
        for (const auto& [v, position] : input.junction_protection->offset_positions)
            protected_positions_fixed &= input.vertices.row(v).transpose() == position;
    }
    report["no_collapse_offset_vertices_relocated"] = relocated_no_collapse_vertices.size();
    report["relocated_no_collapse_source_vids"] = relocated_no_collapse_vertices;
    if (hybrid.report.contains("junction_protection")) {
        auto& protection = hybrid.report["junction_protection"];
        protection["offset_positions_fixed"] = protected_positions_fixed;
        protection["no_collapse_offsets_may_relocate"] = true;
    }
    logger().info(
        "Bad-quality relocation: {} domain moves, {} prism-tet vertex moves, {} tet-face vertex "
        "moves",
        report["collapse_edge_domain"]["accepted_moves"].get<size_t>(),
        report["prism_tet_relocation"]["accepted_moves"].get<size_t>(),
        report["tet_face_relocation"]["accepted_moves"].get<size_t>());
}
void expand_bad_tet_transition_regions(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    const BadQualityRelocationOptions& options,
    int rounds)
{
    if (rounds < 0 || rounds > 5)
        throw std::invalid_argument("Transition expansion rounds must be in [0,5]");
    if (rounds == 0) return;
    validate_options(options, min_tet_volume);
    validate_prism_dominant_mesh(input, hybrid, min_tet_volume);
    if (hybrid.report.is_null()) hybrid.report = nlohmann::json::object();
    const auto shell_tets = [&]() {
        std::vector<size_t> ids;
        for (const auto& cell : hybrid.cells)
            if (cell.type == HybridCellType::Tetrahedron &&
                input.offset_tet_tags.at(cell.source_tets.at(0)) == 1)
                ids.push_back(cell.source_tets[0]);
        return ids;
    };
    const auto source_ids = [&](const std::set<size_t>& vertices) {
        std::vector<int64_t> ids;
        for (size_t v : vertices) ids.push_back(input.source_vertex_ids.at(v));
        return ids;
    };
    const auto counts = [&]() {
        return nlohmann::json{
            {"prisms", hybrid.report.value("prisms", size_t(0))},
            {"pyramids", hybrid.report.value("pyramids", size_t(0))},
            {"band_tetrahedra", shell_tets().size()}};
    };
    nlohmann::json history = nlohmann::json::array();
    nlohmann::json result = {
        {"requested_rounds", rounds},
        {"amips_threshold", options.amips_threshold},
        {"growth", "one incident offset-triangle vertex ring from frozen bad-tet offset seeds"},
        {"marker_field", "transition_mark; vertex labels and correspondence unchanged"},
        {"optimization", "repeat configured relocation stages; no collapse or target smoothing"},
        {"displacement_bounds", "reset at entry to each repeated relocation stage"},
        {"initial_shell_tets", statistics(input, shell_tets(), options.amips_threshold)},
        {"initial_native_counts", counts()},
        {"initial_relocation",
         hybrid.report.value("bad_quality_relocation", nlohmann::json::object())}};
    for (int pass = 0; pass < rounds; ++pass) {
        std::vector<size_t> bad;
        const auto before_tets = shell_tets();
        for (size_t tid : before_tets)
            if (quality(input, tid) > options.amips_threshold) bad.push_back(tid);
        if (bad.empty()) {
            result["stop_reason"] = "no_bad_actual_shell_tets";
            break;
        }
        std::set<size_t> seeds;
        nlohmann::json bad_cells = nlohmann::json::array();
        for (size_t tid : bad) {
            std::array<int64_t, 4> ids;
            const auto vertices = tet(input, tid);
            for (size_t j = 0; j < 4; ++j) ids[j] = input.source_vertex_ids[vertices[j]];
            bad_cells.push_back(
                {{"reference_tet_id", tid}, {"source_vids", ids}, {"amips", quality(input, tid)}});
            for (size_t v : tet(input, tid))
                if (input.vertex_tags[v] == 2) seeds.insert(v);
        }
        const OffsetSurface surface(input);
        std::set<size_t> face_ids;
        for (size_t v : seeds)
            face_ids.insert(surface.incident[v].begin(), surface.incident[v].end());
        std::set<size_t> grown = seeds;
        nlohmann::json marked_faces = nlohmann::json::array();
        for (size_t f : face_ids) {
            const auto& face = surface.faces[f];
            grown.insert(face.begin(), face.end());
            marked_faces.push_back(
                std::array<int64_t, 3>{
                    input.source_vertex_ids[face[0]],
                    input.source_vertex_ids[face[1]],
                    input.source_vertex_ids[face[2]]});
        }
        const auto previous_ids =
            hybrid.report.value("marked_offset_source_vids", std::set<int64_t>{});
        std::set<size_t> previous, marks = grown, added;
        for (size_t v : input.offset_vertices)
            if (previous_ids.count(input.source_vertex_ids[v])) {
                previous.insert(v);
                marks.insert(v);
            }
        std::set_difference(
            marks.begin(),
            marks.end(),
            previous.begin(),
            previous.end(),
            std::inserter(added, added.end()));
        nlohmann::json step = {
            {"round", pass + 1},
            {"bad_reference_tet_ids", bad},
            {"bad_tets_before", std::move(bad_cells)},
            {"shell_tets_before", statistics(input, before_tets, options.amips_threshold)},
            {"native_counts_before", counts()},
            {"seed_offset_source_vids", source_ids(seeds)},
            {"one_ring_offset_source_vids", source_ids(grown)},
            {"one_ring_faces_source_vids", marked_faces},
            {"preserved_marked_vertices", previous.size()},
            {"new_marked_source_vids", source_ids(added)},
            {"forced_transition_source_vids", source_ids(marks)}};
        logger().info(
            "Bad-tet transition expansion {}: {} bad tets, {} seed offsets, {} faces, {} new marks",
            pass + 1,
            bad.size(),
            seeds.size(),
            face_ids.size(),
            added.size());
        auto rebuilt = build_prism_dominant_mesh(input, min_tet_volume, marks);
        hybrid = std::move(rebuilt);
        step["native_counts_after_reconstruction"] = counts();
        step["shell_tets_after_reconstruction"] =
            statistics(input, shell_tets(), options.amips_threshold);
        relocate_bad_quality_regions(input, hybrid, min_tet_volume, options);
        step["shell_tets_after_optimization"] =
            statistics(input, shell_tets(), options.amips_threshold);
        step["relocation"] =
            hybrid.report.value("bad_quality_relocation", nlohmann::json::object());
        history.push_back(std::move(step));
    }
    result["performed_rounds"] = history.size();
    if (!result.contains("stop_reason"))
        result["stop_reason"] =
            statistics(input, shell_tets(), options.amips_threshold)["over_threshold"]
                        .get<size_t>() == 0
                ? "no_bad_actual_shell_tets"
                : "round_budget_reached";
    result["rounds"] = std::move(history);
    result["final_shell_tets"] = statistics(input, shell_tets(), options.amips_threshold);
    result["final_native_counts"] = counts();
    hybrid.report["bad_tet_transition_expansion"] = std::move(result);
}
} // namespace wmtk::components::prismatic_mesh
