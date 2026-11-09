#include "junction_protection.hpp"
#include "prism_jacobian.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Edge = std::array<size_t, 2>;
using Tri = std::array<size_t, 3>;
using Tet = std::array<size_t, 4>;
template <typename T>
T sorted(T value)
{
    std::sort(value.begin(), value.end());
    return value;
}

} // namespace

void initialize_junction_protection(PrismaticMeshInput& input)
{
    if (input.junction_protection) return;
    auto state = std::make_shared<JunctionProtection>();
    std::set<Tri> surface_faces;
    std::set<Edge> surface_edges, rod_edges, volume_edges;
    std::set<size_t> surface_vertices;
    std::set<Edge> explicit_surface_edges;
    for (const auto& f : input.source_faces) {
        surface_vertices.insert(f.begin(), f.end());
        for (int j = 0; j < 3; ++j)
            explicit_surface_edges.insert(sorted(Edge{f[j], f[(j + 1) % 3]}));
    }
    for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid) {
        std::vector<size_t> inside;
        for (int j = 0; j < 4; ++j) {
            const size_t v = input.tetrahedra(tid, j);
            if (input.vertex_tags[v] == 1) inside.push_back(v);
        }
        if (inside.size() >= 3) {
            surface_vertices.insert(inside.begin(), inside.end());
            for (size_t a = 0; a < inside.size(); ++a)
                for (size_t b = a + 1; b < inside.size(); ++b)
                    surface_edges.insert(sorted(Edge{inside[a], inside[b]}));
        }
        if (inside.size() == 4 && input.input_cells[tid] == 1)
            for (size_t a = 0; a < 4; ++a)
                for (size_t b = a + 1; b < 4; ++b)
                    volume_edges.insert(sorted(Edge{inside[a], inside[b]}));
        if (inside.size() == 3 && input.offset_tet_tags[tid] == 1) {
            surface_faces.insert(sorted(Tri{inside[0], inside[1], inside[2]}));
            surface_vertices.insert(inside.begin(), inside.end());
        }
    }
    for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid) {
        if (input.offset_tet_tags[tid] != 1) continue;
        std::vector<size_t> inside;
        for (int j = 0; j < 4; ++j) {
            const size_t v = input.tetrahedra(tid, j);
            if (input.vertex_tags[v] == 1) inside.push_back(v);
        }
        if (inside.size() == 2 && !surface_edges.count(sorted(Edge{inside[0], inside[1]})))
            rod_edges.insert(sorted(Edge{inside[0], inside[1]}));
    }
    if (!input.source_edges.empty()) {
        rod_edges.clear();
        for (auto e : input.source_edges) {
            e = sorted(e);
            if (input.vertex_tags[e[0]] == 1 && input.vertex_tags[e[1]] == 1 &&
                !explicit_surface_edges.count(e) && !volume_edges.count(e))
                rod_edges.insert(e);
        }
    }
    std::map<size_t, size_t> rod_degree;
    for (const auto& edge : rod_edges)
        for (size_t v : edge) ++rod_degree[v];
    for (const auto& [v, degree] : rod_degree) {
        if (surface_vertices.count(v)) state->rod_shell_vertices.insert(v);
        // Path stops and protected connections are different sets. A free end
        // stops traversal but has a regular terminal frame, not a frozen star.
        if (degree != 2 || surface_vertices.count(v)) state->rod_end_vertices.insert(v);
        if (degree == 1 && !surface_vertices.count(v)) state->rod_free_end_vertices.insert(v);
        if (degree > 2) state->rod_joint_vertices.insert(v);
    }
    std::map<Edge, size_t> face_counts;
    for (const auto& face : surface_faces)
        for (size_t j = 0; j < 3; ++j) ++face_counts[sorted(Edge{face[j], face[(j + 1) % 3]})];
    std::map<size_t, size_t> intersection_degree;
    for (const auto& [edge, count] : face_counts) {
        if (count == 1 && !volume_edges.count(edge))
            state->open_boundary_vertices.insert(edge.begin(), edge.end());
        if (count > 2)
            for (size_t v : edge) ++intersection_degree[v];
    }
    for (const auto& [v, degree] : intersection_degree)
        if (degree == 1) {
            state->shell_curve_end_vertices.insert(v);
            if (!state->open_boundary_vertices.count(v)) state->shell_terminal_vertices.insert(v);
        }
    state->centers = state->rod_shell_vertices;
    state->centers.insert(state->rod_joint_vertices.begin(), state->rod_joint_vertices.end());
    state->layered_centers = input.layered_junction_vertices;
    state->centers.insert(state->layered_centers.begin(), state->layered_centers.end());
    state->centers.insert(
        state->shell_terminal_vertices.begin(),
        state->shell_terminal_vertices.end());
    for (size_t v : state->layered_centers)
        if (input.vertex_tags[v] == 2) state->offset_vertices.insert(v);
    for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid) {
        if (input.offset_tet_tags[tid] != 1 || input.input_cells[tid] == 1) continue;
        Tet tet;
        for (size_t j = 0; j < 4; ++j) tet[j] = input.tetrahedra(tid, j);
        if (!std::any_of(tet.begin(), tet.end(), [&](size_t v) { return state->centers.count(v); }))
            continue;
        state->original_band_cells.push_back(sorted(tet));
        for (size_t v : tet)
            if (input.vertex_tags[v] == 2) state->offset_vertices.insert(v);
    }
    for (size_t v : state->offset_vertices)
        state->offset_positions.emplace(v, input.vertices.row(v));
    input.junction_protection = std::move(state);
}

bool junction_tet_protected(const PrismaticMeshInput& input, const Tet& tet)
{
    return input.junction_protection && std::any_of(tet.begin(), tet.end(), [&](size_t v) {
               return input.junction_protection->centers.count(v);
           });
}

bool junction_collapse_allowed(const PrismaticMeshInput& input, size_t removed)
{
    return !input.junction_protection || !input.junction_protection->offset_vertices.count(removed);
}

void verify_junction_one_rings(PrismaticMeshInput& input)
{
    if (!input.junction_protection) return;
    std::set<Tet> live;
    for (Eigen::Index i = 0; i < input.tetrahedra.rows(); ++i) {
        Tet tet;
        for (size_t j = 0; j < 4; ++j) tet[j] = input.tetrahedra(i, j);
        live.insert(sorted(tet));
    }
    for (const auto& tet : input.junction_protection->original_band_cells)
        if (!live.count(tet))
            throw std::runtime_error("Connection-point one-ring changed during collapse/unlock");
    for (const auto& [v, position] : input.junction_protection->offset_positions)
        if (!(input.vertices.row(v).transpose().array() == position.array()).all())
            throw std::runtime_error("Protected connection offset vertex moved");
    input.junction_protection->collapse_connectivity_verified = true;
}

void apply_junction_targets(PrismaticMeshInput& input)
{
    if (!input.junction_protection) return;
    // Ordinary shell component targets assume all same-parent points can merge.
    // These stars cannot collapse, so retain distinct positions instead of
    // squeezing several surviving vertices toward one component target.
    for (const auto& [v, position] : input.junction_protection->offset_positions) {
        if (v >= input.vertex_component_ids.size() || input.vertex_component_ids[v] < 0) continue;
        input.target_positions.row(v) = position.transpose();
        input.singular_vertex_tags[v] = 1;
    }
}

nlohmann::json junction_protection_report(const PrismaticMeshInput& input)
{
    if (!input.junction_protection) return nullptr;
    const auto source_ids = [&](const auto& vertices) {
        std::vector<int64_t> ids;
        for (size_t v : vertices) ids.push_back(input.source_vertex_ids[v]);
        return ids;
    };
    const auto& state = *input.junction_protection;
    return {
        {"rod_shell_source_vids", source_ids(state.rod_shell_vertices)},
        {"rod_end_source_vids", source_ids(state.rod_end_vertices)},
        {"rod_free_end_source_vids", source_ids(state.rod_free_end_vertices)},
        {"rod_joint_source_vids", source_ids(state.rod_joint_vertices)},
        {"layered_junction_source_vids", source_ids(state.layered_centers)},
        {"layered_correspondence", input.layered_correspondence_report},
        {"shell_curve_end_source_vids", source_ids(state.shell_curve_end_vertices)},
        {"open_boundary_source_vids", source_ids(state.open_boundary_vertices)},
        {"shell_terminal_source_vids", source_ids(state.shell_terminal_vertices)},
        {"protected_center_source_vids", source_ids(state.centers)},
        {"no_collapse_offset_source_vids", source_ids(state.offset_vertices)},
        {"protected_one_ring_cells", state.original_band_cells.size()},
        {"offset_positions_fixed", true},
        {"collapse_connectivity_verified", state.collapse_connectivity_verified}};
}

void smooth_transition_regions(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    int iterations)
{
    if (iterations < 0 || iterations > 100)
        throw std::runtime_error("transition smoothing: iterations must be in [0,100]");
    if (iterations == 0) return;
    const MatrixXd original = input.vertices;
    const size_t nv = original.rows(), nt = input.tetrahedra.rows();
    std::vector<Tet> tets(nt);
    std::vector<std::vector<size_t>> incident_tets(nv), incident_cells(nv);
    std::vector<std::set<size_t>> stars(nv), surface_neighbors(nv);
    std::map<Tri, std::pair<size_t, Tri>> boundary_faces;
    std::set<size_t> seeds, frame_anchors;
    if (input.junction_protection) seeds = input.junction_protection->offset_vertices;
    if (hybrid.report.contains("marked_offset_source_vids")) {
        const auto ids = hybrid.report["marked_offset_source_vids"].get<std::set<int64_t>>();
        for (size_t v = 0; v < nv; ++v)
            if (ids.count(input.source_vertex_ids[v])) seeds.insert(v);
    }
    const auto faces = [](const Tet& t) -> std::array<Tri, 4> {
        return {{{t[1], t[2], t[3]}, {t[0], t[3], t[2]}, {t[0], t[1], t[3]}, {t[0], t[2], t[1]}}};
    };
    for (size_t tid = 0; tid < nt; ++tid) {
        auto& tet = tets[tid];
        for (size_t j = 0; j < 4; ++j) tet[j] = input.tetrahedra(tid, j);
        for (size_t v : tet) incident_tets[v].push_back(tid);
        // Use the band/solid union, even when exterior background is retained.
        if (input.offset_tet_tags[tid] != 1 && input.input_cells[tid] != 1) continue;
        for (size_t v : tet) stars[v].insert(tet.begin(), tet.end());
        for (const auto& f : faces(tet)) {
            auto& entry = boundary_faces[sorted(f)];
            ++entry.first;
            entry.second = f;
        }
    }
    for (const auto& [key, entry] : boundary_faces) {
        if (entry.first != 1) continue;
        for (size_t v : key)
            for (size_t w : key)
                if (v != w) surface_neighbors[v].insert(w);
    }
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci) {
        const auto& cell = hybrid.cells[ci];
        for (size_t v : cell.vertices) incident_cells[v].push_back(ci);
        // A completed rod section is an interface anchor, not part of the cap fairing.
        if (cell.type == HybridCellType::Prism &&
            std::count_if(cell.vertices.begin(), cell.vertices.end(), [&](size_t v) {
                return input.vertex_tags[v] == 1;
            }) == 2)
            frame_anchors.insert(cell.vertices.begin(), cell.vertices.end());
    }
    auto region = seeds;
    for (size_t v : seeds) region.insert(surface_neighbors[v].begin(), surface_neighbors[v].end());
    std::map<size_t, double> movable;
    for (size_t v : region) {
        if (input.vertex_tags[v] != 2 || surface_neighbors[v].empty() || frame_anchors.count(v) ||
            input.corr_input_vertex[v] < 0)
            continue;
        double scale = 0;
        for (size_t w : surface_neighbors[v])
            scale += (Vector3d(original.row(w)) - Vector3d(original.row(v))).norm();
        movable[v] = scale / surface_neighbors[v].size();
    }
    // Positive heights across an oriented surface hinge measure a concave fold.
    // Penalize both directions; already convex caps need not be flattened.
    struct Hinge
    {
        Tri first, second;
        size_t opposite_first, opposite_second;
    };
    std::map<Edge, std::vector<Tri>> edge_faces;
    for (const auto& [key, entry] : boundary_faces)
        if (entry.first == 1)
            for (size_t j = 0; j < 3; ++j)
                edge_faces[sorted(Edge{key[j], key[(j + 1) % 3]})].push_back(entry.second);
    std::vector<Hinge> hinges;
    std::vector<std::vector<size_t>> vertex_hinges(nv);
    for (const auto& [edge, fs] : edge_faces) {
        if (fs.size() != 2) continue;
        const auto opposite = [&](const Tri& f) {
            return *std::find_if(f.begin(), f.end(), [&](size_t v) {
                return v != edge[0] && v != edge[1];
            });
        };
        const size_t a = opposite(fs[0]), b = opposite(fs[1]);
        const Tet corners = {edge[0], edge[1], a, b};
        if (std::none_of(corners.begin(), corners.end(), [&](size_t v) {
                return movable.count(v);
            }))
            continue;
        for (size_t v : corners) vertex_hinges[v].push_back(hinges.size());
        hinges.push_back({fs[0], fs[1], a, b});
    }
    const auto hinge_energy = [&](size_t hi) {
        const auto& h = hinges[hi];
        const auto height = [&](const Tri& f, size_t opposite) {
            const Vector3d a = input.vertices.row(f[0]);
            const Vector3d b = input.vertices.row(f[1]);
            const Vector3d c = input.vertices.row(f[2]);
            const Vector3d n = (b - a).cross(c - a);
            if (n.norm() == 0) return std::numeric_limits<double>::infinity();
            return std::max(0., n.normalized().dot(Vector3d(input.vertices.row(opposite)) - a));
        };
        const double a = height(h.first, h.opposite_second);
        const double b = height(h.second, h.opposite_first);
        return a * a + b * b;
    };
    const auto local_energy = [&](size_t v) {
        double value = 0;
        for (size_t hi : vertex_hinges[v]) value += hinge_energy(hi);
        return value;
    };
    const auto total_energy = [&]() {
        double value = 0;
        for (size_t hi = 0; hi < hinges.size(); ++hi) value += hinge_energy(hi);
        return value;
    };
    const double initial_concavity = total_energy();

    // Intersect a correspondence ray with the convex hull of its input vertex's
    // active one-ring. Supporting planes are found directly in these small stars;
    // the immutable snapshot prevents inflation propagating from star to star.
    struct Plane
    {
        Vector3d normal;
        Vector3d origin;
    };
    std::map<size_t, std::vector<Plane>> hulls;
    const auto hull = [&](size_t parent) -> const std::vector<Plane>& {
        auto [it, inserted] = hulls.try_emplace(parent);
        if (!inserted) return it->second;
        const std::vector<size_t> cloud(stars[parent].begin(), stars[parent].end());
        double diameter = 0;
        const Vector3d center = original.row(parent);
        for (size_t v : cloud)
            diameter = std::max(diameter, (Vector3d(original.row(v)) - center).norm());
        const double tolerance = 1e-10 * diameter;
        for (size_t i = 0; i < cloud.size(); ++i)
            for (size_t j = i + 1; j < cloud.size(); ++j)
                for (size_t k = j + 1; k < cloud.size(); ++k) {
                    const Vector3d a = original.row(cloud[i]);
                    const Vector3d b = original.row(cloud[j]);
                    const Vector3d c = original.row(cloud[k]);
                    Vector3d n = (b - a).cross(c - a);
                    if (n.norm() <= tolerance * diameter) continue;
                    n.normalize();
                    double low = 0, high = 0;
                    for (size_t v : cloud) {
                        const double d = n.dot(Vector3d(original.row(v)) - a);
                        low = std::min(low, d);
                        high = std::max(high, d);
                    }
                    if (low < -tolerance && high > tolerance) continue;
                    if (high > tolerance) n = -n;
                    if (std::none_of(it->second.begin(), it->second.end(), [&](const Plane& p) {
                            return (p.normal - n).norm() < 1e-8 &&
                                   std::abs(n.dot(p.origin - a)) < tolerance;
                        }))
                        it->second.push_back({n, a});
                }
        return it->second;
    };
    std::map<size_t, Vector3d> targets;
    double initial_dent = 0;
    for (const auto& [v, local_scale] : movable) {
        const int64_t parent = input.corr_input_vertex[v];
        if (parent < 0) continue;
        const Vector3d origin = original.row(parent);
        const Vector3d ray = Vector3d(original.row(v)) - origin;
        if (ray.norm() == 0) continue;
        double distance = std::numeric_limits<double>::infinity();
        for (const auto& plane : hull(parent)) {
            const double slope = plane.normal.dot(ray);
            if (slope > 1e-12 * ray.norm())
                distance = std::min(distance, plane.normal.dot(plane.origin - origin) / slope);
        }
        if (!std::isfinite(distance) || distance <= 1 + 1e-8) continue;
        // Bound the total displacement as well as each step. No offset ray shrinks.
        const double depth = std::min((distance - 1) * ray.norm(), .5 * local_scale);
        targets[v] = Vector3d(original.row(v)) + depth * ray.normalized();
        initial_dent = std::max(initial_dent, depth);
    }

    const auto determinant = [&](const Tet& t) {
        const Vector3d a = input.vertices.row(t[0]);
        const Vector3d b = input.vertices.row(t[1]);
        const Vector3d c = input.vertices.row(t[2]);
        const Vector3d d = input.vertices.row(t[3]);
        return (b - a).cross(c - a).dot(d - a);
    };
    const auto pyramid_tets = [](const HybridCell& c) -> std::array<Tet, 4> {
        const auto& v = c.vertices;
        return {
            {{v[0], v[1], v[2], v[4]},
             {v[0], v[2], v[3], v[4]},
             {v[0], v[1], v[3], v[4]},
             {v[1], v[2], v[3], v[4]}}};
    };
    const auto shape_quality = [&](const HybridCell& c) {
        const auto& v = c.vertices;
        const auto point = [&](size_t j) -> Vector3d { return input.vertices.row(v[j]); };
        if (c.type == HybridCellType::Tetrahedron) {
            double edges = 0;
            for (size_t i = 0; i < 4; ++i)
                for (size_t j = i + 1; j < 4; ++j) edges += (point(i) - point(j)).squaredNorm();
            const double det = determinant({v[0], v[1], v[2], v[3]});
            return .5 * edges / std::cbrt(2 * det * det);
        }
        if (c.type == HybridCellType::Pyramid) {
            // Native collapsed-square AMIPS: the four corner paths give its
            // maximum, including the apex limit. No reference-tet surrogate.
            double worst = 0;
            for (const auto& uv :
                 std::array<std::array<double, 2>, 4>{{{0, 0}, {1, 0}, {1, 1}, {0, 1}}}) {
                const double u = uv[0], w = uv[1];
                const Vector3d q = (1 - u) * (1 - w) * point(0) + u * (1 - w) * point(1) +
                                   u * w * point(2) + (1 - u) * w * point(3);
                const Vector3d du = (1 - w) * (point(1) - point(0)) + w * (point(2) - point(3));
                const Vector3d dv = (1 - u) * (point(3) - point(0)) + u * (point(2) - point(1));
                const Vector3d dz = std::sqrt(2.) * (point(4) - q + (u - .5) * du + (w - .5) * dv);
                const double det = du.cross(dv).dot(dz);
                worst = std::max(
                    worst,
                    (du.squaredNorm() + dv.squaredNorm() + dz.squaredNorm()) /
                        std::cbrt(det * det));
            }
            return worst;
        }
        // A conservative native-prism AMIPS bound: ||F||^2 is maximized at
        // one of the six corners; use the analytic global minimum det(F).
        double norm = 0;
        for (size_t t = 0; t < 2; ++t)
            for (size_t j = 0; j < 3; ++j) {
                const Vector3d dr = point(3 * t + 1) - point(3 * t);
                const Vector3d ds = (2 * (point(3 * t + 2) - point(3 * t)) - dr) / std::sqrt(3.);
                const Vector3d dt = point(j + 3) - point(j);
                norm = std::max(norm, dr.squaredNorm() + ds.squaredNorm() + dt.squaredNorm());
            }
        const double det =
            2 / std::sqrt(3.) * minimum_prism_jacobian(input.vertices, c).determinant;
        return norm / std::cbrt(det * det);
    };
    std::vector<double> tet_floors(nt), native_floors(hybrid.cells.size(), 0);
    std::vector<double> quality_limits(hybrid.cells.size(), 0);
    for (size_t tid = 0; tid < nt; ++tid) tet_floors[tid] = .25 * determinant(tets[tid]);
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci) {
        const auto& c = hybrid.cells[ci];
        quality_limits[ci] = std::max(10., shape_quality(c));
        if (c.type == HybridCellType::Prism)
            native_floors[ci] = .25 * minimum_prism_jacobian(input.vertices, c).determinant;
        else if (c.type == HybridCellType::Pyramid) {
            native_floors[ci] = std::numeric_limits<double>::infinity();
            for (const auto& t : pyramid_tets(c))
                native_floors[ci] = std::min(native_floors[ci], .25 * determinant(t));
        }
    }
    const auto valid_move = [&](size_t v) {
        for (size_t tid : incident_tets[v]) {
            const double floor = input.offset_tet_tags[tid] == 1 ? min_tet_volume : 0;
            if (!(determinant(tets[tid]) >= tet_floors[tid]) ||
                !tet_volume_above_threshold(input.vertices, tets[tid], floor))
                return false;
        }
        for (size_t ci : incident_cells[v]) {
            const auto& c = hybrid.cells[ci];
            if (c.type == HybridCellType::Prism) {
                const double value = minimum_prism_jacobian(input.vertices, c).determinant;
                if (!std::isfinite(value) || value < native_floors[ci] || value <= 0) return false;
            } else if (c.type == HybridCellType::Pyramid) {
                for (const auto& t : pyramid_tets(c))
                    if (!(determinant(t) >= native_floors[ci]) ||
                        !tet_volume_above_threshold(input.vertices, t, min_tet_volume))
                        return false;
            }
            if (!(shape_quality(c) <= quality_limits[ci] * (1 + 1e-12))) return false;
        }
        return true;
    };
    size_t moves = 0, blocked = 0;
    for (int pass = 0; pass < iterations; ++pass) {
        size_t accepted = 0;
        for (const auto& [v, target] : targets) {
            const Vector3d before = input.vertices.row(v);
            const Vector3d step = .5 * (target - before);
            if (step.norm() <= 1e-8 * (target - Vector3d(original.row(v))).norm()) continue;
            bool moved = false;
            double alpha = 1;
            for (int bt = 0; bt < 20; ++bt, alpha *= .5) {
                input.vertices.row(v) = (before + alpha * step).transpose();
                if (valid_move(v)) {
                    moved = true;
                    break;
                }
                input.vertices.row(v) = before.transpose();
            }
            if (moved)
                ++accepted;
            else
                ++blocked;
        }
        moves += accepted;
        if (!accepted) break;
    }
    double envelope_gap = 0;
    for (const auto& [v, target] : targets)
        envelope_gap = std::max(envelope_gap, (target - Vector3d(input.vertices.row(v))).norm());
    const double envelope_concavity = total_energy();
    size_t fairing_moves = 0;
    for (int pass = 0; pass < iterations; ++pass) {
        size_t accepted = 0;
        for (const auto& [v, scale] : movable) {
            const double energy = local_energy(v);
            if (energy <= 1e-14 * scale * scale) continue;
            const Vector3d before = input.vertices.row(v);
            Vector3d gradient;
            const double epsilon = 1e-5 * scale;
            for (int j = 0; j < 3; ++j) {
                input.vertices(v, j) = before[j] + epsilon;
                const double plus = local_energy(v);
                input.vertices(v, j) = before[j] - epsilon;
                const double minus = local_energy(v);
                input.vertices(v, j) = before[j];
                gradient[j] = (plus - minus) / (2 * epsilon);
            }
            Vector3d step = -.25 * gradient;
            if (!step.allFinite() || step.norm() <= 1e-10 * scale) continue;
            if (step.norm() > .2 * scale) step *= .2 * scale / step.norm();
            Vector3d displacement = before + step - Vector3d(original.row(v));
            if (displacement.norm() > .5 * scale) displacement *= .5 * scale / displacement.norm();
            step = Vector3d(original.row(v)) + displacement - before;
            const Vector3d parent = input.vertices.row(input.corr_input_vertex[v]);
            const Vector3d initial_ray = Vector3d(original.row(v)) - parent;
            bool moved = false;
            double alpha = 1;
            for (int bt = 0; bt < 20; ++bt, alpha *= .5) {
                input.vertices.row(v) = (before + alpha * step).transpose();
                const Vector3d ray = Vector3d(input.vertices.row(v)) - parent;
                // Do not squeeze a protected column onto its input vertex.
                if (ray.dot(initial_ray) >= .5 * initial_ray.squaredNorm() &&
                    local_energy(v) < energy && valid_move(v)) {
                    moved = true;
                    break;
                }
                input.vertices.row(v) = before.transpose();
            }
            accepted += moved;
        }
        fairing_moves += accepted;
        if (!accepted) break;
    }
    validate_prism_dominant_mesh(input, hybrid, min_tet_volume);
    nlohmann::json moved_vertices = nlohmann::json::array();
    size_t protected_moved = 0;
    for (const auto& [v, scale] : movable) {
        if ((input.vertices.row(v) - original.row(v)).norm() == 0) continue;
        std::vector<double> before(3), after(3);
        for (int j = 0; j < 3; ++j) {
            before[j] = original(v, j);
            after[j] = input.vertices(v, j);
        }
        moved_vertices.push_back(
            {{"source_vid", input.source_vertex_ids[v]}, {"before", before}, {"after", after}});
        protected_moved +=
            input.junction_protection && input.junction_protection->offset_vertices.count(v);
    }
    hybrid.report["transition_smoothing"] = {
        {"method", "local convex-envelope initialization followed by concave-hinge energy descent"},
        {"iterations", iterations},
        {"target_vertices", targets.size()},
        {"accepted_moves", moves},
        {"accepted_fairing_moves", fairing_moves},
        {"blocked_moves", blocked},
        {"protected_vertices_moved", protected_moved},
        {"maximum_initial_target_gap", initial_dent},
        {"maximum_envelope_target_gap", envelope_gap},
        {"initial_concavity_energy", initial_concavity},
        {"concavity_energy_after_envelope", envelope_concavity},
        {"final_concavity_energy", total_energy()},
        {"native_quality_limit", "max(10, initial AMIPS); conservative native bound for prisms"},
        {"minimum_relative_jacobian", .25},
        {"maximum_displacement_in_local_edge_lengths", .5},
        {"moved_vertices", moved_vertices},
        {"input_positions_preserved", true},
        {"topology_preserved", true},
        {"rod_prism_frames_preserved", true},
        {"native_elements_validated", true}};
    if (!moved_vertices.empty()) {
        hybrid.report["checks"]["existing_vertex_positions_preserved"] = false;
        hybrid.report["checks"]["positions_preserved_during_reconstruction"] = true;
        // A band-only exterior may intentionally change shape during smoothing.
        // Retained full-background exterior faces remain fixed unless they also
        // contain a moved offset vertex. Keep the reconstruction proof separate.
        std::map<Tri, size_t> exterior_counts;
        for (const auto& tet : tets)
            for (const auto& f : faces(tet)) ++exterior_counts[sorted(f)];
        std::set<size_t> moved;
        for (const auto& [v, scale] : movable)
            if ((input.vertices.row(v) - original.row(v)).norm() > 0) moved.insert(v);
        bool exterior_changed = false;
        for (const auto& [f, count] : exterior_counts)
            if (count == 1)
                for (size_t v : f) exterior_changed |= moved.count(v) != 0;
        hybrid.report["checks"]["volume_boundary_preserved_during_reconstruction"] = true;
        hybrid.report["checks"]["volume_boundary_preserved"] = !exterior_changed;
    }
    if (hybrid.report.contains("junction_protection")) {
        hybrid.report["junction_protection"]["positions_held_during_collapse"] = true;
        hybrid.report["junction_protection"]["offset_positions_fixed"] = protected_moved == 0;
    }
    double minimum = std::numeric_limits<double>::infinity();
    double minimum_sampled = std::numeric_limits<double>::infinity();
    for (const auto& c : hybrid.cells)
        if (c.type == HybridCellType::Prism) {
            minimum = std::min(minimum, minimum_prism_jacobian(input.vertices, c).determinant);
            for (const auto& q : prism_jacobian_sample_points())
                minimum_sampled =
                    std::min(minimum_sampled, prism_jacobian(input.vertices, c, q).determinant);
        }
    hybrid.report["minimum_output_prism_global_det_j"] =
        std::isfinite(minimum) ? nlohmann::json(minimum) : nlohmann::json(nullptr);
    hybrid.report["prism_jacobian_check"]["minimum_output_prism_det_j"] =
        std::isfinite(minimum_sampled) ? nlohmann::json(minimum_sampled) : nlohmann::json(nullptr);
    logger().info(
        "Transition smoothing: {} vertices moved ({} protected), concavity energy {} -> {}",
        moved_vertices.size(),
        protected_moved,
        initial_concavity,
        total_energy());
}
} // namespace wmtk::components::prismatic_mesh
