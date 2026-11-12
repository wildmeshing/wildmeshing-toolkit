#include "rod_mesh.hpp"
#include "junction_protection.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {

void average_rod_vertex_frames(RodMeshState& state)
{
    if (state.edge_frames.size() != state.directed_input_edges.size())
        throw std::runtime_error("rod mesh: incomplete edge frame field");
    std::map<size_t, std::vector<Eigen::Quaterniond>> incident;
    for (size_t i = 0; i < state.edge_frames.size(); ++i)
        for (size_t v : state.directed_input_edges[i])
            incident[v].push_back(state.edge_frames[i].rotation.normalized());
    state.vertex_rotations.clear();
    for (const auto& [v, rotations] : incident) {
        Eigen::Quaterniond q = rotations.front();
        if (rotations.size() == 2) {
            auto next = rotations[1];
            if (q.dot(next) < 0) next.coeffs() *= -1;
            q = q.slerp(.5, next).normalized();
        } else if (rotations.size() > 2) {
            Eigen::Matrix4d M = Eigen::Matrix4d::Zero();
            for (const auto& r : rotations) M += r.coeffs() * r.coeffs().transpose();
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(M);
            if (solver.info() != Eigen::Success)
                throw std::runtime_error("rod mesh: vertex quaternion mean failed");
            q.coeffs() = solver.eigenvectors().col(3);
            q.normalize();
            if (q.dot(rotations.front()) < 0) q.coeffs() *= -1;
        }
        state.vertex_rotations[v] = q;
    }
}

namespace {
using Edge = std::array<size_t, 2>;
using Tri = std::array<size_t, 3>;
using Tet = std::array<size_t, 4>;
constexpr double pi = 3.14159265358979323846;

template <class T>
T sorted(T x)
{
    std::sort(x.begin(), x.end());
    return x;
}

void require(bool ok, const std::string& why)
{
    if (!ok) throw std::runtime_error("rod mesh: " + why);
}

Tet tet_at(const PrismaticMeshInput& input, size_t i)
{
    Tet t;
    for (int j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
    return t;
}

Vector3d position(const PrismaticMeshInput& input, size_t v)
{
    return input.vertices.row(v).transpose();
}

std::vector<size_t> input_corners(const PrismaticMeshInput& input, const Tet& t)
{
    std::vector<size_t> r;
    for (size_t v : t)
        if (input.vertex_tags.at(v) == 1) r.push_back(v);
    return r;
}

bool protected_tet(const PrismaticMeshInput& input, const Tet& t)
{
    if (junction_tet_protected(input, t)) return true;
    for (size_t v : t)
        if (input.rod_state->protected_input.count(v)) return true;
    return false;
}

void rebuild(PrismaticMeshInput& input)
{
    std::vector<Tet> cells(input.tetrahedra.rows());
    for (size_t i = 0; i < cells.size(); ++i) cells[i] = tet_at(input, i);
    input.mesh = std::make_unique<TetMesh>();
    input.mesh->init_with_isolated_vertices(input.vertices.rows(), cells);
    input.offset_face_tags.clear();
}

PrismaticMeshInput copy_input(const PrismaticMeshInput& input)
{
    PrismaticMeshInput r;
    r.vertices = input.vertices;
    r.tetrahedra = input.tetrahedra;
    r.vertex_tags = input.vertex_tags;
    r.source_vertex_ids = input.source_vertex_ids;
    r.corr_input_vid = input.corr_input_vid;
    r.source_edges = input.source_edges;
    r.source_faces = input.source_faces;
    r.layered_junction_vertices = input.layered_junction_vertices;
    r.layered_correspondence_report = input.layered_correspondence_report;
    r.relative_tet_volume_floor = input.relative_tet_volume_floor;
    r.corr_input_vertex = input.corr_input_vertex;
    r.input_vertices = input.input_vertices;
    r.offset_vertices = input.offset_vertices;
    r.input_to_offset_vertices = input.input_to_offset_vertices;
    r.input_cells = input.input_cells;
    r.offset_tet_tags = input.offset_tet_tags;
    r.offset_components = input.offset_components;
    r.input_to_components = input.input_to_components;
    r.vertex_component_ids = input.vertex_component_ids;
    r.singular_vertex_tags = input.singular_vertex_tags;
    r.optimal_normals = input.optimal_normals;
    r.target_positions = input.target_positions;
    r.fixed_background_vertices = input.fixed_background_vertices;
    r.input_average_edge_length = input.input_average_edge_length;
    r.target_thickness = input.target_thickness;
    r.optimization_iterations = input.optimization_iterations;
    r.jacobian_smoothing_report = input.jacobian_smoothing_report;
    r.background_remeshing_report = input.background_remeshing_report;
    r.rod_state = input.rod_state;
    r.junction_protection = input.junction_protection;
    return r;
}

size_t append_vertex(PrismaticMeshInput& input, const Vector3d& p, int label, int64_t parent = -1)
{
    const size_t v = input.vertices.rows();
    require(
        p.allFinite() && v < static_cast<size_t>(std::numeric_limits<int>::max()),
        "invalid new vertex");
    const auto maximum =
        *std::max_element(input.source_vertex_ids.begin(), input.source_vertex_ids.end());
    require(maximum < 9007199254740991LL, "source vertex IDs exhausted");
    input.vertices.conservativeResize(v + 1, 3);
    input.vertices.row(v) = p.transpose();
    input.vertex_tags.push_back(label);
    input.source_vertex_ids.push_back(maximum + 1);
    input.corr_input_vertex.push_back(parent);
    input.corr_input_vid.push_back(parent < 0 ? -1 : input.source_vertex_ids.at(parent));
    input.input_to_offset_vertices.resize(v + 1);
    if (!input.input_to_components.empty()) input.input_to_components.resize(v + 1);
    if (!input.vertex_component_ids.empty()) input.vertex_component_ids.resize(v + 1, -1);
    if (!input.singular_vertex_tags.empty()) input.singular_vertex_tags.resize(v + 1, -1);
    if (!input.fixed_background_vertices.empty())
        input.fixed_background_vertices.resize(v + 1, false);
    if (input.optimal_normals.rows()) {
        input.optimal_normals.conservativeResize(v + 1, 3);
        input.optimal_normals.row(v).setZero();
    }
    if (input.target_positions.rows()) {
        input.target_positions.conservativeResize(v + 1, 3);
        input.target_positions.row(v) = p.transpose();
    }
    if (label == 1) input.input_vertices.push_back(v);
    if (label == 2) {
        require(parent >= 0 && input.vertex_tags.at(parent) == 1, "new frame has invalid parent");
        input.offset_vertices.push_back(v);
        input.input_to_offset_vertices[parent].push_back(v);
    }
    return v;
}

bool split_construction_edge(
    PrismaticMeshInput& input,
    Edge e,
    int label,
    int64_t parent,
    double floor)
{
    PrismaticMeshInput r = copy_input(input);
    const size_t m =
        append_vertex(r, (position(input, e[0]) + position(input, e[1])) / 2, label, parent);
    std::vector<Tet> cells;
    std::vector<int> regions, band;
    size_t changed = 0;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        const Tet t = tet_at(input, i);
        const bool incident = std::find(t.begin(), t.end(), e[0]) != t.end() &&
                              std::find(t.begin(), t.end(), e[1]) != t.end();
        if (incident && protected_tet(input, t)) return false;
        const auto add = [&](Tet child) {
            if (!tet_volume_above_threshold(
                    r.vertices,
                    child,
                    input.offset_tet_tags[i] == 1 ? floor : 0))
                return false;
            cells.push_back(child);
            regions.push_back(input.input_cells[i]);
            band.push_back(input.offset_tet_tags[i]);
            return true;
        };
        if (!incident) {
            cells.push_back(t);
            regions.push_back(input.input_cells[i]);
            band.push_back(input.offset_tet_tags[i]);
        } else {
            Tet a = t, b = t;
            *std::find(a.begin(), a.end(), e[0]) = m;
            *std::find(b.begin(), b.end(), e[1]) = m;
            if (!add(a) || !add(b)) return false;
            ++changed;
        }
    }
    if (!changed) return false;
    r.tetrahedra.resize(cells.size(), 4);
    for (size_t i = 0; i < cells.size(); ++i)
        for (int j = 0; j < 4; ++j) r.tetrahedra(i, j) = cells[i][j];
    r.input_cells = std::move(regions);
    r.offset_tet_tags = std::move(band);
    rebuild(r);
    input = std::move(r);
    return true;
}

bool split_ring_edge(PrismaticMeshInput& input, Edge e, double floor)
{
    if (input.vertex_tags[e[0]] != 2 || input.vertex_tags[e[1]] != 2 ||
        input.corr_input_vertex[e[0]] != input.corr_input_vertex[e[1]] ||
        input.rod_state->frozen_offset.count(e[0]) || input.rod_state->frozen_offset.count(e[1]))
        return false;
    return split_construction_edge(input, e, 2, input.corr_input_vertex[e[0]], floor);
}

void classify_graph(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    std::set<Edge> edges, surface_edges;
    std::set<size_t> surface_vertices;
    std::set<Edge> explicit_surface_edges, volume_edges;
    for (const auto& f : input.source_faces) {
        surface_vertices.insert(f.begin(), f.end());
        for (int j = 0; j < 3; ++j)
            explicit_surface_edges.insert(sorted(Edge{f[j], f[(j + 1) % 3]}));
    }
    for (size_t tid = 0; tid < input.input_cells.size(); ++tid)
        if (input.input_cells[tid] == 1) {
            const auto t = tet_at(input, tid);
            surface_vertices.insert(t.begin(), t.end());
            for (int a = 0; a < 4; ++a)
                for (int b = a + 1; b < 4; ++b) volume_edges.insert(sorted(Edge{t[a], t[b]}));
        }
    // Input faces/solids own their boundary edges. Only maximal one-dimensional
    // simplices seed rod paths; a shared attachment vertex is an interface port.
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        const auto in = input_corners(input, tet_at(input, i));
        if (in.size() < 3) continue;
        surface_vertices.insert(in.begin(), in.end());
        for (size_t a = 0; a < in.size(); ++a)
            for (size_t b = a + 1; b < in.size(); ++b)
                surface_edges.insert(sorted(Edge{in[a], in[b]}));
    }
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        if (input.offset_tet_tags[i] != 1) continue;
        const auto in = input_corners(input, tet_at(input, i));
        if (in.size() == 2 && !surface_edges.count(sorted(Edge{in[0], in[1]})))
            edges.insert(sorted(Edge{in[0], in[1]}));
    }
    if (!input.source_edges.empty()) {
        edges.clear();
        for (auto e : input.source_edges) {
            e = sorted(e);
            if (input.vertex_tags[e[0]] == 1 && input.vertex_tags[e[1]] == 1 &&
                !explicit_surface_edges.count(e) && !volume_edges.count(e))
                edges.insert(e);
        }
    }
    s.input_edges.assign(edges.begin(), edges.end());
    std::map<size_t, std::vector<size_t>> incident;
    double sum = 0;
    for (size_t i = 0; i < s.input_edges.size(); ++i) {
        const auto& e = s.input_edges[i];
        const double length = (position(input, e[1]) - position(input, e[0])).norm();
        require(std::isfinite(length) && length > 0, "degenerate input edge");
        sum += length;
        for (size_t v : e) incident[v].push_back(i);
    }
    for (const auto& [v, ids] : incident) {
        s.input_vertices.insert(v);
        if (surface_vertices.count(v)) s.interface_vertices.insert(v);
        if (ids.size() == 1) s.endpoints.insert(v);
        if (ids.size() == 1 && !surface_vertices.count(v)) s.free_endpoints.insert(v);
        if (ids.size() > 2) s.joints.insert(v);
    }
    s.protected_input = s.joints;
    s.protected_input.insert(s.interface_vertices.begin(), s.interface_vertices.end());
    auto path_stops = s.protected_input;
    path_stops.insert(s.endpoints.begin(), s.endpoints.end());
    std::vector<bool> visited(s.input_edges.size(), false);
    s.directed_input_edges.resize(s.input_edges.size());
    const auto follow = [&](size_t start, size_t first) {
        std::vector<size_t> path;
        size_t v = start, edge = first;
        while (!visited[edge]) {
            visited[edge] = true;
            path.push_back(edge);
            const auto e = s.input_edges[edge];
            const size_t next_vertex = e[0] == v ? e[1] : e[0];
            s.directed_input_edges[edge] = {v, next_vertex};
            v = next_vertex;
            if (path_stops.count(v)) break;
            const auto& next = incident.at(v);
            edge = next[0] == edge ? next[1] : next[0];
        }
        s.rods.push_back(std::move(path));
    };
    for (size_t v : path_stops)
        for (size_t e : incident[v])
            if (!visited[e]) follow(v, e);
    for (size_t e = 0; e < s.input_edges.size(); ++e)
        if (!visited[e]) follow(s.input_edges[e][0], e);
    s.report["rod_paths"] = nlohmann::json::array();
    for (size_t r = 0; r < s.rods.size(); ++r) {
        const auto& path = s.rods[r];
        nlohmann::json path_edges = nlohmann::json::array();
        for (size_t order = 0; order < path.size(); ++order) {
            const auto& e = s.directed_input_edges[path[order]];
            if (order > 0)
                require(
                    s.directed_input_edges[path[order - 1]][1] == e[0],
                    "discontinuous directed rod path");
            path_edges.push_back(
                {{"edge_index", path[order]},
                 {"edge_order", order},
                 {"from_vid", input.source_vertex_ids[e[0]]},
                 {"to_vid", input.source_vertex_ids[e[1]]}});
        }
        s.report["rod_paths"].push_back(
            {{"rod", r},
             {"closed",
              s.directed_input_edges[path.front()][0] == s.directed_input_edges[path.back()][1]},
             {"edges", std::move(path_edges)}});
    }
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        const auto t = tet_at(input, i);
        if (input.offset_tet_tags[i] == 1 &&
            std::any_of(t.begin(), t.end(), [&](size_t v) { return s.protected_input.count(v); }))
            for (size_t v : t)
                if (input.vertex_tags[v] == 2) s.frozen_offset.insert(v);
    }
    s.radius = s.options.radius > 0
                   ? s.options.radius
                   : s.options.thicknessratio * (edges.empty() ? 1. : sum / edges.size());
    require(std::isfinite(s.radius) && s.radius > 0, "radius must be finite and positive");
    s.report["input_edges"] = s.input_edges.size();
    s.report["joints"] = s.joints.size();
    s.report["endpoints"] = s.endpoints.size();
    s.report["free_endpoints"] = s.free_endpoints.size();
    s.report["endpoint_protection_rule"] = "non_rod_attachments_and_branch_joints_only";
    s.report["free_endpoint_source_vids"] = nlohmann::json::array();
    for (size_t v : s.free_endpoints)
        s.report["free_endpoint_source_vids"].push_back(input.source_vertex_ids[v]);
    s.report["rods"] = s.rods.size();
    s.report["joint_vertices"] = s.joints;
    s.report["joint_source_vertex_ids"] = nlohmann::json::array();
    for (size_t v : s.joints)
        s.report["joint_source_vertex_ids"].push_back(input.source_vertex_ids[v]);
    s.report["radius"] = s.radius;
    s.report["interface_vertices"] = s.interface_vertices.size();
    s.report["unified_optimization"] = true;
}

std::vector<size_t> ordered_ring(const std::vector<Edge>& edges)
{
    std::map<size_t, std::vector<size_t>> adj;
    for (const auto& e : edges) {
        adj[e[0]].push_back(e[1]);
        adj[e[1]].push_back(e[0]);
    }
    if (adj.size() < 3 || edges.size() != adj.size()) return {};
    for (const auto& [v, ns] : adj)
        if (ns.size() != 2 || ns[0] == ns[1]) return {};
    std::vector<size_t> ring{adj.begin()->first};
    size_t previous = ring[0], next = adj.begin()->second[0];
    while (next != ring[0]) {
        if (ring.size() >= adj.size()) return {};
        ring.push_back(next);
        const auto& ns = adj.at(next);
        const size_t after = ns[0] == previous ? ns[1] : ns[0];
        previous = next;
        next = after;
    }
    return ring.size() == adj.size() ? ring : std::vector<size_t>{};
}

std::vector<Tri> rod_offset_front(const PrismaticMeshInput& input)
{
    std::map<Tri, size_t> counts;
    for (size_t tid = 0; tid < input.offset_tet_tags.size(); ++tid) {
        if (input.offset_tet_tags[tid] != 1) continue;
        const auto t = tet_at(input, tid);
        for (size_t omit = 0; omit < 4; ++omit) {
            Tri f;
            size_t j = 0;
            for (size_t k = 0; k < 4; ++k)
                if (k != omit) f[j++] = t[k];
            if (std::all_of(f.begin(), f.end(), [&](size_t v) {
                    return input.vertex_tags[v] == 2;
                }))
                ++counts[sorted(f)];
        }
    }
    std::vector<Tri> result;
    for (const auto& [f, count] : counts)
        if (count == 1) result.push_back(f);
    return result;
}

struct VertexRegion
{
    std::set<size_t> vertices;
    std::set<Edge> edges;
    std::set<Tri> triangles;
};
struct RegionSurface
{
    std::map<size_t, VertexRegion> regions;
    std::map<size_t, std::set<size_t>> adjacent;
    std::map<Edge, std::set<size_t>> opposite;
};
RegionSurface region_surface(const PrismaticMeshInput& input)
{
    RegionSurface result;
    for (const auto& f : rod_offset_front(input)) {
        for (size_t v : f) {
            const auto parent = input.corr_input_vertex[v];
            if (parent >= 0 && input.rod_state->input_vertices.count(parent))
                result.regions[parent].vertices.insert(v);
        }
        for (size_t j = 0; j < 3; ++j) {
            const size_t a = f[j], b = f[(j + 1) % 3], c = f[(j + 2) % 3];
            result.adjacent[a].insert(b);
            result.adjacent[b].insert(a);
            result.opposite[sorted(Edge{a, b})].insert(c);
            const auto parent = input.corr_input_vertex[a];
            if (parent >= 0 && parent == input.corr_input_vertex[b] &&
                input.rod_state->input_vertices.count(parent))
                result.regions[parent].edges.insert(sorted(Edge{a, b}));
        }
        const auto parent = input.corr_input_vertex[f[0]];
        if (parent >= 0 && parent == input.corr_input_vertex[f[1]] &&
            parent == input.corr_input_vertex[f[2]] &&
            input.rod_state->input_vertices.count(parent))
            result.regions[parent].triangles.insert(sorted(f));
    }
    return result;
}
bool valid_region_complex(const VertexRegion& r)
{
    if (r.vertices.size() < 3 || r.vertices.size() + r.triangles.size() != r.edges.size())
        return false;
    std::map<size_t, std::set<size_t>> adj;
    for (const auto& e : r.edges) {
        adj[e[0]].insert(e[1]);
        adj[e[1]].insert(e[0]);
    }
    std::set<size_t> visited{*r.vertices.begin()};
    std::vector<size_t> queue{*r.vertices.begin()};
    for (size_t i = 0; i < queue.size(); ++i)
        for (size_t v : adj[queue[i]])
            if (visited.insert(v).second) queue.push_back(v);
    return visited == r.vertices;
}
std::vector<size_t> region_ring(const VertexRegion& r)
{
    if (!r.triangles.empty() || !valid_region_complex(r)) return {};
    const auto ring = ordered_ring(std::vector<Edge>(r.edges.begin(), r.edges.end()));
    return ring.size() == r.vertices.size() ? ring : std::vector<size_t>{};
}
bool collapse_preserves_region(const VertexRegion& r, size_t removed, size_t survivor)
{
    VertexRegion next;
    next.vertices = r.vertices;
    next.vertices.erase(removed);
    for (auto e : r.edges) {
        for (auto& v : e)
            if (v == removed) v = survivor;
        if (e[0] != e[1]) next.edges.insert(sorted(e));
    }
    for (auto f : r.triangles) {
        for (auto& v : f)
            if (v == removed) v = survivor;
        if (std::set<size_t>(f.begin(), f.end()).size() == 3) next.triangles.insert(sorted(f));
    }
    return next.triangles.size() <= r.triangles.size() && valid_region_complex(next);
}
nlohmann::json region_report(const PrismaticMeshInput& input, const RegionSurface& surface)
{
    auto result = nlohmann::json::array();
    for (const auto& [parent, r] : surface.regions) {
        const auto ring = region_ring(r);
        std::vector<int64_t> ids;
        for (size_t v : ring) ids.push_back(input.source_vertex_ids[v]);
        result.push_back(
            {{"center_vid", input.source_vertex_ids[parent]},
             {"vertices", r.vertices.size()},
             {"edges", r.edges.size()},
             {"faces", r.triangles.size()},
             {"euler",
              int64_t(r.vertices.size()) - int64_t(r.edges.size()) + int64_t(r.triangles.size())},
             {"protected", input.rod_state->protected_input.count(parent) != 0},
             {"cycle", !ring.empty()},
             {"ring_vids", ids}});
    }
    return result;
}
size_t collapse_vertex_regions(PrismaticMeshInput& input)
{
    auto& state = *input.rod_state;
    const auto initial = region_surface(input);
    evaluate_target_positions(input, state.options.thicknessratio);
    std::map<size_t, std::vector<size_t>> neighbors;
    for (const auto& e : state.input_edges) {
        neighbors[e[0]].push_back(e[1]);
        neighbors[e[1]].push_back(e[0]);
    }
    struct Candidate
    {
        double cost;
        size_t parent, a, b;
    };
    size_t accepted = 0, attempts = 0;
    auto operations = nlohmann::json::array();
    state.collapsing_regions = true;
    const size_t budget = input.offset_vertices.size();
    for (size_t step = 0; step < budget; ++step) {
        const auto surface = region_surface(input);
        std::vector<Candidate> candidates;
        for (const auto& [parent, r] : surface.regions) {
            if (neighbors[parent].size() != 2 || state.protected_input.count(parent) ||
                !valid_region_complex(r) || !region_ring(r).empty())
                continue;
            const Vector3d center = position(input, parent);
            const Vector3d axis =
                (position(input, neighbors[parent][1]) - position(input, neighbors[parent][0]))
                    .normalized();
            for (const auto& e : r.edges) {
                if (state.frozen_offset.count(e[0]) || state.frozen_offset.count(e[1]) ||
                    !junction_collapse_allowed(input, e[0]) ||
                    !junction_collapse_allowed(input, e[1]))
                    continue;
                const auto& opposite = surface.opposite.at(e);
                if (opposite.size() != 2) continue;
                std::set<size_t> common;
                const auto& a = surface.adjacent.at(e[0]);
                const auto& b = surface.adjacent.at(e[1]);
                std::set_intersection(
                    a.begin(),
                    a.end(),
                    b.begin(),
                    b.end(),
                    std::inserter(common, common.end()));
                if (common != opposite || !collapse_preserves_region(r, e[0], e[1])) continue;
                Vector3d x = position(input, e[0]) - center, y = position(input, e[1]) - center;
                x -= axis * x.dot(axis);
                y -= axis * y.dot(axis);
                if (!(x.norm() > 0 && y.norm() > 0)) continue;
                const double angle =
                    std::acos(std::clamp(x.normalized().dot(y.normalized()), -1., 1.));
                const double length = (position(input, e[0]) - position(input, e[1])).norm();
                candidates.push_back(
                    {angle * angle + .001 * length / state.radius, parent, e[0], e[1]});
            }
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            return std::tie(a.cost, a.parent, a.a, a.b) < std::tie(b.cost, b.parent, b.a, b.b);
        });
        bool changed = false;
        for (const auto& c : candidates) {
            for (int side = 0; side < 2; ++side) {
                const size_t removed = side ? c.b : c.a, survivor = side ? c.a : c.b;
                ++attempts;
                if (!try_collapse_offset_edge(
                        input,
                        removed,
                        survivor,
                        state.options.min_tet_volume))
                    continue;
                operations.push_back(
                    {{"center_vid", input.source_vertex_ids[c.parent]},
                     {"removed_vid", input.source_vertex_ids[removed]},
                     {"survivor_vid", input.source_vertex_ids[survivor]}});
                ++accepted;
                changed = true;
                break;
            }
            if (changed) break;
        }
        if (!changed) break;
    }
    state.collapsing_regions = false;
    const auto final = region_surface(input);
    auto& report = state.report["vertex_regions"];
    if (!report.contains("initial_regions"))
        report["initial_regions"] = region_report(input, initial);
    report["regions"] = region_report(input, final);
    report["passes"].push_back(
        {{"collapses", accepted}, {"attempts", attempts}, {"operations", operations}});
    report["original_correspondence_preserved"] = true;
    size_t rings = 0, regular = 0;
    for (const auto& [parent, r] : final.regions)
        if (neighbors[parent].size() == 2) {
            ++regular;
            rings += !region_ring(r).empty();
        }
    report["regular_regions"] = regular;
    report["regular_cycles"] = rings;
    logger().info(
        "Rod regions: {} collapses, {} / {} regular regions are cycles",
        accepted,
        rings,
        regular);
    return accepted;
}

Vector3d normal_basis(const Vector3d& t);

void build_edge_and_vertex_frames(
    PrismaticMeshInput& input,
    const std::map<Edge, std::vector<Edge>>& links)
{
    auto& s = *input.rod_state;
    s.edge_frames.resize(s.input_edges.size());
    for (const auto& path : s.rods) {
        Eigen::Quaterniond previous = Eigen::Quaterniond::Identity();
        Vector3d previous_t = Vector3d::Zero();
        for (size_t index : path) {
            const auto& directed = s.directed_input_edges[index];
            const Vector3d t =
                (position(input, directed[1]) - position(input, directed[0])).normalized();
            Eigen::Matrix3d basis;
            basis.col(0) = normal_basis(t);
            basis.col(1) = t.cross(basis.col(0));
            basis.col(2) = t;
            const Eigen::Quaterniond transported =
                previous_t.isZero() ? Eigen::Quaterniond(basis)
                                    : Eigen::Quaterniond::FromTwoVectors(previous_t, t) * previous;
            auto& result = s.edge_frames[index];
            result.rotation = transported.normalized();
            const auto found = links.find(s.input_edges[index]);
            auto ring = found == links.end() ? std::vector<size_t>{} : ordered_ring(found->second);
            result.degree = ring.size();
            result.from_link = !ring.empty();
            if (!ring.empty()) {
                const Vector3d center =
                    (position(input, directed[0]) + position(input, directed[1])) / 2;
                Vector3d area = Vector3d::Zero();
                for (size_t k = 0; k < ring.size(); ++k)
                    area += (position(input, ring[k]) - center)
                                .cross(position(input, ring[(k + 1) % ring.size()]) - center);
                if (area.dot(t) < 0) std::reverse(ring.begin() + 1, ring.end());
                const Vector3d x = basis.col(0), y = basis.col(1);
                double real = 0, imag = 0;
                for (size_t k = 0; k < ring.size(); ++k) {
                    const Vector3d radial = position(input, ring[k]) - center;
                    const double angle =
                        std::atan2(radial.dot(y), radial.dot(x)) - 2 * pi * k / ring.size();
                    real += std::cos(angle);
                    imag += std::sin(angle);
                }
                const Eigen::Quaterniond fitted =
                    Eigen::Quaterniond(basis) *
                    Eigen::AngleAxisd(std::atan2(imag, real), Vector3d::UnitZ());
                double best = std::numeric_limits<double>::infinity();
                for (size_t shift = 0; shift < ring.size(); ++shift) {
                    const Eigen::Quaterniond candidate =
                        fitted * Eigen::AngleAxisd(2 * pi * shift / ring.size(), Vector3d::UnitZ());
                    const double cost = transported.angularDistance(candidate);
                    if (cost < best) {
                        best = cost;
                        result.rotation = candidate.normalized();
                    }
                }
            }
            if (!previous_t.isZero() && previous.dot(result.rotation) < 0)
                result.rotation.coeffs() *= -1;
            previous = result.rotation;
            previous_t = t;
        }
    }
    average_rod_vertex_frames(s);
}

void extract_frames(PrismaticMeshInput& input, bool preserve_rotations = false)
{
    auto& s = *input.rod_state;
    // Retain material labels when only geometry changes. Re-fitting the cyclic
    // origin after a partial relocation would change correspondence between
    // already aligned sections. An intentional pose update still reindexes.
    using CycleKey = std::pair<size_t, std::vector<size_t>>;
    std::map<CycleKey, std::pair<Eigen::Quaterniond, std::vector<size_t>>> material_cycles;
    if (preserve_rotations)
        for (const auto& f : s.frames)
            if (f.valid)
                material_cycles.emplace(
                    CycleKey{f.center, sorted(f.ring)},
                    std::make_pair(f.rotation, f.ring));
    std::map<Edge, std::vector<Edge>> links;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        if (input.offset_tet_tags[i] != 1) continue;
        const auto t = tet_at(input, i);
        const auto in = input_corners(input, t);
        if (in.size() != 2) continue;
        Edge off;
        size_t n = 0;
        for (size_t v : t)
            if (input.vertex_tags[v] == 2 && n < 2) off[n++] = v;
        if (n == 2) links[sorted(Edge{in[0], in[1]})].push_back(off);
    }
    const auto edge_frames = preserve_rotations ? s.edge_frames : std::vector<RodEdgeFrame>{};
    const auto rotations = s.vertex_rotations;
    build_edge_and_vertex_frames(input, links);
    if (preserve_rotations) {
        for (size_t i = 0; i < edge_frames.size(); ++i)
            s.edge_frames[i].rotation = edge_frames[i].rotation;
        s.vertex_rotations = rotations;
    }
    s.frames.clear();
    s.vertex_frame.clear();
    std::vector<size_t> rod(s.input_edges.size()), order(s.input_edges.size());
    for (size_t i = 0; i < s.rods.size(); ++i)
        for (size_t k = 0; k < s.rods[i].size(); ++k) {
            const size_t e = s.rods[i][k];
            rod[e] = i;
            order[e] = k;
        }
    const auto orient_frame = [&](RodFrame& f) {
        f.rotation = s.vertex_rotations.at(f.center);
        f.tangent = f.rotation * Vector3d::UnitZ();
        Vector3d area = Vector3d::Zero();
        for (size_t k = 0; k < f.ring.size(); ++k)
            area += (position(input, f.ring[k]) - position(input, f.center))
                        .cross(
                            position(input, f.ring[(k + 1) % f.ring.size()]) -
                            position(input, f.center));
        if (area.dot(f.tangent) < 0) std::reverse(f.ring.begin() + 1, f.ring.end());
        // Existing cycles retain their transported material labels. A new
        // cycle (or intentional pose change) fits degree zero to that field,
        // rather than using the smallest vertex ID as its cyclic origin.
        size_t shift = 0;
        bool preserved_cycle = false;
        const auto previous = material_cycles.find({f.center, sorted(f.ring)});
        if (previous != material_cycles.end() &&
            previous->second.first.angularDistance(f.rotation) < 1e-12) {
            const auto& expected = previous->second.second;
            const auto zero = std::find(f.ring.begin(), f.ring.end(), expected.front());
            if (zero != f.ring.end()) {
                auto cycle = f.ring;
                std::rotate(cycle.begin(), cycle.begin() + (zero - f.ring.begin()), cycle.end());
                if (cycle == expected) {
                    shift = zero - f.ring.begin();
                    preserved_cycle = true;
                }
            }
        }
        double best = std::numeric_limits<double>::infinity();
        for (size_t j = 0; !preserved_cycle && j < f.ring.size(); ++j) {
            double cost = 0;
            for (size_t k = 0; k < f.ring.size(); ++k) {
                const double angle = 2 * pi * k / f.ring.size();
                const Vector3d target =
                    position(input, f.center) +
                    s.radius * (f.rotation * Vector3d(std::cos(angle), std::sin(angle), 0));
                cost += (position(input, f.ring[(k + j) % f.ring.size()]) - target).squaredNorm();
            }
            if (cost < best) {
                best = cost;
                shift = j;
            }
        }
        std::rotate(f.ring.begin(), f.ring.begin() + shift, f.ring.end());
        if (s.protected_input.count(f.center))
            f.reason = "protected_rod_connection";
        else if (std::any_of(f.ring.begin(), f.ring.end(), [&](size_t v) {
                     return s.frozen_offset.count(v) || !junction_collapse_allowed(input, v);
                 }))
            f.reason = "protected_joint_one_ring";
        else
            f.valid = true;
    };
    const auto surface = region_surface(input);
    // A free end's region is a disk, including its offset cone cap. Its material
    // ring is the boundary against the neighboring region, not the whole disk.
    std::map<size_t, std::set<Edge>> end_rims;
    for (const auto& [edge, opposite] : surface.opposite) {
        const auto parent = input.corr_input_vertex[edge[0]];
        if (parent < 0 || parent != input.corr_input_vertex[edge[1]] ||
            !s.free_endpoints.count(parent))
            continue;
        for (size_t v : opposite) {
            const auto other = input.corr_input_vertex[v];
            if (other >= 0 && other != parent &&
                std::binary_search(
                    s.input_edges.begin(),
                    s.input_edges.end(),
                    sorted(Edge{size_t(parent), size_t(other)})))
                end_rims[parent].insert(edge);
        }
    }
    for (const auto& [center, region] : surface.regions) {
        const auto it =
            std::find_if(s.input_edges.begin(), s.input_edges.end(), [&](const Edge& e) {
                return e[0] == center || e[1] == center;
            });
        if (it == s.input_edges.end()) continue;
        const size_t e = it - s.input_edges.begin();
        RodFrame frame;
        frame.edge = *it;
        frame.edge_index = e;
        frame.center = center;
        frame.rod = rod[e];
        frame.order = order[e];
        frame.ring = region_ring(region);
        if (frame.ring.empty() && s.free_endpoints.count(center)) {
            const auto& rim = end_rims[center];
            frame.ring = ordered_ring(std::vector<Edge>(rim.begin(), rim.end()));
        }
        frame.initial_degree = frame.ring.size();
        if (frame.ring.empty())
            frame.reason = "region_not_single_cycle";
        else
            orient_frame(frame);
        if (frame.valid) s.vertex_frame[center] = s.frames.size();
        s.frames.push_back(std::move(frame));
    }
}

size_t equalize_degrees(PrismaticMeshInput& input, bool preserve_rotations = false)
{
    auto& s = *input.rod_state;
    std::vector<size_t> target(s.rods.size(), 0);
    const auto eligible = [&](const RodFrame& f) { return f.valid; };
    for (const auto& f : s.frames)
        if (eligible(f)) target[f.rod] = std::max(target[f.rod], f.ring.size());
    s.report["target_degrees"] = target;
    nlohmann::json operations = nlohmann::json::array();
    size_t splits = 0, blocked = 0;
    for (size_t index = 0; index < s.frames.size(); ++index) {
        while (index < s.frames.size() && eligible(s.frames[index]) &&
               s.frames[index].ring.size() < target[s.frames[index].rod]) {
            const auto f = s.frames[index];
            std::vector<std::pair<double, Edge>> candidates;
            for (size_t k = 0; k < f.ring.size(); ++k) {
                const size_t a = f.ring[k], b = f.ring[(k + 1) % f.ring.size()];
                Vector3d x = position(input, a) - position(input, f.center);
                Vector3d y = position(input, b) - position(input, f.center);
                x -= f.tangent * x.dot(f.tangent);
                y -= f.tangent * y.dot(f.tangent);
                double angle = std::atan2(f.tangent.dot(x.cross(y)), x.dot(y));
                if (angle <= 0) angle += 2 * pi;
                candidates.push_back({-angle, {a, b}});
            }
            std::sort(candidates.begin(), candidates.end());
            bool accepted = false;
            for (const auto& [angle, edge] : candidates) {
                if (split_ring_edge(input, edge, s.options.min_tet_volume)) {
                    accepted = true;
                    ++splits;
                    operations.push_back(
                        {{"center_vid", input.source_vertex_ids[f.center]},
                         {"degree_before", f.ring.size()},
                         {"target_degree", target[f.rod]},
                         {"split_edge_vids",
                          {input.source_vertex_ids[edge[0]], input.source_vertex_ids[edge[1]]}},
                         {"new_vertex_vid", input.source_vertex_ids.back()}});
                    extract_frames(input, preserve_rotations);
                    break;
                }
            }
            if (!accepted) {
                ++blocked;
                break;
            }
        }
    }
    s.report["ring_edge_splits"] = s.report.value("ring_edge_splits", size_t(0)) + splits;
    s.report["degree_alignment_blocked"] = blocked;
    nlohmann::json remaining = nlohmann::json::array();
    for (auto& f : s.frames)
        if (f.valid && f.ring.size() != target[f.rod]) {
            // A valid section can still match an adjacent section of the same
            // degree when the rod-wide target is blocked by a fixed junction.
            f.reason = "degree_alignment_limited";
            remaining.push_back(
                {{"center_vid", input.source_vertex_ids[f.center]},
                 {"degree", f.ring.size()},
                 {"target_degree", target[f.rod]}});
        }
    s.report["degree_alignment_passes"].push_back(
        {{"splits", splits}, {"operations", operations}, {"remaining_frames", remaining}});
    return splits;
}

Vector3d normal_basis(const Vector3d& t)
{
    int axis = 0;
    t.cwiseAbs().minCoeff(&axis);
    Vector3d d = Vector3d::Unit(axis);
    return (d - t * d.dot(t)).normalized();
}

void relocate_frames(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    const size_t count = s.frames.size();
    size_t fitted = 0, limited = 0;
    double worst_remaining = 0;
    // Use the material degree indices assigned during frame extraction. Neither
    // the poses nor the correspondence may be reselected during relocation.
    for (size_t i = 0; i < count; ++i) {
        auto& f = s.frames[i];
        if (!f.valid) continue;
        std::vector<Vector3d> old, target;
        for (size_t k = 0; k < f.ring.size(); ++k) {
            old.push_back(position(input, f.ring[k]));
            const double angle = 2 * pi * k / f.ring.size();
            target.push_back(
                position(input, f.center) +
                s.radius * (f.rotation * Vector3d(std::cos(angle), std::sin(angle), 0)));
        }
        std::set<size_t> incident;
        for (size_t v : f.ring)
            for (size_t t : input.mesh->get_one_ring_tids_for_vertex(v)) incident.insert(t);
        double alpha = 1;
        bool accepted = false;
        for (int backtrack = 0; backtrack <= 24; ++backtrack, alpha *= 0.5) {
            for (size_t k = 0; k < f.ring.size(); ++k)
                input.vertices.row(f.ring[k]) = (old[k] + alpha * (target[k] - old[k])).transpose();
            accepted = std::all_of(incident.begin(), incident.end(), [&](size_t t) {
                return tet_volume_above_threshold(
                    input.vertices,
                    tet_at(input, t),
                    input.offset_tet_tags[t] == 1 ? s.options.min_tet_volume : 0);
            });
            if (accepted) break;
        }
        if (!accepted) {
            alpha = 0;
            for (size_t k = 0; k < f.ring.size(); ++k)
                input.vertices.row(f.ring[k]) = old[k].transpose();
        }
        if (alpha == 1)
            ++fitted;
        else
            ++limited;
        for (size_t k = 0; k < f.ring.size(); ++k)
            worst_remaining =
                std::max(worst_remaining, (position(input, f.ring[k]) - target[k]).norm());
    }
    s.report["circular_frames"] = fitted;
    s.report["relocation_limited_frames"] = limited;
    s.report["maximum_target_displacement_remaining"] = worst_remaining;
}

void flatten_rod_frames(PrismaticMeshInput& input)
{
    auto& state = *input.rod_state;
    size_t flattened = 0, blocked = 0;
    for (const auto& frame : state.frames) {
        if (!frame.valid) continue;
        const Vector3d center = position(input, frame.center);
        std::vector<Vector3d> old;
        std::set<size_t> incident;
        for (size_t v : frame.ring) {
            old.push_back(position(input, v));
            const Vector3d radial = position(input, v) - center;
            input.vertices.row(v) =
                (position(input, v) - frame.tangent * radial.dot(frame.tangent)).transpose();
            for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(v)) incident.insert(tid);
        }
        const bool positive = std::all_of(incident.begin(), incident.end(), [&](size_t tid) {
            return tet_volume_above_threshold(
                input.vertices,
                tet_at(input, tid),
                input.offset_tet_tags[tid] == 1 ? state.options.min_tet_volume : 0);
        });
        if (positive)
            ++flattened;
        else {
            ++blocked;
            for (size_t k = 0; k < frame.ring.size(); ++k)
                input.vertices.row(frame.ring[k]) = old[k].transpose();
        }
    }
    state.report["planar_frames"] = {{"completed", flattened}, {"blocked", blocked}};
}

void record_rod_frames(const PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    s.report["vertex_frame_rule"] = "equal_weight_incident_edge_quaternion_mean";
    s.report["edge_frames"] = nlohmann::json::array();
    std::map<size_t, std::vector<size_t>> incident;
    for (size_t i = 0; i < s.edge_frames.size(); ++i) {
        const auto& q = s.edge_frames[i].rotation;
        const auto& e = s.directed_input_edges[i];
        s.report["edge_frames"].push_back(
            {{"index", i},
             {"from_vid", input.source_vertex_ids[e[0]]},
             {"to_vid", input.source_vertex_ids[e[1]]},
             {"quaternion_wxyz", {q.w(), q.x(), q.y(), q.z()}},
             {"degree", s.edge_frames[i].degree},
             {"from_link", s.edge_frames[i].from_link}});
        for (size_t v : e) incident[v].push_back(i);
    }
    s.report["vertex_frames"] = nlohmann::json::array();
    for (const auto& [v, q] : s.vertex_rotations) {
        const Vector3d normal = q * Vector3d::UnitZ();
        s.report["vertex_frames"].push_back(
            {{"center_vid", input.source_vertex_ids[v]},
             {"incident_edges", incident.at(v)},
             {"quaternion_wxyz", {q.w(), q.x(), q.y(), q.z()}},
             {"normal", {normal.x(), normal.y(), normal.z()}}});
    }
    s.report["frames"] = nlohmann::json::array();
    for (size_t i = 0; i < s.frames.size(); ++i) {
        const auto& f = s.frames[i];
        const auto& directed = s.directed_input_edges[f.edge_index];
        s.report["frames"].push_back(
            {{"input_edge", f.edge},
             {"input_edge_vids",
              {input.source_vertex_ids[f.edge[0]], input.source_vertex_ids[f.edge[1]]}},
             {"center", f.center},
             {"center_vid", nlohmann::json(input.source_vertex_ids[f.center])},
             {"degree", f.ring.size()},
             {"rod", f.rod},
             {"edge_order", f.order},
             {"edge_index", f.edge_index},
             {"from_vid", input.source_vertex_ids[directed[0]]},
             {"to_vid", input.source_vertex_ids[directed[1]]},
             {"tangent", {f.tangent.x(), f.tangent.y(), f.tangent.z()}},
             {"quaternion_wxyz", {f.rotation.w(), f.rotation.x(), f.rotation.y(), f.rotation.z()}},
             {"ring_vids",
              [&]() {
                  nlohmann::json ring = nlohmann::json::array();
                  for (size_t v : f.ring) ring.push_back(input.source_vertex_ids[v]);
                  return ring;
              }()},
             {"valid", f.valid},
             {"reason", f.reason}});
    }
}

void update_frame_targets(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    s.frame_targets.clear();
    s.retained_ring_vertices.clear();
    for (const auto& frame : s.frames)
        if (frame.valid)
            for (size_t k = 0; k < frame.ring.size(); ++k) {
                const double angle = 2 * pi * k / frame.ring.size();
                s.retained_ring_vertices.insert(frame.ring[k]);
                s.frame_targets[frame.ring[k]] =
                    position(input, frame.center) +
                    s.radius * (frame.rotation * Vector3d(std::cos(angle), std::sin(angle), 0));
            }
    s.report["rod_target_vertices"] = s.frame_targets.size();
}
} // namespace

nlohmann::json analyze_rod_tau(const PrismaticMeshInput& input)
{
    std::map<std::string, size_t> counts;
    size_t invalid_parent = 0, wrong_edge_parent = 0, protected_parent = 0, ordinary_tau22 = 0;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        if (input.offset_tet_tags[i] != 1) continue;
        const auto t = tet_at(input, i);
        const auto in = input_corners(input, t);
        size_t offsets = 0;
        std::set<int64_t> parents;
        for (size_t v : t)
            if (input.vertex_tags[v] == 2) {
                ++offsets;
                const auto p = input.corr_input_vertex[v];
                if (p < 0 || p >= input.vertices.rows() || input.vertex_tags[p] != 1 ||
                    input.corr_input_vid[v] != input.source_vertex_ids[p])
                    ++invalid_parent;
                parents.insert(p);
            }
        const std::string name = offsets + in.size() == 4 ? "tau" + std::to_string(offsets) +
                                                                std::to_string(parents.size())
                                                          : "unclassified";
        ++counts[name];
        if (in.size() == 2 && (!input.rod_state || std::binary_search(
                                                       input.rod_state->input_edges.begin(),
                                                       input.rod_state->input_edges.end(),
                                                       sorted(Edge{in[0], in[1]})))) {
            for (auto p : parents)
                if (p != static_cast<int64_t>(in[0]) && p != static_cast<int64_t>(in[1])) {
                    if (protected_tet(input, t))
                        ++protected_parent;
                    else
                        ++wrong_edge_parent;
                }
            if (parents.size() == 2 && input.rod_state && !protected_tet(input, t))
                ++ordinary_tau22;
        }
    }
    return {
        {"counts", counts},
        {"invalid_correspondence", invalid_parent},
        {"edge_parent_mismatch", wrong_edge_parent},
        {"protected_junction_parent_mismatch", protected_parent},
        {"ordinary_tau22", ordinary_tau22}};
}

void preprocess_rod_mesh(PrismaticMeshInput& input, const RodOptions& options)
{
    require(std::isfinite(options.radius) && options.radius >= 0, "invalid radius");
    initialize_junction_protection(input);
    require(
        std::isfinite(options.thicknessratio) && options.thicknessratio > 0,
        "invalid thicknessratio");
    require(
        options.iterations >= 0 && std::isfinite(options.min_tet_volume) &&
            options.min_tet_volume >= 0,
        "invalid iteration count or volume floor");
    for (size_t v = 0; v < static_cast<size_t>(input.vertices.rows()); ++v) {
        if (input.vertex_tags[v] != 2) {
            require(
                input.corr_input_vid[v] == -1 && input.corr_input_vertex[v] == -1,
                "correspondence on a non-offset vertex");
            continue;
        }
        const auto p = input.corr_input_vertex[v];
        require(
            p >= 0 && p < input.vertices.rows() && input.vertex_tags[p] == 1 &&
                input.corr_input_vid[v] == input.source_vertex_ids[p],
            "invalid input correspondence");
    }
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i)
        require(
            tet_volume_above_threshold(input.vertices, tet_at(input, i), 0),
            "nonpositive source tet");
    input.rod_state = std::make_shared<RodMeshState>();
    auto& s = *input.rod_state;
    s.options = options;
    s.report = nlohmann::json::object();
    classify_graph(input);
    s.report["tau_before"] = analyze_rod_tau(input);
    require(
        s.report["tau_before"]["invalid_correspondence"] == 0 &&
            s.report["tau_before"]["edge_parent_mismatch"] == 0,
        "input edge cofaces have inconsistent correspondence");
    if (options.iterations > 0) collapse_vertex_regions(input);
    extract_frames(input);
    s.report["initial_frame_degrees"] = nlohmann::json::array();
    for (const auto& frame : s.frames)
        s.report["initial_frame_degrees"].push_back(frame.ring.size());
    equalize_degrees(input);
    transport_rod_frame_mapping(input);
    extract_frames(input, true);
    update_frame_targets(input);
    // These are surface-normal optimizer diagnostics, used temporarily to reuse
    // the same-parent collapse checks. Rod frames have their own targets/report.
    input.offset_components.clear();
    input.input_to_components.clear();
    input.vertex_component_ids.clear();
    input.singular_vertex_tags.clear();
    input.optimal_normals.resize(0, 3);
    input.target_positions.resize(0, 3);
    s.report["tau_after"] = analyze_rod_tau(input);
    s.report["interface_source_vertex_ids"] = nlohmann::json::array();
    for (size_t v : s.interface_vertices)
        s.report["interface_source_vertex_ids"].push_back(input.source_vertex_ids[v]);
    s.report["protected_offset_source_vertex_ids"] = nlohmann::json::array();
    for (size_t v : s.frozen_offset)
        s.report["protected_offset_source_vertex_ids"].push_back(input.source_vertex_ids[v]);
    record_rod_frames(input);
    label_offset_faces(input);
    logger().info(
        "Rod regions: {} rods, {} joints, {} valid rings; radius {}",
        s.rods.size(),
        s.joints.size(),
        s.vertex_frame.size(),
        s.radius);
}

bool is_rod_offset_vertex(const PrismaticMeshInput& input, size_t v)
{
    return input.rod_state && v < input.vertex_tags.size() && input.vertex_tags[v] == 2 &&
           input.corr_input_vertex[v] >= 0 &&
           input.rod_state->input_vertices.count(static_cast<size_t>(input.corr_input_vertex[v]));
}

bool rod_collapse_allowed(const PrismaticMeshInput& input, size_t removed, size_t survivor)
{
    if (!input.rod_state) return true;
    const auto& s = *input.rod_state;
    if (!is_rod_offset_vertex(input, removed) && !is_rod_offset_vertex(input, survivor)) {
        if (s.frozen_offset.count(removed) || s.frozen_offset.count(survivor)) return false;
        // A shell-owned offset edge can still bound a rod section at an
        // attachment. Collapsing it must not erase a centerline-edge coface.
        for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(removed)) {
            const auto tet = input.mesh->oriented_tet_vids(input.mesh->tuple_from_tet(tid));
            if (std::find(tet.begin(), tet.end(), survivor) == tet.end()) continue;
            const auto corners = input_corners(input, tet);
            for (size_t i = 0; i < corners.size(); ++i)
                for (size_t j = i + 1; j < corners.size(); ++j)
                    if (std::binary_search(
                            s.input_edges.begin(),
                            s.input_edges.end(),
                            sorted(Edge{corners[i], corners[j]})))
                        return false;
        }
        return true;
    }
    return s.collapsing_regions && is_rod_offset_vertex(input, removed) &&
           is_rod_offset_vertex(input, survivor) &&
           input.corr_input_vertex[removed] == input.corr_input_vertex[survivor] &&
           !s.frozen_offset.count(removed) && !s.frozen_offset.count(survivor);
}

void apply_rod_targets(PrismaticMeshInput& input)
{
    if (!input.rod_state) return;
    auto& s = *input.rod_state;
    size_t rod = 0, shell = 0, fixed = 0;
    for (size_t v : input.offset_vertices) {
        if (v >= input.vertex_component_ids.size() || input.vertex_component_ids[v] < 0) continue;
        if (s.frozen_offset.count(v)) {
            input.target_positions.row(v) = input.vertices.row(v);
            input.singular_vertex_tags[v] = 1;
            ++fixed;
        } else if (is_rod_offset_vertex(input, v)) {
            const auto found = s.frame_targets.find(v);
            input.target_positions.row(v) = input.vertices.row(v);
            if (found != s.frame_targets.end())
                input.target_positions.row(v) = found->second.transpose();
            input.singular_vertex_tags[v] = found == s.frame_targets.end() ? 1 : 0;
            if (found != s.frame_targets.end()) {
                const auto parent = input.corr_input_vertex[v];
                input.optimal_normals.row(v) =
                    (found->second - position(input, parent)).normalized().transpose();
                ++rod;
            }
        } else if (input.singular_vertex_tags[v] == 0)
            ++shell;
    }
    s.report["shared_targets"] = {
        {"rod_vertices", rod},
        {"shell_vertices", shell},
        {"protected_vertices", fixed}};
}

void construct_rod_frame_topology(PrismaticMeshInput& input)
{
    if (!input.rod_state || input.rod_state->options.iterations == 0) return;
    const size_t collapses = collapse_vertex_regions(input);
    extract_frames(input, true);
    const size_t splits = equalize_degrees(input, true);
    if (splits) {
        transport_rod_frame_mapping(input);
        extract_frames(input, true);
    }
    update_frame_targets(input);
    relocate_frames(input);
    flatten_rod_frames(input);
    extract_frames(input, true);
    verify_junction_one_rings(input);
    record_rod_frames(input);
    auto& state = *input.rod_state;
    size_t faces = 0;
    for (const auto& [center, region] : region_surface(input).regions)
        if (!state.protected_input.count(center)) faces += region.triangles.size();
    state.report["frame_topology_construction"] = {
        {"method", "vertex_region_to_ring"},
        {"fiber_tets_after", faces},
        {"collapses", collapses},
        {"degree_splits", splits}};
    state.report["frame_topology_rounds"].push_back(state.report["frame_topology_construction"]);
    state.report["tau_after_frame_construction"] = analyze_rod_tau(input);
}
} // namespace wmtk::components::prismatic_mesh
