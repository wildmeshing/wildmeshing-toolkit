#include "rod_mesh.hpp"
#include "background_pyramids.hpp"
#include "junction_protection.hpp"
#include "prism_jacobian.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
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

// A conforming stellar split of an existing ring edge, including every background
// coface. New ring vertices inherit the same parent; centerline splits instead are
// constructed together with their complete new frame in split_rod_intervals().
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
    if (s.options.unified_optimization)
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
        require(
            s.options.unified_optimization || in.size() <= 2,
            "rod_mode requires a curve complex, not input triangles or solids");
        if (in.size() == 2 && !surface_edges.count(sorted(Edge{in[0], in[1]})))
            edges.insert(sorted(Edge{in[0], in[1]}));
    }
    require(
        s.options.unified_optimization || !edges.empty(),
        "no centerline edges in the offset band");
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
        nlohmann::json edges = nlohmann::json::array();
        for (size_t order = 0; order < path.size(); ++order) {
            const auto& e = s.directed_input_edges[path[order]];
            if (order > 0)
                require(
                    s.directed_input_edges[path[order - 1]][1] == e[0],
                    "discontinuous directed rod path");
            edges.push_back(
                {{"edge_index", path[order]},
                 {"edge_order", order},
                 {"from_vid", input.source_vertex_ids[e[0]]},
                 {"to_vid", input.source_vertex_ids[e[1]]}});
        }
        s.report["rod_paths"].push_back(
            {{"rod", r},
             {"closed",
              s.directed_input_edges[path.front()][0] == s.directed_input_edges[path.back()][1]},
             {"edges", std::move(edges)}});
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
    s.report["unified_optimization"] = s.options.unified_optimization;
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
        if (s.degree_blocked_centers.count(f.center))
            f.reason = "degree_alignment_blocked";
        else if (s.protected_input.count(f.center))
            f.reason = "protected_rod_connection";
        else if (std::any_of(f.ring.begin(), f.ring.end(), [&](size_t v) {
                     return s.frozen_offset.count(v) || !junction_collapse_allowed(input, v);
                 }))
            f.reason = "protected_joint_one_ring";
        else
            f.valid = true;
    };
    for (size_t i = 0; i < s.input_edges.size(); ++i) {
        RodFrame f;
        f.edge = s.input_edges[i];
        f.edge_index = i;
        f.rod = rod[i];
        f.order = order[i];
        f.ring = ordered_ring(links[f.edge]);
        f.initial_degree = f.ring.size();
        std::set<int64_t> parents;
        for (size_t v : f.ring) parents.insert(input.corr_input_vertex[v]);
        if (f.ring.empty())
            f.reason = "noncyclic_link";
        else if (parents.size() != 1)
            f.reason = "tau22_in_link";
        else if (
            *parents.begin() < 0 || (static_cast<size_t>(*parents.begin()) != f.edge[0] &&
                                     static_cast<size_t>(*parents.begin()) != f.edge[1]))
            f.reason = "parent_not_on_input_edge";
        else {
            f.center = static_cast<size_t>(*parents.begin());
            orient_frame(f);
        }
        s.frames.push_back(std::move(f));
    }
    // Incoming/outgoing edge links may give two candidate sections at one regular
    // center. Keep one material frame; the other same-parent vertices remain
    // eligible for the checked fiber simplification instead of freezing both rings.
    for (size_t i = 0; i < s.frames.size(); ++i) {
        auto& f = s.frames[i];
        if (f.valid && s.vertex_frame.count(f.center)) {
            f.valid = false;
            f.reason = "secondary_edge_link";
        }
        if (f.valid) s.vertex_frame[f.center] = i;
    }
    // An edge link supplies only one of its two material sections. At a free
    // endpoint the other section may have no incoming edge to supply it. Recover
    // that rim from the lateral offset front's correspondence boundary, including
    // when a same-parent cap continues beyond it. Never use a geometric angle cap.
    std::map<Tri, size_t> front;
    for (size_t tid = 0; tid < static_cast<size_t>(input.tetrahedra.rows()); ++tid) {
        if (input.offset_tet_tags[tid] != 1) continue;
        const auto t = tet_at(input, tid);
        for (size_t omitted = 0; omitted < 4; ++omitted) {
            Tri f;
            size_t j = 0;
            for (size_t k = 0; k < 4; ++k)
                if (k != omitted) f[j++] = t[k];
            if (std::all_of(f.begin(), f.end(), [&](size_t v) {
                    return input.vertex_tags[v] == 2;
                }))
                ++front[sorted(f)];
        }
    }
    std::map<size_t, std::map<size_t, std::set<Edge>>> rims;
    for (const auto& [f, count] : front) {
        if (count != 1) continue;
        for (size_t j = 0; j < 3; ++j) {
            const size_t a = f[j], b = f[(j + 1) % 3], c = f[(j + 2) % 3];
            const int64_t parent = input.corr_input_vertex[a];
            const int64_t neighbor = input.corr_input_vertex[c];
            if (parent < 0 || neighbor < 0 || parent == neighbor ||
                parent != input.corr_input_vertex[b] || !s.input_vertices.count(parent) ||
                s.protected_input.count(parent) ||
                !std::binary_search(
                    s.input_edges.begin(),
                    s.input_edges.end(),
                    sorted(Edge{static_cast<size_t>(parent), static_cast<size_t>(neighbor)})))
                continue;
            rims[parent][neighbor].insert(sorted(Edge{a, b}));
        }
    }
    for (size_t v : s.input_vertices) {
        if (s.protected_input.count(v)) continue;
        if (s.vertex_frame.count(v)) {
            s.frames[s.vertex_frame.at(v)].terminal_section = s.free_endpoints.count(v);
            continue;
        }
        std::vector<size_t> chosen;
        size_t neighbor = v;
        double best = std::numeric_limits<double>::infinity();
        const Vector3d normal = s.vertex_rotations.at(v) * Vector3d::UnitZ();
        for (const auto& [other, edges] : rims[v]) {
            auto ring = ordered_ring({edges.begin(), edges.end()});
            if (ring.empty()) continue;
            double cost = 0;
            for (size_t w : ring) {
                const Vector3d radial = position(input, w) - position(input, v);
                cost += std::pow(radial.dot(normal), 2) + std::pow(radial.norm() - s.radius, 2);
            }
            if (cost / ring.size() < best) {
                best = cost / ring.size();
                chosen = std::move(ring);
                neighbor = other;
            }
        }
        const auto edge =
            std::find_if(s.input_edges.begin(), s.input_edges.end(), [&](const Edge& e) {
                return (e[0] == v || e[1] == v) &&
                       (neighbor == v || e[0] == neighbor || e[1] == neighbor);
            });
        RodFrame f;
        f.edge = *edge;
        f.edge_index = edge - s.input_edges.begin();
        f.rod = rod[f.edge_index];
        f.order = order[f.edge_index];
        f.center = v;
        f.terminal_section = s.free_endpoints.count(v);
        f.ring = std::move(chosen);
        f.initial_degree = f.ring.size();
        if (f.ring.empty())
            f.reason = "noncyclic_correspondence_section";
        else
            orient_frame(f);
        if (f.valid) s.vertex_frame[v] = s.frames.size();
        s.frames.push_back(std::move(f));
    }
}

void equalize_degrees(
    PrismaticMeshInput& input,
    const std::vector<size_t>& minimum_degrees = {},
    bool preserve_rotations = false,
    bool include_secondary = false)
{
    auto& s = *input.rod_state;
    std::vector<size_t> target(s.rods.size(), 0);
    const auto eligible = [&](const RodFrame& f) {
        return f.valid || (include_secondary && f.reason == "secondary_edge_link");
    };
    for (const auto& f : s.frames)
        if (eligible(f)) target[f.rod] = std::max(target[f.rod], f.ring.size());
    for (size_t i = 0; i < minimum_degrees.size(); ++i)
        target[i] = std::max(target[i], minimum_degrees[i]);
    s.report["target_degrees"] = target;
    size_t splits = 0, blocked = 0;
    for (size_t index = 0; index < s.frames.size(); ++index) {
        while (eligible(s.frames[index]) &&
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
                size_t fibers = 0;
                if (include_secondary)
                    for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(a)) {
                        const auto tet = tet_at(input, tid);
                        if (input.offset_tet_tags[tid] != 1 ||
                            std::find(tet.begin(), tet.end(), b) == tet.end())
                            continue;
                        const auto in = input_corners(input, tet);
                        if (in.size() == 1 && in[0] == f.center &&
                            std::all_of(tet.begin(), tet.end(), [&](size_t v) {
                                return v == f.center ||
                                       input.corr_input_vertex[v] == static_cast<int64_t>(f.center);
                            }))
                            ++fibers;
                    }
                candidates.push_back({1e6 * fibers - angle, {a, b}});
            }
            std::sort(candidates.begin(), candidates.end());
            bool accepted = false;
            for (const auto& [angle, edge] : candidates) {
                // A shared ring edge can refine a second edge coface as well. Do
                // not grow an already aligned section past the common degree.
                if (include_secondary &&
                    std::any_of(s.frames.begin(), s.frames.end(), [&](const RodFrame& other) {
                        if (!eligible(other) || other.ring.size() < target[other.rod]) return false;
                        for (size_t k = 0; k < other.ring.size(); ++k)
                            if (sorted(
                                    Edge{other.ring[k], other.ring[(k + 1) % other.ring.size()]}) ==
                                sorted(edge))
                                return true;
                        return false;
                    }))
                    continue;
                if (split_ring_edge(input, edge, s.options.min_tet_volume)) {
                    accepted = true;
                    ++splits;
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
    s.report["ring_edge_splits"] = splits;
    s.report["degree_alignment_blocked"] = blocked;
    for (auto& f : s.frames)
        if (f.valid && f.ring.size() != target[f.rod]) {
            // A valid section can still match an adjacent section of the same
            // degree when the rod-wide target is blocked by a fixed junction.
            f.reason = "degree_alignment_limited";
        }
}

void simplify_fibers(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    size_t collapses = 0;
    // Reuse the exact link/volume checked collapse. Frame ring vertices and all
    // joint one-ring vertices survive; only redundant same-parent vertices collapse.
    for (int round = 0; round < s.options.iterations; ++round) {
        evaluate_target_positions(input, s.options.thicknessratio);
        std::set<size_t> retained = s.frozen_offset;
        for (const auto& f : s.frames)
            if (f.valid) retained.insert(f.ring.begin(), f.ring.end());
        std::vector<std::pair<double, Edge>> candidates;
        for (const auto& e : input.mesh->get_edges()) {
            size_t a = e.vid(*input.mesh), b = e.switch_vertex(*input.mesh).vid(*input.mesh);
            if (input.vertex_tags[a] != 2 || input.vertex_tags[b] != 2 ||
                input.corr_input_vertex[a] != input.corr_input_vertex[b] ||
                (s.options.unified_optimization && !is_rod_offset_vertex(input, a)) ||
                s.frozen_offset.count(a) || s.frozen_offset.count(b))
                continue;
            if (retained.count(a)) std::swap(a, b);
            if (retained.count(a)) continue;
            // Prefer a ring survivor before shortening non-ring edges.
            const double cost = (position(input, a) - position(input, b)).squaredNorm();
            candidates.push_back({cost + (retained.count(b) ? 0 : 1e6), {a, b}});
        }
        std::sort(candidates.begin(), candidates.end());
        size_t accepted = 0;
        for (const auto& [cost, edge] : candidates)
            if (try_collapse_offset_edge(input, edge[0], edge[1], s.options.min_tet_volume))
                ++accepted;
        collapses += accepted;
        extract_frames(input);
        if (!accepted) break;
    }
    s.report["fiber_collapses"] = collapses;
}

void normalize_tau22(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    size_t unlocked = 0, attempted = 0;
    for (int round = 0; round < s.options.iterations; ++round) {
        std::vector<Tet> pending;
        for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
            const auto t = tet_at(input, i);
            if (input.offset_tet_tags[i] != 1 || protected_tet(input, t) ||
                !classify_tau22(input, i))
                continue;
            if (s.options.unified_optimization) {
                const auto in = input_corners(input, t);
                if (in.size() != 2 || !std::binary_search(
                                          s.input_edges.begin(),
                                          s.input_edges.end(),
                                          sorted(Edge{in[0], in[1]})))
                    continue;
            }
            if (std::any_of(t.begin(), t.end(), [&](size_t v) { return s.frozen_offset.count(v); }))
                continue;
            pending.push_back(sorted(t));
        }
        if (pending.empty()) break;
        evaluate_target_positions(input, s.options.thicknessratio);
        std::set<Tet> protected_cells;
        for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i)
            if (protected_tet(input, tet_at(input, i)))
                protected_cells.insert(sorted(tet_at(input, i)));
        size_t accepted = 0;
        for (const auto& key : pending) {
            std::optional<size_t> tid;
            for (size_t i : input.mesh->get_one_ring_tids_for_vertex(key[0]))
                if (sorted(tet_at(input, i)) == key) tid = i;
            if (!tid || !classify_tau22(input, *tid)) continue;
            ++attempted;
            for (int side = 0; side < 2; ++side) {
                auto candidate = copy_input(input);
                rebuild(candidate);
                if (!try_unlock_tau22(candidate, *tid, side, s.options.min_tet_volume)) continue;
                std::set<Tet> preserved;
                for (size_t i = 0; i < static_cast<size_t>(candidate.tetrahedra.rows()); ++i)
                    if (protected_tet(candidate, tet_at(candidate, i)))
                        preserved.insert(sorted(tet_at(candidate, i)));
                if (preserved != protected_cells) continue;
                const auto tau = analyze_rod_tau(candidate);
                if (tau["invalid_correspondence"] != 0 || tau["edge_parent_mismatch"] != 0)
                    continue;
                input = std::move(candidate);
                ++accepted;
                break;
            }
        }
        unlocked += accepted;
        if (!accepted) break;
    }
    s.report["tau22_unlock_attempts"] = attempted;
    s.report["tau22_unlocked"] = unlocked;
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
    // Vertex poses are fixed incident-edge averages. Only the cyclic assignment
    // of existing ring vertices is aligned; no independent vertex twist fit.
    for (size_t i = 0; i < count; ++i) {
        auto& f = s.frames[i];
        if (!f.valid) continue;
        const Vector3d center = position(input, f.center);
        double best = std::numeric_limits<double>::infinity();
        size_t shift = 0;
        for (size_t j = 0; j < f.ring.size(); ++j) {
            double cost = 0;
            for (size_t k = 0; k < f.ring.size(); ++k) {
                const double a = 2 * pi * k / f.ring.size();
                const Vector3d target =
                    center + s.radius * (f.rotation * Vector3d(std::cos(a), std::sin(a), 0));
                cost += (position(input, f.ring[(k + j) % f.ring.size()]) - target).squaredNorm();
            }
            if (cost < best) {
                best = cost;
                shift = j;
            }
        }
        std::rotate(f.ring.begin(), f.ring.begin() + shift, f.ring.end());
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

std::map<Tri, int> patch_boundary(const std::vector<Tet>& cells)
{
    std::map<Tri, int> result;
    for (const auto& t : cells) {
        const std::array<Tri, 4> faces{
            {{t[1], t[2], t[3]}, {t[0], t[3], t[2]}, {t[0], t[1], t[3]}, {t[0], t[2], t[1]}}};
        for (const auto& f : faces) {
            int parity = 0;
            for (int i = 0; i < 3; ++i)
                for (int j = i + 1; j < 3; ++j) parity += f[i] > f[j];
            result[sorted(f)] += parity % 2 ? -1 : 1;
        }
    }
    for (auto it = result.begin(); it != result.end();) {
        if (it->second == 0)
            it = result.erase(it);
        else
            ++it;
    }
    return result;
}

struct RodFrontTriangle
{
    Tri face;
    size_t tid;
};

// Exchange one lateral diagonal by reconstructing its complete band/background
// cavity. This is the volumetric counterpart of a split followed by a collapse
// on the annular front; it constructs the required product connectivity rather
// than waiting for an original three-tet partition to have that connectivity.
bool flip_rod_front(
    PrismaticMeshInput& input,
    const Edge& diagonal,
    const std::vector<RodFrontTriangle>& triangles)
{
    const auto reject = [&](const char* why) {
        auto& reasons = input.rod_state->report["annulus_exchange_rejections"];
        if (reasons.is_null()) reasons = nlohmann::json::object();
        reasons[why] = reasons.value(why, size_t(0)) + 1;
        return false;
    };
    if (triangles.size() != 2) return false;
    const auto& first = triangles[0];
    const auto& second = triangles[1];
    const auto cap = [&](const RodFrontTriangle& f) -> std::optional<size_t> {
        const auto in = input_corners(input, tet_at(input, f.tid));
        if (in.size() != 1) return {};
        return in[0];
    };
    const auto apex = cap(first);
    if (!apex || cap(second) != apex) return reject("different_band_apices");
    size_t c = 0, d = 0;
    for (size_t v : first.face)
        if (v != diagonal[0] && v != diagonal[1]) c = v;
    for (size_t v : second.face)
        if (v != diagonal[0] && v != diagonal[1]) d = v;
    if (c == d || input.corr_input_vertex[c] == input.corr_input_vertex[d]) return false;
    auto f = first.face;
    std::rotate(f.begin(), std::find(f.begin(), f.end(), c), f.end());
    const std::array<size_t, 4> quad{c, f[1], d, f[2]};
    const std::array<Tri, 2> new_faces{{{c, f[1], d}, {c, d, f[2]}}};
    std::vector<Tet> added;
    for (const auto& face : new_faces) {
        const Tet t{*apex, face[0], face[1], face[2]};
        if (!tet_volume_above_threshold(input.vertices, t, input.rod_state->options.min_tet_volume))
            return reject("new_band_tet_not_positive");
        added.push_back(t);
    }
    std::set<size_t> removed{first.tid, second.tid};
    std::vector<int> regions(2, input.input_cells[first.tid]), bands(2, 1);
    auto candidate = copy_input(input);
    const detail::BackgroundQuad request{quad, {first.face, second.face}};
    const auto plans = detail::plan_background_pyramids(input, {request}, 0);
    if (plans.accepted.empty())
        input.rod_state->report["last_annulus_background_plan"] = plans.report;
    if (!plans.accepted.empty()) {
        const auto& plan = plans.accepted.begin()->second;
        const size_t p = append_vertex(candidate, plan.point, -1);
        removed.insert(plan.cavity.begin(), plan.cavity.end());
        const auto add_background = [&](Tri face) {
            const Tet t{p, face[0], face[1], face[2]};
            if (!tet_volume_above_threshold(candidate.vertices, t, 0)) return false;
            added.push_back(t);
            regions.push_back(input.input_cells[plan.cavity.front()]);
            bands.push_back(input.offset_tet_tags[plan.cavity.front()]);
            return true;
        };
        for (const auto& face : plan.boundary)
            if (sorted(face) != sorted(first.face) && sorted(face) != sorted(second.face))
                if (!add_background(face)) return reject("new_background_tet_not_positive");
        for (auto face : new_faces) {
            std::swap(face[1], face[2]);
            if (!add_background(face)) return reject("new_background_tet_not_positive");
        }
    } else {
        // A convex band-side 2-3 exchange needs no background cavity. Its thin
        // outer wedge becomes background and retains the old outer triangles.
        Tet wedge{c, d, diagonal[0], diagonal[1]};
        if (!tet_volume_above_threshold(candidate.vertices, wedge, 0))
            std::swap(wedge[0], wedge[1]);
        if (!tet_volume_above_threshold(candidate.vertices, wedge, 0)) return false;
        added.push_back(wedge);
        regions.push_back(0);
        bands.push_back(-1);
    }
    std::vector<Tet> old;
    for (size_t tid : removed) {
        const auto t = tet_at(input, tid);
        if (protected_tet(input, t)) return false;
        old.push_back(t);
    }
    if (patch_boundary(old) != patch_boundary(added)) return reject("cavity_boundary_mismatch");
    std::vector<Tet> cells;
    std::vector<int> all_regions, all_bands;
    for (size_t tid = 0; tid < input.offset_tet_tags.size(); ++tid) {
        if (removed.count(tid)) continue;
        cells.push_back(tet_at(input, tid));
        all_regions.push_back(input.input_cells[tid]);
        all_bands.push_back(input.offset_tet_tags[tid]);
    }
    cells.insert(cells.end(), added.begin(), added.end());
    all_regions.insert(all_regions.end(), regions.begin(), regions.end());
    all_bands.insert(all_bands.end(), bands.begin(), bands.end());
    candidate.tetrahedra.resize(cells.size(), 4);
    for (size_t i = 0; i < cells.size(); ++i)
        for (int k = 0; k < 4; ++k) candidate.tetrahedra(i, k) = cells[i][k];
    candidate.input_cells = std::move(all_regions);
    candidate.offset_tet_tags = std::move(all_bands);
    rebuild(candidate);
    input = std::move(candidate);
    return true;
}

void align_rod_annuli(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    size_t flips = 0, failed = 0;
    size_t operation_budget = 0;
    for (const auto& axis : s.input_edges) {
        if (!s.vertex_frame.count(axis[0]) || !s.vertex_frame.count(axis[1])) continue;
        const size_t degree = s.frames[s.vertex_frame.at(axis[0])].ring.size();
        operation_budget += 2 * degree * (degree / 2) * (degree / 2);
    }
    const auto energy = [](size_t i, size_t j, size_t degree) {
        const size_t d = (i + degree - j) % degree;
        const size_t distance = std::min(d, degree - d);
        return distance * distance;
    };
    for (size_t operation = 0; operation < operation_budget; ++operation) {
        std::map<Tri, std::vector<RodFrontTriangle>> owners;
        for (size_t tid = 0; tid < input.offset_tet_tags.size(); ++tid) {
            if (input.offset_tet_tags[tid] != 1) continue;
            const auto t = tet_at(input, tid);
            const std::array<Tri, 4> faces{
                {{t[1], t[2], t[3]}, {t[0], t[3], t[2]}, {t[0], t[1], t[3]}, {t[0], t[2], t[1]}}};
            for (const auto& f : faces)
                if (std::all_of(f.begin(), f.end(), [&](size_t v) {
                        return input.vertex_tags[v] == 2;
                    }))
                    owners[sorted(f)].push_back({f, tid});
        }
        std::map<Edge, std::map<Edge, std::vector<RodFrontTriangle>>> annuli;
        for (const auto& [key, front] : owners) {
            if (front.size() != 1) continue;
            const auto& f = front[0].face;
            std::set<size_t> parents;
            for (size_t v : f) parents.insert(input.corr_input_vertex[v]);
            if (parents.size() != 2) continue;
            const Edge axis{*parents.begin(), *parents.rbegin()};
            if (!std::binary_search(s.input_edges.begin(), s.input_edges.end(), axis)) continue;
            for (int k = 0; k < 3; ++k) {
                const size_t a = f[k], b = f[(k + 1) % 3];
                if (input.corr_input_vertex[a] != input.corr_input_vertex[b])
                    annuli[axis][sorted(Edge{a, b})].push_back(front[0]);
            }
        }
        bool changed = false;
        for (const auto& [axis, edges] : annuli) {
            if (!s.vertex_frame.count(axis[0]) || !s.vertex_frame.count(axis[1])) continue;
            const auto& a = s.frames[s.vertex_frame.at(axis[0])].ring;
            const auto& b = s.frames[s.vertex_frame.at(axis[1])].ring;
            const size_t degree = a.size();
            if (degree < 3 || b.size() != degree) continue;
            std::map<size_t, size_t> lower, upper;
            for (size_t k = 0; k < degree; ++k) {
                lower[a[k]] = k;
                upper[b[k]] = k;
            }
            const auto indices = [&](Edge e) -> std::optional<Edge> {
                if (upper.count(e[0])) std::swap(e[0], e[1]);
                if (!lower.count(e[0]) || !upper.count(e[1])) return {};
                return Edge{lower.at(e[0]), upper.at(e[1])};
            };
            size_t best = std::numeric_limits<size_t>::max(), shift = 0;
            for (size_t offset = 0; offset < degree; ++offset) {
                size_t score = 0;
                for (const auto& [e, fs] : edges) {
                    const auto ij = indices(e);
                    if (ij)
                        score += energy((*ij)[0], ((*ij)[1] + degree - offset) % degree, degree);
                }
                if (score < best) {
                    best = score;
                    shift = offset;
                }
            }
            for (const auto& [e, fs] : edges) {
                if (fs.size() != 2) continue;
                const auto old = indices(e);
                if (!old) continue;
                Edge opposite;
                for (int k = 0; k < 2; ++k)
                    for (size_t v : fs[k].face)
                        if (v != e[0] && v != e[1]) opposite[k] = v;
                const auto next = indices(opposite);
                if (!next || edges.count(sorted(opposite))) continue;
                if (energy((*next)[0], ((*next)[1] + degree - shift) % degree, degree) >=
                    energy((*old)[0], ((*old)[1] + degree - shift) % degree, degree))
                    continue;
                if (!flip_rod_front(input, e, fs)) {
                    ++failed;
                    continue;
                }
                ++flips;
                changed = true;
                break;
            }
            if (changed) break;
        }
        if (!changed) break;
    }
    s.report["annulus_diagonal_exchanges"] = {{"applied", flips}, {"rejected", failed}};
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

// Extract columns only through the selected rings. Thus an equal cardinality alone
// cannot claim a prism: all three source tets must realize the same correspondence.
void discover_columns(PrismaticMeshInput& input)
{
    auto& s = *input.rod_state;
    std::set<Tet> band_keys;
    for (size_t i = 0; i < input.offset_tet_tags.size(); ++i)
        if (input.offset_tet_tags[i] == 1) band_keys.insert(sorted(tet_at(input, i)));
    std::map<Edge, std::map<size_t, size_t>> correspondences;
    const auto matching = [&](size_t u, size_t v) -> const std::map<size_t, size_t>& {
        const Edge key{u, v};
        const auto found = correspondences.find(key);
        if (found != correspondences.end()) return found->second;
        auto& result = correspondences[key];
        const auto& first = s.frames[s.vertex_frame.at(u)];
        const auto& second = s.frames[s.vertex_frame.at(v)];
        auto a = first.ring, b = second.ring;
        Vector3d ta = first.tangent, tb = second.tangent;
        // Rings already have the winding of their directed rod edge. Orientations
        // must not be inferred again from the center-to-center chord: at a right
        // angle its dot product with one tangent is zero, leaving an ambiguity.
        const auto transport = Eigen::Quaterniond::FromTwoVectors(ta, tb);
        double best = std::numeric_limits<double>::infinity();
        size_t best_sectors = 0;
        size_t shift = 0;
        for (size_t offset = 0; offset < a.size(); ++offset) {
            double cost = 0;
            size_t sectors = 0;
            for (size_t k = 0; k < a.size(); ++k) {
                const auto vertices = sorted(
                    std::array<size_t, 6>{
                        u,
                        a[k],
                        a[(k + 1) % a.size()],
                        v,
                        b[(k + offset) % b.size()],
                        b[(k + 1 + offset) % b.size()]});
                size_t tets = 0;
                for (size_t i = 0; i < 3; ++i)
                    for (size_t j = i + 1; j < 4; ++j)
                        for (size_t l = j + 1; l < 5; ++l)
                            for (size_t m = l + 1; m < 6; ++m)
                                tets += band_keys.count(
                                    {vertices[i], vertices[j], vertices[l], vertices[m]});
                sectors += tets == 3;
            }
            for (size_t k = 0; k < a.size(); ++k)
                cost += (transport * (position(input, a[k]) - position(input, u)) -
                         (position(input, b[(k + offset) % b.size()]) - position(input, v)))
                            .squaredNorm();
            if (sectors > best_sectors || (sectors == best_sectors && cost < best)) {
                best_sectors = sectors;
                best = cost;
                shift = offset;
            }
        }
        for (size_t k = 0; k < a.size(); ++k) result[a[k]] = b[(k + shift) % b.size()];
        return result;
    };
    std::map<Tri, std::vector<size_t>> faces;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i)
        if (input.offset_tet_tags[i] == 1)
            for (int omitted = 0; omitted < 4; ++omitted) {
                Tri f;
                int k = 0;
                for (int j = 0; j < 4; ++j)
                    if (j != omitted) f[k++] = input.tetrahedra(i, j);
                faces[sorted(f)].push_back(i);
            }
    std::map<std::array<size_t, 6>, std::array<size_t, 6>> unique;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        if (input.offset_tet_tags[i] != 1) continue;
        const auto t = tet_at(input, i);
        const auto in = input_corners(input, t);
        if (in.size() != 2 || protected_tet(input, t)) continue;
        if (!std::binary_search(
                s.input_edges.begin(),
                s.input_edges.end(),
                sorted(Edge{in[0], in[1]})))
            continue;
        std::vector<size_t> off;
        for (size_t v : t)
            if (input.vertex_tags[v] == 2) off.push_back(v);
        if (off.size() != 2 || input.corr_input_vertex[off[0]] != input.corr_input_vertex[off[1]])
            continue;
        const size_t v = input.corr_input_vertex[off[0]];
        if (v != in[0] && v != in[1]) continue;
        const size_t u = in[0] == v ? in[1] : in[0];
        if (!s.vertex_frame.count(u) || !s.vertex_frame.count(v)) continue;
        const auto& bottom = s.frames[s.vertex_frame.at(u)];
        const auto& top = s.frames[s.vertex_frame.at(v)];
        if (bottom.ring.size() != top.ring.size()) continue;
        const auto& mapped = matching(u, v);
        const std::set<size_t> lower(bottom.ring.begin(), bottom.ring.end()),
            upper(top.ring.begin(), top.ring.end());
        if (!upper.count(off[0]) || !upper.count(off[1])) continue;
        for (int side = 0; side < 2; ++side) {
            const size_t a = off[side], b = off[1 - side];
            for (size_t j : faces[sorted(Tri{u, a, b})]) {
                if (j == i) continue;
                size_t x = input.vertices.rows();
                for (size_t w : tet_at(input, j))
                    if (w != u && w != a && w != b) x = w;
                if (!lower.count(x) || mapped.at(x) != a) continue;
                for (size_t k : faces[sorted(Tri{u, x, b})]) {
                    if (k == j) continue;
                    size_t y = input.vertices.rows();
                    for (size_t w : tet_at(input, k))
                        if (w != u && w != x && w != b) y = w;
                    if (!lower.count(y) || y == x || mapped.at(y) != b) continue;
                    const std::array<size_t, 6> column{u, x, y, v, a, b};
                    unique.emplace(sorted(column), column);
                }
            }
        }
    }
    s.columns.clear();
    for (const auto& [key, column] : unique) s.columns.push_back(column);
    s.report["matched_column_candidates"] = s.columns.size();
    for (size_t r = 0; r < s.rods.size(); ++r) {
        const auto& path = s.rods[r];
        const size_t start = s.directed_input_edges[path.front()][0];
        const size_t end = s.directed_input_edges[path.back()][1];
        const auto kind = [&](size_t v) {
            if (s.interface_vertices.count(v)) return "non_rod_attachment";
            if (s.joints.count(v)) return "rod_joint";
            if (s.free_endpoints.count(v)) return "free_end";
            return "periodic_frame";
        };
        auto& report = s.report["rod_paths"][r];
        report["start_kind"] = kind(start);
        report["end_kind"] = kind(end);
        report["frame_sequence"] = nlohmann::json::array();
        const auto append = [&](size_t v) {
            report["frame_sequence"].push_back(
                {{"center_vid", input.source_vertex_ids[v]},
                 {"frame_index",
                  s.vertex_frame.count(v) ? nlohmann::json(s.vertex_frame.at(v))
                                          : nlohmann::json(nullptr)},
                 {"one_ring_protected", s.protected_input.count(v) != 0}});
        };
        append(start);
        for (size_t e : path) append(s.directed_input_edges[e][1]);
        if (start == end) {
            report["periodic_shared_frame"] = s.vertex_frame.count(start) != 0;
            const auto closing = sorted(s.input_edges[path.back()]);
            report["closing_interval_candidate_columns"] =
                std::count_if(s.columns.begin(), s.columns.end(), [&](const auto& c) {
                    return sorted(Edge{c[0], c[3]}) == closing;
                });
        }
    }
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
             {"center_vid",
              f.valid ? nlohmann::json(input.source_vertex_ids[f.center])
                      : nlohmann::json(nullptr)},
             {"degree", f.ring.size()},
             {"rod", f.rod},
             {"edge_order", f.order},
             {"edge_index", f.edge_index},
             {"terminal_section", f.terminal_section},
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
        if (in.size() == 2 && (!input.rod_state || !input.rod_state->options.unified_optimization ||
                               std::binary_search(
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

void prepare_rod_mesh(PrismaticMeshInput& input, const RodOptions& options)
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
    require(
        options.unified_optimization || std::none_of(
                                            input.input_cells.begin(),
                                            input.input_cells.end(),
                                            [](int tag) { return tag == 1; }),
        "rod_mode does not accept input volume cells");
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
    normalize_tau22(input);
    extract_frames(input);
    s.report["initial_frame_degrees"] = nlohmann::json::array();
    for (const auto& f : s.frames) s.report["initial_frame_degrees"].push_back(f.ring.size());
    equalize_degrees(input, {}, false, true);
    if (!options.unified_optimization) {
        simplify_fibers(input);
        relocate_frames(input);
    } else {
        // Fit only cyclic vertex indexing. Geometry is moved later by the shared
        // per-vertex smoother, with shell, interface and background cells present.
        for (auto& f : s.frames) {
            if (!f.valid) continue;
            double best = std::numeric_limits<double>::infinity();
            size_t shift = 0;
            for (size_t j = 0; j < f.ring.size(); ++j) {
                double cost = 0;
                for (size_t k = 0; k < f.ring.size(); ++k) {
                    const double a = 2 * pi * k / f.ring.size();
                    const Vector3d target =
                        position(input, f.center) +
                        s.radius * (f.rotation * Vector3d(std::cos(a), std::sin(a), 0));
                    cost +=
                        (position(input, f.ring[(k + j) % f.ring.size()]) - target).squaredNorm();
                }
                if (cost < best) {
                    best = cost;
                    shift = j;
                }
            }
            std::rotate(f.ring.begin(), f.ring.begin() + shift, f.ring.end());
            for (size_t k = 0; k < f.ring.size(); ++k) {
                const double a = 2 * pi * k / f.ring.size();
                s.retained_ring_vertices.insert(f.ring[k]);
                s.frame_targets[f.ring[k]] =
                    position(input, f.center) +
                    s.radius * (f.rotation * Vector3d(std::cos(a), std::sin(a), 0));
            }
        }
        s.report["rod_target_vertices"] = s.frame_targets.size();
    }
    discover_columns(input);
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
        "Rod construction: {} rods, {} joints, {} frames, {} candidate columns; radius {}",
        s.rods.size(),
        s.joints.size(),
        s.vertex_frame.size(),
        s.columns.size(),
        s.radius);
}

void preprocess_rod_mesh(PrismaticMeshInput& input, RodOptions options)
{
    options.unified_optimization = true;
    prepare_rod_mesh(input, options);
}

bool is_rod_offset_vertex(const PrismaticMeshInput& input, size_t v)
{
    return input.rod_state && v < input.vertex_tags.size() && input.vertex_tags[v] == 2 &&
           input.corr_input_vertex[v] >= 0 &&
           input.rod_state->input_vertices.count(static_cast<size_t>(input.corr_input_vertex[v]));
}

bool rod_collapse_allowed(const PrismaticMeshInput& input, size_t removed, size_t survivor)
{
    if (!input.rod_state || !input.rod_state->options.unified_optimization) return true;
    const auto& s = *input.rod_state;
    return !s.frozen_offset.count(removed) && !s.frozen_offset.count(survivor) &&
           !s.retained_ring_vertices.count(removed) &&
           is_rod_offset_vertex(input, removed) == is_rod_offset_vertex(input, survivor);
}

void apply_rod_targets(PrismaticMeshInput& input)
{
    if (!input.rod_state || !input.rod_state->options.unified_optimization) return;
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

void refresh_rod_prism_candidates(PrismaticMeshInput& input)
{
    if (!input.rod_state) return;
    discover_columns(input);
    input.rod_state->report["tau_after_shared_optimization"] = analyze_rod_tau(input);
}

void construct_rod_frame_topology(PrismaticMeshInput& input)
{
    if (!input.rod_state || !input.rod_state->options.unified_optimization ||
        input.rod_state->options.iterations == 0)
        return;
    std::set<size_t> required_frames, eligible_edges;
    for (const auto& [v, index] : input.rod_state->vertex_frame) required_frames.insert(v);
    for (size_t e = 0; e < input.rod_state->input_edges.size(); ++e) {
        const auto& f = input.rod_state->frames[e];
        if (f.valid || f.reason == "secondary_edge_link") eligible_edges.insert(e);
    }
    std::set<size_t> free_cap_vertices;
    for (size_t v : input.rod_state->free_endpoints) {
        const auto& state = *input.rod_state;
        if (!state.vertex_frame.count(v)) continue;
        const auto& ring = state.frames[state.vertex_frame.at(v)].ring;
        const std::set<size_t> selected(ring.begin(), ring.end());
        for (size_t w : input.input_to_offset_vertices[v])
            if (!selected.count(w) && !input.mesh->get_one_ring_tids_for_vertex(w).empty())
                free_cap_vertices.insert(w);
    }
    const auto regular = [&](size_t v) {
        const auto& s = *input.rod_state;
        return required_frames.count(v) && !s.protected_input.count(v);
    };
    const auto fiber_tets = [&](const PrismaticMeshInput& data) {
        std::vector<Tet> result;
        for (size_t tid = 0; tid < data.offset_tet_tags.size(); ++tid) {
            if (data.offset_tet_tags[tid] != 1) continue;
            const auto t = tet_at(data, tid);
            const auto in = input_corners(data, t);
            if (in.size() != 1 || !regular(in[0])) continue;
            if (std::any_of(t.begin(), t.end(), [&](size_t v) {
                    return free_cap_vertices.count(v);
                }))
                continue;
            if (std::all_of(t.begin(), t.end(), [&](size_t v) {
                    return v == in[0] || (data.vertex_tags[v] == 2 &&
                                          data.corr_input_vertex[v] == static_cast<int64_t>(in[0]));
                }))
                result.push_back(t);
        }
        return result;
    };
    std::vector<size_t> degrees =
        input.rod_state->report["target_degrees"].get<std::vector<size_t>>();
    const size_t before = fiber_tets(input).size();
    std::set<Tet> existing_offset_only;
    for (size_t tid = 0; tid < input.offset_tet_tags.size(); ++tid)
        if (input.offset_tet_tags[tid] == 1 && input_corners(input, tet_at(input, tid)).empty())
            existing_offset_only.insert(sorted(tet_at(input, tid)));
    size_t collapses = 0, split_collapses = 0, attempts = 0;
    std::map<std::string, size_t> rejected;
    const auto admissible = [&](PrismaticMeshInput& candidate, size_t old_count) {
        if (fiber_tets(candidate).size() >= old_count) {
            ++rejected["no_fiber_reduction"];
            return false;
        }
        for (size_t tid = 0; tid < candidate.offset_tet_tags.size(); ++tid)
            if (candidate.offset_tet_tags[tid] == 1 &&
                input_corners(candidate, tet_at(candidate, tid)).empty() &&
                !existing_offset_only.count(sorted(tet_at(candidate, tid)))) {
                ++rejected["offset_only_tet"];
                return false;
            }
        extract_frames(candidate, true);
        const auto& state = *candidate.rod_state;
        for (size_t e = 0; e < state.input_edges.size(); ++e) {
            if (!eligible_edges.count(e)) continue;
            const auto& f = state.frames[e];
            if (state.protected_input.count(f.edge[0]) || state.protected_input.count(f.edge[1]))
                continue;
            if (f.ring.size() < 3 || (!f.valid && f.reason != "secondary_edge_link")) {
                ++rejected["invalid_edge_section"];
                return false;
            }
        }
        for (size_t v : required_frames)
            if (!state.vertex_frame.count(v)) {
                ++rejected["missing_vertex_section"];
                return false;
            }
        return true;
    };
    // A chosen vertex ID is not a topological constraint. The edge-coface cycle,
    // correspondence and complete background link are the constraints instead.
    for (size_t operation = 0; operation < before; ++operation) {
        const auto fibers = fiber_tets(input);
        if (fibers.empty()) break;
        std::set<Edge> edges;
        for (const auto& t : fibers)
            for (size_t a : t)
                for (size_t b : t)
                    if (a < b && input.vertex_tags[a] == 2 && input.vertex_tags[b] == 2)
                        edges.insert({a, b});
        bool changed = false;
        for (const auto& e : edges) {
            const auto& state = *input.rod_state;
            if (state.frozen_offset.count(e[0]) || state.frozen_offset.count(e[1]) ||
                input.vertex_component_ids[e[0]] < 0 ||
                input.vertex_component_ids[e[0]] != input.vertex_component_ids[e[1]])
                continue;
            if (!input.mesh->link_condition(input.mesh->tuple_from_edge(e))) {
                ++rejected["collapse_link"];
                continue;
            }
            for (int side = 0; side < 2; ++side) {
                if (!junction_collapse_allowed(input, e[side])) continue;
                for (double alpha : {0., .5, .25, .75, .125, .875}) {
                    if (alpha != 0 && !junction_collapse_allowed(input, e[1 - side])) continue;
                    auto candidate = copy_input(input);
                    candidate.rod_state = std::make_shared<RodMeshState>(*input.rod_state);
                    rebuild(candidate);
                    const size_t removed = e[side], survivor = e[1 - side];
                    candidate.rod_state->retained_ring_vertices.erase(removed);
                    if (alpha != 0) {
                        candidate.vertices.row(survivor) =
                            ((1 - alpha) * position(input, survivor) +
                             alpha * position(input, removed))
                                .transpose();
                        bool positive = true;
                        for (size_t tid : candidate.mesh->get_one_ring_tids_for_vertex(survivor))
                            positive &= tet_volume_above_threshold(
                                candidate.vertices,
                                tet_at(candidate, tid),
                                0);
                        if (!positive) continue;
                    }
                    ++attempts;
                    if (!try_collapse_offset_edge(
                            candidate,
                            removed,
                            survivor,
                            candidate.rod_state->options.min_tet_volume)) {
                        auto tuple = input.mesh->tuple_from_edge(e);
                        ++rejected
                            [input.mesh->link_condition(tuple) ? "collapse_geometry_or_component"
                                                               : "collapse_link"];
                        continue;
                    }
                    bool floors = true;
                    std::set<size_t> affected;
                    for (size_t v : e)
                        for (size_t tid : input.mesh->get_one_ring_tids_for_vertex(v))
                            affected.insert(tid);
                    for (size_t tid : affected) {
                        auto t = tet_at(input, tid);
                        if (std::find(t.begin(), t.end(), removed) != t.end() &&
                            std::find(t.begin(), t.end(), survivor) != t.end())
                            continue;
                        Eigen::Matrix<long double, 3, 3> matrix;
                        for (int j = 0; j < 3; ++j)
                            for (int k = 0; k < 3; ++k)
                                matrix(j, k) =
                                    static_cast<long double>(input.vertices(t[j + 1], k)) -
                                    input.vertices(t[0], k);
                        const double floor = std::max(
                            input.offset_tet_tags[tid] == 1
                                ? candidate.rod_state->options.min_tet_volume
                                : 0.,
                            static_cast<double>(
                                input.relative_tet_volume_floor * matrix.determinant() / 6));
                        for (auto& v : t)
                            if (v == removed) v = survivor;
                        floors &= tet_volume_above_threshold(candidate.vertices, t, floor);
                    }
                    if (!floors) continue;
                    if (!admissible(candidate, fibers.size())) continue;
                    input = std::move(candidate);
                    ++collapses;
                    changed = true;
                    break;
                }
                if (changed) break;
            }
            if (changed) break;
        }
        if (changed) continue;
        // Shell an ear of a regular fiber: split its radial edge, then collapse
        // the temporary point toward an adjacent offset vertex. This changes the
        // edge link without identifying two preselected frame vertices at once.
        for (const auto& t : fibers) {
            const size_t center = input_corners(input, t)[0];
            for (size_t a : t) {
                if (a == center) continue;
                for (size_t b : t) {
                    if (b == center || b == a) continue;
                    auto candidate = copy_input(input);
                    candidate.rod_state = std::make_shared<RodMeshState>(*input.rod_state);
                    rebuild(candidate);
                    const size_t x = candidate.vertices.rows();
                    ++attempts;
                    if (!split_construction_edge(
                            candidate,
                            {center, a},
                            2,
                            center,
                            candidate.rod_state->options.min_tet_volume))
                        continue;
                    const int64_t cid = candidate.vertex_component_ids[b];
                    if (cid < 0) continue;
                    candidate.vertex_component_ids[x] = cid;
                    candidate.offset_components[cid].vertices.push_back(x);
                    if (!try_collapse_offset_edge(
                            candidate,
                            x,
                            b,
                            candidate.rod_state->options.min_tet_volume)) {
                        ++rejected["split_collapse_preflight"];
                        continue;
                    }
                    if (!admissible(candidate, fibers.size())) continue;
                    input = std::move(candidate);
                    ++split_collapses;
                    changed = true;
                    break;
                }
                if (changed) break;
            }
            if (changed) break;
        }
        if (!changed) break;
    }
    auto& s = *input.rod_state;
    if (collapses || split_collapses) {
        const auto previous_splits = s.report.value("ring_edge_splits", size_t(0));
        equalize_degrees(input, degrees, true, true);
        s.report["ring_edge_splits"] = previous_splits + s.report["ring_edge_splits"].get<size_t>();
        s.retained_ring_vertices.clear();
        s.frame_targets.clear();
        for (const auto& f : s.frames) {
            if (!f.valid) continue;
            for (size_t k = 0; k < f.ring.size(); ++k) {
                const double angle = 2 * pi * k / f.ring.size();
                s.retained_ring_vertices.insert(f.ring[k]);
                s.frame_targets[f.ring[k]] =
                    position(input, f.center) +
                    s.radius * (f.rotation * Vector3d(std::cos(angle), std::sin(angle), 0));
            }
        }
        relocate_frames(input);
        label_offset_faces(input);
        evaluate_target_positions(input, s.options.thicknessratio);
    }
    s.report["frame_topology_construction"] = {
        {"fiber_tets_before", before},
        {"fiber_tets_after", fiber_tets(input).size()},
        {"collapses", collapses},
        {"split_collapses", split_collapses},
        {"attempts", attempts},
        {"rejections", rejected}};
    align_rod_annuli(input);
    flatten_rod_frames(input);
    verify_junction_one_rings(input);
    discover_columns(input);
    record_rod_frames(input);
    s.report["tau_after_frame_construction"] = analyze_rod_tau(input);
    s.report["frame_topology_rounds"].push_back(s.report["frame_topology_construction"]);
    logger().info(
        "Rod frame topology: {} -> {} regular fiber tets, {} collapses, {} split/collapses",
        before,
        fiber_tets(input).size(),
        collapses,
        split_collapses);
}

namespace {
std::vector<std::vector<size_t>> hybrid_faces(const HybridCell& c)
{
    const auto& v = c.vertices;
    if (c.type == HybridCellType::Tetrahedron)
        return {{v[1], v[2], v[3]}, {v[0], v[3], v[2]}, {v[0], v[1], v[3]}, {v[0], v[2], v[1]}};
    if (c.type == HybridCellType::Pyramid)
        return {
            {v[0], v[3], v[2], v[1]},
            {v[0], v[1], v[4]},
            {v[1], v[2], v[4]},
            {v[2], v[3], v[4]},
            {v[3], v[0], v[4]}};
    return {
        {v[0], v[2], v[1]},
        {v[3], v[4], v[5]},
        {v[0], v[1], v[4], v[3]},
        {v[1], v[2], v[5], v[4]},
        {v[2], v[0], v[3], v[5]}};
}

// A single global vertex order gives both users of every quad the same diagonal.
// A rejected positive-J prism keeps its original representation if this particular
// reference triangulation would introduce a nonpositive tet.
std::vector<Tet> pulling_split(const HybridCell& cell)
{
    if (cell.type == HybridCellType::Tetrahedron)
        return {{cell.vertices[0], cell.vertices[1], cell.vertices[2], cell.vertices[3]}};
    const size_t apex = *std::min_element(cell.vertices.begin(), cell.vertices.end());
    std::vector<Tet> result;
    for (auto face : hybrid_faces(cell)) {
        if (std::find(face.begin(), face.end(), apex) != face.end()) continue;
        std::rotate(face.begin(), std::min_element(face.begin(), face.end()), face.end());
        for (size_t k = 1; k + 1 < face.size(); ++k)
            result.push_back({apex, face[0], face[k], face[k + 1]});
    }
    return result;
}
std::map<Tri, size_t> boundary_faces(const PrismaticMeshInput& input)
{
    std::map<Tri, size_t> counts;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        const auto t = tet_at(input, i);
        for (int omitted = 0; omitted < 4; ++omitted) {
            Tri face;
            int k = 0;
            for (int j = 0; j < 4; ++j)
                if (j != omitted) face[k++] = t[j];
            ++counts[sorted(face)];
        }
    }
    for (auto it = counts.begin(); it != counts.end();)
        if (it->second != 1)
            it = counts.erase(it);
        else
            ++it;
    return counts;
}
double triangle_area(const PrismaticMeshInput& input, const Tri& t)
{
    return (position(input, t[1]) - position(input, t[0]))
               .cross(position(input, t[2]) - position(input, t[0]))
               .norm() /
           2;
}
double boundary_intersection_area(
    const PrismaticMeshInput& a,
    const Tri& ta,
    const PrismaticMeshInput& b,
    const Tri& tb)
{
    const Vector3d origin = position(a, ta[0]);
    const Vector3d x = position(a, ta[1]) - origin, y = position(a, ta[2]) - origin;
    const Vector3d cross = x.cross(y);
    const double length = cross.norm(), scale = std::max(x.norm(), y.norm());
    if (!(length > 0)) return 0;
    const Vector3d normal = cross / length;
    for (size_t v : tb)
        if (std::abs((position(b, v) - origin).dot(normal)) > 1e-10 * scale) return 0;
    const Vector3d u = x.normalized(), v = normal.cross(u);
    const auto projected = [&](const Vector3d& p) {
        const Vector3d q = p - origin;
        return Eigen::Vector2d(q.dot(u), q.dot(v));
    };
    std::vector<Eigen::Vector2d> polygon;
    for (size_t k : tb) polygon.push_back(projected(position(b, k)));
    const auto cross2 = [](const Eigen::Vector2d& p, const Eigen::Vector2d& q) {
        return p.x() * q.y() - p.y() * q.x();
    };
    for (size_t k = 0; k < 3 && !polygon.empty(); ++k) {
        const auto p = projected(position(a, ta[k])), q = projected(position(a, ta[(k + 1) % 3]));
        const Eigen::Vector2d edge = q - p;
        std::vector<Eigen::Vector2d> clipped;
        for (size_t j = 0; j < polygon.size(); ++j) {
            const auto c = polygon[j], d = polygon[(j + 1) % polygon.size()];
            const double dc = cross2(edge, c - p), dd = cross2(edge, d - p);
            if (dc >= 0) clipped.push_back(c);
            if ((dc >= 0) != (dd >= 0)) clipped.push_back(c + dc / (dc - dd) * (d - c));
        }
        polygon = std::move(clipped);
    }
    double area = 0;
    for (size_t k = 0; k < polygon.size(); ++k)
        area += cross2(polygon[k], polygon[(k + 1) % polygon.size()]);
    return std::abs(area) / 2;
}
bool same_boundary(const PrismaticMeshInput& a, const PrismaticMeshInput& b)
{
    auto first = boundary_faces(a), second = boundary_faces(b);
    for (auto it = first.begin(); it != first.end();) {
        if (second.erase(it->first))
            it = first.erase(it);
        else
            ++it;
    }
    if (first.empty() && second.empty()) return true;
    const auto covered =
        [&](const auto& source, const auto& faces, const auto& target, const auto& others) {
            for (const auto& [f, n] : faces) {
                const double area = triangle_area(source, f);
                double coverage = 0;
                for (const auto& [g, m] : others)
                    coverage += boundary_intersection_area(source, f, target, g);
                if (std::abs(area - coverage) > 1e-8 * area) return false;
            }
            return true;
        };
    return covered(a, first, b, second) && covered(b, second, a, first);
}
long double volume_sum(const PrismaticMeshInput& input)
{
    long double sum = 0;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        const auto t = tet_at(input, i);
        Eigen::Matrix<long double, 3, 3> e;
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                e(j, k) =
                    static_cast<long double>(input.vertices(t[j + 1], k)) - input.vertices(t[0], k);
        sum += e.determinant() / 6;
    }
    return sum;
}
bool pyramid_positive(const PrismaticMeshInput& input, const HybridCell& c, double floor)
{
    const auto& v = c.vertices;
    for (const auto& t : std::vector<Tet>{
             {v[0], v[1], v[2], v[4]},
             {v[0], v[2], v[3], v[4]},
             {v[0], v[1], v[3], v[4]},
             {v[1], v[2], v[3], v[4]}})
        if (!tet_volume_above_threshold(input.vertices, t, floor)) return false;
    return true;
}
struct IntervalPlan
{
    Edge axis;
    std::vector<size_t> prisms;
    std::map<size_t, size_t> columns;
    std::vector<size_t> ring;
    bool enabled = true;
};
using Midpoints = std::map<Edge, size_t>;

// Number of cut edges of a quad; two opposite cuts are a complete new frame face.
std::vector<size_t> quad_cuts(const std::vector<size_t>& q, const Midpoints& mids)
{
    std::vector<size_t> r;
    for (size_t k = 0; k < 4; ++k)
        if (mids.count(sorted(Edge{q[k], q[(k + 1) % 4]}))) r.push_back(k);
    return r;
}
std::array<std::vector<size_t>, 2> split_quad(std::vector<size_t> q, const Midpoints& mids)
{
    const auto cuts = quad_cuts(q, mids);
    require(cuts.size() == 2 && (cuts[0] + 2) % 4 == cuts[1], "incomplete quad frame cut");
    std::rotate(q.begin(), q.begin() + cuts[0], q.end());
    const size_t a = mids.at(sorted(Edge{q[0], q[1]})), b = mids.at(sorted(Edge{q[2], q[3]}));
    return {{{q[0], a, b, q[3]}, {a, q[1], q[2], b}}};
}

bool construct_split_mesh(
    const PrismaticMeshInput& source,
    const PrismDominantMesh& original,
    const std::vector<IntervalPlan>& plans,
    PrismaticMeshInput& target,
    PrismDominantMesh& hybrid,
    std::set<size_t>& reject)
{
    target = copy_input(source);
    target.rod_state = std::make_shared<RodMeshState>(*source.rod_state);
    target.rod_state->report["midpoint_vertex_frames"] = nlohmann::json::array();
    Midpoints mids;
    std::map<Edge, size_t> owner;
    for (size_t i = 0; i < plans.size(); ++i) {
        const auto& plan = plans[i];
        if (!plan.enabled) continue;
        require(
            target.source_vertex_ids.size() == static_cast<size_t>(target.vertices.rows()) &&
                target.vertex_tags.size() == target.source_vertex_ids.size(),
            "incomplete vertex state before frame insertion");
        const size_t w = append_vertex(
            target,
            (position(source, plan.axis[0]) + position(source, plan.axis[1])) / 2,
            1);
        mids[plan.axis] = w;
        owner[plan.axis] = i;
        const Vector3d center = position(target, w);
        const auto& state = *source.rod_state;
        const auto edge = std::find(state.input_edges.begin(), state.input_edges.end(), plan.axis);
        require(edge != state.input_edges.end(), "split interval has no parent edge frame");
        // Both child edges inherit this pose, so their midpoint mean is exactly it.
        const auto rotation = state.edge_frames.at(edge - state.input_edges.begin()).rotation;
        target.rod_state->vertex_rotations[w] = rotation;
        target.rod_state->report["midpoint_vertex_frames"].push_back(
            {{"center_vid", target.source_vertex_ids[w]},
             {"parent_edge_vids",
              {source.source_vertex_ids[plan.axis[0]], source.source_vertex_ids[plan.axis[1]]}},
             {"quaternion_wxyz", {rotation.w(), rotation.x(), rotation.y(), rotation.z()}},
             {"rule", "mean_of_two_children_inheriting_parent_edge_pose"}});
        const Vector3d tangent = rotation * Vector3d::UnitZ();
        auto ring = plan.ring;
        const auto interpolant = [&](size_t a) {
            return ((position(source, a) + position(source, plan.columns.at(a))) / 2 - center)
                .eval();
        };
        Vector3d area = Vector3d::Zero();
        for (size_t k = 0; k < ring.size(); ++k)
            area += interpolant(ring[k]).cross(interpolant(ring[(k + 1) % ring.size()]));
        if (area.dot(tangent) < 0) std::reverse(ring.begin() + 1, ring.end());
        double best = std::numeric_limits<double>::infinity();
        size_t shift = 0;
        for (size_t j = 0; j < ring.size(); ++j) {
            double cost = 0;
            for (size_t k = 0; k < ring.size(); ++k) {
                const double angle = 2 * pi * k / ring.size();
                cost += (interpolant(ring[(k + j) % ring.size()]) -
                         state.radius * (rotation * Vector3d(std::cos(angle), std::sin(angle), 0)))
                            .squaredNorm();
            }
            if (cost < best) {
                best = cost;
                shift = j;
            }
        }
        std::rotate(ring.begin(), ring.begin() + shift, ring.end());
        for (size_t k = 0; k < ring.size(); ++k) {
            const size_t a = ring[k], b = plan.columns.at(a);
            const Edge e = sorted(Edge{a, b});
            if (mids.count(e)) {
                reject.insert(i);
                reject.insert(owner.at(e));
                continue;
            }
            const double angle = 2 * pi * k / ring.size();
            mids[e] = append_vertex(
                target,
                center + state.radius * (rotation * Vector3d(std::cos(angle), std::sin(angle), 0)),
                2,
                w);
            owner[e] = i;
        }
    }
    const auto reject_cell = [&](const HybridCell& c) {
        for (size_t a : c.vertices)
            for (size_t b : c.vertices) {
                const auto found = owner.find(sorted(Edge{a, b}));
                if (found != owner.end()) reject.insert(found->second);
            }
    };
    const auto old_split = [&](const HybridCell& cell) {
        std::vector<Tet> result;
        for (size_t tid : cell.source_tets) result.push_back(tet_at(source, tid));
        return result;
    };
    const auto add_cell = [&](HybridCell cell,
                              size_t original_cell,
                              const std::vector<Tet>& explicit_split = {},
                              bool unchanged = false) {
        const auto& old = original.cells[original_cell];
        const bool band = source.offset_tet_tags[old.source_tets[0]] == 1;
        const double floor = band && !unchanged ? source.rod_state->options.min_tet_volume : 0;
        if ((cell.type == HybridCellType::Prism &&
             !(minimum_prism_jacobian(target.vertices, cell).determinant > 0)) ||
            (cell.type == HybridCellType::Pyramid && !pyramid_positive(target, cell, floor))) {
            reject_cell(old);
            return false;
        }
        const auto split = explicit_split.empty() ? pulling_split(cell) : explicit_split;
        for (const auto& t : split)
            if (!tet_volume_above_threshold(target.vertices, t, floor)) {
                reject_cell(old);
                return false;
            }
        cell.source_tets.clear();
        for (const auto& t : split) {
            const size_t tid = target.input_cells.size();
            target.tetrahedra.conservativeResize(tid + 1, 4);
            for (int j = 0; j < 4; ++j) target.tetrahedra(tid, j) = t[j];
            target.input_cells.push_back(source.input_cells[old.source_tets[0]]);
            target.offset_tet_tags.push_back(source.offset_tet_tags[old.source_tets[0]]);
            cell.source_tets.push_back(tid);
            hybrid.source_tet_to_cell.push_back(hybrid.cells.size());
        }
        hybrid.cells.push_back(std::move(cell));
        return true;
    };
    target.tetrahedra.resize(0, 4);
    target.input_cells.clear();
    target.offset_tet_tags.clear();
    for (size_t ci = 0; ci < original.cells.size(); ++ci) {
        const auto& c = original.cells[ci];
        const auto& v = c.vertices;
        std::set<Edge> cut;
        for (size_t a : v)
            for (size_t b : v) {
                const Edge e = sorted(Edge{a, b});
                if (mids.count(e)) cut.insert(e);
            }
        if (c.type == HybridCellType::Prism) {
            if (cut.empty()) {
                if (!add_cell(c, ci, old_split(c), true)) return false;
                continue;
            }
            const std::set<Edge> columns{
                sorted(Edge{v[0], v[3]}),
                sorted(Edge{v[1], v[4]}),
                sorted(Edge{v[2], v[5]})};
            if (cut != columns) {
                reject_cell(c);
                continue;
            }
            const std::vector<size_t> m{
                mids.at(sorted(Edge{v[0], v[3]})),
                mids.at(sorted(Edge{v[1], v[4]})),
                mids.at(sorted(Edge{v[2], v[5]}))};
            HybridCell a = c, b = c;
            a.vertices = {v[0], v[1], v[2], m[0], m[1], m[2]};
            b.vertices = {m[0], m[1], m[2], v[3], v[4], v[5]};
            if (!add_cell(a, ci) || !add_cell(b, ci)) return false;
        } else if (c.type == HybridCellType::Pyramid) {
            if (cut.empty()) {
                if (!add_cell(c, ci, old_split(c), true)) return false;
                continue;
            }
            const std::vector<size_t> base{v[0], v[1], v[2], v[3]};
            const auto cuts = quad_cuts(base, mids);
            if (cut.size() != 2 || cuts.size() != 2 || (cuts[0] + 2) % 4 != cuts[1]) {
                reject_cell(c);
                continue;
            }
            for (auto q : split_quad(base, mids)) {
                HybridCell child = c;
                child.vertices = q;
                child.vertices.push_back(v[4]);
                if (!add_cell(child, ci)) return false;
            }
        } else {
            std::vector<Tet> split{{v[0], v[1], v[2], v[3]}};
            for (const auto& e : cut) {
                const size_t n = split.size();
                for (size_t k = 0; k < n; ++k) {
                    auto& t = split[k];
                    const auto a = std::find(t.begin(), t.end(), e[0]),
                               b = std::find(t.begin(), t.end(), e[1]);
                    if (a == t.end() || b == t.end()) continue;
                    Tet next = t;
                    *std::find(next.begin(), next.end(), e[1]) = mids.at(e);
                    *a = mids.at(e);
                    split.push_back(next);
                }
            }
            for (const auto& t : split) {
                HybridCell child = c;
                child.vertices.assign(t.begin(), t.end());
                if (!add_cell(child, ci, {t}, cut.empty())) return false;
            }
        }
    }
    if (!reject.empty()) return false;
    return true;
}
} // namespace

void split_rod_intervals(PrismaticMeshInput& input, PrismDominantMesh& hybrid)
{
    require(input.rod_state != nullptr, "missing rod state");
    auto& state = *input.rod_state;
    std::map<Edge, size_t> groups;
    std::vector<IntervalPlan> plans;
    for (size_t i = 0; i < hybrid.cells.size(); ++i) {
        const auto& c = hybrid.cells[i];
        if (c.type != HybridCellType::Prism || input.vertex_tags[c.vertices[0]] != 1 ||
            input.vertex_tags[c.vertices[3]] != 1)
            continue;
        const Edge axis = sorted(Edge{c.vertices[0], c.vertices[3]});
        if (!groups.count(axis)) {
            groups[axis] = plans.size();
            plans.push_back({axis, {}, {}, {}, true});
        }
        auto& plan = plans[groups.at(axis)];
        plan.prisms.push_back(i);
        for (int k = 1; k < 3; ++k) {
            size_t a = c.vertices[k], b = c.vertices[k + 3];
            if (input.corr_input_vertex[a] != static_cast<int64_t>(axis[0])) std::swap(a, b);
            if (input.corr_input_vertex[a] != static_cast<int64_t>(axis[0]) ||
                input.corr_input_vertex[b] != static_cast<int64_t>(axis[1]) ||
                (plan.columns.count(a) && plan.columns[a] != b))
                plan.enabled = false;
            plan.columns[a] = b;
        }
    }
    for (auto& plan : plans) {
        std::vector<Edge> lower, upper;
        for (size_t ci : plan.prisms) {
            const auto& v = hybrid.cells[ci].vertices;
            if (v[0] == plan.axis[0]) {
                lower.push_back({v[1], v[2]});
                upper.push_back({v[4], v[5]});
            } else {
                lower.push_back({v[4], v[5]});
                upper.push_back({v[1], v[2]});
            }
        }
        plan.ring = ordered_ring(lower);
        if (plan.ring.empty() || ordered_ring(upper).empty() ||
            plan.columns.size() != plan.prisms.size())
            plan.enabled = false;
    }
    size_t rejected =
        std::count_if(plans.begin(), plans.end(), [](const auto& p) { return !p.enabled; });
    const long double old_volume = volume_sum(input);
    while (std::any_of(plans.begin(), plans.end(), [](const auto& p) { return p.enabled; })) {
        PrismaticMeshInput candidate;
        PrismDominantMesh result;
        std::set<size_t> reject;
        bool built = false;
        try {
            built = construct_split_mesh(input, hybrid, plans, candidate, result, reject);
        } catch (const std::exception& error) {
            throw std::runtime_error(std::string("rod interval construction: ") + error.what());
        }
        if (!reject.empty()) {
            for (size_t i : reject)
                if (plans[i].enabled) {
                    plans[i].enabled = false;
                    ++rejected;
                }
            continue;
        }
        if (!built) {
            state.report["axial_split_rejection"] = "no_positive_common_reference_split";
            break;
        }
        try {
            require(
                same_boundary(input, candidate),
                "axial split changes domain boundary geometry");
            require(
                std::abs(volume_sum(candidate) - old_volume) <= 1e-10L * std::abs(old_volume),
                "axial split changes domain volume");
            validate_prism_dominant_mesh(candidate, result, state.options.min_tet_volume);
            const auto tau = analyze_rod_tau(candidate);
            require(
                tau.at("invalid_correspondence") == 0 && tau.at("edge_parent_mismatch") == 0,
                "axial split produced inconsistent correspondence");
            rebuild(candidate);
            require(
                candidate.mesh->check_mesh_connectivity_validity(),
                "invalid axial split connectivity");
        } catch (const std::exception& error) {
            state.report["axial_split_rejection"] = error.what();
            break;
        }
        result.report = hybrid.report;
        result.report["retriangulated_regions"] = nlohmann::json::array();
        result.report["transition_and_tet_regions"] = nlohmann::json::array();
        result.report["source_tetrahedra"] = candidate.tetrahedra.rows();
        result.report["output_cells"] = result.cells.size();
        for (const char* key :
             {"prisms",
              "pyramids",
              "shell_pyramids",
              "background_pyramids",
              "band_tetrahedra",
              "input_tetrahedra",
              "background_tetrahedra"})
            result.report[key] = 0;
        double min_j = std::numeric_limits<double>::infinity();
        double min_sampled_j = std::numeric_limits<double>::infinity();
        const auto sample_points = prism_jacobian_sample_points();
        for (const auto& c : result.cells) {
            const bool band = candidate.offset_tet_tags[c.source_tets[0]] == 1;
            if (c.type == HybridCellType::Prism) {
                result.report["prisms"] = result.report["prisms"].get<size_t>() + 1;
                min_j = std::min(min_j, minimum_prism_jacobian(candidate.vertices, c).determinant);
                for (const auto& p : sample_points)
                    min_sampled_j = std::min(
                        min_sampled_j,
                        prism_jacobian(candidate.vertices, c, p).determinant);
            } else if (c.type == HybridCellType::Pyramid) {
                result.report["pyramids"] = result.report["pyramids"].get<size_t>() + 1;
                const char* key = band ? "shell_pyramids" : "background_pyramids";
                result.report[key] = result.report[key].get<size_t>() + 1;
            } else {
                const char* key = band ? "band_tetrahedra"
                                  : candidate.input_cells[c.source_tets[0]] == 1
                                      ? "input_tetrahedra"
                                      : "background_tetrahedra";
                result.report[key] = result.report[key].get<size_t>() + 1;
            }
        }
        result.report["minimum_output_prism_global_det_j"] =
            std::isfinite(min_j) ? nlohmann::json(min_j) : nlohmann::json(nullptr);
        result.report["prism_jacobian_check"]["minimum_output_prism_det_j"] =
            std::isfinite(min_sampled_j) ? nlohmann::json(min_sampled_j) : nlohmann::json(nullptr);
        result.report["axial_reference_rebuilt"] = true;
        auto& accepted_state = *candidate.rod_state;
        accepted_state.report["axially_split_intervals"] =
            std::count_if(plans.begin(), plans.end(), [](const auto& p) { return p.enabled; });
        accepted_state.report["axial_split_rejected_intervals"] = rejected;
        accepted_state.report["new_frame_vertices"] =
            candidate.vertices.rows() - input.vertices.rows();
        accepted_state.report["tau_after_axial_split"] = analyze_rod_tau(candidate);
        input = std::move(candidate);
        label_offset_faces(input);
        hybrid = std::move(result);
        return;
    }
    state.report["axially_split_intervals"] = 0;
    state.report["axial_split_rejected_intervals"] = plans.size();
    state.report["new_frame_vertices"] = 0;
}

void finalize_rod_terminal_frames(PrismaticMeshInput& input, PrismDominantMesh& hybrid)
{
    require(input.rod_state != nullptr, "missing rod state");
    auto& state = *input.rod_state;
    state.terminal_frame_vertices.clear();
    state.terminal_prism_cells.clear();
    state.report["terminal_frames"] = nlohmann::json::array();
    if (state.free_endpoints.empty()) return;
    const auto parent = [&](const auto& vertices) -> int64_t {
        int64_t common = -1;
        for (size_t v : vertices) {
            const int64_t p = input.vertex_tags[v] == 1   ? static_cast<int64_t>(v)
                              : input.vertex_tags[v] == 2 ? input.corr_input_vertex[v]
                                                          : -1;
            if (p < 0 || !state.free_endpoints.count(p) || (common >= 0 && common != p)) return -1;
            common = p;
        }
        return common;
    };
    // Keep only faces in a free endpoint's fiber. The complete background still
    // contributes their other owners, without storing unrelated far faces.
    std::map<Tri, std::vector<size_t>> owners;
    for (size_t ci = 0; ci < hybrid.cells.size(); ++ci)
        for (const auto& f : hybrid_faces(hybrid.cells[ci]))
            if (f.size() == 3 && parent(f) >= 0)
                owners[sorted(Tri{f[0], f[1], f[2]})].push_back(ci);
    std::set<size_t> returned_cells;
    size_t completed = 0;
    for (size_t v : state.free_endpoints) {
        nlohmann::json report = {
            {"center_vid", input.source_vertex_ids[v]},
            {"definition", "material_frame_disk_through_input_endpoint"}};
        const auto fail = [&](const char* reason) {
            report["status"] = reason;
            state.report["terminal_frames"].push_back(report);
        };
        if (!state.vertex_frame.count(v)) {
            fail("missing_terminal_frame");
            continue;
        }
        const auto& frame = state.frames.at(state.vertex_frame.at(v));
        const Vector3d center = position(input, v);
        const double tolerance = 1e-8 * state.radius;
        bool valid = frame.valid && frame.ring.size() >= 3;
        for (size_t w : frame.ring)
            valid &= std::abs((position(input, w) - center).dot(frame.tangent)) <= tolerance;
        if (!valid) {
            fail("terminal_frame_not_planar");
            continue;
        }
        const auto is_band = [&](size_t ci) {
            return input.offset_tet_tags[hybrid.cells[ci].source_tets.front()] == 1;
        };
        const auto is_cap = [&](size_t ci) {
            const auto& cell = hybrid.cells[ci];
            if (!is_band(ci) || cell.type != HybridCellType::Tetrahedron ||
                parent(cell.vertices) != static_cast<int64_t>(v))
                return false;
            bool outside = false;
            const auto& edge = state.directed_input_edges[frame.edge_index];
            const Vector3d outward = edge[0] == v ? -frame.tangent : frame.tangent;
            for (size_t w : cell.vertices) {
                const double d = (position(input, w) - center).dot(outward);
                if (d < -tolerance) return false;
                outside |= d > tolerance;
            }
            return outside;
        };
        std::set<Tri> disk;
        std::set<size_t> prisms, cap;
        std::vector<size_t> pending;
        for (size_t k = 0; k < frame.ring.size() && valid; ++k) {
            const Tri face = sorted(Tri{v, frame.ring[k], frame.ring[(k + 1) % frame.ring.size()]});
            disk.insert(face);
            const auto found = owners.find(face);
            if (found == owners.end() || found->second.empty() || found->second.size() > 2) {
                valid = false;
                break;
            }
            size_t prism_count = 0;
            for (size_t ci : found->second) {
                const auto& cell = hybrid.cells[ci];
                if (is_band(ci) && cell.type == HybridCellType::Prism &&
                    minimum_prism_jacobian(input.vertices, cell).determinant > 0) {
                    prisms.insert(ci);
                    ++prism_count;
                } else if (is_cap(ci))
                    pending.push_back(ci);
                else if (is_band(ci))
                    valid = false;
            }
            valid &= prism_count == 1;
        }
        if (!valid) {
            fail("unmatched_terminal_prism_sector");
            continue;
        }
        while (!pending.empty() && valid) {
            const size_t ci = pending.back();
            pending.pop_back();
            if (!cap.insert(ci).second) continue;
            for (const auto& f : hybrid_faces(hybrid.cells[ci])) {
                const Tri key = sorted(Tri{f[0], f[1], f[2]});
                if (disk.count(key)) continue;
                for (size_t neighbor : owners.at(key)) {
                    if (neighbor == ci) continue;
                    if (is_cap(neighbor))
                        pending.push_back(neighbor);
                    else if (is_band(neighbor))
                        valid = false;
                }
            }
        }
        if (!valid) {
            fail("offset_cap_has_another_band_interface");
            continue;
        }
        // The complete material frame defines the rod boundary. Its disk is
        // covered by real prism caps; exterior cone tets remain background tets.
        returned_cells.insert(cap.begin(), cap.end());
        state.terminal_prism_cells.insert(prisms.begin(), prisms.end());
        state.terminal_frame_vertices.insert(v);
        state.terminal_frame_vertices.insert(frame.ring.begin(), frame.ring.end());
        report["status"] = "prismatic_flat_end";
        report["degree"] = frame.ring.size();
        report["prism_cells"] = prisms;
        report["offset_cap_cells_returned_to_background"] = cap;
        report["ring_vids"] = nlohmann::json::array();
        for (size_t w : frame.ring) report["ring_vids"].push_back(input.source_vertex_ids[w]);
        state.report["terminal_frames"].push_back(std::move(report));
        ++completed;
    }
    const auto old_tags = input.offset_tet_tags;
    for (size_t ci : returned_cells)
        for (size_t tid : hybrid.cells[ci].source_tets) input.offset_tet_tags[tid] = -1;
    try {
        validate_prism_dominant_mesh(input, hybrid, state.options.min_tet_volume);
    } catch (...) {
        input.offset_tet_tags = old_tags;
        state.terminal_frame_vertices.clear();
        state.terminal_prism_cells.clear();
        throw;
    }
    size_t band_tets = 0, background_tets = 0;
    for (const auto& cell : hybrid.cells)
        if (cell.type == HybridCellType::Tetrahedron) {
            const size_t tid = cell.source_tets.front();
            band_tets += input.offset_tet_tags[tid] == 1;
            background_tets += input.offset_tet_tags[tid] != 1 && input.input_cells[tid] != 1;
        }
    hybrid.report["band_tetrahedra"] = band_tets;
    hybrid.report["background_tetrahedra"] = background_tets;
    state.report["completed_prismatic_free_ends"] = completed;
    state.report["offset_cap_tets_returned_to_background"] = returned_cells.size();
    hybrid.report["rod_terminal_frames"] = state.report["terminal_frames"];
    hybrid.report["checks"]["free_end_frame_disks_checked"] = true;
    logger().info(
        "Rod terminal frames: {}/{} prismatic ends; {} offset cap tets returned to background",
        completed,
        state.free_endpoints.size(),
        returned_cells.size());
}

} // namespace wmtk::components::prismatic_mesh
