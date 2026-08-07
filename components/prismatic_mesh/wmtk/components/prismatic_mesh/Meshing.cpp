#include "OffsetOperations.hpp"
#include "Recovery.hpp"
#include "Types.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>
#include <optional>
#include <queue>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

namespace wmtk::components::prismatic_mesh {
namespace {

using Edge = std::array<size_t, 2>;
using Face = std::array<size_t, 3>;
using ShellVertexKey = std::tuple<size_t, int, size_t>;
constexpr double PI = 3.141592653589793238462643383279502884;

struct EdgeHash
{
    size_t operator()(const Edge& e) const
    {
        return std::hash<size_t>()(e[0]) ^ (std::hash<size_t>()(e[1]) << 1);
    }
};

Edge sorted_edge(size_t a, size_t b)
{
    if (a > b) std::swap(a, b);
    return {{a, b}};
}

Face sorted_face(Face face)
{
    std::sort(face.begin(), face.end());
    return face;
}

struct IncidentTet
{
    size_t tet = 0;
    size_t opposite_vertex = 0;
};

std::map<Face, std::vector<IncidentTet>> tet_face_incidence(const PrismaticMeshInput& input)
{
    std::map<Face, std::vector<IncidentTet>> result;
    static constexpr std::array<std::array<size_t, 3>, 4> local_faces = {
        {{{1, 2, 3}}, {{0, 3, 2}}, {{0, 1, 3}}, {{0, 2, 1}}}};
    for (size_t t = 0; t < input.tets.size(); ++t) {
        for (size_t opposite = 0; opposite < 4; ++opposite) {
            Face face;
            for (size_t i = 0; i < 3; ++i) {
                face[i] = input.tets[t][local_faces[opposite][i]];
            }
            result[sorted_face(face)].push_back({t, input.tets[t][opposite]});
        }
    }
    return result;
}

size_t append_vertex(std::vector<Vector3d>& vertices, const Vector3d& p)
{
    vertices.push_back(p);
    return vertices.size() - 1;
}

template <size_t N>
std::array<Vector3d, N> cell_points(
    const std::vector<Vector3d>& vertices,
    const std::array<size_t, N>& cell)
{
    std::array<Vector3d, N> result;
    for (size_t i = 0; i < N; ++i) result[i] = vertices[cell[i]];
    return result;
}

bool point_in_closed_tet(const Vector3d& point, const std::array<Vector3d, 4>& tet)
{
    Matrix3d basis;
    basis.col(0) = tet[1] - tet[0];
    basis.col(1) = tet[2] - tet[0];
    basis.col(2) = tet[3] - tet[0];
    const double determinant = basis.determinant();
    if (std::abs(determinant) <= 1e-30) return false;
    const Vector3d barycentric = basis.fullPivLu().solve(point - tet[0]);
    const double tolerance = 1e-10;
    return barycentric.minCoeff() >= -tolerance && barycentric.sum() <= 1 + tolerance;
}

void orient_and_append_tet(
    HybridVolumeMesh& mesh,
    const std::vector<Vector3d>& vertices,
    std::array<size_t, 4> tet,
    const std::string& region)
{
    auto points = cell_points(vertices, tet);
    if (!is_valid_tet(points)) {
        std::swap(tet[2], tet[3]);
        points = cell_points(vertices, tet);
    }
    if (!is_valid_tet(points)) {
        log_and_throw_error("Generated a degenerate fallback tetrahedron in region '{}'.", region);
    }
    mesh.tets.push_back(tet);
    mesh.tet_region_tags.push_back(region);
}

void fallback_prism_to_tets(
    HybridVolumeMesh& mesh,
    const std::vector<Vector3d>& vertices,
    const std::array<size_t, 6>& p,
    const std::string& region,
    PrismaticMeshReport& report)
{
    orient_and_append_tet(mesh, vertices, {{p[0], p[1], p[2], p[3]}}, region);
    orient_and_append_tet(mesh, vertices, {{p[1], p[2], p[3], p[4]}}, region);
    orient_and_append_tet(mesh, vertices, {{p[2], p[3], p[4], p[5]}}, region);
    report.fallback_tets += 3;
}

bool append_prism(
    HybridVolumeMesh& mesh,
    const std::vector<Vector3d>& vertices,
    std::array<size_t, 6> prism,
    const std::string& region,
    const PrismaticMeshParameters& parameters,
    PrismaticMeshReport& report,
    const bool is_rod)
{
    auto points = cell_points(vertices, prism);
    if (!is_valid_prism(points, parameters.jacobian_tolerance, parameters.validity_max_depth)) {
        std::swap(prism[1], prism[2]);
        std::swap(prism[4], prism[5]);
        points = cell_points(vertices, prism);
    }
    if (!is_valid_prism(points, parameters.jacobian_tolerance, parameters.validity_max_depth)) {
        ++report.invalid_prism_candidates;
        fallback_prism_to_tets(mesh, vertices, prism, region, report);
        return false;
    }

    mesh.prisms.push_back(prism);
    mesh.prism_region_tags.push_back(region);
    if (is_rod)
        ++report.rod_prisms;
    else
        ++report.shell_prisms;
    return true;
}

bool append_recovered_prism(
    HybridVolumeMesh& mesh,
    const std::vector<Vector3d>& vertices,
    const std::array<size_t, 6>& prism,
    const std::array<bool, 3>& marked,
    const std::string& region,
    const PrismaticMeshParameters& parameters,
    PrismaticMeshReport& report)
{
    auto partition = recover_prism_partition(prism, marked);
    if (!partition.prisms.empty()) {
        return append_prism(mesh, vertices, prism, region, parameters, report, false);
    }

    if (!partition.pyramids.empty()) {
        auto pyramid = partition.pyramids.front();
        auto points = cell_points(vertices, pyramid);
        if (!is_valid_pyramid(
                points,
                parameters.jacobian_tolerance,
                parameters.validity_max_depth)) {
            std::swap(pyramid[1], pyramid[3]);
            points = cell_points(vertices, pyramid);
        }
        auto tet = partition.tets.front();
        auto tet_points = cell_points(vertices, tet);
        if (!is_valid_tet(tet_points)) std::swap(tet[2], tet[3]);
        tet_points = cell_points(vertices, tet);
        if (is_valid_pyramid(
                points,
                parameters.jacobian_tolerance,
                parameters.validity_max_depth) &&
            is_valid_tet(tet_points)) {
            mesh.pyramids.push_back(pyramid);
            mesh.pyramid_region_tags.push_back(region);
            mesh.tets.push_back(tet);
            mesh.tet_region_tags.push_back(region);
            ++report.shell_pyramids;
            ++report.fallback_tets;
            return true;
        }

        // Algorithm 9 marks the entire candidate after a failed pyramid/tet
        // Jacobian check, which deterministically promotes it to pure tets.
        ++report.invalid_prism_candidates;
        partition = recover_prism_partition(prism, {{true, true, true}});
    }

    for (auto tet : partition.tets) {
        orient_and_append_tet(mesh, vertices, tet, region);
        ++report.fallback_tets;
    }
    return false;
}

bool prism_has_valid_orientation(
    const std::vector<Vector3d>& vertices,
    std::array<size_t, 6> prism,
    const PrismaticMeshParameters& parameters)
{
    auto points = cell_points(vertices, prism);
    if (is_valid_prism(points, parameters.jacobian_tolerance, parameters.validity_max_depth)) {
        return true;
    }
    std::swap(prism[1], prism[2]);
    std::swap(prism[4], prism[5]);
    points = cell_points(vertices, prism);
    return is_valid_prism(points, parameters.jacobian_tolerance, parameters.validity_max_depth);
}

std::optional<double> segment_triangle_parameter(
    const Vector3d& origin,
    const Vector3d& displacement,
    const Vector3d& a,
    const Vector3d& b,
    const Vector3d& c);

double tet_amips(const std::array<Vector3d, 4>& input)
{
    auto points = input;
    if (!is_valid_tet(points)) std::swap(points[2], points[3]);
    std::array<double, 12> coordinates;
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 3; ++j) coordinates[3 * i + j] = points[i][j];
    }
    return wmtk::AMIPS_energy(coordinates);
}

double prism_max_amips(const std::vector<Vector3d>& vertices, const std::array<size_t, 6>& prism)
{
    return std::max(
        {tet_amips(cell_points(
             vertices,
             std::array<size_t, 4>{{prism[0], prism[1], prism[2], prism[3]}})),
         tet_amips(cell_points(
             vertices,
             std::array<size_t, 4>{{prism[1], prism[2], prism[3], prism[4]}})),
         tet_amips(cell_points(
             vertices,
             std::array<size_t, 4>{{prism[2], prism[3], prism[4], prism[5]}}))});
}

double pyramid_max_amips(
    const std::vector<Vector3d>& vertices,
    const std::array<size_t, 5>& pyramid)
{
    return std::max(
        tet_amips(cell_points(
            vertices,
            std::array<size_t, 4>{{pyramid[0], pyramid[1], pyramid[2], pyramid[4]}})),
        tet_amips(cell_points(
            vertices,
            std::array<size_t, 4>{{pyramid[0], pyramid[2], pyramid[3], pyramid[4]}})));
}

void improve_shell_vertices(
    HybridVolumeMesh& mesh,
    std::vector<Vector3d>& vertices,
    const std::map<size_t, size_t>& movable_sources,
    const MatrixXd& input_vertices,
    const std::vector<Face>& collision_faces,
    const PrismaticMeshParameters& parameters,
    PrismaticMeshReport& report)
{
    std::map<size_t, std::set<size_t>> neighbors;
    std::map<size_t, std::vector<size_t>> incident_prisms;
    std::map<size_t, std::vector<size_t>> incident_pyramids;
    std::map<size_t, std::vector<size_t>> incident_tets;
    for (size_t p = 0; p < mesh.prisms.size(); ++p) {
        const auto& prism = mesh.prisms[p];
        for (size_t i = 3; i < 6; ++i) {
            if (movable_sources.count(prism[i]) == 0) continue;
            incident_prisms[prism[i]].push_back(p);
            neighbors[prism[i]].insert(prism[3 + (i - 2) % 3]);
            neighbors[prism[i]].insert(prism[3 + (i - 1) % 3]);
        }
    }
    for (size_t p = 0; p < mesh.pyramids.size(); ++p) {
        for (const size_t vertex : mesh.pyramids[p]) {
            if (movable_sources.count(vertex) == 0) continue;
            incident_pyramids[vertex].push_back(p);
            for (const size_t neighbor : mesh.pyramids[p]) {
                if (neighbor != vertex) neighbors[vertex].insert(neighbor);
            }
        }
    }
    for (size_t t = 0; t < mesh.tets.size(); ++t) {
        for (const size_t vertex : mesh.tets[t]) {
            if (movable_sources.count(vertex) == 0) continue;
            incident_tets[vertex].push_back(t);
            for (const size_t neighbor : mesh.tets[t]) {
                if (neighbor != vertex) neighbors[vertex].insert(neighbor);
            }
        }
    }

    const size_t maximum_iterations = std::min<size_t>(10, parameters.max_shell_iterations);
    for (size_t iteration = 0; iteration < maximum_iterations; ++iteration) {
        size_t accepted = 0;
        for (const auto& [vertex, source] : movable_sources) {
            if (neighbors[vertex].empty() ||
                (incident_prisms[vertex].empty() && incident_pyramids[vertex].empty() &&
                 incident_tets[vertex].empty())) {
                continue;
            }
            Vector3d average = Vector3d::Zero();
            for (const size_t neighbor : neighbors[vertex]) average += vertices[neighbor];
            average /= static_cast<double>(neighbors[vertex].size());
            const Vector3d previous = vertices[vertex];
            const Vector3d proposal = 0.5 * (previous + average);
            if ((proposal - previous).norm() <= 1e-14) continue;

            double before = 0;
            for (const size_t prism : incident_prisms[vertex]) {
                before = std::max(before, prism_max_amips(vertices, mesh.prisms[prism]));
            }
            for (const size_t pyramid : incident_pyramids[vertex]) {
                before = std::max(before, pyramid_max_amips(vertices, mesh.pyramids[pyramid]));
            }
            for (const size_t tet : incident_tets[vertex]) {
                before = std::max(before, tet_amips(cell_points(vertices, mesh.tets[tet])));
            }
            ++report.shell_smoothing_attempts;
            bool collision = false;
            for (const Face& obstacle : collision_faces) {
                if (std::find(obstacle.begin(), obstacle.end(), source) != obstacle.end()) continue;
                collision = collision || segment_triangle_parameter(
                                             previous,
                                             proposal - previous,
                                             input_vertices.row(obstacle[0]),
                                             input_vertices.row(obstacle[1]),
                                             input_vertices.row(obstacle[2]))
                                             .has_value();
            }
            if (collision) {
                ++report.shell_smoothing_collision_rejects;
                continue;
            }
            vertices[vertex] = proposal;
            bool valid = true;
            double after = 0;
            for (const size_t prism : incident_prisms[vertex]) {
                valid =
                    valid && prism_has_valid_orientation(vertices, mesh.prisms[prism], parameters);
                after = std::max(after, prism_max_amips(vertices, mesh.prisms[prism]));
            }
            for (const size_t pyramid : incident_pyramids[vertex]) {
                valid = valid && is_valid_pyramid(
                                     cell_points(vertices, mesh.pyramids[pyramid]),
                                     parameters.jacobian_tolerance,
                                     parameters.validity_max_depth);
                after = std::max(after, pyramid_max_amips(vertices, mesh.pyramids[pyramid]));
            }
            for (const size_t tet : incident_tets[vertex]) {
                valid = valid && is_valid_tet(cell_points(vertices, mesh.tets[tet]));
                after = std::max(after, tet_amips(cell_points(vertices, mesh.tets[tet])));
            }
            if (!valid || !std::isfinite(after) || after + 1e-12 >= before) {
                vertices[vertex] = previous;
                continue;
            }
            ++accepted;
            ++report.shell_smoothing_accepts;
        }
        ++report.shell_smoothing_iterations;
        if (accepted == 0) break;
    }
}

bool selected_solid(const std::set<std::string>& groups, const std::set<std::string>& selected)
{
    for (const auto& group : groups) {
        if (selected.count(group) != 0) return true;
    }
    return false;
}

void validate_input_complex(const PrismaticMeshInput& input)
{
    const size_t vertex_count = static_cast<size_t>(input.vertices.rows());
    std::set<Face> tet_faces;
    std::set<Edge> tet_edges;
    for (size_t tet_index = 0; tet_index < input.tets.size(); ++tet_index) {
        const auto& tet = input.tets[tet_index];
        std::set<size_t> distinct;
        for (const size_t vertex : tet) {
            if (vertex >= vertex_count) {
                log_and_throw_error(
                    "Tetrahedron {} references out-of-range vertex {}.",
                    tet_index,
                    vertex);
            }
            distinct.insert(vertex);
        }
        if (distinct.size() != 4) {
            log_and_throw_error("Tetrahedron {} has repeated vertices.", tet_index);
        }
        const std::array<Vector3d, 4> points = {
            input.vertices.row(tet[0]),
            input.vertices.row(tet[1]),
            input.vertices.row(tet[2]),
            input.vertices.row(tet[3])};
        if (!is_valid_tet(points) && !is_valid_tet({points[0], points[1], points[3], points[2]})) {
            log_and_throw_error("Tetrahedron {} is degenerate.", tet_index);
        }
        tet_faces.insert(sorted_face({{tet[0], tet[1], tet[2]}}));
        tet_faces.insert(sorted_face({{tet[0], tet[1], tet[3]}}));
        tet_faces.insert(sorted_face({{tet[0], tet[2], tet[3]}}));
        tet_faces.insert(sorted_face({{tet[1], tet[2], tet[3]}}));
        for (size_t i = 0; i < 4; ++i) {
            for (size_t j = i + 1; j < 4; ++j) {
                tet_edges.insert(sorted_edge(tet[i], tet[j]));
            }
        }
    }

    for (const auto& [group, faces] : input.face_groups) {
        std::set<Face> unique;
        for (const Face& face : faces) {
            for (const size_t vertex : face) {
                if (vertex >= vertex_count) {
                    log_and_throw_error(
                        "Face group '{}' references out-of-range vertex {}.",
                        group,
                        vertex);
                }
            }
            const Face key = sorted_face(face);
            if (key[0] == key[1] || key[1] == key[2]) {
                log_and_throw_error("Face group '{}' contains a degenerate face.", group);
            }
            if (!unique.insert(key).second) {
                log_and_throw_error("Face group '{}' contains a duplicate face.", group);
            }
            if (!input.tets.empty() && tet_faces.count(key) == 0) {
                log_and_throw_error(
                    "Face group '{}' contains a triangle that is not a tetrahedral simplex.",
                    group);
            }
        }
    }
    for (const auto& [group, edges] : input.edge_groups) {
        std::set<Edge> unique;
        for (const Edge& edge : edges) {
            if (edge[0] >= vertex_count || edge[1] >= vertex_count || edge[0] == edge[1]) {
                log_and_throw_error("Edge group '{}' contains an invalid edge.", group);
            }
            const Edge key = sorted_edge(edge[0], edge[1]);
            if (!unique.insert(key).second) {
                log_and_throw_error("Edge group '{}' contains a duplicate edge.", group);
            }
            if (!input.tets.empty() && tet_edges.count(key) == 0) {
                log_and_throw_error(
                    "Edge group '{}' contains an edge that is not a tetrahedral simplex.",
                    group);
            }
        }
    }
}

void compact_output_vertices(HybridVolumeMesh& mesh)
{
    std::vector<bool> used(mesh.vertices.rows(), false);
    const auto mark = [&used](const auto& cells) {
        for (const auto& cell : cells) {
            for (const size_t v : cell) used[v] = true;
        }
    };
    mark(mesh.tets);
    mark(mesh.pyramids);
    mark(mesh.prisms);

    std::vector<size_t> remap(used.size(), std::numeric_limits<size_t>::max());
    size_t count = 0;
    for (size_t v = 0; v < used.size(); ++v) {
        if (used[v]) remap[v] = count++;
    }
    MatrixXd compact(count, 3);
    for (size_t v = 0; v < used.size(); ++v) {
        if (used[v]) compact.row(remap[v]) = mesh.vertices.row(v);
    }
    const auto apply = [&remap](auto& cells) {
        for (auto& cell : cells) {
            for (size_t& v : cell) v = remap[v];
        }
    };
    apply(mesh.tets);
    apply(mesh.pyramids);
    apply(mesh.prisms);
    mesh.vertices = std::move(compact);
}

template <size_t N>
size_t remove_duplicate_cells(
    std::vector<std::array<size_t, N>>& cells,
    std::vector<std::string>& regions,
    const MatrixXd& vertices,
    const double epsilon,
    const int element_type,
    std::set<std::string>& accepted)
{
    std::vector<std::array<size_t, N>> unique_cells;
    std::vector<std::string> unique_regions;
    unique_cells.reserve(cells.size());
    unique_regions.reserve(regions.size());
    size_t removed = 0;
    for (size_t cell_index = 0; cell_index < cells.size(); ++cell_index) {
        std::array<std::array<int64_t, 3>, N> points;
        for (size_t i = 0; i < N; ++i) {
            for (size_t axis = 0; axis < 3; ++axis) {
                points[i][axis] = static_cast<int64_t>(
                    std::llround(vertices(cells[cell_index][i], axis) / epsilon));
            }
        }
        std::sort(points.begin(), points.end());
        const std::string region =
            cell_index < regions.size() ? regions[cell_index] : std::string{};
        std::ostringstream key;
        key << element_type << ':' << region << ':';
        for (const auto& point : points) {
            key << point[0] << ',' << point[1] << ',' << point[2] << ';';
        }
        if (!accepted.insert(key.str()).second) {
            ++removed;
            continue;
        }
        unique_cells.push_back(cells[cell_index]);
        unique_regions.push_back(region);
    }
    cells = std::move(unique_cells);
    regions = std::move(unique_regions);
    return removed;
}

struct ShellSectorData
{
    std::map<std::pair<size_t, size_t>, size_t> sector_by_vertex_face;
    std::map<std::pair<size_t, size_t>, Vector3d> direction_by_vertex_sector;
    std::map<std::pair<size_t, size_t>, bool> singular_by_vertex_sector;
};

ShellSectorData shell_sectors(
    const MatrixXd& V,
    const std::vector<std::array<size_t, 3>>& faces,
    PrismaticMeshReport& report)
{
    std::vector<Vector3d> face_normals(faces.size());
    std::map<Edge, std::vector<size_t>> edge_faces;
    std::map<size_t, std::vector<size_t>> vertex_faces;
    for (size_t face_index = 0; face_index < faces.size(); ++face_index) {
        const auto& f = faces[face_index];
        const Vector3d a = V.row(f[0]);
        const Vector3d b = V.row(f[1]);
        const Vector3d c = V.row(f[2]);
        Vector3d n = (b - a).cross(c - a);
        if (n.norm() <= 1e-15) {
            log_and_throw_error("Shell annotation contains a degenerate triangle.");
        }
        n.normalize();
        face_normals[face_index] = n;
        for (const size_t v : f) vertex_faces[v].push_back(face_index);
        edge_faces[sorted_edge(f[0], f[1])].push_back(face_index);
        edge_faces[sorted_edge(f[1], f[2])].push_back(face_index);
        edge_faces[sorted_edge(f[2], f[0])].push_back(face_index);
    }
    for (const auto& [_, incident] : edge_faces) {
        report.nonmanifold_shell_edges += incident.size() > 2;
    }

    ShellSectorData result;
    size_t next_sector = 0;
    for (const auto& [v, incident_faces] : vertex_faces) {
        std::map<size_t, std::set<size_t>> adjacency;
        for (const size_t face : incident_faces) adjacency[face];
        for (const auto& [edge, edge_incident_faces] : edge_faces) {
            if (edge[0] != v && edge[1] != v) continue;
            if (edge_incident_faces.size() != 2) continue;
            adjacency[edge_incident_faces[0]].insert(edge_incident_faces[1]);
            adjacency[edge_incident_faces[1]].insert(edge_incident_faces[0]);
        }

        std::set<size_t> unvisited(incident_faces.begin(), incident_faces.end());
        while (!unvisited.empty()) {
            const size_t seed = *unvisited.begin();
            std::vector<size_t> component;
            std::queue<size_t> queue;
            queue.push(seed);
            unvisited.erase(seed);
            while (!queue.empty()) {
                const size_t face = queue.front();
                queue.pop();
                component.push_back(face);
                for (const size_t neighbor : adjacency[face]) {
                    if (unvisited.erase(neighbor) != 0) queue.push(neighbor);
                }
            }

            std::vector<Vector3d> ns;
            std::set<size_t> neighbors;
            for (const size_t face : component) {
                ns.push_back(face_normals[face]);
                for (const size_t vertex : faces[face]) {
                    if (vertex != v) neighbors.insert(vertex);
                }
            }
            MatrixXd C(ns.size(), 3);
            for (size_t i = 0; i < ns.size(); ++i) C.row(i) = ns[i];
            const VectorXd ones = VectorXd::Ones(ns.size());
            Vector3d direction = C.completeOrthogonalDecomposition().solve(ones);

            bool singular = !direction.allFinite() || direction.norm() <= 1e-14;
            if (!singular) {
                direction.normalize();
                for (const auto& n : ns) {
                    singular = singular || direction.dot(n) < 1e-3;
                }
            }
            if (singular) {
                direction.setZero();
                for (const auto& n : ns) direction += n;
                Vector3d laplacian = Vector3d::Zero();
                for (const size_t neighbor : neighbors) {
                    laplacian += V.row(v) - V.row(neighbor);
                }
                if (!neighbors.empty()) laplacian /= static_cast<double>(neighbors.size());
                if (laplacian.dot(direction) < 0) laplacian = -laplacian;
                if (laplacian.norm() > 1e-14) direction += laplacian.normalized();
                if (direction.norm() <= 1e-14) direction = ns.front();
                direction.normalize();
                ++report.singular_shell_vertices;
            } else {
                ++report.least_squares_shell_vertices;
            }
            const size_t sector = next_sector++;
            for (const size_t face : component) {
                result.sector_by_vertex_face[{v, face}] = sector;
            }
            result.direction_by_vertex_sector[{v, sector}] = direction;
            result.singular_by_vertex_sector[{v, sector}] = singular;
        }
    }
    report.shell_vertex_sectors += next_sector;
    return result;
}

struct TopologicalShellData
{
    struct Vertex
    {
        Vector3d position;
        int64_t source = -1;
        bool original_input = false;
        int label = 0;
    };

    struct Tet
    {
        std::array<size_t, 4> vertices;
        bool selected_solid = false;
    };

    struct OffsetFace
    {
        std::array<size_t, 3> vertices;
        std::array<size_t, 3> sources;
    };

    std::map<size_t, std::vector<Vector3d>> targets;
    std::set<size_t> marked_sources;
    std::set<size_t> marked_vertices;
    std::map<size_t, Vertex> vertices;
    std::vector<Tet> offset_tets;
    std::vector<OffsetFace> offset_faces;
};

TopologicalShellData topological_shell_data(
    const PrismaticMeshInput& input,
    const std::vector<Face>& faces,
    const double thickness,
    const size_t max_iterations,
    const std::set<std::string>& solid_groups,
    PrismaticMeshReport& report)
{
    MatrixXi tets(input.tets.size(), 4);
    for (size_t i = 0; i < input.tets.size(); ++i) {
        for (size_t j = 0; j < 4; ++j) tets(i, j) = static_cast<int>(input.tets[i][j]);
    }
    const std::vector<std::string> solid_names(solid_groups.begin(), solid_groups.end());
    MatrixSi tags(input.tets.size(), solid_names.size());
    tags.setZero();
    for (size_t t = 0; t < input.tets.size(); ++t) {
        for (size_t tag = 0; tag < solid_names.size(); ++tag) {
            tags.coeffRef(t, tag) = input.tet_groups[t].count(solid_names[tag]) != 0;
        }
    }
    MatrixXd envelope_vertices;
    MatrixXi envelope_faces;

    wmtk::components::topological_offset::Parameters offset_parameters;
    offset_parameters.respect_all_topologies = true;
    offset_parameters.offset_in = true;
    offset_parameters.offset_out = true;
    offset_parameters.target_distance = thickness;
    offset_parameters.target_distance_rel = -1;
    offset_parameters.relative_ball_threshold = 0.5;
    offset_parameters.edge_search_term_len = std::max(1e-12, thickness * 1e-6);
    offset_parameters.sorted_marching = true;
    offset_parameters.save_vtu = false;
    offset_parameters.debug_output = false;
    offset_parameters.protected_tags = solid_groups;
    const VectorXd minimum = input.vertices.colwise().minCoeff().transpose();
    const VectorXd maximum = input.vertices.colwise().maxCoeff().transpose();
    offset_parameters.init(minimum, maximum);

    PrismOffsetTetMesh mesh(offset_parameters, 0);
    mesh.init_from_image(
        input.vertices,
        tets,
        tags,
        envelope_vertices,
        envelope_faces,
        solid_names);
    mesh.label_input_complex(faces, {});
    mesh.init_input_complex_bvh();
    mesh.execute_offset({});
    ++report.topological_offset_runs;

    const auto boundary_stats = mesh.handle_open_boundaries(faces);
    report.open_boundary_edges += boundary_stats.boundary_edges;
    report.open_boundary_candidate_tets += boundary_stats.candidate_tets;
    report.open_boundary_removed_tets += boundary_stats.removed_tets;

    // Preserve the geometric target candidates produced by the offset before
    // topology editing. A collapse chooses one of these existing positions, so
    // retaining the full candidate set cannot weaken the later closest-target
    // selection and avoids coupling target quality to a topological survivor id.
    TopologicalShellData result;
    for (const auto& vertex : mesh.get_vertices()) {
        const auto& attributes = mesh.m_vertex_attribute[vertex.vid(mesh)];
        if (attributes.label != 2 || attributes.source_vid < 0 ||
            static_cast<size_t>(attributes.source_vid) >= input.vertices.rows()) {
            continue;
        }
        result.targets[static_cast<size_t>(attributes.source_vid)].push_back(attributes.m_posf);
        ++report.topological_mapped_vertices;
    }

    const auto operation_stats = mesh.simplify_offset(max_iterations);
    report.offset_operation_iterations += operation_stats.iterations;
    report.equal_source_collapse_attempts += operation_stats.equal_source_attempts;
    report.equal_source_collapses += operation_stats.equal_source_collapses;
    report.unlock_attempts += operation_stats.unlock_attempts;
    report.unlocks += operation_stats.unlocks;
    report.unlock_rollbacks += operation_stats.unlock_rollbacks;
    report.remaining_equal_source_edges += operation_stats.remaining_equal_source_edges;
    report.remaining_tau_2_2 += operation_stats.remaining_tau_2_2;
    for (const int64_t source : mesh.non_bijective_sources()) {
        if (source >= 0 && static_cast<size_t>(source) < input.vertices.rows()) {
            result.marked_sources.insert(static_cast<size_t>(source));
        }
    }
    result.marked_vertices = mesh.non_bijective_vertices();
    report.marked_shell_sources += result.marked_sources.size();

    std::set<Face> annotated_faces;
    for (const Face& face : faces) annotated_faces.insert(sorted_face(face));

    for (const auto& vertex : mesh.get_vertices()) {
        const size_t id = vertex.vid(mesh);
        const auto& attributes = mesh.m_vertex_attribute[id];
        result.vertices.emplace(
            id,
            TopologicalShellData::Vertex{
                attributes.m_posf,
                attributes.source_vid,
                attributes.original_input,
                attributes.label});
    }

    for (const auto& tet : mesh.get_tets()) {
        const auto& attributes = mesh.m_tet_attribute[tet.tid(mesh)];
        if (attributes.label != 2) continue;
        bool is_selected_solid = false;
        for (size_t tag = 0; tag < solid_names.size(); ++tag) {
            is_selected_solid =
                is_selected_solid || attributes.tag.count(static_cast<int64_t>(tag + 1)) != 0;
        }
        result.offset_tets.push_back({mesh.oriented_tet_vids(tet), is_selected_solid});
        ++report.topological_offset_tets;
    }

    for (const auto& face : mesh.get_faces()) {
        const auto face_vertices = mesh.get_face_vids(face);
        bool all_offset = true;
        std::array<size_t, 3> sources;
        for (size_t i = 0; i < 3; ++i) {
            const auto& attributes = mesh.m_vertex_attribute[face_vertices[i]];
            all_offset = all_offset && attributes.label == 2 && attributes.source_vid >= 0;
            if (attributes.source_vid >= 0) {
                sources[i] = static_cast<size_t>(attributes.source_vid);
            }
        }
        if (!all_offset || annotated_faces.count(sorted_face(sources)) == 0) continue;

        const bool current_is_offset = mesh.m_tet_attribute[face.tid(mesh)].label == 2;
        const auto other = face.switch_tetrahedron(mesh);
        const bool other_is_offset = other && mesh.m_tet_attribute[other->tid(mesh)].label == 2;
        if (current_is_offset == other_is_offset) continue;
        result.offset_faces.push_back({face_vertices, sources});
    }
    return result;
}

struct ShellSideFace
{
    Face face;
    std::array<size_t, 3> sectors;
    int side = 1;
};

std::optional<double> segment_triangle_parameter(
    const Vector3d& origin,
    const Vector3d& displacement,
    const Vector3d& a,
    const Vector3d& b,
    const Vector3d& c)
{
    const Vector3d edge0 = b - a;
    const Vector3d edge1 = c - a;
    const Vector3d cross = displacement.cross(edge1);
    const double determinant = edge0.dot(cross);
    const double scale = std::max({edge0.norm(), edge1.norm(), displacement.norm(), 1.0});
    const double epsilon = 1e-12 * scale * scale;
    if (std::abs(determinant) <= epsilon) return std::nullopt;
    const double inverse = 1 / determinant;
    const Vector3d delta = origin - a;
    const double u = delta.dot(cross) * inverse;
    if (u < -1e-12 || u > 1 + 1e-12) return std::nullopt;
    const double v = displacement.dot(delta.cross(edge0)) * inverse;
    if (v < -1e-12 || u + v > 1 + 1e-12) return std::nullopt;
    const double t = edge1.dot(delta.cross(edge0)) * inverse;
    if (t <= 1e-10 || t > 1 + 1e-12) return std::nullopt;
    return t;
}

std::vector<ShellSideFace> classify_shell_sides(
    const PrismaticMeshInput& input,
    const std::vector<Face>& faces,
    const ShellSectorData& sectors,
    const std::set<std::string>& solid_groups,
    const std::map<Face, std::vector<IncidentTet>>& incidence,
    PrismaticMeshReport& report)
{
    std::vector<ShellSideFace> result;
    result.reserve(2 * faces.size());
    for (size_t face_index = 0; face_index < faces.size(); ++face_index) {
        const Face& face = faces[face_index];
        std::array<size_t, 3> face_sectors;
        for (size_t i = 0; i < 3; ++i) {
            face_sectors[i] = sectors.sector_by_vertex_face.at({face[i], face_index});
        }
        const Vector3d a = input.vertices.row(face[0]);
        const Vector3d b = input.vertices.row(face[1]);
        const Vector3d c = input.vertices.row(face[2]);
        const Vector3d normal = (b - a).cross(c - a);
        bool positive_side_is_solid = false;
        bool negative_side_is_solid = false;
        const auto found = incidence.find(sorted_face(face));
        if (found != incidence.end()) {
            for (const IncidentTet& incident : found->second) {
                if (!selected_solid(input.tet_groups[incident.tet], solid_groups)) continue;
                const Vector3d apex = input.vertices.row(incident.opposite_vertex);
                const double orientation = normal.dot(apex - a);
                positive_side_is_solid = positive_side_is_solid || orientation > 0;
                negative_side_is_solid = negative_side_is_solid || orientation < 0;
            }
        }

        if (!positive_side_is_solid) result.push_back({face, face_sectors, 1});
        if (!negative_side_is_solid) result.push_back({face, face_sectors, -1});
        report.solid_blocked_shell_sides += static_cast<size_t>(positive_side_is_solid) +
                                            static_cast<size_t>(negative_side_is_solid);
        if (positive_side_is_solid && negative_side_is_solid) {
            report.warnings.push_back(
                "Shell face in group is enclosed by selected solids on both sides; no layer was "
                "emitted.");
        }
    }
    return result;
}

std::map<ShellVertexKey, double> shrink_shell_layer(
    const MatrixXd& input_vertices,
    const std::vector<ShellSideFace>& faces,
    const std::map<ShellVertexKey, Vector3d>& targets,
    const std::vector<Face>& collision_faces,
    const PrismaticMeshParameters& parameters,
    PrismaticMeshReport& report)
{
    std::map<ShellVertexKey, double> scales;
    for (const auto& candidate : faces) {
        for (size_t i = 0; i < 3; ++i) {
            scales[{candidate.face[i], candidate.side, candidate.sectors[i]}] = 1;
        }
    }

    // The offset segment is a conservative proxy for the swept shell surface.
    // Clip before optimization so no vertex can cross another annotated sheet.
    for (auto& [key, scale] : scales) {
        const size_t source = std::get<0>(key);
        const Vector3d origin = input_vertices.row(source);
        const Vector3d displacement = targets.at(key) - origin;
        double clipped = scale;
        for (const Face& obstacle : collision_faces) {
            if (std::find(obstacle.begin(), obstacle.end(), source) != obstacle.end()) continue;
            const auto hit = segment_triangle_parameter(
                origin,
                displacement,
                input_vertices.row(obstacle[0]),
                input_vertices.row(obstacle[1]),
                input_vertices.row(obstacle[2]));
            if (hit) clipped = std::min(clipped, 0.8 * *hit);
        }
        if (clipped < scale) {
            scale = clipped;
            ++report.collision_clipped_shell_vertices;
            report.minimum_shell_scale = std::min(report.minimum_shell_scale, scale);
        }
    }

    for (size_t iteration = 0; iteration < parameters.max_shell_iterations; ++iteration) {
        std::set<ShellVertexKey> shrink;
        for (const auto& candidate : faces) {
            std::array<Vector3d, 6> points;
            for (size_t i = 0; i < 3; ++i) {
                const size_t v = candidate.face[i];
                const ShellVertexKey key = {v, candidate.side, candidate.sectors[i]};
                points[i] = input_vertices.row(v);
                points[i + 3] = points[i] + scales.at(key) * (targets.at(key) - points[i]);
            }
            if (candidate.side < 0) {
                std::swap(points[1], points[2]);
                std::swap(points[4], points[5]);
            }
            if (!is_valid_prism(
                    points,
                    parameters.jacobian_tolerance,
                    parameters.validity_max_depth)) {
                for (size_t i = 0; i < 3; ++i) {
                    shrink.insert({candidate.face[i], candidate.side, candidate.sectors[i]});
                }
            }
        }
        if (shrink.empty()) return scales;
        ++report.shell_shrink_iterations;
        for (const auto& key : shrink) {
            scales[key] *= 0.5;
            report.minimum_shell_scale = std::min(report.minimum_shell_scale, scales[key]);
        }
    }
    report.warnings.push_back(
        "Kernel shrinking reached max_shell_iterations; unresolved cells use tetrahedral "
        "fallback.");
    return scales;
}

struct Frame
{
    Vector3d center;
    Vector3d tangent;
    Vector3d u;
    Vector3d v;
    double chain_parameter = 0;
};

std::vector<std::vector<size_t>> rod_chains(
    const std::vector<Edge>& edges,
    size_t& irregular_count,
    std::map<size_t, size_t>& valences)
{
    std::map<size_t, std::vector<size_t>> adjacency;
    std::unordered_set<Edge, EdgeHash> unused;
    for (const auto& raw : edges) {
        if (raw[0] == raw[1]) continue;
        const Edge e = sorted_edge(raw[0], raw[1]);
        if (!unused.insert(e).second) continue;
        adjacency[e[0]].push_back(e[1]);
        adjacency[e[1]].push_back(e[0]);
    }
    for (auto& [_, neighbors] : adjacency) {
        std::sort(neighbors.begin(), neighbors.end());
    }

    std::set<size_t> irregular;
    for (const auto& [v, neighbors] : adjacency) {
        valences[v] = neighbors.size();
        if (neighbors.size() != 2) irregular.insert(v);
    }
    irregular_count += irregular.size();

    std::vector<std::vector<size_t>> result;
    auto trace = [&adjacency, &unused](const size_t start, const size_t first) {
        std::vector<size_t> chain = {start, first};
        unused.erase(sorted_edge(start, first));
        size_t previous = start;
        size_t current = first;
        while (adjacency[current].size() == 2) {
            const size_t next =
                adjacency[current][0] == previous ? adjacency[current][1] : adjacency[current][0];
            const Edge e = sorted_edge(current, next);
            if (unused.erase(e) == 0) break;
            chain.push_back(next);
            previous = current;
            current = next;
        }
        return chain;
    };

    for (const size_t start : irregular) {
        for (const size_t neighbor : adjacency[start]) {
            if (unused.count(sorted_edge(start, neighbor)) != 0) {
                result.push_back(trace(start, neighbor));
            }
        }
    }
    while (!unused.empty()) {
        const Edge seed = *std::min_element(unused.begin(), unused.end());
        result.push_back(trace(seed[0], seed[1]));
    }
    return result;
}

std::vector<Frame>
make_frames(const MatrixXd& V, const std::vector<size_t>& chain, const size_t subdivisions)
{
    std::vector<Vector3d> centers;
    for (size_t i = 0; i + 1 < chain.size(); ++i) {
        const Vector3d a = V.row(chain[i]);
        const Vector3d b = V.row(chain[i + 1]);
        centers.push_back(a);
        for (size_t j = 0; j < subdivisions; ++j) {
            const double t = static_cast<double>(j + 1) / (subdivisions + 1);
            centers.push_back((1 - t) * a + t * b);
        }
    }
    centers.push_back(V.row(chain.back()));

    std::vector<double> arc(centers.size(), 0);
    for (size_t i = 1; i < centers.size(); ++i) {
        arc[i] = arc[i - 1] + (centers[i] - centers[i - 1]).norm();
    }
    const double total = arc.back();
    if (!(total > 0)) return {};

    std::vector<Frame> frames(centers.size());
    for (size_t i = 0; i < centers.size(); ++i) {
        Vector3d tangent;
        if (i == 0)
            tangent = centers[1] - centers[0];
        else if (i + 1 == centers.size())
            tangent = centers[i] - centers[i - 1];
        else
            tangent = centers[i + 1] - centers[i - 1];
        if (tangent.norm() <= 1e-15) return {};
        tangent.normalize();

        Vector3d u;
        if (i == 0) {
            const Vector3d axis =
                std::abs(tangent.x()) < 0.8 ? Vector3d::UnitX() : Vector3d::UnitY();
            u = tangent.cross(axis).normalized();
        } else {
            u = frames[i - 1].u - tangent * tangent.dot(frames[i - 1].u);
            if (u.norm() <= 1e-12) {
                const Vector3d axis =
                    std::abs(tangent.x()) < 0.8 ? Vector3d::UnitX() : Vector3d::UnitY();
                u = tangent.cross(axis);
            }
            u.normalize();
        }
        frames[i] = {centers[i], tangent, u, tangent.cross(u).normalized(), arc[i] / total};
    }
    return frames;
}

void append_rod_chain(
    HybridVolumeMesh& mesh,
    std::vector<Vector3d>& vertices,
    const MatrixXd& input_vertices,
    const std::vector<size_t>& chain,
    const RodDefinition& rod,
    const std::map<size_t, size_t>& valences,
    std::map<size_t, size_t>& shared_joint_vertices,
    const PrismaticMeshParameters& parameters,
    PrismaticMeshReport& report)
{
    const auto frames = make_frames(input_vertices, chain, rod.subdivisions);
    if (frames.size() < 2) {
        report.warnings.push_back("Skipped zero-length rod chain in group '" + rod.group + "'.");
        return;
    }

    struct RodRing
    {
        Frame frame;
        size_t center = 0;
        std::vector<size_t> vertices;
    };
    const auto make_ring = [&](const Frame& frame) {
        RodRing result;
        result.frame = frame;
        result.center = append_vertex(vertices, frame.center);
        result.vertices.reserve(rod.cross_section.size());
        const double interior_weight =
            4 * frame.chain_parameter * (1 - frame.chain_parameter) * rod.circular_blend;
        for (size_t i = 0; i < rod.cross_section.size(); ++i) {
            const Vector2d source = rod.cross_section[i];
            const double radius = source.norm();
            const double angle = 2 * PI * static_cast<double>(i) / rod.cross_section.size();
            const Vector2d circular(radius * std::cos(angle), radius * std::sin(angle));
            const Vector2d profile = (1 - interior_weight) * source + interior_weight * circular;
            result.vertices.push_back(append_vertex(
                vertices,
                frame.center + rod.thickness * (profile.x() * frame.u + profile.y() * frame.v)));
        }
        return result;
    };

    std::vector<RodRing> rings;
    rings.reserve(frames.size());
    for (const Frame& frame : frames) rings.push_back(make_ring(frame));

    const auto midpoint_frame = [](const Frame& left, const Frame& right) {
        Frame result;
        result.center = 0.5 * (left.center + right.center);
        result.chain_parameter = 0.5 * (left.chain_parameter + right.chain_parameter);
        result.tangent = right.center - left.center;
        if (result.tangent.norm() <= 1e-15) result.tangent = left.tangent + right.tangent;
        result.tangent.normalize();
        result.u = left.u - result.tangent * result.tangent.dot(left.u);
        if (result.u.norm() <= 1e-12) {
            result.u = right.u - result.tangent * result.tangent.dot(right.u);
        }
        if (result.u.norm() <= 1e-12) {
            const Vector3d axis =
                std::abs(result.tangent.x()) < 0.8 ? Vector3d::UnitX() : Vector3d::UnitY();
            result.u = result.tangent.cross(axis);
        }
        result.u.normalize();
        result.v = result.tangent.cross(result.u).normalized();
        if (result.v.dot(left.v + right.v) < 0) {
            result.u = -result.u;
            result.v = -result.v;
        }
        return result;
    };

    std::function<void(const RodRing&, const RodRing&, size_t)> append_segment;
    append_segment = [&](const RodRing& first, const RodRing& second, const size_t depth) {
        bool valid = true;
        for (size_t i = 0; i < rod.cross_section.size(); ++i) {
            const size_t next = (i + 1) % rod.cross_section.size();
            valid = valid && prism_has_valid_orientation(
                                 vertices,
                                 {{first.center,
                                   first.vertices[i],
                                   first.vertices[next],
                                   second.center,
                                   second.vertices[i],
                                   second.vertices[next]}},
                                 parameters);
        }
        if (!valid && depth < parameters.validity_max_depth) {
            ++report.rod_refined_segments;
            report.rod_refinement_max_depth = std::max(report.rod_refinement_max_depth, depth + 1);
            const RodRing middle = make_ring(midpoint_frame(first.frame, second.frame));
            append_segment(first, middle, depth + 1);
            append_segment(middle, second, depth + 1);
            return;
        }
        for (size_t i = 0; i < rod.cross_section.size(); ++i) {
            const size_t next = (i + 1) % rod.cross_section.size();
            append_prism(
                mesh,
                vertices,
                {{first.center,
                  first.vertices[i],
                  first.vertices[next],
                  second.center,
                  second.vertices[i],
                  second.vertices[next]}},
                "rod:" + rod.group,
                parameters,
                report,
                true);
        }
    };

    const auto is_joint = [&valences](const size_t vertex) {
        const auto found = valences.find(vertex);
        return found != valences.end() && found->second > 2;
    };
    const bool front_joint = is_joint(chain.front());
    const bool back_joint = is_joint(chain.back());
    if (front_joint && back_joint && rings.size() == 2) {
        rings.insert(
            rings.begin() + 1,
            make_ring(midpoint_frame(rings.front().frame, rings.back().frame)));
    }
    const auto joint_vertex = [&](const size_t input_vertex) {
        const auto found = shared_joint_vertices.find(input_vertex);
        if (found != shared_joint_vertices.end()) return found->second;
        const size_t output_vertex = append_vertex(vertices, input_vertices.row(input_vertex));
        shared_joint_vertices.emplace(input_vertex, output_vertex);
        return output_vertex;
    };
    const auto append_joint_fan = [&](const size_t apex, const RodRing& ring) {
        for (size_t i = 0; i < rod.cross_section.size(); ++i) {
            const size_t next = (i + 1) % rod.cross_section.size();
            orient_and_append_tet(
                mesh,
                vertices,
                {{apex, ring.center, ring.vertices[i], ring.vertices[next]}},
                "rod-joint:" + rod.group);
            ++report.rod_joint_tets;
        }
    };

    for (size_t k = 0; k + 1 < rings.size(); ++k) {
        if (k == 0 && front_joint) {
            append_joint_fan(joint_vertex(chain.front()), rings[k + 1]);
            continue;
        }
        if (k + 2 == rings.size() && back_joint) {
            append_joint_fan(joint_vertex(chain.back()), rings[k]);
            continue;
        }
        append_segment(rings[k], rings[k + 1], 0);
    }
}

struct SharedOffsetComponent
{
    std::vector<Face> faces;
    std::vector<Edge> edges;
    std::set<std::string> shell_groups;
    std::set<std::string> rod_groups;
    double thickness = 0;
    size_t junctions = 0;
    size_t count = 1;
};

std::vector<SharedOffsetComponent> rod_shell_components(
    const PrismaticMeshInput& input,
    const PrismaticMeshParameters& parameters)
{
    struct Annotation
    {
        bool shell = false;
        std::string group;
        double thickness = 0;
        std::set<size_t> vertices;
    };

    std::vector<Annotation> annotations;
    annotations.reserve(parameters.shells.size() + parameters.rods.size());
    for (const ShellDefinition& shell : parameters.shells) {
        Annotation annotation{true, shell.group, shell.thickness, {}};
        const auto found = input.face_groups.find(shell.group);
        if (found != input.face_groups.end()) {
            for (const Face& face : found->second) {
                annotation.vertices.insert(face.begin(), face.end());
            }
        }
        annotations.push_back(std::move(annotation));
    }
    for (const RodDefinition& rod : parameters.rods) {
        Annotation annotation{false, rod.group, rod.thickness, {}};
        const auto found = input.edge_groups.find(rod.group);
        if (found != input.edge_groups.end()) {
            for (const Edge& edge : found->second) {
                annotation.vertices.insert(edge.begin(), edge.end());
            }
        }
        annotations.push_back(std::move(annotation));
    }

    std::vector<size_t> parent(annotations.size());
    std::iota(parent.begin(), parent.end(), 0);
    const std::function<size_t(size_t)> root = [&parent, &root](const size_t i) -> size_t {
        if (parent[i] != i) parent[i] = root(parent[i]);
        return parent[i];
    };
    const auto unite = [&parent, &root](const size_t a, const size_t b) {
        const size_t ra = root(a);
        const size_t rb = root(b);
        if (ra != rb) parent[std::max(ra, rb)] = std::min(ra, rb);
    };
    for (size_t i = 0; i < annotations.size(); ++i) {
        for (size_t j = i + 1; j < annotations.size(); ++j) {
            bool intersects = false;
            auto a = annotations[i].vertices.begin();
            auto b = annotations[j].vertices.begin();
            while (a != annotations[i].vertices.end() && b != annotations[j].vertices.end()) {
                if (*a == *b) {
                    intersects = true;
                    break;
                }
                if (*a < *b)
                    ++a;
                else
                    ++b;
            }
            if (intersects) unite(i, j);
        }
    }

    std::map<size_t, std::vector<size_t>> indices_by_component;
    for (size_t i = 0; i < annotations.size(); ++i) indices_by_component[root(i)].push_back(i);

    std::vector<SharedOffsetComponent> result;
    for (const auto& [_, indices] : indices_by_component) {
        bool has_shell = false;
        bool has_rod = false;
        for (const size_t i : indices) {
            has_shell = has_shell || annotations[i].shell;
            has_rod = has_rod || !annotations[i].shell;
        }
        if (!has_shell || !has_rod) continue;

        SharedOffsetComponent component;
        component.thickness = annotations[indices.front()].thickness;
        std::set<size_t> shell_vertices;
        std::set<size_t> rod_vertices;
        for (const size_t i : indices) {
            const Annotation& annotation = annotations[i];
            const double scale = std::max({1.0, component.thickness, annotation.thickness});
            if (std::abs(annotation.thickness - component.thickness) > 1e-12 * scale) {
                log_and_throw_error(
                    "Connected rod-shell annotations require a common thickness; group '{}' uses "
                    "{} while the junction component uses {}.",
                    annotation.group,
                    annotation.thickness,
                    component.thickness);
            }
            if (annotation.shell) {
                component.shell_groups.insert(annotation.group);
                shell_vertices.insert(annotation.vertices.begin(), annotation.vertices.end());
                const auto found = input.face_groups.find(annotation.group);
                if (found != input.face_groups.end()) {
                    component.faces.insert(
                        component.faces.end(),
                        found->second.begin(),
                        found->second.end());
                }
            } else {
                component.rod_groups.insert(annotation.group);
                rod_vertices.insert(annotation.vertices.begin(), annotation.vertices.end());
                const auto found = input.edge_groups.find(annotation.group);
                if (found != input.edge_groups.end()) {
                    component.edges.insert(
                        component.edges.end(),
                        found->second.begin(),
                        found->second.end());
                }
            }
        }
        for (const size_t vertex : shell_vertices) {
            component.junctions += rod_vertices.count(vertex) != 0;
        }
        result.push_back(std::move(component));
    }
    return result;
}

std::string joined_groups(const std::set<std::string>& groups)
{
    std::ostringstream stream;
    bool first = true;
    for (const std::string& group : groups) {
        if (!first) stream << '+';
        stream << group;
        first = false;
    }
    return stream.str();
}

void append_shared_offset_component(
    HybridVolumeMesh& output,
    std::vector<Vector3d>& output_vertices,
    const PrismaticMeshInput& input,
    const SharedOffsetComponent& component,
    const std::set<std::string>& solid_groups,
    const size_t max_iterations,
    PrismaticMeshReport& report)
{
    MatrixXi tets(input.tets.size(), 4);
    for (size_t i = 0; i < input.tets.size(); ++i) {
        for (size_t j = 0; j < 4; ++j) tets(i, j) = static_cast<int>(input.tets[i][j]);
    }
    const std::vector<std::string> solid_names(solid_groups.begin(), solid_groups.end());
    MatrixSi tags(input.tets.size(), solid_names.size());
    tags.setZero();
    for (size_t t = 0; t < input.tets.size(); ++t) {
        for (size_t tag = 0; tag < solid_names.size(); ++tag) {
            if (input.tet_groups[t].count(solid_names[tag]) != 0) {
                tags.coeffRef(t, tag) = 1;
            }
        }
    }

    wmtk::components::topological_offset::Parameters offset_parameters;
    offset_parameters.respect_all_topologies = true;
    offset_parameters.offset_in = true;
    offset_parameters.offset_out = true;
    offset_parameters.target_distance = component.thickness;
    offset_parameters.target_distance_rel = -1;
    offset_parameters.relative_ball_threshold = 0.5;
    offset_parameters.edge_search_term_len = std::max(1e-12, component.thickness * 1e-6);
    offset_parameters.sorted_marching = true;
    offset_parameters.save_vtu = false;
    offset_parameters.debug_output = false;
    const VectorXd minimum = input.vertices.colwise().minCoeff().transpose();
    const VectorXd maximum = input.vertices.colwise().maxCoeff().transpose();
    offset_parameters.init(minimum, maximum);

    MatrixXd envelope_vertices;
    MatrixXi envelope_faces;
    PrismOffsetTetMesh mesh(offset_parameters, 0);
    mesh.init_from_image(
        input.vertices,
        tets,
        tags,
        envelope_vertices,
        envelope_faces,
        solid_names);
    mesh.label_input_complex(component.faces, component.edges);
    mesh.init_input_complex_bvh();
    mesh.execute_offset({});
    ++report.topological_offset_runs;

    const auto operation_stats = mesh.simplify_offset(max_iterations);
    report.offset_operation_iterations += operation_stats.iterations;
    report.equal_source_collapse_attempts += operation_stats.equal_source_attempts;
    report.equal_source_collapses += operation_stats.equal_source_collapses;
    report.unlock_attempts += operation_stats.unlock_attempts;
    report.unlocks += operation_stats.unlocks;
    report.unlock_rollbacks += operation_stats.unlock_rollbacks;
    report.remaining_equal_source_edges += operation_stats.remaining_equal_source_edges;
    report.remaining_tau_2_2 += operation_stats.remaining_tau_2_2;

    std::map<size_t, size_t> output_vertex;
    const auto map_vertex = [&](const size_t vertex) {
        const auto found = output_vertex.find(vertex);
        if (found != output_vertex.end()) return found->second;
        const auto& attributes = mesh.m_vertex_attribute[vertex];
        size_t mapped;
        if (attributes.original_input && vertex < static_cast<size_t>(input.vertices.rows())) {
            mapped = vertex;
        } else {
            mapped = append_vertex(output_vertices, attributes.m_posf);
        }
        output_vertex.emplace(vertex, mapped);
        if (attributes.label == 2 && attributes.source_vid >= 0) {
            ++report.topological_mapped_vertices;
        }
        return mapped;
    };

    const std::string junction_region = "rod-shell:" + joined_groups(component.shell_groups) + "|" +
                                        joined_groups(component.rod_groups);
    for (const auto& tet : mesh.get_tets()) {
        const size_t tet_id = tet.tid(mesh);
        const auto& attributes = mesh.m_tet_attribute[tet_id];
        const bool is_offset = attributes.label == 2;
        std::string solid_region;
        for (size_t tag = 0; tag < solid_names.size(); ++tag) {
            if (attributes.tag.count(static_cast<int64_t>(tag + 1)) != 0) {
                solid_region = "solid:" + solid_names[tag];
                break;
            }
        }
        if (!is_offset && solid_region.empty()) continue;

        auto ids = mesh.oriented_tet_vids(tet);
        for (size_t& vertex : ids) vertex = map_vertex(vertex);
        orient_and_append_tet(
            output,
            output_vertices,
            ids,
            solid_region.empty() ? junction_region : solid_region);
        if (!solid_region.empty()) {
            ++report.solid_tets;
        } else {
            ++report.fallback_tets;
            ++report.rod_shell_transition_tets;
        }
        report.topological_offset_tets += is_offset;
    }

    report.rod_shell_junctions += component.junctions;
    report.rod_shell_components += component.count;
    report.warnings.push_back(
        "Rod-shell component '" + joined_groups(component.shell_groups) + "|" +
        joined_groups(component.rod_groups) +
        "' was retained as conforming tetrahedra from shared offset topology because no "
        "conforming prism partition was proven.");
}

} // namespace

PrismaticMeshResult generate_prismatic_mesh(
    const PrismaticMeshInput& input,
    const PrismaticMeshParameters& parameters)
{
    if (input.vertices.cols() != 3 || input.tets.size() != input.tet_groups.size()) {
        log_and_throw_error("Invalid PrismaticMeshInput.");
    }
    validate_input_complex(input);

    PrismaticMeshResult result;
    result.report.input_tets = input.tets.size();
    std::vector<Vector3d> vertices(input.vertices.rows());
    for (size_t i = 0; i < vertices.size(); ++i) vertices[i] = input.vertices.row(i);

    const std::set<std::string> solid_groups(
        parameters.solid_groups.begin(),
        parameters.solid_groups.end());
    auto shared_components = rod_shell_components(input, parameters);
    if (!shared_components.empty() && !parameters.use_topological_offset) {
        log_and_throw_error(
            "Rod-shell junctions require use_topological_offset=true so their transition cells "
            "share one editable tetrahedral topology.");
    }
    if (!shared_components.empty() && input.tets.empty()) {
        log_and_throw_error(
            "Rod-shell junctions require an ambient tetrahedral mesh around the annotated "
            "simplices.");
    }
    if (!solid_groups.empty() && shared_components.size() > 1) {
        SharedOffsetComponent combined = shared_components.front();
        for (size_t i = 1; i < shared_components.size(); ++i) {
            const auto& component = shared_components[i];
            const double scale = std::max({1.0, combined.thickness, component.thickness});
            if (std::abs(component.thickness - combined.thickness) > 1e-12 * scale) {
                log_and_throw_error(
                    "Disconnected rod-shell components with different thicknesses cannot share "
                    "the selected-solid transition topology.");
            }
            combined.faces.insert(
                combined.faces.end(),
                component.faces.begin(),
                component.faces.end());
            combined.edges.insert(
                combined.edges.end(),
                component.edges.begin(),
                component.edges.end());
            combined.shell_groups.insert(
                component.shell_groups.begin(),
                component.shell_groups.end());
            combined.rod_groups.insert(component.rod_groups.begin(), component.rod_groups.end());
            combined.junctions += component.junctions;
            combined.count += component.count;
        }
        shared_components = {std::move(combined)};
    }
    std::set<std::string> shared_shell_groups;
    std::set<std::string> shared_rod_groups;
    for (const auto& component : shared_components) {
        shared_shell_groups.insert(component.shell_groups.begin(), component.shell_groups.end());
        shared_rod_groups.insert(component.rod_groups.begin(), component.rod_groups.end());
        append_shared_offset_component(
            result.mesh,
            vertices,
            input,
            component,
            solid_groups,
            parameters.max_shell_iterations,
            result.report);
    }
    const auto face_incidence = tet_face_incidence(input);
    std::vector<Face> collision_faces;
    for (const auto& shell : parameters.shells) {
        const auto found = input.face_groups.find(shell.group);
        if (found != input.face_groups.end()) {
            collision_faces.insert(
                collision_faces.end(),
                found->second.begin(),
                found->second.end());
        }
    }
    for (size_t i = 0; i < input.tets.size(); ++i) {
        if (!shared_components.empty() && !solid_groups.empty()) continue;
        if (!selected_solid(input.tet_groups[i], solid_groups)) continue;
        auto tet = input.tets[i];
        auto points = cell_points(vertices, tet);
        if (!is_valid_tet(points)) std::swap(tet[2], tet[3]);
        result.mesh.tets.push_back(tet);
        std::string region = "solid";
        for (const auto& group : input.tet_groups[i]) {
            if (solid_groups.count(group) != 0) {
                region = "solid:" + group;
                break;
            }
        }
        result.mesh.tet_region_tags.push_back(region);
        ++result.report.solid_tets;
    }

    std::map<size_t, size_t> movable_shell_vertices;
    for (const auto& shell : parameters.shells) {
        if (shared_shell_groups.count(shell.group) != 0) continue;
        const auto found = input.face_groups.find(shell.group);
        if (found == input.face_groups.end() || found->second.empty()) {
            log_and_throw_error("Shell physical group '{}' is missing or empty.", shell.group);
        }
        const auto sectors = shell_sectors(input.vertices, found->second, result.report);
        const auto topology = parameters.use_topological_offset
                                  ? topological_shell_data(
                                        input,
                                        found->second,
                                        shell.thickness,
                                        parameters.max_shell_iterations,
                                        solid_groups,
                                        result.report)
                                  : TopologicalShellData{};
        const auto side_faces = classify_shell_sides(
            input,
            found->second,
            sectors,
            solid_groups,
            face_incidence,
            result.report);

        if (parameters.use_topological_offset) {
            struct Candidate
            {
                const TopologicalShellData::OffsetFace* offset = nullptr;
                const ShellSideFace* side = nullptr;
            };

            std::map<Face, size_t> input_face_index;
            for (size_t face_index = 0; face_index < found->second.size(); ++face_index) {
                input_face_index.emplace(sorted_face(found->second[face_index]), face_index);
            }
            std::map<std::pair<Face, int>, const ShellSideFace*> allowed_sides;
            for (const ShellSideFace& side : side_faces) {
                allowed_sides.emplace(std::make_pair(sorted_face(side.face), side.side), &side);
            }

            std::vector<Candidate> candidates;
            std::set<std::array<size_t, 3>> seen_offset_faces;
            for (const auto& offset_face : topology.offset_faces) {
                auto face_key = offset_face.vertices;
                std::sort(face_key.begin(), face_key.end());
                if (!seen_offset_faces.insert(face_key).second) continue;
                const Face source_face = sorted_face(offset_face.sources);
                const auto face_it = input_face_index.find(source_face);
                if (face_it == input_face_index.end()) continue;
                const Face& oriented = found->second[face_it->second];
                const Vector3d a = input.vertices.row(oriented[0]);
                const Vector3d b = input.vertices.row(oriented[1]);
                const Vector3d c = input.vertices.row(oriented[2]);
                Vector3d offset_center = Vector3d::Zero();
                for (const size_t vertex : offset_face.vertices) {
                    offset_center += topology.vertices.at(vertex).position;
                }
                offset_center /= 3;
                const double signed_side =
                    (b - a).cross(c - a).dot(offset_center - (a + b + c) / 3);
                if (std::abs(signed_side) <= 1e-14) continue;
                const int side = signed_side > 0 ? 1 : -1;
                const auto allowed = allowed_sides.find({source_face, side});
                if (allowed != allowed_sides.end()) {
                    candidates.push_back({&offset_face, allowed->second});
                }
            }

            // Assign one target to every actual offset-surface vertex. Sharing
            // follows the edited tetrahedral topology; it is no longer inferred
            // from an input (source,side,sector) key.
            std::map<size_t, Vector3d> target_sum;
            std::map<size_t, size_t> target_count;
            std::set<size_t> singular_top_vertices;
            std::map<size_t, std::set<size_t>> top_neighbors;
            for (const Candidate& candidate : candidates) {
                for (size_t i = 0; i < 3; ++i) {
                    top_neighbors[candidate.offset->vertices[i]].insert(
                        candidate.offset->vertices[(i + 1) % 3]);
                    top_neighbors[candidate.offset->vertices[i]].insert(
                        candidate.offset->vertices[(i + 2) % 3]);
                }
                for (size_t i = 0; i < 3; ++i) {
                    const size_t top = candidate.offset->vertices[i];
                    const size_t source = candidate.offset->sources[i];
                    size_t local = 0;
                    while (candidate.side->face[local] != source) ++local;
                    const size_t sector = candidate.side->sectors[local];
                    const Vector3d direction =
                        sectors.direction_by_vertex_sector.at({source, sector});
                    if (target_count[top] == 0) target_sum[top] = Vector3d::Zero();
                    const Vector3d origin = input.vertices.row(source);
                    Vector3d target_direction = candidate.side->side * direction;
                    if (sectors.singular_by_vertex_sector.at({source, sector})) {
                        target_direction = topology.vertices.at(top).position - origin;
                        if (target_direction.norm() <= 1e-14) {
                            target_direction = candidate.side->side * direction;
                        }
                        target_direction.normalize();
                        singular_top_vertices.insert(top);
                    }
                    target_sum[top] += origin + shell.thickness * target_direction;
                    ++target_count[top];
                }
            }

            std::map<size_t, Vector3d> top_position;
            for (const auto& [vertex, count] : target_count) {
                top_position[vertex] = target_sum.at(vertex) / static_cast<double>(count);
            }

            // PositionAdjustment: a regular target must remain in the kernel
            // represented by the original one-ring of its offset vertex. Shrink
            // toward the corresponding input vertex; if the one-ring cannot
            // contain the target, switch to the paper's singular sphere target.
            for (auto& [vertex, position] : top_position) {
                if (singular_top_vertices.count(vertex) != 0) continue;
                const size_t source = static_cast<size_t>(topology.vertices.at(vertex).source);
                const Vector3d origin = input.vertices.row(source);
                bool in_kernel = false;
                for (size_t iteration = 0; iteration < parameters.max_shell_iterations;
                     ++iteration) {
                    for (const auto& tet : topology.offset_tets) {
                        if (tet.selected_solid ||
                            std::find(tet.vertices.begin(), tet.vertices.end(), vertex) ==
                                tet.vertices.end()) {
                            continue;
                        }
                        std::array<Vector3d, 4> tet_points;
                        for (size_t i = 0; i < 4; ++i) {
                            tet_points[i] = topology.vertices.at(tet.vertices[i]).position;
                        }
                        if (point_in_closed_tet(position, tet_points)) {
                            in_kernel = true;
                            break;
                        }
                    }
                    if (in_kernel) break;
                    position = 0.5 * (origin + position);
                    ++result.report.shell_shrink_iterations;
                    result.report.minimum_shell_scale = std::min(
                        result.report.minimum_shell_scale,
                        (position - origin).norm() / shell.thickness);
                }
                if (!in_kernel) {
                    Vector3d direction = topology.vertices.at(vertex).position - origin;
                    if (direction.norm() <= 1e-14) {
                        direction = position - origin;
                    }
                    if (direction.norm() > 1e-14) {
                        position = origin + shell.thickness * direction.normalized();
                    }
                    singular_top_vertices.insert(vertex);
                }
            }

            // The paper resolves the non-unique spherical target with one
            // Laplacian round. Reprojecting the averaged direction retains the
            // prescribed shell radius while regularizing neighboring targets.
            const auto pre_laplacian = top_position;
            for (const size_t vertex : singular_top_vertices) {
                if (top_neighbors[vertex].empty()) continue;
                Vector3d average = Vector3d::Zero();
                size_t count = 0;
                for (const size_t neighbor : top_neighbors[vertex]) {
                    const auto position = pre_laplacian.find(neighbor);
                    if (position == pre_laplacian.end()) continue;
                    average += position->second;
                    ++count;
                }
                if (count == 0) continue;
                average /= static_cast<double>(count);
                const size_t source = static_cast<size_t>(topology.vertices.at(vertex).source);
                const Vector3d origin = input.vertices.row(source);
                Vector3d direction = average - origin;
                if (direction.norm() > 1e-14) {
                    top_position[vertex] = origin + shell.thickness * direction.normalized();
                }
            }

            // PositionAdjustment is accepted only while every editable offset
            // tetrahedron preserves its original orientation. Prism validity
            // alone is insufficient because collapsed columns can leave
            // unmatched tetrahedra that are emitted by the conservative
            // fallback below. Roll back all relocated vertices of an invalid
            // tetrahedron toward their proven-valid offset positions.
            for (size_t iteration = 0; iteration < parameters.max_shell_iterations; ++iteration) {
                std::set<size_t> rollback;
                for (const auto& tet : topology.offset_tets) {
                    if (tet.selected_solid) continue;
                    std::array<Vector3d, 4> original_points;
                    std::array<Vector3d, 4> target_points;
                    for (size_t i = 0; i < tet.vertices.size(); ++i) {
                        original_points[i] = topology.vertices.at(tet.vertices[i]).position;
                        const auto target = top_position.find(tet.vertices[i]);
                        target_points[i] =
                            target == top_position.end() ? original_points[i] : target->second;
                    }
                    if (!is_valid_tet(original_points)) {
                        std::swap(original_points[2], original_points[3]);
                        std::swap(target_points[2], target_points[3]);
                    }
                    if (is_valid_tet(target_points)) continue;
                    for (const size_t vertex : tet.vertices) {
                        if (top_position.count(vertex) != 0) rollback.insert(vertex);
                    }
                }
                if (rollback.empty()) break;
                ++result.report.shell_shrink_iterations;
                for (const size_t vertex : rollback) {
                    const Vector3d original = topology.vertices.at(vertex).position;
                    top_position.at(vertex) = 0.5 * (original + top_position.at(vertex));
                }
                if (iteration + 1 == parameters.max_shell_iterations) {
                    for (const size_t vertex : rollback) {
                        top_position.at(vertex) = topology.vertices.at(vertex).position;
                    }
                    result.report.warnings.push_back(
                        "Shell target relocation was rolled back to preserve offset-tet "
                        "orientation.");
                }
            }

            std::map<size_t, size_t> input_vertex_by_source;
            for (const auto& [vertex, attributes] : topology.vertices) {
                if (attributes.original_input && attributes.source >= 0) {
                    input_vertex_by_source.emplace(static_cast<size_t>(attributes.source), vertex);
                }
            }
            std::map<size_t, size_t> output_vertex_by_topology;
            const auto map_topology_vertex = [&](const size_t vertex) {
                const auto existing = output_vertex_by_topology.find(vertex);
                if (existing != output_vertex_by_topology.end()) return existing->second;
                const auto& attributes = topology.vertices.at(vertex);
                size_t output_vertex;
                if (attributes.original_input && attributes.source >= 0 &&
                    static_cast<size_t>(attributes.source) < input.vertices.rows()) {
                    output_vertex = static_cast<size_t>(attributes.source);
                } else {
                    const auto target = top_position.find(vertex);
                    output_vertex = append_vertex(
                        vertices,
                        target == top_position.end() ? attributes.position : target->second);
                }
                output_vertex_by_topology.emplace(vertex, output_vertex);
                if (!attributes.original_input && attributes.label == 2 && attributes.source >= 0 &&
                    static_cast<size_t>(attributes.source) < input.vertices.rows()) {
                    movable_shell_vertices.emplace(
                        output_vertex,
                        static_cast<size_t>(attributes.source));
                }
                return output_vertex;
            };

            std::set<size_t> represented_tets;
            for (const Candidate& candidate : candidates) {
                std::array<size_t, 6> prism;
                std::set<size_t> column_vertices;
                for (size_t i = 0; i < 3; ++i) {
                    const size_t source = candidate.offset->sources[i];
                    prism[i] = source;
                    prism[i + 3] = map_topology_vertex(candidate.offset->vertices[i]);
                    column_vertices.insert(candidate.offset->vertices[i]);
                    const auto input_vertex = input_vertex_by_source.find(source);
                    if (input_vertex != input_vertex_by_source.end()) {
                        column_vertices.insert(input_vertex->second);
                    }
                    movable_shell_vertices.emplace(prism[i + 3], source);
                }
                std::vector<size_t> column_tets;
                for (size_t tet_index = 0; tet_index < topology.offset_tets.size(); ++tet_index) {
                    const auto& tet = topology.offset_tets[tet_index];
                    bool inside_column = !tet.selected_solid;
                    for (const size_t vertex : tet.vertices) {
                        inside_column = inside_column && column_vertices.count(vertex) != 0;
                    }
                    if (inside_column) column_tets.push_back(tet_index);
                }

                // A recovered hybrid cell may replace tetrahedra only when the
                // two representations have exactly the same triangulated
                // boundary. A mere vertex-subset test is not sufficient after
                // embedding splits: it can emit a prism over tet fallback
                // cells that contain additional vertices, creating overlap.
                HybridVolumeMesh trial;
                PrismaticMeshReport trial_report;
                append_recovered_prism(
                    trial,
                    vertices,
                    prism,
                    {{topology.marked_vertices.count(candidate.offset->vertices[0]) != 0,
                      topology.marked_vertices.count(candidate.offset->vertices[1]) != 0,
                      topology.marked_vertices.count(candidate.offset->vertices[2]) != 0}},
                    "shell:" + shell.group,
                    parameters,
                    trial_report);

                using Triangle = std::array<size_t, 3>;
                const auto boundary_triangles = [](const std::vector<std::array<size_t, 4>>& tets) {
                    std::map<Triangle, size_t> counts;
                    static constexpr std::array<std::array<size_t, 3>, 4> local_faces = {
                        {{{0, 1, 2}}, {{0, 1, 3}}, {{0, 2, 3}}, {{1, 2, 3}}}};
                    for (const auto& tet : tets) {
                        for (const auto& local : local_faces) {
                            Triangle face = {{tet[local[0]], tet[local[1]], tet[local[2]]}};
                            std::sort(face.begin(), face.end());
                            ++counts[face];
                        }
                    }
                    std::set<Triangle> boundary;
                    bool valid = true;
                    for (const auto& [face, count] : counts) {
                        valid = valid && count <= 2;
                        if (count == 1) boundary.insert(face);
                    }
                    return std::make_pair(valid, boundary);
                };

                std::vector<std::array<size_t, 4>> actual_tets;
                bool disjoint = !column_tets.empty();
                for (const size_t tet_index : column_tets) {
                    disjoint = disjoint && represented_tets.count(tet_index) == 0;
                    std::array<size_t, 4> tet;
                    for (size_t i = 0; i < tet.size(); ++i) {
                        tet[i] = map_topology_vertex(topology.offset_tets[tet_index].vertices[i]);
                    }
                    actual_tets.push_back(tet);
                }

                std::vector<std::array<size_t, 4>> trial_tets = trial.tets;
                for (const auto& p : trial.prisms) {
                    trial_tets.push_back({{p[0], p[1], p[2], p[3]}});
                    trial_tets.push_back({{p[1], p[2], p[3], p[4]}});
                    trial_tets.push_back({{p[2], p[3], p[4], p[5]}});
                }
                for (const auto& p : trial.pyramids) {
                    trial_tets.push_back({{p[0], p[1], p[2], p[4]}});
                    trial_tets.push_back({{p[0], p[2], p[3], p[4]}});
                }
                const auto actual_boundary = boundary_triangles(actual_tets);
                const auto trial_boundary = boundary_triangles(trial_tets);
                if (!disjoint || !actual_boundary.first || !trial_boundary.first ||
                    actual_boundary.second != trial_boundary.second) {
                    continue;
                }

                append_recovered_prism(
                    result.mesh,
                    vertices,
                    prism,
                    {{topology.marked_vertices.count(candidate.offset->vertices[0]) != 0,
                      topology.marked_vertices.count(candidate.offset->vertices[1]) != 0,
                      topology.marked_vertices.count(candidate.offset->vertices[2]) != 0}},
                    "shell:" + shell.group,
                    parameters,
                    result.report);
                for (const size_t tet_index : column_tets) {
                    represented_tets.insert(tet_index);
                }
            }

            // Any offset tetrahedron not proven to belong to a recovered
            // prism/pyramid column remains an actual tetrahedron from the edited
            // topology. Selected solid cells are emitted by the solid path.
            for (size_t tet_index = 0; tet_index < topology.offset_tets.size(); ++tet_index) {
                const auto& tet = topology.offset_tets[tet_index];
                if (tet.selected_solid || represented_tets.count(tet_index) != 0) continue;
                std::array<size_t, 4> output_tet;
                for (size_t i = 0; i < 4; ++i) {
                    output_tet[i] = map_topology_vertex(tet.vertices[i]);
                }
                orient_and_append_tet(result.mesh, vertices, output_tet, "shell:" + shell.group);
                ++result.report.fallback_tets;
            }
            continue;
        }

        std::map<ShellVertexKey, Vector3d> targets;
        for (const auto& candidate : side_faces) {
            for (size_t local_vertex = 0; local_vertex < 3; ++local_vertex) {
                const size_t source = candidate.face[local_vertex];
                const size_t sector = candidate.sectors[local_vertex];
                const ShellVertexKey key = {source, candidate.side, sector};
                if (targets.count(key) != 0) continue;
                const Vector3d origin = input.vertices.row(source);
                const Vector3d direction = sectors.direction_by_vertex_sector.at({source, sector});
                const Vector3d desired = origin + candidate.side * shell.thickness * direction;
                Vector3d selected = desired;
                bool found_target = false;
                const auto candidates = topology.targets.find(source);
                if (candidates != topology.targets.end()) {
                    double best_distance = std::numeric_limits<double>::infinity();
                    for (const Vector3d& point : candidates->second) {
                        const Vector3d displacement = point - origin;
                        const double length = displacement.norm();
                        if (!point.allFinite() || length < 0.05 * shell.thickness ||
                            length > 2.0 * shell.thickness ||
                            displacement.dot(candidate.side * direction) <= 1e-12) {
                            continue;
                        }
                        const double distance = (point - desired).squaredNorm();
                        if (distance < best_distance) {
                            best_distance = distance;
                            selected = point;
                            found_target = true;
                        }
                    }
                }
                if (parameters.use_topological_offset && !found_target) {
                    ++result.report.topological_target_fallbacks;
                }
                targets.emplace(key, selected);
            }
        }
        const auto scales = shrink_shell_layer(
            input.vertices,
            side_faces,
            targets,
            collision_faces,
            parameters,
            result.report);

        std::map<ShellVertexKey, size_t> offset_vertices;
        const auto offset_vertex = [&](const size_t source, const int side, const size_t sector) {
            const ShellVertexKey key = {source, side, sector};
            const auto found_vertex = offset_vertices.find(key);
            if (found_vertex != offset_vertices.end()) return found_vertex->second;
            const Vector3d p = input.vertices.row(source);
            const size_t id = append_vertex(vertices, p + scales.at(key) * (targets.at(key) - p));
            offset_vertices.emplace(key, id);
            movable_shell_vertices.emplace(id, source);
            return id;
        };

        for (const auto& candidate : side_faces) {
            const auto& f = candidate.face;
            const size_t a = offset_vertex(f[0], candidate.side, candidate.sectors[0]);
            const size_t b = offset_vertex(f[1], candidate.side, candidate.sectors[1]);
            const size_t c = offset_vertex(f[2], candidate.side, candidate.sectors[2]);
            const std::array<size_t, 6> prism =
                candidate.side > 0 ? std::array<size_t, 6>{{f[0], f[1], f[2], a, b, c}}
                                   : std::array<size_t, 6>{{f[0], f[2], f[1], a, c, b}};
            append_recovered_prism(
                result.mesh,
                vertices,
                prism,
                {{topology.marked_sources.count(f[0]) != 0,
                  topology.marked_sources.count(f[1]) != 0,
                  topology.marked_sources.count(f[2]) != 0}},
                "shell:" + shell.group,
                parameters,
                result.report);
        }
    }

    improve_shell_vertices(
        result.mesh,
        vertices,
        movable_shell_vertices,
        input.vertices,
        collision_faces,
        parameters,
        result.report);

    for (const auto& rod : parameters.rods) {
        if (shared_rod_groups.count(rod.group) != 0) continue;
        const auto found = input.edge_groups.find(rod.group);
        if (found == input.edge_groups.end() || found->second.empty()) {
            log_and_throw_error("Rod physical group '{}' is missing or empty.", rod.group);
        }
        std::map<size_t, size_t> valences;
        std::map<size_t, size_t> shared_joint_vertices;
        const auto chains =
            rod_chains(found->second, result.report.irregular_rod_vertices, valences);
        for (const auto& chain : chains) {
            append_rod_chain(
                result.mesh,
                vertices,
                input.vertices,
                chain,
                rod,
                valences,
                shared_joint_vertices,
                parameters,
                result.report);
        }
    }

    result.mesh.vertices.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) result.mesh.vertices.row(i) = vertices[i];

    if (result.mesh.vertices.rows() != 0) {
        const Vector3d output_minimum = result.mesh.vertices.colwise().minCoeff().transpose();
        const Vector3d output_maximum = result.mesh.vertices.colwise().maxCoeff().transpose();
        const double duplicate_epsilon =
            std::max(1e-10, 1e-8 * (output_maximum - output_minimum).norm());
        std::set<std::string> accepted_cells;
        result.report.duplicate_cells_removed += remove_duplicate_cells(
            result.mesh.tets,
            result.mesh.tet_region_tags,
            result.mesh.vertices,
            duplicate_epsilon,
            4,
            accepted_cells);
        result.report.duplicate_cells_removed += remove_duplicate_cells(
            result.mesh.pyramids,
            result.mesh.pyramid_region_tags,
            result.mesh.vertices,
            duplicate_epsilon,
            7,
            accepted_cells);
        result.report.duplicate_cells_removed += remove_duplicate_cells(
            result.mesh.prisms,
            result.mesh.prism_region_tags,
            result.mesh.vertices,
            duplicate_epsilon,
            6,
            accepted_cells);
    }
    if (result.report.duplicate_cells_removed != 0) {
        result.report.warnings.push_back(
            "Coincident duplicate output cells were removed after conservative recovery.");
    }

    const auto accumulate_amips =
        [&result](const std::vector<Vector3d>& all_vertices, std::array<size_t, 4> tet) {
            auto p = cell_points(all_vertices, tet);
            if (!is_valid_tet(p)) {
                std::swap(p[2], p[3]);
            }
            std::array<double, 12> coordinates;
            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < 3; ++j) coordinates[3 * i + j] = p[i][j];
            }
            const double energy = wmtk::AMIPS_energy(coordinates);
            if (std::isfinite(energy)) {
                result.report.max_amips = std::max(result.report.max_amips, energy);
            }
        };
    for (const auto& tet : result.mesh.tets) {
        accumulate_amips(vertices, tet);
    }
    for (const auto& p : result.mesh.prisms) {
        accumulate_amips(vertices, {{p[0], p[1], p[2], p[3]}});
        accumulate_amips(vertices, {{p[1], p[2], p[3], p[4]}});
        accumulate_amips(vertices, {{p[2], p[3], p[4], p[5]}});
    }
    for (const auto& p : result.mesh.pyramids) {
        accumulate_amips(vertices, {{p[0], p[1], p[2], p[4]}});
        accumulate_amips(vertices, {{p[0], p[2], p[3], p[4]}});
    }
    compact_output_vertices(result.mesh);
    return result;
}

} // namespace wmtk::components::prismatic_mesh
