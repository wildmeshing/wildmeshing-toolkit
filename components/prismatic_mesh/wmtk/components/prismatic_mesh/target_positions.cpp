#include "prismatic_mesh.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <unordered_set>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
constexpr double direction_epsilon = 1e-8;
constexpr double rank_epsilon = 1e-10;
constexpr double geometry_epsilon = 1e-12;

bool is_input_face(const PrismaticMeshInput& input, const std::array<size_t, 3>& vertices)
{
    return std::all_of(vertices.begin(), vertices.end(), [&](size_t v) {
        return input.vertex_tags.at(v) == 1;
    });
}

// Direction from an input triangle into an incident band tet, independent of face winding.
bool normal_into_tet(
    const PrismaticMeshInput& input,
    const std::array<size_t, 3>& face,
    size_t tid,
    Vector3d& normal)
{
    const Vector3d a = input.vertices.row(face[0]);
    const Vector3d e1 = input.vertices.row(face[1]).transpose() - a;
    const Vector3d e2 = input.vertices.row(face[2]).transpose() - a;
    const double scale = std::max(e1.norm(), e2.norm());
    if (!(scale > 0) || !std::isfinite(scale)) return false;
    normal = (e1 / scale).cross(e2 / scale);
    const double length = normal.norm();
    if (!(length > geometry_epsilon) || !std::isfinite(length)) return false;
    normal /= length;
    for (size_t v : input.mesh->oriented_tet_vids(tid)) {
        if (std::find(face.begin(), face.end(), v) != face.end()) continue;
        const Vector3d towards_band = input.vertices.row(v).transpose() - a;
        const double side = normal.dot(towards_band / scale);
        if (!std::isfinite(side) || std::abs(side) <= geometry_epsilon) return false;
        if (side < 0) normal = -normal;
        return true;
    }
    return false;
}

void build_components(PrismaticMeshInput& input)
{
    const size_t n = input.vertices.rows();
    input.offset_components.clear();
    input.input_to_components.assign(n, {});
    input.vertex_component_ids.assign(n, -1);
    std::vector<std::vector<size_t>> adjacency(n);
    for (const auto& edge : input.mesh->get_edges()) {
        const size_t a = edge.vid(*input.mesh);
        const size_t b = edge.switch_vertex(*input.mesh).vid(*input.mesh);
        if (input.vertex_tags[a] != 2 || input.vertex_tags[b] != 2 ||
            input.corr_input_vid[a] != input.corr_input_vid[b])
            continue;
        adjacency[a].push_back(b);
        adjacency[b].push_back(a);
    }
    for (const auto& vertex : input.mesh->get_vertices()) {
        const size_t seed = vertex.vid(*input.mesh);
        if (input.vertex_tags[seed] != 2 || input.vertex_component_ids[seed] != -1) continue;
        const int64_t parent = input.corr_input_vertex.at(seed);
        if (parent < 0 || static_cast<size_t>(parent) >= n || input.vertex_tags[parent] != 1 ||
            input.source_vertex_ids[parent] != input.corr_input_vid[seed]) {
            log_and_throw_error("Invalid input correspondence for offset vertex {}", seed);
        }
        const size_t id = input.offset_components.size();
        OffsetComponent component;
        component.input_vertex = static_cast<size_t>(parent);
        component.vertices.push_back(seed);
        input.vertex_component_ids[seed] = static_cast<int64_t>(id);
        for (size_t i = 0; i < component.vertices.size(); ++i) {
            for (size_t next : adjacency[component.vertices[i]]) {
                if (input.vertex_component_ids[next] != -1) continue;
                input.vertex_component_ids[next] = static_cast<int64_t>(id);
                component.vertices.push_back(next);
            }
        }
        input.input_to_components[parent].push_back(id);
        input.offset_components.push_back(std::move(component));
    }
}

// Input faces separate sectors in an input vertex's tetrahedral one-ring. Gather the
// input face normals bounding each sector, directed into that sector, and attach them
// only to correspondence components touching that sector. This handles two-sided sheets
// without a centroid-based sign guess and never uses input vertices to connect components.
void gather_component_normals(
    const PrismaticMeshInput& input,
    std::vector<std::map<std::pair<size_t, size_t>, Vector3d>>& normals,
    std::vector<bool>& invalid_geometry)
{
    for (size_t p = 0; p < input.input_to_components.size(); ++p) {
        if (input.input_to_components[p].empty()) continue;
        std::unordered_set<size_t> visited;
        for (size_t seed : input.mesh->get_one_ring_tids_for_vertex(p)) {
            if (!visited.insert(seed).second) continue;
            std::vector<size_t> sector{seed};
            std::set<size_t> components;
            std::map<std::pair<size_t, size_t>, Vector3d> sector_normals;
            bool invalid = false;
            for (size_t i = 0; i < sector.size(); ++i) {
                const size_t tid = sector[i];
                for (size_t v : input.mesh->oriented_tet_vids(tid)) {
                    const int64_t cid = input.vertex_component_ids[v];
                    if (cid >= 0 && input.offset_components[cid].input_vertex == p) {
                        components.insert(static_cast<size_t>(cid));
                    }
                }
                for (int local = 0; local < 4; ++local) {
                    const auto face = input.mesh->tuple_from_face(tid, local);
                    const auto fv = input.mesh->get_face_vids(face);
                    if (std::find(fv.begin(), fv.end(), p) == fv.end()) continue;
                    if (is_input_face(input, fv)) {
                        Vector3d normal;
                        if (!normal_into_tet(input, fv, tid, normal))
                            invalid = true;
                        else
                            sector_normals.emplace(
                                std::make_pair(face.fid(*input.mesh), tid),
                                normal);
                        continue; // Do not cross the input sheet to the other side.
                    }
                    const auto neighbor = face.switch_tetrahedron(*input.mesh);
                    if (neighbor && visited.insert(neighbor->tid(*input.mesh)).second) {
                        sector.push_back(neighbor->tid(*input.mesh));
                    }
                }
            }
            for (size_t cid : components) {
                normals[cid].insert(sector_normals.begin(), sector_normals.end());
                invalid_geometry[cid] = invalid_geometry[cid] || invalid;
            }
        }
    }
}
} // namespace

bool solve_target_direction(const std::vector<Vector3d>& normals, Vector3d& direction)
{
    direction.setZero();
    if (normals.empty()) return false;
    std::vector<Vector3d> unit_normals;
    for (const auto& n : normals) {
        const double length = n.norm();
        if (!n.allFinite() || !(length > 0) || !std::isfinite(length)) return false;
        unit_normals.push_back(n / length);
    }
    // Restrict the normal equations to the span of the normals. This is the minimum-norm
    // LS solution and avoids treating a planar rank-one normal set as singular. The small
    // reduced normal equations are solved with dense full-pivoting LU, without damping.
    Eigen::Matrix3d basis = Eigen::Matrix3d::Zero();
    int rank = 0;
    while (rank < 3) {
        Vector3d best = Vector3d::Zero();
        for (const auto& n : unit_normals) {
            Vector3d residual = n;
            for (int pass = 0; pass < 2; ++pass) {
                for (int j = 0; j < rank; ++j)
                    residual -= basis.col(j).dot(residual) * basis.col(j);
            }
            if (residual.squaredNorm() > best.squaredNorm()) best = residual;
        }
        if (best.norm() <= rank_epsilon) break;
        basis.col(rank++) = best.normalized();
    }
    if (rank == 0) return false;
    MatrixXd lhs = MatrixXd::Zero(rank, rank);
    VectorXd rhs = VectorXd::Zero(rank);
    for (const auto& n : unit_normals) {
        const VectorXd row = basis.leftCols(rank).transpose() * n;
        lhs += row * row.transpose();
        rhs += row;
    }
    Eigen::FullPivLU<MatrixXd> lu(lhs);
    lu.setThreshold(1e-12);
    if (lu.rank() != rank) return false;
    const VectorXd x = lu.solve(rhs);
    Vector3d candidate = basis.leftCols(rank) * x;
    const double length = candidate.norm();
    if (!candidate.allFinite() || !(length > 1e-12) || !std::isfinite(length)) return false;
    candidate /= length;
    for (const auto& n : unit_normals) {
        if (n.dot(candidate) <= direction_epsilon) return false;
    }
    direction = candidate;
    return true;
}

void evaluate_target_positions(PrismaticMeshInput& input, double thicknessratio)
{
    if (!std::isfinite(thicknessratio) || thicknessratio <= 0) {
        log_and_throw_error("thicknessratio must be finite and positive.");
    }
    build_components(input);
    input.singular_vertex_tags.assign(input.vertices.rows(), -1);
    input.optimal_normals = MatrixXd::Zero(input.vertices.rows(), 3);
    input.target_positions = input.vertices;
    std::set<std::array<size_t, 2>> edges;
    for (const auto& face : input.mesh->get_faces()) {
        const auto fv = input.mesh->get_face_vids(face);
        if (!is_input_face(input, fv)) continue;
        for (size_t j = 0; j < 3; ++j) {
            std::array<size_t, 2> edge{fv[j], fv[(j + 1) % 3]};
            std::sort(edge.begin(), edge.end());
            edges.insert(edge);
        }
    }
    long double length_sum = 0;
    for (const auto& edge : edges) {
        length_sum += (input.vertices.row(edge[0]) - input.vertices.row(edge[1])).norm();
    }
    input.input_average_edge_length =
        edges.empty() ? 0 : static_cast<double>(length_sum / edges.size());
    input.target_thickness = thicknessratio * input.input_average_edge_length;
    if (!std::isfinite(input.target_thickness))
        log_and_throw_error("Target thickness is not finite.");
    std::vector<std::map<std::pair<size_t, size_t>, Vector3d>> normal_maps(
        input.offset_components.size());
    std::vector<bool> invalid_geometry(input.offset_components.size(), false);
    gather_component_normals(input, normal_maps, invalid_geometry);
    size_t singular_count = 0;
    for (size_t cid = 0; cid < input.offset_components.size(); ++cid) {
        auto& component = input.offset_components[cid];
        const Vector3d p = input.vertices.row(component.input_vertex);
        component.target_position = p; // invalid placeholder until a direction is found
        std::vector<Vector3d> normals;
        for (const auto& entry : normal_maps[cid]) normals.push_back(entry.second);
        if (!(input.target_thickness > 0))
            component.singular_reason = "no_input_surface_edges";
        else if (invalid_geometry[cid])
            component.singular_reason = "degenerate_input_face_or_tet";
        else if (normals.empty())
            component.singular_reason = "no_adjacent_input_faces";
        else if (!solve_target_direction(normals, component.optimal_normal)) {
            component.singular_reason = "no_valid_least_squares_direction";
        } else {
            component.target_position = p + input.target_thickness * component.optimal_normal;
            if (component.target_position.allFinite())
                component.singular = false;
            else {
                component.optimal_normal.setZero();
                component.target_position = p;
                component.singular_reason = "non_finite_target_position";
            }
        }
        if (component.singular) ++singular_count;
        for (size_t v : component.vertices) {
            input.singular_vertex_tags[v] = component.singular ? 1 : 0;
            if (!component.singular) {
                input.optimal_normals.row(v) = component.optimal_normal.transpose();
                input.target_positions.row(v) = component.target_position.transpose();
            }
        }
    }
    logger().info(
        "Target positions: {} components, {} valid, {} singular; input average edge length {}, "
        "thickness {}",
        input.offset_components.size(),
        input.offset_components.size() - singular_count,
        singular_count,
        input.input_average_edge_length,
        input.target_thickness);
}
} // namespace wmtk::components::prismatic_mesh
