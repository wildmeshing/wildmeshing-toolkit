#include "rod_mesh.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
constexpr double pi = 3.14159265358979323846;

Vector3d position(const PrismaticMeshInput& input, size_t v)
{
    return input.vertices.row(v).transpose();
}

// The centerline meets the target plane at its center. Project a radial ray
// axially onto that plane; this is the radial part of the line/plane hit and
// avoids subtracting nearly equal world positions. Its scale is immaterial.
std::optional<Vector3d>
ray_map(const Vector3d& radial, const Vector3d& axis, const Vector3d& target_normal)
{
    const double denominator = target_normal.dot(axis);
    if (!std::isfinite(denominator) || std::abs(denominator) <= 1e-12) return {};
    const Vector3d hit = radial - axis * (target_normal.dot(radial) / denominator);
    const double length = hit.norm();
    if (!hit.allFinite() || !(length > 1e-12)) return {};
    return Vector3d(hit / length);
}

double phase(const Eigen::Quaterniond& basis, const Vector3d& radial)
{
    return std::atan2(radial.dot(basis * Vector3d::UnitY()), radial.dot(basis * Vector3d::UnitX()));
}

nlohmann::json vector_json(const Vector3d& p)
{
    return {p.x(), p.y(), p.z()};
}
} // namespace

void transport_rod_frame_mapping(PrismaticMeshInput& input)
{
    if (!input.rod_state) return;
    auto& state = *input.rod_state;
    if (state.edge_frames.size() != state.directed_input_edges.size())
        throw std::runtime_error("rod mapping: incomplete edge frame field");
    average_rod_vertex_frames(state);
    const auto initial_edges = state.edge_frames;
    const auto initial_vertices = state.vertex_rotations;
    auto paths = nlohmann::json::array();
    size_t completed = 0, skipped = 0;
    double maximum_change = 0, maximum_grid_phase_change = 0;
    for (size_t rod = 0; rod < state.rods.size(); ++rod) {
        const auto& path = state.rods[rod];
        nlohmann::json report = {{"rod", rod}, {"completed", false}};
        if (path.empty()) {
            report["reason"] = "empty_path";
            paths.push_back(std::move(report));
            ++skipped;
            continue;
        }
        const auto& first_axis = state.directed_input_edges.at(path.front());
        const auto& last_axis = state.directed_input_edges.at(path.back());
        const bool closed = first_axis[0] == last_axis[1];
        report["closed"] = closed;
        size_t degree = 0;
        for (size_t e : path) degree = std::max(degree, initial_edges.at(e).degree);
        if (degree < 3 || std::any_of(
                              path.begin(),
                              path.end(),
                              [&](size_t e) {
                                  return !initial_edges.at(e).from_link ||
                                         initial_edges[e].degree != degree;
                              })) {
            report["reason"] = "incompatible_frame_degrees";
            paths.push_back(std::move(report));
            ++skipped;
            continue;
        }
        std::vector<double> lengths, arc{0};
        std::vector<Vector3d> tangents;
        bool valid = true;
        for (size_t k = 0; k < path.size(); ++k) {
            const auto& e = state.directed_input_edges.at(path[k]);
            const Vector3d d = position(input, e[1]) - position(input, e[0]);
            const double length = d.norm();
            if (!(length > 0) || !std::isfinite(length) ||
                (k && state.directed_input_edges[path[k - 1]][1] != e[0])) {
                valid = false;
                break;
            }
            lengths.push_back(length);
            tangents.push_back(d / length);
            if (k) arc.push_back(arc.back() + .5 * (lengths[k - 1] + length));
        }
        if (!valid) {
            report["reason"] = "invalid_directed_path";
            paths.push_back(std::move(report));
            ++skipped;
            continue;
        }
        // Stations are the edge midpoints. The intermediate vertex is reached
        // after half an edge; it is not an extra centerline segment.
        const double total = arc.back() + (closed ? .5 * (lengths.back() + lengths.front()) : 0);
        std::vector<Vector3d> rays{initial_edges[path.front()].rotation * Vector3d::UnitX()};
        std::vector<double> mapped_phase{0};
        auto transitions = nlohmann::json::array();
        const size_t count = path.size() - 1 + (closed ? 1 : 0);
        for (size_t k = 0; k < count; ++k) {
            const size_t next = (k + 1) % path.size();
            const size_t vertex = state.directed_input_edges[path[k]][1];
            const Vector3d normal = initial_vertices.at(vertex) * Vector3d::UnitZ();
            const auto at_vertex = ray_map(rays.back(), tangents[k], normal);
            const auto at_edge = at_vertex ? ray_map(*at_vertex, tangents[next], tangents[next])
                                           : std::optional<Vector3d>{};
            if (!at_edge) {
                valid = false;
                report["reason"] = "parallel_or_degenerate_projection";
                report["blocked_vertex_vid"] = input.source_vertex_ids.at(vertex);
                break;
            }
            double angle = phase(initial_edges[path[next]].rotation, *at_edge);
            angle = mapped_phase.back() + std::remainder(angle - mapped_phase.back(), 2 * pi);
            mapped_phase.push_back(angle);
            rays.push_back(*at_edge);
            transitions.push_back(
                {{"vertex_vid", input.source_vertex_ids.at(vertex)},
                 {"arc_length", arc[k] + .5 * lengths[k]},
                 {"vertex_normal", vector_json(normal)},
                 {"vertex_ray", vector_json(*at_vertex)},
                 {"next_edge_ray", vector_json(*at_edge)}});
        }
        if (!valid) {
            report["transitions"] = std::move(transitions);
            paths.push_back(std::move(report));
            ++skipped;
            continue;
        }
        const double step = 2 * pi / degree;
        const double terminal_phase = mapped_phase.back();
        const auto terminal_index = static_cast<int64_t>(std::llround(terminal_phase / step));
        const double snapped_phase = terminal_index * step;
        const double delta = std::remainder(snapped_phase - terminal_phase, 2 * pi);
        auto frames = nlohmann::json::array();
        double energy = 0, maximum_density_error = 0;
        for (size_t k = 0; k < path.size(); ++k) {
            const size_t e = path[k];
            const double correction = total > 0 ? arc[k] * delta / total : 0;
            Vector3d x = Eigen::AngleAxisd(correction, tangents[k]) * rays[k];
            x -= tangents[k] * tangents[k].dot(x);
            x.normalize();
            Eigen::Matrix3d basis;
            basis.col(0) = x;
            basis.col(1) = tangents[k].cross(x);
            basis.col(2) = tangents[k];
            Eigen::Quaterniond rotation(basis);
            rotation.normalize();
            const auto& previous =
                k ? state.edge_frames[path[k - 1]].rotation : initial_edges[e].rotation;
            if (rotation.dot(previous) < 0) rotation.coeffs() *= -1;
            state.edge_frames[e].rotation = rotation;
            const double change = rotation.angularDistance(initial_edges[e].rotation);
            const double grid_change = std::remainder(mapped_phase[k] + correction, step);
            maximum_change = std::max(maximum_change, change);
            maximum_grid_phase_change = std::max(maximum_grid_phase_change, std::abs(grid_change));
            frames.push_back(
                {{"edge_index", e},
                 {"arc_length", arc[k]},
                 {"mapped_phase_radians", mapped_phase[k]},
                 {"correction_radians", correction},
                 {"phase_change_mod_degree_radians", grid_change},
                 {"rotation_change_radians", change}});
        }
        // min sum (alpha[i+1]-alpha[i])^2 / ds[i], with endpoint alpha={0,delta}.
        // Include the ghost closing station for a loop, without overwriting its
        // shared first frame. The terminal degree shift records that permutation.
        for (size_t k = 1; k < rays.size(); ++k) {
            const double a = k < arc.size() ? arc[k] : total;
            const double b = arc[k - 1];
            const double difference = (a - b) * delta / total;
            energy += difference * difference / (a - b);
            maximum_density_error =
                std::max(maximum_density_error, std::abs(difference / (a - b) - delta / total));
        }
        const size_t last = closed ? 0 : path.size() - 1;
        const Vector3d corrected_terminal = Eigen::AngleAxisd(delta, tangents[last]) * rays.back();
        const double closure_error = std::abs(
            std::remainder(
                phase(initial_edges[path[last]].rotation, corrected_terminal) - snapped_phase,
                2 * pi));
        report.update(
            {{"completed", true},
             {"degree", degree},
             {"station_rule", "edge_midpoints_with_intermediate_vertex_planes"},
             {"span_length", total},
             {"terminal_phase_radians", terminal_phase},
             {"terminal_degree_shift",
              (terminal_index % static_cast<int64_t>(degree) + static_cast<int64_t>(degree)) %
                  static_cast<int64_t>(degree)},
             {"delta_theta_radians", delta},
             {"closure_error_radians", closure_error},
             {"weighted_twist_energy", energy},
             {"maximum_twist_density_error", maximum_density_error},
             {"frames", std::move(frames)},
             {"transitions", std::move(transitions)}});
        paths.push_back(std::move(report));
        ++completed;
    }
    average_rod_vertex_frames(state);
    state.report["frame_mapping"] = {
        {"method", "raycast_via_vertex_mean_then_length_weighted_terminal_closure"},
        {"completed_paths", completed},
        {"skipped_paths", skipped},
        {"maximum_edge_rotation_change_radians", maximum_change},
        {"maximum_phase_change_mod_degree_radians", maximum_grid_phase_change},
        {"paths", std::move(paths)}};
    logger().info(
        "Rod frame mapping: {} paths completed, {} skipped; maximum edge rotation change {} rad",
        completed,
        skipped,
        maximum_change);
}
void transport_rod_section_mapping(PrismaticMeshInput& input)
{
    if (!input.rod_state) return;
    auto& state = *input.rod_state;
    state.mapping_controls.clear();
    state.layer_slot_shifts.clear();
    const auto is_control = [&](size_t v) {
        return state.vertex_frame.count(v) &&
               (state.section_edges.count(v) || state.free_endpoints.count(v));
    };
    const auto fitted_pose = [&](size_t v, const Vector3d& normal) {
        const auto& f = state.frames.at(state.vertex_frame.at(v));
        const Eigen::Quaterniond basis =
            Eigen::Quaterniond::FromTwoVectors(f.rotation * Vector3d::UnitZ(), normal) * f.rotation;
        const Vector3d x = basis * Vector3d::UnitX(), y = basis * Vector3d::UnitY();
        double re = 0, im = 0;
        for (size_t k = 0; k < f.ring.size(); ++k) {
            const Vector3d d = position(input, f.ring[k]) - position(input, v);
            const double a = std::atan2(d.dot(y), d.dot(x)) - 2 * pi * k / f.ring.size();
            re += std::cos(a);
            im += std::sin(a);
        }
        // Continuous least-squares pose of the fixed endpoint labels. This
        // neither snaps an angle nor selects another cyclic vertex origin.
        return Eigen::Quaterniond(basis * Eigen::AngleAxisd(std::atan2(im, re), Vector3d::UnitZ()));
    };
    auto paths = nlohmann::json::array();
    size_t completed = 0, skipped = 0;
    for (size_t rod = 0; rod < state.rods.size(); ++rod) {
        const auto& path = state.rods[rod];
        if (path.empty()) continue;
        std::vector<size_t> vertices{state.directed_input_edges[path.front()][0]};
        for (size_t e : path) vertices.push_back(state.directed_input_edges[e][1]);
        const bool closed = vertices.front() == vertices.back();
        auto report = nlohmann::json{{"rod", rod}, {"closed", closed}, {"completed", false}};
        const auto fail = [&](const std::string& reason) {
            report["reason"] = reason;
            paths.push_back(report);
            ++skipped;
            for (size_t v : vertices) {
                const auto found = state.vertex_frame.find(v);
                if (found == state.vertex_frame.end()) continue;
                auto& frame = state.frames[found->second];
                frame.valid = false;
                frame.reason = "material_mapping_" + reason;
                state.vertex_frame.erase(found);
            }
        };
        if (closed) vertices.pop_back();
        const auto first = std::find_if(vertices.begin(), vertices.end(), is_control);
        if (first == vertices.end()) {
            fail("no_section_controls");
            continue;
        }
        if (closed) {
            std::rotate(vertices.begin(), first, vertices.end());
            vertices.push_back(vertices.front());
        } else {
            vertices.erase(vertices.begin(), first);
            const auto last = std::find_if(vertices.rbegin(), vertices.rend(), is_control).base();
            vertices.erase(last, vertices.end());
        }
        const size_t start = vertices.front(), end = vertices.back();
        const size_t degree = state.frames.at(state.vertex_frame.at(start)).ring.size();
        bool valid = degree >= 3;
        for (size_t v : vertices)
            if (state.vertex_frame.count(v))
                valid &= state.frames[state.vertex_frame.at(v)].ring.size() == degree;
        if (!valid) {
            fail("incompatible_section_degrees");
            continue;
        }
        std::vector<double> arc{0};
        std::vector<Vector3d> tangents, normals;
        for (size_t k = 1; k < vertices.size(); ++k) {
            const Vector3d edge = position(input, vertices[k]) - position(input, vertices[k - 1]);
            const double length = edge.norm();
            if (!(length > 0)) {
                valid = false;
                break;
            }
            arc.push_back(arc.back() + length);
            tangents.push_back(edge / length);
        }
        if (valid && tangents.empty()) {
            // A path between two joints can contain just one section control.
            normals.push_back(
                state.frames[state.vertex_frame.at(start)].rotation * Vector3d::UnitZ());
        } else if (valid)
            for (size_t k = 0; k < vertices.size(); ++k) {
                Vector3d normal;
                if (closed && (k == 0 || k + 1 == vertices.size()))
                    normal = tangents.back() + tangents.front();
                else if (k == 0)
                    normal = tangents.front();
                else if (k + 1 == vertices.size())
                    normal = tangents.back();
                else
                    normal = tangents[k - 1] + tangents[k];
                if (!normal.allFinite() || normal.norm() <= 1e-12) {
                    valid = false;
                    break;
                }
                normals.push_back(normal.normalized());
            }
        if (!valid) {
            fail("undefined_geometric_section_plane");
            continue;
        }
        const auto first_pose = fitted_pose(start, normals.front()),
                   last_pose = closed ? first_pose : fitted_pose(end, normals.back());
        std::vector<Vector3d> rays{first_pose * Vector3d::UnitX()};
        for (size_t k = 1; k < vertices.size(); ++k) {
            const auto ray = ray_map(rays.back(), tangents[k - 1], normals[k]);
            if (!ray) {
                valid = false;
                break;
            }
            rays.push_back(*ray);
        }
        if (!valid) {
            fail("parallel_or_degenerate_projection");
            continue;
        }
        const double total = arc.back(), step = 2 * pi / degree;
        const double terminal_phase = phase(last_pose, rays.back());
        const int64_t rounded = std::llround(terminal_phase / step);
        const double delta = std::remainder(rounded * step - terminal_phase, 2 * pi);
        const size_t shift =
            size_t((rounded % int64_t(degree) + int64_t(degree)) % int64_t(degree));
        const auto end_before = state.frames[state.vertex_frame.at(end)].ring;
        if (!closed && end != start) {
            auto& ring = state.frames[state.vertex_frame.at(end)].ring;
            std::rotate(ring.begin(), ring.begin() + shift, ring.end());
        } else if (closed && vertices.size() > 1) {
            const size_t u = vertices[vertices.size() - 2], v = vertices.front();
            std::array<size_t, 2> axis{u, v};
            std::sort(axis.begin(), axis.end());
            state.layer_slot_shifts[axis] = u < v ? shift : (degree - shift) % degree;
        }
        auto stations = nlohmann::json::array();
        auto relabelings = nlohmann::json::array();
        const size_t count = vertices.size() - (closed ? 1 : 0);
        for (size_t k = 0; k < count; ++k) {
            const size_t v = vertices[k];
            const double correction = total > 0 ? arc[k] * delta / total : 0;
            Vector3d x = Eigen::AngleAxisd(correction, normals[k]) * rays[k];
            x -= normals[k] * normals[k].dot(x);
            x.normalize();
            Eigen::Matrix3d basis;
            basis.col(0) = x;
            basis.col(1) = normals[k].cross(x);
            basis.col(2) = normals[k];
            const Eigen::Quaterniond q(basis);
            state.vertex_rotations[v] = q;
            if (state.vertex_frame.count(v)) {
                auto& f = state.frames[state.vertex_frame.at(v)];
                f.rotation = q;
                f.tangent = normals[k];
                // A cyclic ring origin is only an index convention. Choose it
                // against the completed continuous field before locking the
                // mapping; do not physically rotate vertices by a whole sector
                // merely to preserve an arbitrary pre-transport vertex zero.
                if (v != start && (closed || v != end) && f.valid) {
                    const auto cost = [&](size_t shift) {
                        double sum = 0;
                        for (size_t j = 0; j < f.ring.size(); ++j) {
                            const double angle = 2 * pi * j / f.ring.size();
                            const Vector3d target =
                                position(input, v) +
                                state.radius * (q * Vector3d(std::cos(angle), std::sin(angle), 0));
                            sum += (position(input, f.ring[(j + shift) % f.ring.size()]) - target)
                                       .squaredNorm();
                        }
                        return sum;
                    };
                    size_t best_shift = 0;
                    const double before_cost = cost(0);
                    double best_cost = before_cost;
                    for (size_t shift = 1; shift < f.ring.size(); ++shift) {
                        const double candidate = cost(shift);
                        if (candidate < best_cost) {
                            best_shift = shift;
                            best_cost = candidate;
                        }
                    }
                    if (best_shift) {
                        relabelings.push_back(
                            {{"center_vid", input.source_vertex_ids[v]},
                             {"cyclic_shift", best_shift},
                             {"target_squared_error_before", before_cost},
                             {"target_squared_error_after", best_cost}});
                        std::rotate(f.ring.begin(), f.ring.begin() + best_shift, f.ring.end());
                    }
                }
            }
            if (is_control(v)) state.mapping_controls.insert(v);
            stations.push_back(
                {{"center_vid", input.source_vertex_ids[v]},
                 {"arc_length", arc[k]},
                 {"correction_radians", correction},
                 {"control", is_control(v)}});
        }
        const Vector3d terminal = Eigen::AngleAxisd(delta, normals.back()) * rays.back();
        report.update(
            {{"completed", true},
             {"degree", degree},
             {"span_length", total},
             {"terminal_phase_radians", terminal_phase},
             {"terminal_degree_shift", shift},
             {"delta_theta_radians", delta},
             {"twist_density", total > 0 ? delta / total : 0},
             {"closure_error_radians",
              std::abs(std::remainder(phase(last_pose, terminal) - rounded * step, 2 * pi))},
             {"frames", stations},
             {"intermediate_cyclic_origins_changed", relabelings.size()},
             {"intermediate_relabelings", relabelings},
             {"per_frame_rounding", false},
             {"DP_pairing", false}});
        if (!closed) {
            const auto ids = [&](const auto& ring) {
                std::vector<int64_t> result;
                for (size_t v : ring) result.push_back(input.source_vertex_ids[v]);
                return result;
            };
            report["terminal_ring_before"] = ids(end_before);
            report["terminal_ring_after"] = ids(state.frames[state.vertex_frame.at(end)].ring);
        }
        paths.push_back(std::move(report));
        ++completed;
    }
    std::vector<int64_t> control_ids;
    for (size_t v : state.mapping_controls) control_ids.push_back(input.source_vertex_ids[v]);
    state.report["material_mapping"] = {
        {"method", "continuous_ray_transport_terminal_rounding_arc_length_twist"},
        {"completed_paths", completed},
        {"skipped_paths", skipped},
        {"paths", paths},
        {"control_center_vids", control_ids},
        {"section_planes", "edge_normal_at_midpoints_and_geometric_tangent_bisector_at_corners"},
        {"cyclic_origins_assigned_after_transport", true},
        {"per_frame_rounding", false},
        {"DP_pairing", false}};
    state.report["vertex_frame_rule"] = "continuous_transported_material_direction";
    state.report["frame_mapping"] = state.report["material_mapping"];
    state.mapping_locked = true;
    logger().info(
        "Rod material mapping: {} paths completed, {} skipped; {} section controls; terminal "
        "rounding only",
        completed,
        skipped,
        state.mapping_controls.size());
}
} // namespace wmtk::components::prismatic_mesh
