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
        const double delta = snapped_phase - terminal_phase;
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
} // namespace wmtk::components::prismatic_mesh
