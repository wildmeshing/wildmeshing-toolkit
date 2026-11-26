#include "junction_protection.hpp"
#include "prism_jacobian.hpp"
#include "rod_fairing.hpp"
#include "rod_joint_radius.hpp"
#include "rod_mesh.hpp"

#include <wmtk/utils/AMIPS.h>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tet = std::array<size_t, 4>;
using Tri = std::array<size_t, 3>;
constexpr double pi = 3.14159265358979323846;
struct LaplaceRow
{
    std::vector<size_t> vertices;
    std::vector<double> coefficients;
    Vector3d residual(const MatrixXd& positions, double radius) const
    {
        Vector3d value = Vector3d::Zero();
        const Vector3d origin = positions.row(vertices.front());
        for (size_t j = 1; j < vertices.size(); ++j)
            value += coefficients[j] * (positions.row(vertices[j]).transpose() - origin);
        return value / radius;
    }
};

Vector3d point(const PrismaticMeshInput& input, size_t v)
{
    return input.vertices.row(v).transpose();
}

// Put the free vertex first using an even permutation. Work in radius units
// and translate before evaluating the generated AMIPS derivatives.
std::array<double, 12> local_cell(
    const PrismaticMeshInput& input,
    Tet tet,
    size_t v,
    const Vector3d& origin,
    double radius)
{
    const auto found = std::find(tet.begin(), tet.end(), v);
    if (found != tet.end() && found != tet.begin()) {
        std::iter_swap(tet.begin(), found);
        std::swap(tet[1], tet[2]);
    }
    std::array<double, 12> data;
    for (size_t j = 0; j < 4; ++j)
        for (size_t k = 0; k < 3; ++k)
            data[3 * j + k] = (input.vertices(tet[j], k) - origin[k]) / radius;
    return data;
}

nlohmann::json candidate_geometry(const PrismaticMeshInput& input)
{
    const auto& state = *input.rod_state;
    size_t eligible = 0, invalid = 0, bad_sectors = 0, unequal = 0;
    for (const auto& axis : state.input_edges) {
        if (!state.vertex_frame.count(axis[0]) || !state.vertex_frame.count(axis[1])) continue;
        const auto [lo, hi] = rod_layer_rings(input, axis);
        if (lo.size() != hi.size()) {
            ++unequal;
            continue;
        }
        ++eligible;
        size_t bad = 0;
        for (size_t k = 0; k < lo.size(); ++k) {
            HybridCell prism;
            prism.type = HybridCellType::Prism;
            prism.vertices =
                {axis[0], lo[k], lo[(k + 1) % lo.size()], axis[1], hi[k], hi[(k + 1) % hi.size()]};
            if (!(minimum_prism_jacobian(input.vertices, prism).determinant > 0)) {
                std::swap(prism.vertices[1], prism.vertices[2]);
                std::swap(prism.vertices[4], prism.vertices[5]);
            }
            bad += !(minimum_prism_jacobian(input.vertices, prism).determinant > 0);
        }
        invalid += bad > 0;
        bad_sectors += bad;
    }
    return {
        {"eligible_layers", eligible},
        {"unequal_degree_layers", unequal},
        {"nonpositive_prism_layers", invalid},
        {"nonpositive_prism_sectors", bad_sectors}};
}
} // namespace

static void optimize_rod_positions_impl(
    PrismaticMeshInput& input,
    bool surface_pass,
    bool joint_pass = false,
    bool coupled_pass = false)
{
    if (!input.rod_state || !input.rod_state->options.edge_based) return;
    const auto start_time = std::chrono::steady_clock::now();
    auto& state = *input.rod_state;
    if (!state.mapping_locked)
        throw std::runtime_error("rod position optimization requires a fixed material mapping");
    const auto& options = state.options;
    const double radius = state.radius;
    const MatrixXd original_positions = input.vertices;
    const MatrixXi original_tets = input.tetrahedra;
    std::vector<std::vector<size_t>> mapping;
    for (const auto& frame : state.frames) mapping.push_back(frame.ring);
    std::vector<Tet> cells(input.tetrahedra.rows());
    std::vector<std::vector<size_t>> incident(input.vertices.rows());
    std::set<size_t> rod_vertices;
    std::map<Tri, size_t> front_faces;
    for (size_t tid = 0; tid < cells.size(); ++tid) {
        auto& tet = cells[tid];
        for (size_t j = 0; j < 4; ++j) {
            tet[j] = input.tetrahedra(tid, j);
            incident[tet[j]].push_back(tid);
            if (is_rod_band_tet(input, tid)) rod_vertices.insert(tet[j]);
        }
        if (!is_rod_band_tet(input, tid)) continue;
        for (size_t omit = 0; omit < 4; ++omit) {
            Tri face;
            size_t n = 0;
            for (size_t j = 0; j < 4; ++j)
                if (j != omit) face[n++] = tet[j];
            if (std::all_of(face.begin(), face.end(), [&](size_t v) {
                    return input.vertex_tags[v] == 2;
                })) {
                std::sort(face.begin(), face.end());
                ++front_faces[face];
            }
        }
    }
    std::vector<size_t> movable;
    std::set<size_t> active_cells;
    std::set<size_t> joint_offsets;
    if (joint_pass || coupled_pass)
        for (size_t tid = 0; tid < cells.size(); ++tid) {
            if (!is_rod_band_tet(input, tid) ||
                std::none_of(cells[tid].begin(), cells[tid].end(), [&](size_t v) {
                    return state.joints.count(v);
                }))
                continue;
            for (size_t v : cells[tid])
                if (input.vertex_tags[v] == 2) joint_offsets.insert(v);
        }
    for (size_t v : joint_offsets) state.joint_position_reference.try_emplace(v, point(input, v));
    for (size_t v : rod_vertices) {
        if (input.vertex_tags[v] == 1) continue;
        if (state.fixed_shell_vertices.count(v)) continue;
        const bool protected_point =
            state.frozen_offset.count(v) || !junction_collapse_allowed(input, v);
        if (joint_pass ? !joint_offsets.count(v)
                       : (protected_point && !(coupled_pass && joint_offsets.count(v))))
            continue;
        movable.push_back(v);
        active_cells.insert(incident[v].begin(), incident[v].end());
    }
    const size_t movable_rod_count = movable.size();
    RodBackgroundNeighborhood background_region;
    if (coupled_pass) {
        background_region = rod_background_neighborhood(input);
        for (size_t v : background_region.movable_vertices) {
            movable.push_back(v);
            active_cells.insert(incident[v].begin(), incident[v].end());
        }
    } else if (surface_pass || state.mapping_locked) {
        std::set<size_t> outer, nearby_background;
        for (const auto& face : input.mesh->get_faces())
            if (face.is_boundary_face(*input.mesh))
                for (const auto& v : input.mesh->get_face_vertices(face))
                    outer.insert(v.vid(*input.mesh));
        for (size_t v : movable)
            for (size_t tid : incident[v])
                for (size_t u : cells[tid])
                    if (input.vertex_tags[u] == -1 && !outer.count(u) &&
                        !state.fixed_shell_vertices.count(u) &&
                        std::all_of(incident[u].begin(), incident[u].end(), [&](size_t t) {
                            return input.input_cells[t] != 1 && input.offset_tet_tags[t] != 1;
                        }))
                        nearby_background.insert(u);
        movable.insert(movable.end(), nearby_background.begin(), nearby_background.end());
        for (size_t v : nearby_background)
            active_cells.insert(incident[v].begin(), incident[v].end());
    }
    std::map<size_t, Vector3d> targets;
    if (state.mapping_locked) state.frame_targets.clear();
    std::set<size_t> frame_vertices;
    std::map<size_t, std::vector<std::array<size_t, 2>>> carriers;
    for (const auto& edge : state.input_edges)
        for (size_t v : edge) carriers[v].push_back(edge);
    const auto carrier_foot = [&](size_t parent, const Vector3d& p) {
        Vector3d best = point(input, parent);
        double distance = std::numeric_limits<double>::infinity();
        for (const auto& edge : carriers.at(parent)) {
            const Vector3d a = point(input, edge[0]), d = point(input, edge[1]) - a;
            const Vector3d foot = a + std::clamp((p - a).dot(d) / d.squaredNorm(), 0., 1.) * d;
            if ((p - foot).squaredNorm() < distance) {
                best = foot;
                distance = (p - foot).squaredNorm();
            }
        }
        return best;
    };
    // Every valid ring can be a native prism cap, including transition rings.
    // Give every radial edge its material-slot target, not only the source
    // section controls. These poses already contain the single terminal
    // rounding and distributed twist; never refit their phase to the mesh.
    size_t control_target_vertices = 0, transition_target_vertices = 0;
    for (const auto& frame : state.frames) {
        if (!frame.valid) continue;
        const Vector3d x = frame.rotation * Vector3d::UnitX(),
                       y = frame.rotation * Vector3d::UnitY();
        (state.mapping_controls.count(frame.center) ? control_target_vertices
                                                    : transition_target_vertices) +=
            frame.ring.size();
        for (size_t k = 0; k < frame.ring.size(); ++k) {
            const double angle = 2 * pi * k / frame.ring.size();
            targets[frame.ring[k]] =
                point(input, frame.center) + radius * (std::cos(angle) * x + std::sin(angle) * y);
            frame_vertices.insert(frame.ring[k]);
            state.frame_targets[frame.ring[k]] = targets[frame.ring[k]];
        }
    }
    // Front points outside reconstruction rings have a carrier-radius penalty;
    // Laplace supplies their motion without introducing another frame fit.
    for (const auto& [face, count] : front_faces) {
        if (count != 1) continue;
        for (size_t v : face) {
            if (targets.count(v)) continue;
            const int64_t parent = input.corr_input_vertex[v];
            if (parent < 0 || !state.vertex_rotations.count(parent)) continue;
            const Vector3d foot = carrier_foot(parent, point(input, v));
            const Vector3d d = point(input, v) - foot;
            if (d.norm() > 0) targets[v] = foot + radius * d.normalized();
        }
    }
    size_t targeted = 0;
    state.report["rod_target_vertices"] = frame_vertices.size();
    for (size_t v : movable) targeted += targets.count(v);
    for (const auto& [v, target] : targets)
        if (input.target_positions.rows() == input.vertices.rows())
            input.target_positions.row(v) = target.transpose();
    std::vector<double> volume_floors(cells.size(), 0);
    for (size_t tid : active_cells) {
        if (input.offset_tet_tags[tid] == 1) volume_floors[tid] = options.min_tet_volume;
        if (input.relative_tet_volume_floor == 0) continue;
        Eigen::Matrix<long double, 3, 3> edges;
        for (int j = 0; j < 3; ++j)
            for (int k = 0; k < 3; ++k)
                edges(j, k) = static_cast<long double>(input.vertices(cells[tid][j + 1], k)) -
                              input.vertices(cells[tid][0], k);
        volume_floors[tid] = std::max(
            volume_floors[tid],
            static_cast<double>(input.relative_tet_volume_floor * edges.determinant() / 6));
    }
    const double coefficient =
        targeted ? options.position_weight * active_cells.size() / targeted : 0;
    if (!std::isfinite(coefficient))
        throw std::runtime_error("rod position optimization: nonfinite geometry coefficient");
    std::vector<RodFairingRow> fairing;
    std::vector<std::vector<std::pair<size_t, size_t>>> fairing_incident(input.vertices.rows());
    const std::set<size_t> free_vertices(movable.begin(), movable.end());
    if (options.position_smoothing_weight > 0)
        for (const auto& row : rod_fairing_rows(input)) {
            if (std::none_of(row.vertices.begin(), row.vertices.end(), [&](size_t v) {
                    return free_vertices.count(v);
                }))
                continue;
            for (size_t j = 0; j < 3; ++j)
                fairing_incident[row.vertices[j]].push_back({fairing.size(), j});
            fairing.push_back(row);
        }
    const double fairing_coefficient =
        fairing.empty() ? 0
                        : options.position_smoothing_weight * active_cells.size() / fairing.size();
    if (!std::isfinite(fairing_coefficient))
        throw std::runtime_error("rod position optimization: nonfinite fairing coefficient");
    std::vector<LaplaceRow> laplace;
    std::vector<std::vector<std::pair<size_t, double>>> laplace_incident(input.vertices.rows());
    std::map<size_t, std::set<size_t>> surface_neighbors;
    for (const auto& [face, count] : front_faces)
        if (count == 1)
            for (size_t a : face)
                for (size_t b : face)
                    if (a != b) surface_neighbors[a].insert(b);
    if (state.mapping_locked && options.laplace_weight > 0)
        for (size_t v : movable) {
            if (!rod_vertices.count(v) || (!joint_pass && !coupled_pass && frame_vertices.count(v)))
                continue;
            std::set<size_t> neighbors = surface_neighbors[v];
            if (neighbors.empty())
                for (size_t tid : incident[v])
                    for (size_t u : cells[tid])
                        if (u != v) neighbors.insert(u);
            if (neighbors.empty()) continue;
            LaplaceRow row;
            row.vertices.push_back(v);
            row.coefficients.push_back(1);
            for (size_t u : neighbors) {
                row.vertices.push_back(u);
                row.coefficients.push_back(-1. / neighbors.size());
            }
            for (size_t j = 0; j < row.vertices.size(); ++j)
                laplace_incident[row.vertices[j]].push_back({laplace.size(), row.coefficients[j]});
            laplace.push_back(std::move(row));
        }
    const double laplace_coefficient =
        laplace.empty() ? 0 : options.laplace_weight * active_cells.size() / laplace.size();
    const auto geometry =
        [&](size_t v, const Vector3d& p, Vector3d* gradient, Eigen::Matrix3d* hessian) {
            const int64_t parent = input.corr_input_vertex[v];
            if ((joint_pass || coupled_pass) && options.joint_blend_radius_ratio > 0 &&
                !frame_vertices.count(v) && parent >= 0 && state.joints.count(parent) &&
                carriers.at(parent).size() > 2) {
                std::vector<std::array<Vector3d, 2>> segments;
                for (const auto& edge : carriers.at(parent))
                    segments.push_back({point(input, edge[0]), point(input, edge[1])});
                const auto sample =
                    rod_joint_radius_sample(p, segments, radius, options.joint_blend_radius_ratio);
                if (gradient) *gradient = 2 * sample.residual * sample.gradient;
                if (hessian) *hessian = 2 * sample.gradient * sample.gradient.transpose();
                return sample.residual * sample.residual;
            }
            if (state.mapping_locked && !frame_vertices.count(v)) {
                const Vector3d d = (p - carrier_foot(input.corr_input_vertex[v], p)) / radius;
                const double length = d.norm();
                const Vector3d u = length > 1e-12 ? Vector3d(d / length) : Vector3d::UnitX();
                if (gradient) *gradient = 2 * (length - 1) * u;
                if (hessian) *hessian = 2 * u * u.transpose();
                return (length - 1) * (length - 1);
            }
            const Vector3d error = (p - targets.at(v)) / radius;
            if (gradient) *gradient = 2 * error;
            if (hessian) *hessian = 2 * Eigen::Matrix3d::Identity();
            return error.squaredNorm();
        };
    const auto total_energy = [&]() {
        double amips = 0, shape = 0, smooth = 0, harmonic = 0;
        for (size_t tid : active_cells) {
            const auto local =
                local_cell(input, cells[tid], cells[tid][0], point(input, cells[tid][0]), radius);
            double value = AMIPS_energy(local);
            if (!std::isfinite(value)) value = std::cbrt(AMIPS_energy_stable_p3<Rational>(local));
            amips += value;
        }
        for (size_t v : movable)
            if (targets.count(v))
                shape += (surface_pass || state.mapping_locked)
                             ? geometry(v, point(input, v), nullptr, nullptr)
                             : (point(input, v) - targets.at(v)).squaredNorm() / (radius * radius);
        for (const auto& row : fairing)
            smooth += row.weight * row.residual(input.vertices).squaredNorm();
        for (const auto& row : laplace)
            harmonic += row.residual(input.vertices, radius).squaredNorm();
        return std::array<double, 5>{
            amips + coefficient * shape + fairing_coefficient * smooth +
                laplace_coefficient * harmonic,
            amips,
            shape,
            smooth,
            harmonic};
    };
    const auto before = total_energy();
    if (!std::isfinite(before[0]))
        throw std::runtime_error("rod position optimization: nonfinite initial energy");
    auto report = nlohmann::json{
        {"stage",
         coupled_pass ? "after_local_background_remeshing_before_native_reconstruction"
         : joint_pass ? "joint_geometry_after_regular_positions_before_background_remeshing"
                      : "after_degree_alignment_and_fixed_mapping_before_native_prisms"},
        {"movable_rod_vertices", movable_rod_count},
        {"movable_background_vertices", movable.size() - movable_rod_count},
        {"background_neighborhood_radius", background_region.radius},
        {"background_neighborhood_tets", background_region.selected_tets},
        {"background_patch_boundary_vertices", background_region.boundary_vertices},
        {"surface_energy_crosses_tet_prism_regions", coupled_pass},
        {"coupled_tetrahedra", active_cells.size()},
        {"geometry_target_vertices", targeted},
        {"position_weight", options.position_weight},
        {"frame_target_vertices", frame_vertices.size()},
        {"control_frame_target_vertices", control_target_vertices},
        {"transition_frame_target_vertices", transition_target_vertices},
        {"intermediate_laplace_vertices", laplace.size()},
        {"laplace_weight", options.laplace_weight},
        {"laplace_energy_coefficient", laplace_coefficient},
        {"geometry_definition",
         state.mapping_locked ? "all_prism_cap_material_slots_and_noncap_carrier_radius"
         : surface_pass       ? "radius_and_section_plane_with_weak_material_slot_tether"
                              : "fixed_mapped_circle_targets"},
        {"geometry_energy_coefficient", coefficient},
        {"position_smoothing_weight", options.position_smoothing_weight},
        {"fairing_rows", fairing.size()},
        {"fairing_energy_coefficient", fairing_coefficient},
        {"energy_terms",
         {"total",
          "tet_AMIPS_sum",
          "geometry_squared_error",
          "surface_fairing_squared_error",
          "laplace_squared_error"}},
        {"iterations_requested", options.position_iterations},
        {"iterations_completed", 0},
        {"joint_offsets_released", joint_offsets.size()},
        {"joint_blend_radius_ratio",
         (joint_pass || coupled_pass) ? options.joint_blend_radius_ratio : 0},
        {"candidate_geometry_before", candidate_geometry(input)},
        {"history", nlohmann::json::array()}};
    if (report["candidate_geometry_before"]["unequal_degree_layers"] != 0)
        throw std::runtime_error(
            "rod position optimization: ring degrees must be aligned before mapping");
    size_t total_moves = 0;
    for (int sweep = 0; sweep < options.position_iterations && !movable.empty(); ++sweep) {
        size_t moves = 0, backtracks = 0;
        for (size_t v : movable) {
            const Vector3d start = point(input, v);
            std::vector<std::array<double, 12>> local;
            Vector3d gradient = Vector3d::Zero();
            Eigen::Matrix3d hessian = Eigen::Matrix3d::Zero();
            double initial = 0;
            for (size_t tid : incident[v]) {
                const auto data = local_cell(input, cells[tid], v, start, radius);
                initial += AMIPS_energy(data);
                Vector3d g;
                Eigen::Matrix3d h;
                AMIPS_jacobian(data, g);
                AMIPS_hessian(data, h);
                gradient += g;
                hessian += h;
                local.push_back(data);
            }
            const bool has_target = targets.count(v);
            const Vector3d error =
                has_target ? Vector3d((start - targets.at(v)) / radius) : Vector3d::Zero();
            if (has_target) {
                if (surface_pass || state.mapping_locked) {
                    Vector3d g;
                    Eigen::Matrix3d h;
                    initial += coefficient * geometry(v, start, &g, &h);
                    gradient += coefficient * g;
                    hessian += coefficient * h;
                } else {
                    initial += coefficient * error.squaredNorm();
                    gradient += 2 * coefficient * error;
                    hessian.diagonal().array() += 2 * coefficient;
                }
            }
            std::vector<Vector3d> fairing_errors;
            for (const auto& [row_id, j] : fairing_incident[v]) {
                const auto& row = fairing[row_id];
                const Vector3d residual = row.residual(input.vertices);
                const Eigen::Matrix3d derivative = radius * row.coefficients[j];
                const double weight = fairing_coefficient * row.weight;
                initial += weight * residual.squaredNorm();
                gradient += 2 * weight * derivative.transpose() * residual;
                hessian += 2 * weight * derivative.transpose() * derivative;
                fairing_errors.push_back(residual);
            }
            std::vector<Vector3d> laplace_errors;
            for (const auto& [row_id, c] : laplace_incident[v]) {
                const Vector3d residual = laplace[row_id].residual(input.vertices, radius);
                initial += laplace_coefficient * residual.squaredNorm();
                gradient += 2 * laplace_coefficient * c * residual;
                hessian.diagonal().array() += 2 * laplace_coefficient * c * c;
                laplace_errors.push_back(residual);
            }
            if (!std::isfinite(initial) || !gradient.allFinite() || !hessian.allFinite()) continue;
            const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
                (hessian + hessian.transpose()) / 2);
            if (solver.info() != Eigen::Success) continue;
            const double floor = std::max(1e-6, solver.eigenvalues().cwiseAbs().maxCoeff() * 1e-6);
            const Vector3d eigenvalues = solver.eigenvalues().cwiseMax(floor);
            Vector3d direction =
                -solver.eigenvectors() *
                ((solver.eigenvectors().transpose() * gradient).cwiseQuotient(eigenvalues));
            if (!direction.allFinite() || !(direction.dot(gradient) < 0)) continue;
            if (direction.norm() > .25) direction *= .25 / direction.norm();
            if (direction.norm() < 1e-10) continue;
            double alpha = 1;
            for (int attempt = 0; attempt <= 30; ++attempt, alpha *= .5) {
                const Vector3d step = alpha * direction;
                double energy =
                    has_target
                        ? coefficient * ((surface_pass || state.mapping_locked)
                                             ? geometry(v, start + radius * step, nullptr, nullptr)
                                             : (error + step).squaredNorm())
                        : 0;
                for (auto data : local) {
                    for (size_t k = 0; k < 3; ++k) data[k] = step[k];
                    energy += AMIPS_energy(data);
                }
                for (size_t k = 0; k < fairing_incident[v].size(); ++k) {
                    const auto [row_id, j] = fairing_incident[v][k];
                    const auto& row = fairing[row_id];
                    const Vector3d residual =
                        fairing_errors[k] + radius * row.coefficients[j] * step;
                    energy += fairing_coefficient * row.weight * residual.squaredNorm();
                }
                for (size_t k = 0; k < laplace_incident[v].size(); ++k) {
                    const double c = laplace_incident[v][k].second;
                    energy += laplace_coefficient * (laplace_errors[k] + c * step).squaredNorm();
                }
                if (!std::isfinite(energy) ||
                    !(energy < initial - 1e-12 * std::max(1., std::abs(initial))))
                    continue;
                const Vector3d candidate = start + radius * step;
                if (candidate == start) break;
                input.vertices.row(v) = candidate.transpose();
                bool valid = true;
                for (size_t tid : incident[v])
                    if (!tet_volume_above_threshold(
                            input.vertices,
                            cells[tid],
                            volume_floors[tid])) {
                        valid = false;
                        break;
                    }
                if (!valid) {
                    input.vertices.row(v) = start.transpose();
                    continue;
                }
                ++moves;
                backtracks += attempt;
                break;
            }
        }
        total_moves += moves;
        const auto energy = total_energy();
        report["history"].push_back(
            {{"iteration", sweep + 1},
             {"accepted_moves", moves},
             {"backtracks", backtracks},
             {"total_energy", energy[0]},
             {"tet_AMIPS_sum", energy[1]},
             {"geometry_squared_error", energy[2]},
             {"surface_fairing_squared_error", energy[3]},
             {"laplace_squared_error", energy[4]}});
        report["iterations_completed"] = sweep + 1;
        if (!moves) break;
    }
    const auto after = total_energy();
    for (size_t v = 0; v < input.vertex_tags.size(); ++v)
        if (input.vertex_tags[v] == 1 &&
            !(input.vertices.row(v).array() == original_positions.row(v).array()).all())
            throw std::runtime_error("rod position optimization moved an input vertex");
    for (size_t i = 0; i < state.frames.size(); ++i)
        if (state.frames[i].ring != mapping[i])
            throw std::runtime_error("rod position optimization changed the mapping");
    if (input.tetrahedra != original_tets)
        throw std::runtime_error("rod position optimization changed connectivity");
    if (after[0] > before[0] + 1e-9 * std::max(1., std::abs(before[0])))
        throw std::runtime_error("rod position optimization increased total energy");
    if ((joint_pass || coupled_pass) && input.junction_protection) {
        // Connectivity is still protected. Only this explicitly requested
        // geometry stage can replace the coordinate snapshot, after positivity
        // and fixed-input/mapping/connectivity checks have passed.
        nlohmann::json moved = nlohmann::json::array();
        double maximum_displacement = 0;
        double maximum_total_displacement = 0;
        for (size_t v : joint_offsets) {
            const double displacement =
                (point(input, v) - Vector3d(original_positions.row(v))).norm();
            maximum_total_displacement = std::max(
                maximum_total_displacement,
                (point(input, v) - state.joint_position_reference.at(v)).norm());
            if (!(displacement > 0)) continue;
            moved.push_back(input.source_vertex_ids[v]);
            maximum_displacement = std::max(maximum_displacement, displacement);
            input.junction_protection->offset_positions.at(v) = point(input, v);
        }
        report["protected_offset_vertices_moved"] = moved.size();
        report["moved_joint_source_vertex_ids"] = moved;
        report["maximum_joint_displacement"] = maximum_displacement;
        report["maximum_total_joint_displacement"] = maximum_total_displacement;
        report["maximum_joint_displacement_limit"] = nullptr;
        report["joint_displacement_policy"] = "geometry energy, descent and positive incident tets";
        report["protection_geometry_snapshot_refreshed"] = true;
    }
    verify_junction_one_rings(input);
    report["energy_before"] = before;
    report["energy_after"] = after;
    report["accepted_moves"] = total_moves;
    report["candidate_geometry_after"] = candidate_geometry(input);
    report["mapping_unchanged"] = true;
    report["connectivity_unchanged"] = true;
    report["elapsed_seconds"] =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    state.report["tet_position_optimization"] = std::move(report);
    logger().info(
        "{}: {} sweeps, {} moves; total energy {} -> {}",
        coupled_pass   ? "Coupled rod/background optimization"
        : joint_pass   ? "Rod joint smoothing"
        : surface_pass ? "Rod surface smoothing"
                       : "Rod tet position optimization",
        state.report["tet_position_optimization"]["iterations_completed"].get<int>(),
        total_moves,
        before[0],
        after[0]);
}

void optimize_rod_background_coupled(PrismaticMeshInput& input)
{
    if (!input.rod_state || input.rod_state->options.coupled_rounds == 0) return;
    auto& state = *input.rod_state;
    const RodOptions saved = state.options;
    if (!saved.edge_based || !saved.background_remeshing.enabled)
        throw std::runtime_error(
            "Coupled optimization requires edge-based rods and background remeshing.");
    finalize_rod_frame_mapping(input);
    const auto regular_report = state.report["tet_position_optimization"];
    const auto started = std::chrono::steady_clock::now();
    auto report = nlohmann::json{
        {"method", "local_background_remeshing_then_shared_surface_and_background_geometry"},
        {"rounds_requested", saved.coupled_rounds},
        {"rounds_completed", 0},
        {"native_reconstruction_after_all_rounds", true},
        {"rounds", nlohmann::json::array()}};
    try {
        for (int round = 0; round < saved.coupled_rounds; ++round) {
            const size_t old_vertices = input.vertices.rows();
            remesh_rod_background(input);
            const MatrixXd remeshed_positions = input.vertices;
            state.options.position_iterations = saved.coupled_iterations;
            state.options.position_weight = saved.coupled_geometry_weight;
            state.options.laplace_weight = saved.coupled_smoothing_weight;
            state.options.position_smoothing_weight = 0;
            optimize_rod_positions_impl(input, false, false, true);
            auto geometry = state.report["tet_position_optimization"];
            size_t new_points_moved = 0;
            for (size_t v = old_vertices; v < size_t(input.vertices.rows()); ++v)
                new_points_moved += input.vertices.row(v) != remeshed_positions.row(v);
            geometry["new_background_vertices_available"] = input.vertices.rows() - old_vertices;
            geometry["new_background_vertices_moved"] = new_points_moved;
            report["rounds"].push_back(
                {{"round", round + 1},
                 {"remeshing", state.report["background_neighborhood_remeshing"]},
                 {"geometry", geometry}});
            report["rounds_completed"] = round + 1;
            state.options = saved;
        }
        report["elapsed_seconds"] =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
        state.report["coupled_optimization"] = std::move(report);
        state.report["tet_position_optimization"] = regular_report;
        state.options = saved;
    } catch (...) {
        state.options = saved;
        state.report["tet_position_optimization"] = regular_report;
        throw;
    }
}

void optimize_rod_joint_positions(PrismaticMeshInput& input)
{
    if (!input.rod_state || !input.rod_state->options.edge_based ||
        input.rod_state->options.joint_smoothing_iterations == 0 || input.rod_state->joints.empty())
        return;
    auto& state = *input.rod_state;
    const RodOptions saved = state.options;
    const auto regular_report = state.report["tet_position_optimization"];
    try {
        state.options.position_iterations = saved.joint_smoothing_iterations;
        state.options.position_weight = saved.joint_geometry_weight;
        state.options.laplace_weight = saved.joint_smoothing_weight;
        state.options.position_smoothing_weight = 0;
        optimize_rod_positions_impl(input, false, true);
        state.report["joint_smoothing"] = state.report["tet_position_optimization"];
        state.report["tet_position_optimization"] = regular_report;
        state.options = saved;
    } catch (...) {
        state.options = saved;
        throw;
    }
}

void optimize_rod_tet_positions(PrismaticMeshInput& input)
{
    if (!input.rod_state || !input.rod_state->options.edge_based) return;
    finalize_rod_frame_mapping(input);
    auto& state = *input.rod_state;
    const RodOptions saved = state.options;
    try {
        // Fairing is an additional, independently enabled warm-started pass;
        // it never substitutes for the primary mapped-frame/Laplace solve.
        state.options.position_smoothing_weight = 0;
        optimize_rod_positions_impl(input, false);
        const auto baseline_report = state.report["tet_position_optimization"];
        if (saved.position_smoothing_weight > 0 && saved.position_smoothing_iterations > 0) {
            state.options = saved;
            state.options.position_iterations = saved.position_smoothing_iterations;
            state.options.position_weight = saved.position_smoothing_geometry_weight;
            optimize_rod_positions_impl(input, true);
            state.report["tet_surface_smoothing"] = state.report["tet_position_optimization"];
            state.report["tet_surface_smoothing"]["stage"] =
                "after_tet_position_optimization_before_native_prisms";
            state.report["tet_position_optimization"] = baseline_report;
        }
        state.options = saved;
    } catch (...) {
        state.options = saved;
        throw;
    }
}
} // namespace wmtk::components::prismatic_mesh
