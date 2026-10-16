#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <wmtk/utils/Logger.hpp>
#include "prism_jacobian.hpp"

namespace wmtk::components::prismatic_mesh {
namespace {
using Mat = Eigen::Matrix3d;
struct Plane
{
    Vector3d a;
    double b;
}; // a.dot(d) >= b
struct Residual
{
    Vector3d g;
    double b;
}; // max(0,b-g.dot(d))

bool add_plane(std::vector<Plane>& planes, Vector3d a, double b)
{
    const double norm = a.norm();
    if (!std::isfinite(norm) || !std::isfinite(b)) return false;
    if (norm == 0) return b <= 0;
    planes.push_back({a / norm, b / norm});
    return true;
}

// Phase I: cyclic halfspace projections. Failure means no feasible point was found
// within the budget, not a proof that the polytope is empty.
bool feasible_point(const std::vector<Plane>& planes, Vector3d& x)
{
    for (int sweep = 0; sweep < 2000; ++sweep) {
        double violation = 0;
        for (const auto& p : planes) {
            const double gap = p.b - p.a.dot(x);
            violation = std::max(violation, gap);
            if (gap > 0) x += gap * p.a;
        }
        if (!x.allFinite()) return false;
        if (violation < 1e-11) return true;
    }
    return false;
}

// Primal active-set QP in three position variables. The current point is feasible.
// At most three independent hard constraints enter the working set.
bool quadratic_step(const Mat& h, const Vector3d& c, const std::vector<Plane>& planes, Vector3d& x)
{
    const Mat inverse = h.ldlt().solve(Mat::Identity());
    std::vector<size_t> active;
    for (int iteration = 0; iteration < 128; ++iteration) {
        const Vector3d gradient = h * x + c;
        Vector3d direction = -inverse * gradient;
        Eigen::VectorXd multipliers;
        if (!active.empty()) {
            Eigen::MatrixXd a(active.size(), 3);
            for (size_t i = 0; i < active.size(); ++i) a.row(i) = planes[active[i]].a.transpose();
            const Eigen::MatrixXd gram = a * inverse * a.transpose();
            multipliers = gram.ldlt().solve(a * inverse * gradient);
            direction += inverse * a.transpose() * multipliers;
        }
        if (!direction.allFinite()) return false;
        if (direction.norm() < 1e-10) {
            if (active.empty()) return true;
            Eigen::Index worst;
            const double minimum = multipliers.minCoeff(&worst);
            if (minimum >= -1e-9) return true;
            active.erase(active.begin() + worst);
            continue;
        }
        double alpha = 1;
        size_t blocker = planes.size();
        for (size_t i = 0; i < planes.size(); ++i) {
            if (std::find(active.begin(), active.end(), i) != active.end()) continue;
            const double rate = planes[i].a.dot(direction);
            if (rate >= -1e-12) continue;
            const double limit = std::max(0., (planes[i].a.dot(x) - planes[i].b) / (-rate));
            if (limit < alpha) {
                alpha = limit;
                blocker = i;
            }
        }
        x += alpha * direction;
        if (blocker != planes.size()) {
            Eigen::MatrixXd a(active.size() + 1, 3);
            for (size_t i = 0; i < active.size(); ++i) a.row(i) = planes[active[i]].a.transpose();
            a.row(active.size()) = planes[blocker].a.transpose();
            Eigen::FullPivLU<Eigen::MatrixXd> rank(a);
            rank.setThreshold(1e-10);
            if (rank.rank() != a.rows()) return false;
            active.push_back(blocker);
        }
    }
    return false;
}

double model_energy(
    const std::vector<Residual>& residuals,
    const Vector3d& d,
    const Vector3d& displacement,
    double weight)
{
    double result = weight * (displacement + d).squaredNorm();
    for (const auto& r : residuals) {
        const double z = std::max(0., r.b - r.g.dot(d));
        result += z * z;
    }
    return result;
}

// Eliminate the nonnegative slack variables analytically. The squared hinge
// objective is convex and piecewise quadratic; solve each Newton model with the
// hard halfspaces and use an Armijo line search on the actual hinge objective.
bool solve_local_qp(
    const std::vector<Residual>& residuals,
    const std::vector<Plane>& planes,
    const Vector3d& displacement,
    double weight,
    Vector3d& d,
    std::string& failure)
{
    d.setZero();
    if (!feasible_point(planes, d)) {
        failure = "constraint_feasibility_not_found";
        return false;
    }
    for (int iteration = 0; iteration < 60; ++iteration) {
        Mat h = 2 * weight * Mat::Identity();
        Vector3d gradient = 2 * weight * (displacement + d);
        for (const auto& r : residuals) {
            const double z = r.b - r.g.dot(d);
            if (z > 0) {
                h += 2 * r.g * r.g.transpose();
                gradient -= 2 * z * r.g;
            }
        }
        Vector3d trial = d;
        if (!quadratic_step(h, gradient - h * d, planes, trial)) {
            failure = "qp_active_set_limit";
            return false;
        }
        const Vector3d direction = trial - d;
        if (direction.norm() < 1e-9) return true;
        const double energy = model_energy(residuals, d, displacement, weight);
        double alpha = 1;
        bool accepted = false;
        for (int i = 0; i < 30; ++i, alpha *= .5) {
            if (model_energy(residuals, d + alpha * direction, displacement, weight) <=
                energy + 1e-4 * alpha * gradient.dot(direction)) {
                d += alpha * direction;
                accepted = true;
                break;
            }
        }
        if (!accepted) {
            failure = "qp_line_search_stalled";
            return false;
        }
    }
    failure = "qp_newton_limit";
    return false;
}

struct Quality
{
    PrismJacobianMinimum minimum;
    double energy = 0;
};
Quality quality(
    const PrismaticMeshInput& input,
    const HybridCell& prism,
    double scale,
    double target,
    const std::vector<Vector3d>& samples)
{
    Quality result;
    result.minimum = minimum_prism_jacobian(input.vertices, prism);
    for (const auto& q : samples) {
        const double z =
            std::max(0., target - prism_jacobian(input.vertices, prism, q).determinant / scale);
        result.energy += z * z;
    }
    // These three minima are recomputed after every trial move. They account for
    // moving worst points between the fixed samples in the nonlinear acceptance test.
    for (double j : result.minimum.edge_values) {
        const double z = std::max(0., target - j / scale);
        result.energy += z * z;
    }
    return result;
}

std::array<size_t, 4> tet_at(const PrismaticMeshInput& input, size_t tid)
{
    std::array<size_t, 4> t;
    for (int i = 0; i < 4; ++i) t[i] = input.tetrahedra(tid, i);
    return t;
}

Plane tet_constraint(
    const PrismaticMeshInput& input,
    size_t tid,
    size_t v,
    double length,
    double floor)
{
    const auto t = tet_at(input, tid);
    std::array<Vector3d, 4> p, g;
    for (int i = 0; i < 4; ++i) p[i] = input.vertices.row(t[i]);
    const Vector3d a = p[1] - p[0], b = p[2] - p[0], c = p[3] - p[0];
    g[1] = b.cross(c) / 6;
    g[2] = c.cross(a) / 6;
    g[3] = a.cross(b) / 6;
    g[0] = -g[1] - g[2] - g[3];
    const double volume = a.dot(b.cross(c)) / 6;
    // Keep QP endpoints a little inside the strict threshold; the final comparison
    // uses the existing exact rational volume predicate on the stored doubles.
    const double margin = 1e-10 * std::max(std::abs(volume), floor);
    for (int i = 0; i < 4; ++i)
        if (t[i] == v) return {length * g[i], floor + margin - volume};
    throw std::runtime_error("Jacobian smoothing: vertex absent from incident tet");
}
} // namespace

void smooth_prism_jacobians(
    PrismaticMeshInput& input,
    double min_tet_volume,
    const JacobianSmoothingOptions& options)
{
    if (options.iterations < 0 || !std::isfinite(min_tet_volume) || min_tet_volume < 0 ||
        !std::isfinite(options.target) || options.target <= 0 ||
        !std::isfinite(options.position_weight) || options.position_weight <= 0 ||
        !std::isfinite(options.max_step_ratio) || options.max_step_ratio <= 0 ||
        !std::isfinite(options.max_displacement_ratio) || options.max_displacement_ratio <= 0)
        throw std::invalid_argument("Invalid prism Jacobian smoothing options");
    input.jacobian_smoothing_report = {{"enabled", options.iterations > 0}};
    if (options.iterations == 0) return;
    const auto prisms = prism_candidates_for_smoothing(input);
    const MatrixXd original = input.vertices;
    const auto samples = prism_jacobian_sample_points();
    std::vector<double> scales(prisms.size());
    std::vector<Quality> state(prisms.size());
    std::vector<std::vector<size_t>> incident(input.vertices.rows());
    std::vector<double> length(input.vertices.rows(), 0);
    std::vector<double> before_min(prisms.size());
    for (size_t i = 0; i < prisms.size(); ++i) {
        const auto& p = prisms[i].vertices;
        const Vector3d e1 = original.row(p[1]) - original.row(p[0]);
        const Vector3d e2 = original.row(p[2]) - original.row(p[0]);
        double height = 0;
        for (int j = 0; j < 3; ++j) {
            height += (original.row(p[j + 3]) - original.row(p[j])).norm() / 3;
            incident[p[j + 3]].push_back(i);
            length[p[j + 3]] = input.target_thickness > 0
                                   ? input.target_thickness
                                   : (original.row(p[j + 3]) - original.row(p[j])).norm();
        }
        scales[i] = e1.cross(e2).norm() * height;
        if (!(scales[i] > 0) || !std::isfinite(scales[i]))
            throw std::runtime_error("Jacobian smoothing: invalid fixed prism normalization");
        state[i] = quality(input, prisms[i], scales[i], options.target, samples);
        before_min[i] = state[i].minimum.determinant;
    }
    auto statistics = [&]() {
        size_t bad = 0, below = 0;
        double minimum = std::numeric_limits<double>::infinity(), energy = 0;
        for (size_t i = 0; i < state.size(); ++i) {
            const double q = state[i].minimum.determinant / scales[i];
            bad += !(q > 0);
            below += q < options.target;
            minimum = std::min(minimum, q);
            energy += state[i].energy;
        }
        return nlohmann::json{
            {"invalid_prisms", bad},
            {"below_target_prisms", below},
            {"minimum_normalized_jacobian",
             state.empty() ? nlohmann::json(nullptr) : nlohmann::json(minimum)},
            {"jacobian_energy", energy}};
    };
    const auto initial = statistics();
    logger().info(
        "Prism Jacobian smoothing: {} candidates, {} invalid before smoothing",
        prisms.size(),
        initial["invalid_prisms"].get<size_t>());
    nlohmann::json iterations = nlohmann::json::array();
    std::map<std::string, size_t> total_rejections;
    size_t total_moves = 0;
    for (int round = 0; round < options.iterations; ++round) {
        std::vector<std::pair<double, size_t>> pending;
        for (size_t v = 0; v < incident.size(); ++v) {
            if (incident[v].empty() || input.vertex_tags[v] != 2) continue;
            double worst = std::numeric_limits<double>::infinity();
            for (size_t i : incident[v])
                worst = std::min(worst, state[i].minimum.determinant / scales[i]);
            if (worst < options.target) pending.emplace_back(worst, v);
        }
        std::sort(pending.begin(), pending.end());
        size_t moves = 0, attempts = 0;
        std::map<std::string, size_t> rejections;
        for (const auto& entry : pending) {
            const size_t v = entry.second;
            const auto& neighbors = incident[v];
            double old_energy = 0;
            for (size_t i : neighbors) old_energy += state[i].energy;
            if (old_energy < 1e-18) continue;
            ++attempts;
            const Vector3d start = input.vertices.row(v);
            const Vector3d displacement = (start - original.row(v).transpose()) / length[v];
            const auto star = input.mesh->get_one_ring_tids_for_vertex(v);
            std::vector<std::vector<Vector3d>> extra(neighbors.size());
            std::set<std::string> blocked;
            bool accepted = false;
            for (int cutting_pass = 0; cutting_pass < 3 && !accepted; ++cutting_pass) {
                std::vector<Plane> planes;
                std::vector<Residual> residuals;
                bool constraints_ok = true;
                for (int j = 0; j < 3; ++j) {
                    const double step = options.max_step_ratio / std::sqrt(3.);
                    const double bound = options.max_displacement_ratio / std::sqrt(3.);
                    add_plane(planes, Vector3d::Unit(j), std::max(-step, -bound - displacement[j]));
                    add_plane(planes, -Vector3d::Unit(j), -std::min(step, bound - displacement[j]));
                }
                for (size_t tid : star) {
                    const double floor = input.offset_tet_tags[tid] == 1 ? min_tet_volume : 0;
                    const auto p = tet_constraint(input, tid, v, length[v], floor);
                    constraints_ok &= add_plane(planes, p.a, p.b);
                }
                for (size_t k = 0; k < neighbors.size(); ++k) {
                    const size_t i = neighbors[k];
                    const auto& prism = prisms[i];
                    const auto it = std::find(prism.vertices.begin(), prism.vertices.end(), v);
                    const size_t local = std::distance(prism.vertices.begin(), it);
                    auto points = samples;
                    points.insert(
                        points.end(),
                        state[i].minimum.edge_points.begin(),
                        state[i].minimum.edge_points.end());
                    points.insert(points.end(), extra[k].begin(), extra[k].end());
                    const double current_min = state[i].minimum.determinant / scales[i];
                    for (const auto& q : points) {
                        const auto value = prism_jacobian(input.vertices, prism, q);
                        const double b = value.determinant / scales[i];
                        const Vector3d g = length[v] * value.gradients[local] / scales[i];
                        residuals.push_back({g, options.target - b});
                        if (current_min > 0)
                            constraints_ok &= add_plane(
                                planes,
                                g,
                                std::min(options.target, .5 * current_min) - b);
                    }
                }
                if (!constraints_ok) {
                    blocked.insert("nonfinite_or_constant_constraint");
                    break;
                }
                Vector3d d;
                std::string failure;
                if (!solve_local_qp(
                        residuals,
                        planes,
                        displacement,
                        options.position_weight,
                        d,
                        failure)) {
                    blocked.insert(failure);
                    break;
                }
                if (d.norm() < 1e-10) {
                    blocked.insert("stationary_under_constraints");
                    break;
                }
                const double old_total =
                    old_energy + options.position_weight * displacement.squaredNorm();
                double alpha = 1;
                for (int backtrack = 0; backtrack <= 20; ++backtrack, alpha *= .5) {
                    const Vector3d proposal = start + length[v] * alpha * d;
                    if (!proposal.allFinite() || proposal == start) break;
                    input.vertices.row(v) = proposal.transpose();
                    bool valid = true;
                    for (size_t tid : star) {
                        if (!tet_volume_above_threshold(
                                input.vertices,
                                tet_at(input, tid),
                                input.offset_tet_tags[tid] == 1 ? min_tet_volume : 0)) {
                            blocked.insert("tet_volume");
                            valid = false;
                            break;
                        }
                    }
                    std::vector<Quality> updated;
                    double energy = 0;
                    if (valid)
                        for (size_t k = 0; k < neighbors.size(); ++k) {
                            const size_t i = neighbors[k];
                            updated.push_back(
                                quality(input, prisms[i], scales[i], options.target, samples));
                            energy += updated.back().energy;
                            const double before = state[i].minimum.determinant / scales[i];
                            const double after = updated.back().minimum.determinant / scales[i];
                            if (!std::isfinite(after) ||
                                (before > 0 &&
                                 after < std::min(options.target, .5 * before) * (1 - 1e-8))) {
                                blocked.insert("valid_prism_jacobian");
                                valid = false;
                            }
                            // Add the newly discovered worst reference positions before another QP
                            // solve.
                            if (backtrack == 0)
                                for (const auto& q : updated.back().minimum.edge_points)
                                    if (std::none_of(
                                            extra[k].begin(),
                                            extra[k].end(),
                                            [&](const Vector3d& p) {
                                                return (p - q).norm() < 1e-10;
                                            }))
                                        extra[k].push_back(q);
                        }
                    const double total =
                        energy + options.position_weight * (displacement + alpha * d).squaredNorm();
                    const double tolerance = 1e-14 * std::max(1., old_energy);
                    if (valid && energy < old_energy - tolerance && total < old_total - tolerance) {
                        for (size_t k = 0; k < neighbors.size(); ++k)
                            state[neighbors[k]] = updated[k];
                        accepted = true;
                        ++moves;
                        break;
                    }
                    if (valid) blocked.insert("no_actual_energy_decrease");
                    input.vertices.row(v) = start.transpose();
                }
            }
            if (!accepted)
                for (const auto& reason : blocked) ++rejections[reason];
        }
        total_moves += moves;
        for (const auto& entry : rejections) total_rejections[entry.first] += entry.second;
        auto stats = statistics();
        stats["iteration"] = round + 1;
        stats["attempted_vertices"] = attempts;
        stats["accepted_moves"] = moves;
        stats["rejections"] = rejections;
        iterations.push_back(stats);
        logger().info(
            "Jacobian smoothing {}/{}: {} moves / {} attempts, {} invalid prisms, minimum "
            "normalized J {}",
            round + 1,
            options.iterations,
            moves,
            attempts,
            stats["invalid_prisms"].get<size_t>(),
            stats["minimum_normalized_jacobian"].is_null()
                ? 0
                : stats["minimum_normalized_jacobian"].get<double>());
        if (moves == 0) break;
    }
    nlohmann::json moved = nlohmann::json::array(), columns = nlohmann::json::array();
    double max_move = 0, max_ratio = 0, max_thickness_change = 0;
    for (size_t v = 0; v < incident.size(); ++v) {
        const double distance = (input.vertices.row(v) - original.row(v)).norm();
        if (distance == 0) continue;
        const size_t parent = input.corr_input_vertex[v];
        const double old_h = (original.row(v) - original.row(parent)).norm();
        const double new_h = (input.vertices.row(v) - input.vertices.row(parent)).norm();
        max_move = std::max(max_move, distance);
        max_ratio = std::max(max_ratio, distance / length[v]);
        max_thickness_change = std::max(max_thickness_change, std::abs(new_h - old_h) / old_h);
        std::array<double, 3> before, after;
        for (int j = 0; j < 3; ++j) {
            before[j] = original(v, j);
            after[j] = input.vertices(v, j);
        }
        moved.push_back(
            {{"source_vid", input.source_vertex_ids[v]},
             {"before", before},
             {"after", after},
             {"displacement", distance},
             {"normalizing_length", length[v]},
             {"old_column_length", old_h},
             {"new_column_length", new_h}});
    }
    for (size_t i = 0; i < prisms.size(); ++i) {
        std::vector<int64_t> ids;
        for (size_t v : prisms[i].vertices) ids.push_back(input.source_vertex_ids[v]);
        columns.push_back(
            {{"source_vids", ids},
             {"normalization_scale", scales[i]},
             {"initial_minimum_det_j", before_min[i]},
             {"final_minimum_det_j", state[i].minimum.determinant}});
    }
    input.jacobian_smoothing_report = {
        {"enabled", true},
        {"candidate_prisms", prisms.size()},
        {"method", "single-vertex convex squared-hinge QP; analytic side-edge minima acceptance"},
        {"target_normalized_jacobian", options.target},
        {"position_weight", options.position_weight},
        {"max_step_ratio", options.max_step_ratio},
        {"max_displacement_ratio", options.max_displacement_ratio},
        {"requested_iterations", options.iterations},
        {"initial", initial},
        {"final", statistics()},
        {"iterations", iterations},
        {"accepted_moves", total_moves},
        {"rejections", total_rejections},
        {"moved_vertices", moved},
        {"maximum_displacement", max_move},
        {"maximum_displacement_ratio", max_ratio},
        {"maximum_relative_column_length_change", max_thickness_change},
        {"candidate_quality", columns}};
}
} // namespace wmtk::components::prismatic_mesh
