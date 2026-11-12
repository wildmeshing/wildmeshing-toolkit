#include "thickness_smoothing.hpp"
#include "junction_protection.hpp"
#include "prism_jacobian.hpp"
#include "rod_mesh.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::prismatic_mesh {
namespace {
using Tet = std::array<size_t, 4>;
using Mat = Eigen::Matrix3d;
constexpr double infinity = std::numeric_limits<double>::infinity();
constexpr double thickness_weight = 1000;

struct PrismQuality
{
    double value = -infinity;
    std::array<Vector3d, 6> gradient;
};

std::array<Vector3d, 6> shape_gradients(double r, double s, double t)
{
    const std::array<double, 6> dr{t - 1, 1 - t, 0, -t, t, 0};
    const std::array<double, 6> ds{t - 1, 0, 1 - t, -t, 0, t};
    const std::array<double, 6> dt{r + s - 1, -r, -s, 1 - r - s, r, s};
    std::array<Vector3d, 6> g;
    for (size_t i = 0; i < 6; ++i) g[i] = {dr[i], (2 * ds[i] - dr[i]) / std::sqrt(3.), dt[i]};
    return g;
}

Mat shape_matrix(const MatrixXd& p, const HybridCell& c, const std::array<Vector3d, 6>& g)
{
    Mat f = Mat::Zero();
    for (size_t i = 1; i < 6; ++i)
        f += (p.row(c.vertices[i]) - p.row(c.vertices[0])).transpose() * g[i].transpose();
    return f;
}

// Roots on [0,1], bracketed by derivative stationary points. Include endpoints
// and stationary points too, so almost multiple roots remain harmless samples.
std::vector<double> roots(const std::array<double, 4>& c)
{
    std::vector<double> intervals{0, 1};
    auto add = [&](double t) {
        if (t > 0 && t < 1 && std::isfinite(t)) intervals.push_back(t);
    };
    const double a = 3 * c[3], b = 2 * c[2], d = c[1];
    if (a == 0) {
        if (b != 0) add(-d / b);
    } else if (b * b - 4 * a * d >= 0) {
        const double v = -.5 * (b + std::copysign(std::sqrt(b * b - 4 * a * d), b));
        add(v / a);
        if (v != 0) add(d / v);
    }
    std::sort(intervals.begin(), intervals.end());
    auto result = intervals;
    const auto value = [&](double t) { return ((c[3] * t + c[2]) * t + c[1]) * t + c[0]; };
    for (size_t i = 1; i < intervals.size(); ++i) {
        double lo = intervals[i - 1], hi = intervals[i];
        const bool sign = value(lo) < 0;
        if (sign == (value(hi) < 0)) continue;
        for (int j = 0; j < 45; ++j) {
            const double mid = .5 * (lo + hi);
            if ((value(mid) < 0) == sign)
                lo = mid;
            else
                hi = mid;
        }
        result.push_back(.5 * (lo + hi));
    }
    return result;
}

PrismQuality prism_quality(const MatrixXd& p, const HybridCell& c)
{
    PrismQuality result;
    if (c.type != HybridCellType::Prism || c.vertices.size() != 6)
        throw std::invalid_argument("prism_amips requires six prism corners");
    if (!(minimum_prism_jacobian(p, c).determinant > 0)) {
        result.value = infinity;
        return result;
    }
    for (const auto& rs : std::array<std::array<double, 2>, 3>{{{0, 0}, {1, 0}, {0, 1}}}) {
        std::array<double, 3> norms, dets;
        for (size_t i = 0; i < 3; ++i) {
            const Mat f = shape_matrix(p, c, shape_gradients(rs[0], rs[1], .5 * i));
            norms[i] = f.squaredNorm();
            dets[i] = static_cast<double>(f.cast<long double>().determinant());
        }
        const auto quadratic = [](const auto& y) {
            const double c = 2 * (y[0] + y[2] - 2 * y[1]);
            return std::array<double, 3>{y[0], y[2] - y[0] - c, c};
        };
        const auto n = quadratic(norms), d = quadratic(dets);
        // d/dt (N / D^(2/3)) = 0 iff 3 N' D - 2 N D' = 0.
        for (double t : roots(
                 {3 * n[1] * d[0] - 2 * n[0] * d[1],
                  n[1] * d[1] + 6 * n[2] * d[0] - 4 * n[0] * d[2],
                  4 * n[2] * d[1] - n[1] * d[2],
                  2 * n[2] * d[2]})) {
            const auto g = shape_gradients(rs[0], rs[1], t);
            const Mat f = shape_matrix(p, c, g);
            const double det = static_cast<double>(f.cast<long double>().determinant());
            const double inv = 1 / std::cbrt(det * det);
            const double q = f.squaredNorm() * inv;
            if (q <= result.value) continue;
            result.value = q;
            Mat cof;
            cof.col(0) = f.col(1).cross(f.col(2));
            cof.col(1) = f.col(2).cross(f.col(0));
            cof.col(2) = f.col(0).cross(f.col(1));
            for (size_t i = 0; i < 6; ++i)
                result.gradient[i] = 2 * inv * f * g[i] - (2. / 3) * q / det * cof * g[i];
        }
    }
    return result;
}

struct Target
{
    enum Kind { Shell, Section, Cylinder } kind;
    size_t vertex;
    Vector3d origin, axis, slot;
    double length;
    // A cylinder uses a fixed source centerline segment, not the changing offset surface.
    Vector3d end;
    bool fixed_distance = false, fixed_plane = false;
};

struct Constraint
{
    std::vector<std::pair<size_t, Vector3d>> gradient;
    double bound = 0, multiplier = 0, denominator = 0;
};

std::array<Vector3d, 4> tet_gradient(const MatrixXd& p, const Tet& t, double& det)
{
    using Vec = Eigen::Matrix<long double, 3, 1>;
    const Vec a = p.row(t[0]).transpose().cast<long double>();
    const Vec b = p.row(t[1]).transpose().cast<long double>() - a;
    const Vec c = p.row(t[2]).transpose().cast<long double>() - a;
    const Vec d = p.row(t[3]).transpose().cast<long double>() - a;
    std::array<Vector3d, 4> g;
    g[1] = c.cross(d).cast<double>();
    g[2] = d.cross(b).cast<double>();
    g[3] = b.cross(c).cast<double>();
    g[0] = -g[1] - g[2] - g[3];
    det = static_cast<double>(b.dot(c.cross(d)));
    return g;
}

std::vector<Tet> pyramid_corners(const HybridCell& c)
{
    const auto& v = c.vertices;
    return {
        {v[0], v[1], v[2], v[4]},
        {v[0], v[2], v[3], v[4]},
        {v[0], v[1], v[3], v[4]},
        {v[1], v[2], v[3], v[4]}};
}

nlohmann::json quality_statistics(const MatrixXd& p, const std::vector<HybridCell>& prisms)
{
    std::vector<double> q;
    for (const auto& c : prisms) q.push_back(prism_amips(p, c));
    if (q.empty()) return {{"count", 0}};
    std::sort(q.begin(), q.end());
    double sum = 0;
    for (double v : q) sum += v;
    return {
        {"count", q.size()},
        {"mean", sum / q.size()},
        {"maximum", q.back()},
        {"p95", q[static_cast<size_t>(.95 * (q.size() - 1))]}};
}
} // namespace

double prism_amips(const MatrixXd& vertices, const HybridCell& prism)
{
    return prism_quality(vertices, prism).value;
}

void smooth_thickness_and_prisms(
    PrismaticMeshInput& input,
    PrismDominantMesh& hybrid,
    double min_tet_volume,
    int iterations)
{
    if (iterations < 0 || iterations > 200)
        throw std::invalid_argument("thickness_smoothing_iterations must be in [0,200]");
    if (!std::isfinite(min_tet_volume) || min_tet_volume < 0)
        throw std::invalid_argument("Invalid thickness smoothing volume floor");
    if (iterations == 0) return;
    validate_prism_dominant_mesh(input, hybrid, min_tet_volume);
    const MatrixXd original = input.vertices;
    const size_t nv = original.rows();
    std::vector<bool> movable(nv, false), active(nv, false);
    std::vector<HybridCell> prisms;
    std::vector<Tet> tets;
    std::vector<double> floors;
    std::set<size_t> below_floor_vertices;
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i) {
        Tet t;
        for (size_t j = 0; j < 4; ++j) t[j] = input.tetrahedra(i, j);
        tets.push_back(t);
        double det;
        tet_gradient(original, t, det);
        const double floor = input.offset_tet_tags[i] == 1 ? 6 * min_tet_volume : 0;
        // Preserve an existing below-floor cell unchanged. Every other changed
        // band tet obeys the explicit operation floor, without a shape objective.
        if (det <= floor) below_floor_vertices.insert(t.begin(), t.end());
        floors.push_back(det > floor ? floor : 0);
        for (size_t v : t) active[v] = true;
    }
    for (const auto& c : hybrid.cells) {
        if (c.type == HybridCellType::Prism) prisms.push_back(c);
        if (c.type == HybridCellType::Pyramid) {
            for (const auto& t : pyramid_corners(c)) {
                tets.push_back(t);
                floors.push_back(6 * min_tet_volume);
            }
        }
    }
    for (size_t v = 0; v < nv; ++v)
        movable[v] =
            active[v] && input.vertex_tags[v] == 2 && !below_floor_vertices.count(v) &&
            (!input.junction_protection || !input.junction_protection->offset_vertices.count(v)) &&
            (!input.rod_state || !input.rod_state->frozen_offset.count(v));
    for (size_t i = 0; i < static_cast<size_t>(input.tetrahedra.rows()); ++i)
        if (input.input_cells[i] == 1)
            for (int j = 0; j < 4; ++j) movable[input.tetrahedra(i, j)] = false;

    std::map<size_t, const RodFrame*> frame;
    if (input.rod_state)
        for (const auto& f : input.rod_state->frames)
            if (f.valid)
                for (size_t v : f.ring) frame[v] = &f;
    std::vector<Target> targets;
    std::vector<double> scale(nv, input.target_thickness > 0 ? input.target_thickness : 1);
    for (size_t v = 0; v < nv; ++v) {
        if (!movable[v]) continue;
        const auto parent = input.corr_input_vertex[v];
        if (is_rod_offset_vertex(input, v)) {
            const auto& s = *input.rod_state;
            scale[v] = s.radius;
            if (frame.count(v)) {
                const auto& f = *frame.at(v);
                const auto slot = s.frame_targets.find(v);
                targets.push_back(
                    {Target::Section,
                     v,
                     input.vertices.row(f.center),
                     f.rotation * Vector3d::UnitZ(),
                     slot == s.frame_targets.end() ? Vector3d(input.vertices.row(v)) : slot->second,
                     s.radius,
                     Vector3d::Zero()});
            } else {
                double best = infinity;
                Target target;
                const Vector3d p = input.vertices.row(v);
                for (const auto& e : s.input_edges) {
                    if (e[0] != parent && e[1] != parent) continue;
                    const Vector3d a = input.vertices.row(e[0]), b = input.vertices.row(e[1]);
                    const Vector3d ab = b - a;
                    if (!(ab.squaredNorm() > 0)) continue;
                    const Vector3d foot =
                        a + std::clamp((p - a).dot(ab) / ab.squaredNorm(), 0., 1.) * ab;
                    if ((p - foot).squaredNorm() >= best) continue;
                    best = (p - foot).squaredNorm();
                    target = {Target::Cylinder, v, a, ab.normalized(), p, s.radius, b};
                }
                if (std::isfinite(best)) targets.push_back(target);
            }
        } else if (
            v < input.singular_vertex_tags.size() && input.singular_vertex_tags[v] == 0 &&
            input.optimal_normals.rows() == original.rows() && input.target_thickness > 0) {
            if (parent >= 0)
                targets.push_back(
                    {Target::Shell,
                     v,
                     input.vertices.row(parent),
                     input.optimal_normals.row(v),
                     input.target_positions.row(v),
                     input.target_thickness,
                     Vector3d::Zero()});
        }
    }
    // Preserve already attained physical thickness exactly. Quality smoothing
    // may slide along these planes/circles, but must not spend thickness accuracy.
    for (auto& t : targets) {
        Vector3d d = input.vertices.row(t.vertex).transpose() - t.origin;
        if (t.kind == Target::Shell)
            t.fixed_distance = std::abs(d.dot(t.axis) - t.length) < 1e-9 * t.length;
        else {
            if (t.kind == Target::Section) {
                t.fixed_plane = std::abs(d.dot(t.axis)) < 1e-9 * t.length;
                d -= d.dot(t.axis) * t.axis;
            } else {
                const Vector3d ab = t.end - t.origin;
                d -= std::clamp(d.dot(ab) / ab.squaredNorm(), 0., 1.) * ab;
            }
            t.fixed_distance = std::abs(d.norm() - t.length) < 1e-9 * t.length;
        }
    }
    double quality_ceiling = 0;
    for (const auto& c : prisms)
        quality_ceiling = std::max(quality_ceiling, prism_amips(original, c));
    // All derivatives below are in physical coordinates. A scalar block Hessian
    // keeps the coupled halfspace projection cheap; unframed rod offsets
    // can move to release a blocked section without optimizing their tet shape.
    auto evaluate = [&](MatrixXd* gradient, std::vector<double>* hessian) {
        if (gradient) gradient->setZero(nv, 3);
        if (hessian) {
            hessian->resize(nv);
            for (size_t v = 0; v < nv; ++v) (*hessian)[v] = .01 / (scale[v] * scale[v]);
        }
        double energy = 0;
        const auto residual = [&](size_t v, double value, const Vector3d& g, double weight) {
            energy += weight * value * value;
            if (gradient) gradient->row(v) += (2 * weight * value * g).transpose();
            if (hessian) (*hessian)[v] += 2 * weight * g.squaredNorm();
        };
        for (const auto& t : targets) {
            const Vector3d p = input.vertices.row(t.vertex);
            Vector3d d = p - t.origin;
            if (t.kind == Target::Shell) {
                residual(
                    t.vertex,
                    d.dot(t.axis) / t.length - 1,
                    t.axis / t.length,
                    thickness_weight);
            } else {
                if (t.kind == Target::Section)
                    residual(
                        t.vertex,
                        d.dot(t.axis) / t.length,
                        t.axis / t.length,
                        thickness_weight);
                else {
                    const Vector3d ab = t.end - t.origin;
                    d -= std::clamp(d.dot(ab) / ab.squaredNorm(), 0., 1.) * ab;
                }
                if (t.kind == Target::Section) d -= d.dot(t.axis) * t.axis;
                const double r = d.norm();
                if (r > 0)
                    residual(t.vertex, r / t.length - 1, d / (r * t.length), thickness_weight);
            }
            // Weak material-slot tether; it does not replace the physical distance
            // target. Tangential motions remain available to improve prism shape.
            for (int j = 0; j < 3; ++j)
                residual(
                    t.vertex,
                    (p[j] - t.slot[j]) / t.length,
                    Vector3d::Unit(j) / t.length,
                    .01);
        }
        for (const auto& c : prisms) {
            const auto q = prism_quality(input.vertices, c);
            if (!std::isfinite(q.value)) return infinity;
            energy += q.value * q.value / 9;
            for (size_t j = 0; j < 6; ++j) {
                const size_t v = c.vertices[j];
                if (gradient) gradient->row(v) += (2 * q.value / 9 * q.gradient[j]).transpose();
                if (hessian) (*hessian)[v] += 2. / 9 * q.gradient[j].squaredNorm();
            }
        }
        return energy;
    };
    const auto thickness_statistics = [&]() {
        nlohmann::json out = nlohmann::json::object();
        for (int kind : {Target::Shell, Target::Section, Target::Cylinder}) {
            double sum = 0, maximum = 0, plane_sum = 0, plane_max = 0;
            size_t count = 0;
            for (const auto& t : targets) {
                if (t.kind != kind) continue;
                Vector3d d = input.vertices.row(t.vertex).transpose() - t.origin;
                double error;
                if (t.kind == Target::Shell)
                    error = std::abs(d.dot(t.axis) - t.length);
                else {
                    if (t.kind == Target::Section) {
                        const double z = d.dot(t.axis);
                        plane_sum += z * z;
                        plane_max = std::max(plane_max, std::abs(z));
                        d -= z * t.axis;
                    } else {
                        const Vector3d ab = t.end - t.origin;
                        d -= std::clamp(d.dot(ab) / ab.squaredNorm(), 0., 1.) * ab;
                    }
                    error = std::abs(d.norm() - t.length);
                }
                sum += error * error;
                maximum = std::max(maximum, error);
                ++count;
            }
            const auto name = kind == Target::Shell     ? "shell_offset"
                              : kind == Target::Section ? "rod_sections"
                                                        : "rod_free_surface";
            out[name] = {
                {"count", count},
                {"rms_error", count ? std::sqrt(sum / count) : 0},
                {"maximum_error", maximum}};
            if (kind == Target::Section) {
                out[name]["rms_plane_error"] = count ? std::sqrt(plane_sum / count) : 0;
                out[name]["maximum_plane_error"] = plane_max;
            }
        }
        return out;
    };
    auto& report = hybrid.report["thickness_smoothing"];
    report = {
        {"requested_iterations", iterations},
        {"tet_objective", "validity_only"},
        {"prism_quality_before", quality_statistics(input.vertices, prisms)},
        {"thickness_before", thickness_statistics()},
        {"rod_radius", input.rod_state ? input.rod_state->radius : 0},
        {"shell_offset_thickness", input.target_thickness}};
    report["stop_reason"] = "iteration_limit";
    report["preserve_attained_thickness"] = true;
    report["prism_maximum_ceiling"] = quality_ceiling;
    report["fixed_below_floor_vertices"] = below_floor_vertices.size();
    double energy = evaluate(nullptr, nullptr);
    report["energy_before"] = energy;
    auto history = nlohmann::json::array();
    for (int iteration = 0; iteration < iterations; ++iteration) {
        MatrixXd gradient;
        std::vector<double> hessian;
        energy = evaluate(&gradient, &hessian);
        MatrixXd step = MatrixXd::Zero(nv, 3);
        std::vector<double> inverse(nv, 0);
        for (size_t v = 0; v < nv; ++v)
            if (movable[v]) {
                inverse[v] = 1 / hessian[v];
                step.row(v) = -inverse[v] * gradient.row(v);
            }
        std::vector<Constraint> constraints;
        const auto add = [&](const auto& vertices, const auto& gradients, double bound) {
            Constraint c;
            double norm = 0;
            for (size_t j = 0; j < vertices.size(); ++j)
                if (movable[vertices[j]]) {
                    c.gradient.emplace_back(vertices[j], gradients[j]);
                    norm += gradients[j].squaredNorm();
                }
            if (!(norm > 0)) return;
            norm = std::sqrt(norm);
            c.bound = bound / norm;
            for (auto& [v, g] : c.gradient) {
                g /= norm;
                c.denominator += inverse[v] * g.squaredNorm();
            }
            constraints.push_back(std::move(c));
        };
        for (size_t i = 0; i < tets.size(); ++i) {
            double det;
            const auto g = tet_gradient(input.vertices, tets[i], det);
            add(tets[i], g, -.8 * std::max(0., det - floors[i]));
        }
        for (const auto& c : prisms) {
            const auto minimum = minimum_prism_jacobian(input.vertices, c);
            for (const auto& q : minimum.edge_points) {
                const auto j = prism_jacobian(input.vertices, c, q);
                add(c.vertices, j.gradients, -.8 * j.determinant);
            }
            const auto quality = prism_quality(input.vertices, c);
            auto g = quality.gradient;
            for (auto& v : g) v = -v;
            add(c.vertices, g, quality.value - quality_ceiling);
        }
        for (const auto& t : targets) {
            const auto equality = [&](double error, const Vector3d& normal) {
                for (double sign : {-1., 1.})
                    add(std::array<size_t, 1>{t.vertex},
                        std::array<Vector3d, 1>{sign * normal},
                        -sign * error);
            };
            Vector3d d = input.vertices.row(t.vertex).transpose() - t.origin;
            if (t.kind == Target::Shell) {
                if (t.fixed_distance) equality(d.dot(t.axis) - t.length, t.axis);
            } else {
                if (t.kind == Target::Section) {
                    if (t.fixed_plane) equality(d.dot(t.axis), t.axis);
                    d -= d.dot(t.axis) * t.axis;
                } else {
                    const Vector3d ab = t.end - t.origin;
                    d -= std::clamp(d.dot(ab) / ab.squaredNorm(), 0., 1.) * ab;
                }
                if (t.fixed_distance && d.norm() > 0) equality(d.norm() - t.length, d.normalized());
            }
        }
        for (size_t v = 0; v < nv; ++v)
            if (movable[v])
                for (int j = 0; j < 3; ++j)
                    for (double sign : {-1., 1.})
                        add(std::array<size_t, 1>{v},
                            std::array<Vector3d, 1>{sign * Vector3d::Unit(j)},
                            -.2 * scale[v] / std::sqrt(3.));
        // Dykstra / dual coordinate ascent: project the full proposed displacement
        // in the block Hessian metric. A zero step is always feasible. Unlike
        // pointwise backtracking, this releases neighboring free offset vertices.
        double violation = 0;
        for (int sweep = 0; sweep < 400; ++sweep) {
            violation = 0;
            for (auto& c : constraints) {
                double gap = c.bound;
                for (const auto& [v, g] : c.gradient) gap -= g.dot(step.row(v));
                violation = std::max(violation, gap);
                const double delta = std::max(-c.multiplier, gap / c.denominator);
                c.multiplier += delta;
                for (const auto& [v, g] : c.gradient)
                    step.row(v) += (delta * inverse[v] * g).transpose();
            }
            if (sweep > 10 && violation < 1e-12) break;
        }
        double slope = (gradient.array() * step.array()).sum();
        if (!(slope < 0) || !step.allFinite()) {
            report["stop_reason"] = "no_feasible_descent_found";
            break;
        }
        const MatrixXd before = input.vertices;
        std::vector<double> clearance(tets.size());
        for (size_t i = 0; i < tets.size(); ++i) {
            double det;
            tet_gradient(before, tets[i], det);
            clearance[i] = floors[i] + .1 * std::max(0., det - floors[i]);
        }
        std::vector<double> prism_clearance;
        for (const auto& c : prisms)
            prism_clearance.push_back(.1 * minimum_prism_jacobian(before, c).determinant);
        double alpha = 1, next = energy;
        bool accepted = false;
        for (int backtrack = 0; backtrack < 45; ++backtrack, alpha *= .5) {
            input.vertices = before + alpha * step;
            // Finite coupled moves contain quadratic/cubic volume terms. Repair
            // those terms at the candidate coordinates, rather than letting a
            // nearly flat tet force every independent section's step to zero.
            // This is a feasibility projection, with no tet shape objective.
            for (int sweep = 0; sweep < 60; ++sweep) {
                bool corrected = false;
                const auto repair = [&](const auto& vertices, const auto& g, double gap) {
                    if (!(gap > 0)) return;
                    double denominator = 0;
                    for (size_t j = 0; j < vertices.size(); ++j)
                        denominator += inverse[vertices[j]] * g[j].squaredNorm();
                    if (!(denominator > 0)) return;
                    const double delta = gap / denominator;
                    for (size_t j = 0; j < vertices.size(); ++j)
                        input.vertices.row(vertices[j]) +=
                            (delta * inverse[vertices[j]] * g[j]).transpose();
                    corrected = true;
                };
                for (size_t i = 0; i < tets.size(); ++i) {
                    double det;
                    const auto g = tet_gradient(input.vertices, tets[i], det);
                    repair(tets[i], g, clearance[i] - det);
                }
                for (size_t i = 0; i < prisms.size(); ++i) {
                    const auto& c = prisms[i];
                    const auto minimum = minimum_prism_jacobian(input.vertices, c);
                    for (const auto& q : minimum.edge_points) {
                        const auto j = prism_jacobian(input.vertices, c, q);
                        repair(c.vertices, j.gradients, prism_clearance[i] - j.determinant);
                    }
                    const auto quality = prism_quality(input.vertices, c);
                    if (std::isfinite(quality.value)) {
                        auto g = quality.gradient;
                        for (auto& v : g) v = -v;
                        repair(c.vertices, g, quality.value - quality_ceiling);
                    }
                }
                for (const auto& t : targets) {
                    Vector3d p = input.vertices.row(t.vertex), d = p - t.origin;
                    const Vector3d previous = p;
                    if (t.kind == Target::Shell) {
                        if (t.fixed_distance) p -= (d.dot(t.axis) - t.length) * t.axis;
                    } else {
                        Vector3d foot = t.origin;
                        if (t.kind == Target::Section) {
                            const double z = d.dot(t.axis);
                            if (t.fixed_plane)
                                p -= z * t.axis;
                            else
                                foot += z * t.axis;
                        } else {
                            const Vector3d ab = t.end - t.origin;
                            foot += std::clamp(d.dot(ab) / ab.squaredNorm(), 0., 1.) * ab;
                        }
                        const Vector3d ray = p - foot;
                        if (t.fixed_distance && ray.norm() > 0)
                            p = foot + t.length * ray.normalized();
                    }
                    input.vertices.row(t.vertex) = p.transpose();
                    if ((p - previous).norm() > 1e-12 * t.length) corrected = true;
                }
                if (!corrected || !input.vertices.allFinite()) break;
            }
            bool valid = true;
            for (size_t v = 0; valid && v < nv; ++v)
                valid = (input.vertices.row(v) - before.row(v)).norm() <= .4 * scale[v];
            for (size_t i = 0; valid && i < tets.size(); ++i)
                if (std::any_of(tets[i].begin(), tets[i].end(), [&](size_t v) {
                        return movable[v];
                    }))
                    valid = tet_volume_above_threshold(input.vertices, tets[i], floors[i] / 6);
            for (const auto& c : prisms)
                if (valid) valid = prism_amips(input.vertices, c) <= quality_ceiling * (1 + 1e-10);
            if (!valid) continue;
            next = evaluate(nullptr, nullptr);
            const double actual_slope =
                (gradient.array() * (input.vertices - before).array()).sum();
            if (next < energy && next <= energy + 1e-4 * std::min(0., actual_slope)) {
                accepted = true;
                break;
            }
        }
        if (!accepted) {
            input.vertices = before;
            report["stop_reason"] = "line_search_stalled";
            break;
        }
        history.push_back(
            {{"iteration", iteration + 1},
             {"energy", next},
             {"alpha", alpha},
             {"projection_violation", violation}});
        logger().info(
            "Thickness/prism smoothing {}: energy {} -> {}, alpha {}",
            iteration + 1,
            energy,
            next,
            alpha);
        if (energy - next < 1e-11 * std::max(1., energy)) {
            report["stop_reason"] = "converged";
            break;
        }
    }
    validate_prism_dominant_mesh(input, hybrid, min_tet_volume);
    report["iterations"] = history;
    report["energy_after"] = evaluate(nullptr, nullptr);
    report["prism_quality_after"] = quality_statistics(input.vertices, prisms);
    report["thickness_after"] = thickness_statistics();
    report["maximum_displacement"] = (input.vertices - original).rowwise().norm().maxCoeff();
    report["reference_and_native_cells_positive"] = true;
    report["connectivity_unchanged"] = true;
}
} // namespace wmtk::components::prismatic_mesh
