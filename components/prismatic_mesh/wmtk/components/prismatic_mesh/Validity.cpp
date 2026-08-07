#include "Types.hpp"

#include <wmtk/utils/orient.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace wmtk::components::prismatic_mesh {
namespace {

double cross2(const Vector2d& a, const Vector2d& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

int orientation(const Vector2d& a, const Vector2d& b, const Vector2d& c, const double eps)
{
    const double value = cross2(b - a, c - a);
    return value > eps ? 1 : (value < -eps ? -1 : 0);
}

bool on_segment(const Vector2d& a, const Vector2d& b, const Vector2d& p, const double eps)
{
    return p.x() >= std::min(a.x(), b.x()) - eps && p.x() <= std::max(a.x(), b.x()) + eps &&
           p.y() >= std::min(a.y(), b.y()) - eps && p.y() <= std::max(a.y(), b.y()) + eps &&
           std::abs(cross2(b - a, p - a)) <= eps;
}

bool segments_intersect(
    const Vector2d& a,
    const Vector2d& b,
    const Vector2d& c,
    const Vector2d& d,
    const double eps)
{
    const int o1 = orientation(a, b, c, eps);
    const int o2 = orientation(a, b, d, eps);
    const int o3 = orientation(c, d, a, eps);
    const int o4 = orientation(c, d, b, eps);
    if (o1 != o2 && o3 != o4) return true;
    return (o1 == 0 && on_segment(a, b, c, eps)) || (o2 == 0 && on_segment(a, b, d, eps)) ||
           (o3 == 0 && on_segment(c, d, a, eps)) || (o4 == 0 && on_segment(c, d, b, eps));
}

double
prism_jacobian(const std::array<Vector3d, 6>& p, const double r, const double s, const double t)
{
    const Vector3d dr = (1 - t) * (p[1] - p[0]) + t * (p[4] - p[3]);
    const Vector3d ds = (1 - t) * (p[2] - p[0]) + t * (p[5] - p[3]);
    const Vector3d dt = (1 - r - s) * (p[3] - p[0]) + r * (p[4] - p[1]) + s * (p[5] - p[2]);
    return dr.dot(ds.cross(dt));
}

double pyramid_jacobian(const std::array<Vector3d, 5>& p, const double r, const double s)
{
    const Vector3d base =
        (1 - r) * (1 - s) * p[0] + r * (1 - s) * p[1] + r * s * p[2] + (1 - r) * s * p[3];
    const Vector3d dr = (1 - s) * (p[1] - p[0]) + s * (p[2] - p[3]);
    const Vector3d ds = (1 - r) * (p[3] - p[0]) + r * (p[2] - p[1]);
    return dr.dot(ds.cross(p[4] - base));
}

std::array<double, 3>
quadratic_bernstein_coefficients(const double f0, const double fmid, const double f1)
{
    return {{f0, 2 * fmid - 0.5 * (f0 + f1), f1}};
}

bool valid_prism_patch(
    const std::array<Vector3d, 6>& p,
    const std::array<Vector2d, 3>& triangle,
    const double t0,
    const double t1,
    const double tolerance,
    const size_t depth,
    const size_t max_depth)
{
    std::array<std::array<double, 3>, 3> coefficients;
    bool sampled_non_positive = false;
    for (size_t vertex = 0; vertex < 3; ++vertex) {
        const double r = triangle[vertex].x();
        const double s = triangle[vertex].y();
        const double f0 = prism_jacobian(p, r, s, t0);
        const double fm = prism_jacobian(p, r, s, 0.5 * (t0 + t1));
        const double f1 = prism_jacobian(p, r, s, t1);
        sampled_non_positive =
            sampled_non_positive || f0 <= tolerance || fm <= tolerance || f1 <= tolerance;
        coefficients[vertex] = quadratic_bernstein_coefficients(f0, fm, f1);
    }
    if (sampled_non_positive) return false;

    bool strictly_positive = true;
    for (const auto& row : coefficients) {
        for (const double coefficient : row) {
            strictly_positive = strictly_positive && coefficient > tolerance;
        }
    }
    if (strictly_positive) return true;
    if (depth == max_depth) return false;

    const Vector2d ab = 0.5 * (triangle[0] + triangle[1]);
    const Vector2d bc = 0.5 * (triangle[1] + triangle[2]);
    const Vector2d ca = 0.5 * (triangle[2] + triangle[0]);
    const std::array<std::array<Vector2d, 3>, 4> children = {{
        {{triangle[0], ab, ca}},
        {{ab, triangle[1], bc}},
        {{ca, bc, triangle[2]}},
        {{ab, bc, ca}},
    }};
    const double tm = 0.5 * (t0 + t1);
    for (const auto& child : children) {
        if (!valid_prism_patch(p, child, t0, tm, tolerance, depth + 1, max_depth) ||
            !valid_prism_patch(p, child, tm, t1, tolerance, depth + 1, max_depth)) {
            return false;
        }
    }
    return true;
}

bool valid_pyramid_patch(
    const std::array<Vector3d, 5>& p,
    const double r0,
    const double r1,
    const double s0,
    const double s1,
    const double tolerance,
    const size_t depth,
    const size_t max_depth)
{
    std::array<std::array<double, 3>, 3> samples;
    for (size_t i = 0; i < 3; ++i) {
        const double r = i == 0 ? r0 : (i == 1 ? 0.5 * (r0 + r1) : r1);
        for (size_t j = 0; j < 3; ++j) {
            const double s = j == 0 ? s0 : (j == 1 ? 0.5 * (s0 + s1) : s1);
            samples[i][j] = pyramid_jacobian(p, r, s);
            if (samples[i][j] <= tolerance) return false;
        }
    }

    // Convert the biquadratic polynomial from its 3x3 value lattice to the
    // tensor-product Bernstein basis. Every positive coefficient proves a
    // positive lower bound over the complete parameter patch.
    std::array<std::array<double, 3>, 3> along_s;
    for (size_t i = 0; i < 3; ++i) {
        along_s[i] = quadratic_bernstein_coefficients(samples[i][0], samples[i][1], samples[i][2]);
    }
    std::array<std::array<double, 3>, 3> coefficients;
    for (size_t j = 0; j < 3; ++j) {
        const auto column =
            quadratic_bernstein_coefficients(along_s[0][j], along_s[1][j], along_s[2][j]);
        for (size_t i = 0; i < 3; ++i) coefficients[i][j] = column[i];
    }

    bool strictly_positive = true;
    for (const auto& row : coefficients) {
        for (const double coefficient : row) {
            strictly_positive = strictly_positive && coefficient > tolerance;
        }
    }
    if (strictly_positive) return true;
    if (depth == max_depth) return false;

    const double rm = 0.5 * (r0 + r1);
    const double sm = 0.5 * (s0 + s1);
    return valid_pyramid_patch(p, r0, rm, s0, sm, tolerance, depth + 1, max_depth) &&
           valid_pyramid_patch(p, rm, r1, s0, sm, tolerance, depth + 1, max_depth) &&
           valid_pyramid_patch(p, r0, rm, sm, s1, tolerance, depth + 1, max_depth) &&
           valid_pyramid_patch(p, rm, r1, sm, s1, tolerance, depth + 1, max_depth);
}

} // namespace

bool is_valid_cross_section(const std::vector<Vector2d>& polygon, std::string* reason)
{
    auto fail = [reason](const std::string& message) {
        if (reason != nullptr) *reason = message;
        return false;
    };
    if (polygon.size() < 3) return fail("at least three vertices are required");

    double scale = 0;
    for (const auto& p : polygon) {
        if (!p.allFinite()) return fail("coordinates must be finite");
        scale = std::max(scale, p.norm());
    }
    if (!(scale > 0)) return fail("polygon has zero scale");
    const double eps = std::max(1e-14, scale * 1e-12);

    double area2 = 0;
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Vector2d& a = polygon[i];
        const Vector2d& b = polygon[(i + 1) % polygon.size()];
        if ((a - b).norm() <= eps) return fail("consecutive vertices must be distinct");
        area2 += cross2(a, b);
    }
    if (area2 <= eps * eps) return fail("polygon must be non-degenerate and counter-clockwise");

    for (size_t i = 0; i < polygon.size(); ++i) {
        const size_t i_next = (i + 1) % polygon.size();
        for (size_t j = i + 1; j < polygon.size(); ++j) {
            const size_t j_next = (j + 1) % polygon.size();
            if (i == j || i_next == j || j_next == i) continue;
            if (segments_intersect(polygon[i], polygon[i_next], polygon[j], polygon[j_next], eps)) {
                return fail("polygon self-intersects");
            }
        }
    }

    // For a CCW simple polygon, its kernel is the intersection of the left
    // half-planes of all directed boundary edges.
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Vector2d& a = polygon[i];
        const Vector2d& b = polygon[(i + 1) % polygon.size()];
        if (cross2(b - a, -a) < -eps) {
            return fail("the origin is not in the polygon kernel");
        }
    }
    return true;
}

bool is_valid_tet(const std::array<Vector3d, 4>& p, const double tolerance)
{
    if (tolerance == 0) {
        return wmtk::utils::orient3d(p[0], p[1], p[2], p[3]);
    }
    return (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0])) > tolerance;
}

bool is_valid_prism(
    const std::array<Vector3d, 6>& p,
    const double tolerance,
    const size_t max_depth)
{
    return valid_prism_patch(p, {{{0, 0}, {1, 0}, {0, 1}}}, 0, 1, tolerance, 0, max_depth);
}

bool is_valid_pyramid(
    const std::array<Vector3d, 5>& p,
    const double tolerance,
    const size_t max_depth)
{
    // The collapsed pyramid map contributes a nonnegative (1-t)^2 factor.
    // Its sign is therefore governed by this biquadratic base-domain
    // polynomial, with no singular evaluation at the apex.
    return valid_pyramid_patch(p, 0, 1, 0, 1, tolerance, 0, max_depth);
}

} // namespace wmtk::components::prismatic_mesh
