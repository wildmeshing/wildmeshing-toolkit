#include "BallRefinement.hpp"

#include <Eigen/Geometry> // cross

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace wmtk::utils {

namespace {

template <typename Vec>
double point_segment_distance(const Vec& p, const Vec& a, const Vec& b)
{
    const Vec ab = b - a;
    const double len2 = ab.squaredNorm();
    const double t = len2 > 0 ? std::clamp((p - a).dot(ab) / len2, 0., 1.) : 0.;
    return (p - a - t * ab).norm();
}

double cross2(const Eigen::Vector2d& u, const Eigen::Vector2d& v)
{
    return u[0] * v[1] - u[1] * v[0];
}

double orient3(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
    return (b - a).cross(c - a).dot(d - a);
}

template <int D>
bool certify_on_cell_impl(
    const std::array<Eigen::Matrix<double, D, 1>, D + 1>& cell,
    const std::array<double, D + 1>& corner_values,
    const std::function<double(const Eigen::Matrix<double, D, 1>&)>& f,
    const double level,
    const bool at_most,
    const int max_depth,
    const double margin)
{
    const double lo = level - margin * std::abs(level);
    const double hi = level + margin * std::abs(level);
    // Written so that NaN refutes: a comparison with NaN is false.
    for (const double v : corner_values) {
        if (at_most ? !(v <= lo) : !(v > hi)) {
            return false;
        }
    }
    struct Item
    {
        RefinementBall<D> ball;
        int depth;
    };
    std::vector<Item> stack{{RefinementBall<D>::bounding(cell), 0}};
    while (!stack.empty()) {
        const Item it = stack.back();
        stack.pop_back();
        const RefinementBall<D>& b = it.ball;
        const double fc = f(b.center);
        if (at_most ? fc + b.radius <= lo : fc - b.radius > hi) {
            continue; // certified
        }
        if (point_cell_distance(b.center, cell) == 0. && (at_most ? fc > level : fc <= level)) {
            return false; // a point of the cell on the wrong side
        }
        if (it.depth >= max_depth) {
            return false; // undecided: refuted, conservatively
        }
        for (const RefinementBall<D>& child : b.refined()) {
            if (child.overlaps(cell)) {
                stack.push_back({child, it.depth + 1});
            }
        }
    }
    return true;
}

} // namespace

template <int D>
void fit_box(
    const std::array<Eigen::Matrix<double, D, 1>, D + 1>& cell,
    Eigen::Matrix<double, D, 1>& center,
    double& side)
{
    Eigen::Matrix<double, D, 1> mn = cell[0];
    Eigen::Matrix<double, D, 1> mx = cell[0];
    for (const auto& p : cell) {
        mn = mn.cwiseMin(p);
        mx = mx.cwiseMax(p);
    }
    center = (mn + mx) / 2;
    side = (mx - mn).maxCoeff();
}

template <int D>
RefinementBall<D> RefinementBall<D>::bounding(const Cell& cell)
{
    RefinementBall b;
    double side = 0;
    fit_box<D>(cell, b.center, side);
    b.radius = side * std::sqrt(double(D)) / 2;
    return b;
}

template <int D>
std::array<RefinementBall<D>, (1 << D)> RefinementBall<D>::refined() const
{
    // the sub-squares' centres are a quarter side, radius / (2 sqrt(D)) per axis, away
    const double off = radius / (2 * std::sqrt(double(D)));
    std::array<RefinementBall, (1 << D)> out;
    for (int s = 0; s < (1 << D); ++s) {
        out[s].center = center;
        for (int k = 0; k < D; ++k) {
            out[s].center[k] += (s >> k & 1) ? off : -off;
        }
        out[s].radius = radius / 2;
    }
    return out;
}

template <int D>
bool RefinementBall<D>::overlaps(const Cell& cell) const
{
    return !(point_cell_distance(center, cell) >= radius); // NaN counts as overlapping
}

template struct RefinementBall<2>;
template struct RefinementBall<3>;
template void fit_box<2>(const std::array<Eigen::Vector2d, 3>&, Eigen::Vector2d&, double&);
template void fit_box<3>(const std::array<Eigen::Vector3d, 4>&, Eigen::Vector3d&, double&);

double point_cell_distance(const Eigen::Vector2d& p, const std::array<Eigen::Vector2d, 3>& cell)
{
    // inside: on the same side of each edge as the opposite corner (or on the edge)
    bool inside = true;
    double best = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 3; ++k) {
        const Eigen::Vector2d& a = cell[k];
        const Eigen::Vector2d& b = cell[(k + 1) % 3];
        const Eigen::Vector2d& opp = cell[(k + 2) % 3];
        if (cross2(b - a, opp - a) * cross2(b - a, p - a) < 0) {
            inside = false;
        }
        best = std::min(best, point_segment_distance(p, a, b));
    }
    return inside ? 0. : best;
}

double point_triangle_distance(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    // Ericson, Real-Time Collision Detection 5.1.5: the closest point by Voronoi regions
    const Eigen::Vector3d ab = b - a, ac = c - a, ap = p - a;
    const double d1 = ab.dot(ap), d2 = ac.dot(ap);
    if (d1 <= 0 && d2 <= 0) return ap.norm();
    const Eigen::Vector3d bp = p - b;
    const double d3 = ab.dot(bp), d4 = ac.dot(bp);
    if (d3 >= 0 && d4 <= d3) return bp.norm();
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return point_segment_distance(p, a, b);
    const Eigen::Vector3d cp = p - c;
    const double d5 = ab.dot(cp), d6 = ac.dot(cp);
    if (d6 >= 0 && d5 <= d6) return cp.norm();
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return point_segment_distance(p, a, c);
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && d4 - d3 >= 0 && d5 - d6 >= 0) return point_segment_distance(p, b, c);
    const double denom = va + vb + vc;
    if (!(denom > 0)) { // a degenerate triangle: its edges
        return std::min(
            {point_segment_distance(p, a, b),
             point_segment_distance(p, b, c),
             point_segment_distance(p, c, a)});
    }
    return (p - (a + ab * (vb / denom) + ac * (vc / denom))).norm();
}

double point_cell_distance(const Eigen::Vector3d& p, const std::array<Eigen::Vector3d, 4>& cell)
{
    // inside: on the same side of each face as the opposite corner (or on the face)
    bool inside = true;
    double best = std::numeric_limits<double>::infinity();
    for (int k = 0; k < 4; ++k) {
        const Eigen::Vector3d& a = cell[(k + 1) % 4];
        const Eigen::Vector3d& b = cell[(k + 2) % 4];
        const Eigen::Vector3d& c = cell[(k + 3) % 4];
        if (orient3(a, b, c, cell[k]) * orient3(a, b, c, p) < 0) {
            inside = false;
        }
        best = std::min(best, point_triangle_distance(p, a, b, c));
    }
    return inside ? 0. : best;
}

bool certify_on_cell(
    const std::array<Eigen::Vector2d, 3>& cell,
    const std::array<double, 3>& corner_values,
    const std::function<double(const Eigen::Vector2d&)>& f,
    const double level,
    const bool at_most,
    const int max_depth,
    const double margin)
{
    return certify_on_cell_impl<2>(cell, corner_values, f, level, at_most, max_depth, margin);
}

bool certify_on_cell(
    const std::array<Eigen::Vector3d, 4>& cell,
    const std::array<double, 4>& corner_values,
    const std::function<double(const Eigen::Vector3d&)>& f,
    const double level,
    const bool at_most,
    const int max_depth,
    const double margin)
{
    return certify_on_cell_impl<3>(cell, corner_values, f, level, at_most, max_depth, margin);
}

} // namespace wmtk::utils
