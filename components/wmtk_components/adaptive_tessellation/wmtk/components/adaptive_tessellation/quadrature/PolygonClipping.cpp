#include "PolygonClipping.hpp"
#include <predicates.h>

namespace wmtk {

namespace {

template <int Axis, bool Invert>
struct AlignedHalfPlane
{
    static constexpr int axis = Axis;
    static constexpr bool invert = Invert;
    double coord = 0;
};

inline int point_is_in_half_plane(
    const Eigen::RowVector2d& p,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2)
{
    using Point = Eigen::Matrix<double, 2, 1>;
    Point a{p[0], p[1]};
    Point b{q1[0], q1[1]};
    Point c{q2[0], q2[1]};

    return (int)orient2d(b.data(), c.data(), a.data());
}

template <typename HalfPlaneType>
inline int point_is_in_aligned_half_plane(const Eigen::RowVector2d& p, HalfPlaneType half_plane)
{
    if (p[half_plane.axis] == half_plane.coord) {
        return 0;
    } else if (p[half_plane.axis] > half_plane.coord) {
        return half_plane.invert ? -1 : 1;
    } else {
        return half_plane.invert ? 1 : -1;
    }
}

inline bool intersect_lines(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& p2,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2,
    Eigen::RowVector2d& result)
{
    Eigen::RowVector2d Vp = p2 - p1;
    Eigen::RowVector2d Vq = q2 - q1;
    Eigen::RowVector2d pq = q1 - p1;

    double a = Vp(0);
    double b = -Vq(0);
    double c = Vp(1);
    double d = -Vq(1);

    double delta = a * d - b * c;
    if (delta == 0.0) {
        return false;
    }

    double tp = (d * pq(0) - b * pq(1)) / delta;

    result << (1.0 - tp) * p1(0) + tp * p2(0), (1.0 - tp) * p1(1) + tp * p2(1);

    return true;
}

template <typename HalfPlaneType>
inline bool intersect_line_half_plane(
    const Eigen::RowVector2d& p1,
    const Eigen::RowVector2d& p2,
    HalfPlaneType half_plane,
    Eigen::RowVector2d& result)
{
    constexpr int axis = half_plane.axis;

    if (p1[axis] == p2[axis]) {
        return false;
    }

    const double t = (half_plane.coord - p1[axis]) / (p2[axis] - p1[axis]);
    result = (1.0 - t) * p1 + t * p2;

    return true;
}

double triangle_area_2d(
    const Eigen::RowVector2d& a,
    const Eigen::RowVector2d& b,
    const Eigen::RowVector2d& c)
{
    return ((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y())) / 2.0;
}

template <typename HalfPlaneType>
SmallPolygon2d<7> clip_small_poly_by_aligned_half_plane(
    const SmallPolygon2d<7>& poly,
    HalfPlaneType half_plane)
{
    SmallPolygon2d<7> result(0, 2);

    auto push_back = [&](const Eigen::RowVector2d& p) {
        assert(result.rows() != 7);
        int idx = static_cast<int>(result.rows());
        result.conservativeResize(idx + 1, Eigen::NoChange);
        result.row(idx) = p;
    };

    if (poly.rows() == 0) {
        return result;
    }

    if (poly.rows() == 1) {
        if (point_is_in_aligned_half_plane(poly.row(0), half_plane)) {
            result = poly.row(0);
        }
        return result;
    }

    Eigen::RowVector2d prev_p = poly.row(poly.rows() - 1);
    int prev_status = point_is_in_aligned_half_plane(prev_p, half_plane);

    for (unsigned int i = 0; i < poly.rows(); ++i) {
        Eigen::RowVector2d p = poly.row(i);
        int status = point_is_in_aligned_half_plane(p, half_plane);
        if (status != prev_status && status != 0 && prev_status != 0) {
            Eigen::RowVector2d intersect;
            if (intersect_line_half_plane(prev_p, p, half_plane, intersect)) {
                push_back(intersect);
            }
        }

        switch (status) {
        case -1: break;
        case 0: push_back(p); break;
        case 1: push_back(p); break;
        default: break;
        }

        prev_p = p;
        prev_status = status;
    }

    return result;
}

} // anonymous namespace

// -----------------------------------------------------------------------------

double polygon_signed_area(const Eigen::MatrixXd& P)
{
    double result = 0;
    for (Eigen::Index i = 1; i + 1 < P.rows(); ++i) {
        result += triangle_area_2d(P.row(0), P.row(i), P.row(i + 1));
    }
    return result;
}

void clip_polygon_by_half_plane(
    const Eigen::MatrixXd& P_in,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2,
    Eigen::MatrixXd& P_out)
{
    assert(P_in.cols() == 2);
    std::vector<Eigen::RowVector2d> result;

    if (P_in.rows() == 0) {
        P_out.resize(0, 2);
        return;
    }

    if (P_in.rows() == 1) {
        if (point_is_in_half_plane(P_in.row(0), q1, q2)) {
            P_out.resize(1, 2);
            P_out << P_in.row(0);
        } else {
            P_out.resize(0, 2);
        }
        return;
    }

    Eigen::RowVector2d prev_p = P_in.row(P_in.rows() - 1);
    int prev_status = point_is_in_half_plane(prev_p, q1, q2);

    for (unsigned int i = 0; i < P_in.rows(); ++i) {
        Eigen::RowVector2d p = P_in.row(i);
        int status = point_is_in_half_plane(p, q1, q2);
        if (status != prev_status && status != 0 && prev_status != 0) {
            Eigen::RowVector2d intersect;
            if (intersect_lines(prev_p, p, q1, q2, intersect)) {
                result.push_back(intersect);
            }
        }

        switch (status) {
        case -1: break;
        case 0: result.push_back(p); break;
        case 1: result.push_back(p); break;
        default: break;
        }

        prev_p = p;
        prev_status = status;
    }

    P_out.resize((int)result.size(), 2);
    for (size_t i = 0; i < result.size(); ++i) {
        P_out.row((int)i) = result[i];
    }
}

SmallPolygon2d<7> clip_triangle_by_box(
    const SmallPolygon2d<3>& triangle,
    const Eigen::AlignedBox2d& box)
{
    SmallPolygon2d<7> result = triangle;

    AlignedHalfPlane<0, false> h0{box.min().x()};
    AlignedHalfPlane<0, true> h1{box.max().x()};
    AlignedHalfPlane<1, false> h2{box.min().y()};
    AlignedHalfPlane<1, true> h3{box.max().y()};

    result = clip_small_poly_by_aligned_half_plane(result, h0);
    result = clip_small_poly_by_aligned_half_plane(result, h1);
    result = clip_small_poly_by_aligned_half_plane(result, h2);
    result = clip_small_poly_by_aligned_half_plane(result, h3);

    return result;
}

} // namespace wmtk
