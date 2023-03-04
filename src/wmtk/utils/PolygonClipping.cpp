#include "PolygonClipping.h"

#include <geogram/numerics/predicates.h>

namespace wmtk {

namespace {

inline GEO::Sign point_is_in_half_plane(
    const Eigen::RowVector2d& p,
    const Eigen::RowVector2d& q1,
    const Eigen::RowVector2d& q2)
{
    return GEO::PCK::orient_2d(q1.data(), q2.data(), p.data());
}

inline bool intersect_segments(
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

double triangle_area_2d(
    const Eigen::RowVector2d& a,
    const Eigen::RowVector2d& b,
    const Eigen::RowVector2d& c)
{
    return ((b.x() - a.x()) * (c.y() - a.y()) - (c.x() - a.x()) * (b.y() - a.y())) / 2.0;
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
    using namespace GEO;
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
    Sign prev_status = point_is_in_half_plane(prev_p, q1, q2);

    for (unsigned int i = 0; i < P_in.rows(); ++i) {
        Eigen::RowVector2d p = P_in.row(i);
        Sign status = point_is_in_half_plane(p, q1, q2);
        if (status != prev_status && status != ZERO && prev_status != ZERO) {
            Eigen::RowVector2d intersect;
            if (intersect_segments(prev_p, p, q1, q2, intersect)) {
                result.push_back(intersect);
            }
        }

        switch (status) {
        case NEGATIVE: break;
        case ZERO: result.push_back(p); break;
        case POSITIVE: result.push_back(p); break;
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

} // namespace wmtk
