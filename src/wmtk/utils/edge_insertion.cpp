#include "edge_insertion.hpp"

namespace wmtk::utils {

bool segment_intersection_rational(
    const Vector2r& P1,
    const Vector2r& P2,
    const Vector2r& Q1,
    const Vector2r& Q2,
    Vector2r& intersection)
{
    Vector2r P1P2 = P2 - P1;
    Vector2r P2Q1 = Q1 - P2;
    Vector2r P2Q2 = Q2 - P2;
    Vector2r Q1Q2 = Q2 - Q1;
    Vector2r Q2P1 = P1 - Q2;
    Vector2r Q2P2 = P2 - Q2;
    Vector2r P1Q1 = Q1 - P1;

    Rational o1 = P1P2[0] * P2Q1[1] - P1P2[1] * P2Q1[0];
    Rational o2 = P1P2[0] * P2Q2[1] - P1P2[1] * P2Q2[0];
    Rational o3 = Q1Q2[0] * Q2P1[1] - Q1Q2[1] * Q2P1[0];
    Rational o4 = Q1Q2[0] * Q2P2[1] - Q1Q2[1] * Q2P2[0];

    int o1_sgn = o1.get_sign();
    int o2_sgn = o2.get_sign();
    int o3_sgn = o3.get_sign();
    int o4_sgn = o4.get_sign();

    if (o1_sgn == 0 && o2_sgn == 0) {
        // collinear case
        if (Q1[0] >= std::min(P1[0], P2[0]) && Q1[0] <= std::max(P1[0], P2[0])) {
            // Q1 on P1P2, return Q1
            intersection = Q1;
            return true;
        } else if (Q2[0] >= std::min(P1[0], P2[0]) && Q2[0] <= std::max(P1[0], P2[0])) {
            // Q2 on P1P2, return Q2
            intersection = Q2;
            return true;
        }

        // no overlap
        return false;

    } else if (o1_sgn * o2_sgn <= 0 && o3_sgn * o4_sgn <= 0) {
        // general intersect
        Rational t =
            (P1Q1[0] * Q1Q2[1] - P1Q1[1] * Q1Q2[0]) / (P1P2[0] * Q1Q2[1] - P1P2[1] * Q1Q2[0]);
        intersection = P1 + t * P1P2;
        return true;
    }

    return false;
}
} // namespace wmtk::utils
