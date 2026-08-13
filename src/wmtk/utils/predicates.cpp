#include <wmtk/utils/predicates.hpp>

// The exact-predicate backend, and the ONLY place in the toolkit that names it. Everything
// goes through the wrappers in the header, so changing backend again is a change to this
// file alone rather than to the call sites.
//
// Marco Attene's Indirect_Predicates, reached through fast-envelope, which fetches and pins
// it. orient2d/orient3d here are the direct (non-indirect) predicates: a semi-static double
// filter first, falling back to exact expansion arithmetic only when the filter cannot
// decide the sign.
#include <implicit_point.h>

namespace wmtk::utils::predicates::detail {

int orient2d_sign(
    const double ax,
    const double ay,
    const double bx,
    const double by,
    const double cx,
    const double cy)
{
    return ::orient2d(ax, ay, bx, by, cx, cy);
}

int orient3d_sign(
    const double ax,
    const double ay,
    const double az,
    const double bx,
    const double by,
    const double bz,
    const double cx,
    const double cy,
    const double cz,
    const double dx,
    const double dy,
    const double dz)
{
    // Negated: the backend orients det[b-a; c-a; d-a] where Shewchuk orients
    // det[a-d; b-d; c-d], and the two are opposite. See the note on orient3d() in the header
    // for the measurement.
    return -::orient3d(ax, ay, az, bx, by, bz, cx, cy, cz, dx, dy, dz);
}

} // namespace wmtk::utils::predicates::detail
