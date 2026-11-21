#pragma once

#include "dyadic.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace wmtk::components::tracking {

/// A positively oriented triangle in the plane. Triangles of one triangulation name a shared
/// corner by the same point, bit for bit: that is how a walk finds its way between them.
using Triangle2 = std::array<Eigen::Vector2d, 3>;

/// A triangle with its corners in homogeneous coordinates: (x, y, 1) for a corner in the plane,
/// or the barycentric coordinates of a corner of a piece of a face, in the face. Positively
/// oriented: det[c0; c1; c2] > 0. As for Triangle2, shared corners are the same three doubles.
using TriangleH = std::array<Eigen::Vector3d, 3>;

/// (x, y, 1) for each corner.
TriangleH lift(const Triangle2& t);

/// A point on a triangle, by barycentric coordinates. A point on an edge or a corner of it has
/// exact zeros, and a walk leaves those where they are. The coordinates are homogeneous: the
/// point is sum_k bary[k] c_k, up to a positive factor, exactly as the doubles say -- they need not
/// sum to 1.
struct WalkPoint
{
    size_t tri;
    Eigen::Vector3d bary;
};

/// The part of a segment on one triangle, from one end to the other.
struct WalkPiece
{
    size_t tri;
    std::array<Eigen::Vector3d, 2> bary;
};

/**
 * Exact geometry on points given by non-negative weights on the corners of a triangle,
 * sum_k w_k c_k, in homogeneous coordinates, in adaptive precision.
 *
 * A weight is a number known exactly or only approximately (Hom, Value). Every computation runs
 * at a precision (Precision) with a proven bound on its error, the bound growing with the error
 * of its inputs: a sign is taken when the bound proves it, and a value comes with its bound. The
 * precisions are doubles, double-doubles (a value as hi + lo, about 106 bits) and exact (Dyadic:
 * every value here is a sum of products of doubles, since nothing divides). Where the bound cannot
 * prove a sign, the sign is computed exactly if the inputs are known exactly, and otherwise the
 * computation gives up (undecided()) for the caller to do it again at a higher precision. Nothing
 * is ever decided by a tolerance.
 */
namespace exact {

using Exact3 = std::array<Dyadic, 3>;

/// Homogeneous coordinates: v + lo, componentwise within e of the exact coordinates (lo is zero
/// but for double-double values); x holds them exactly where they are known and v + lo is not
/// exact. So: e == 0 and no x, v + lo is exact; x set, x is exact; e > 0 and no x, only v + lo and
/// its bound are known. A zero in v is an exact zero, and every non-zero v has the sign of the
/// exact coordinate.
///
/// `stored` marks coordinates exactly as a curve holds them (v, lo and the further terms in x, see
/// SurfaceMap): carried along unchanged, they are written back term for term, so a point on the
/// outline of a local map keeps its representation bit for bit.
struct Hom
{
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d lo = Eigen::Vector3d::Zero();
    Eigen::Vector3d e = Eigen::Vector3d::Zero();
    std::shared_ptr<const Exact3> x;
    bool stored = false;

    /// Exact doubles.
    static Hom of(const Eigen::Vector3d& v);
    /// Whether the exact coordinates are known.
    bool known() const { return x || (e.array() == 0).all(); }
    /// The exact coordinates; known() must hold.
    Exact3 value() const;
    int zeros() const { return int(v[0] == 0) + int(v[1] == 0) + int(v[2] == 0); }
};

/// Whether a and b are the same point: by their exact coordinates if both are known; false if
/// their values differ beyond their bounds; gives up (undecided()) if those cannot tell.
bool same(const Hom& a, const Hom& b);

/// A number: v + lo within e of the exact value, which x holds where known and v + lo is not
/// exact. A non-zero v has the exact sign; v == 0 means the value is exactly 0.
struct Value
{
    double v = 0, lo = 0, e = 0;
    std::shared_ptr<const Dyadic> x;
    int sign() const { return (v > 0) - (v < 0); }
    bool known() const { return x || e == 0; }
    Dyadic value() const;
};
/// Coordinate k of h.
Value coordinate(const Hom& h, int k);
/// Coordinates from three values.
Hom hom(const Value& a, const Value& b, const Value& c);
/// -v.
Value negated(Value v);
/// h scaled to sum about 1: divided by its sum (in doubles or double-doubles), or exactly,
/// multiplied by a power of two (to a sum between 1 and 2), so that the exact coordinates stay
/// dyadic. The same point.
Hom normalized(const Hom& h);

/// Whether a computation on this thread gave up since clear_undecided(): it could not go on from
/// approximate values, a sign being too close to call. What it returned then means nothing; the
/// caller computes again at a higher precision. (A flag rather than an exception: it is raised
/// often, and unwinding costs far more than the computation.)
bool undecided();
void clear_undecided();
/// Raises the flag.
void give_up();

/// The precision values are computed at, on this thread, while a Precision is alive: doubles,
/// double-doubles, or exact (every value then has x, from inputs that must be known exactly).
enum Level : int { kDouble = 0, kDoubleDouble = 1, kExact = 2 };
class Precision
{
public:
    explicit Precision(Level level);
    ~Precision();
    Precision(const Precision&) = delete;
    Precision& operator=(const Precision&) = delete;

private:
    Level m_previous;
};
Level precision();
inline bool exact_mode()
{
    return precision() == kExact;
}

/// A point: weights on the corners of a triangle.
struct Point
{
    const TriangleH* t;
    Hom w;
};
/// The corner k of t.
Point corner(const TriangleH& t, int k);

/// The sign of det[a; b; c]: for points of the plane, 1 if (a, b, c) turn counterclockwise, -1 if
/// clockwise, 0 if they are on a line.
int orient(const Point& a, const Point& b, const Point& c);
/// det[a; b; c], with its exact sign and a bound on its error. The filter measures from a corner
/// near the points, so its bound scales with their distances, not with their coordinates.
Value det(const Point& a, const Point& b, const Point& c);
/// a b - c d, for exact doubles b, d.
Value lin2(const Value& a, double b, const Value& c, double d);
/// a + b c, for an exact double c.
Value muladd(const Value& a, const Value& b, double c);

/// How often the filter could not decide a sign at the precision asked, since the start (for
/// statistics; not synchronized).
size_t exact_evaluations();

} // namespace exact

/// A point on a triangle of a triangulation, its coordinates maybe approximate.
struct HWalkPoint
{
    size_t tri;
    exact::Hom bary;
};
/// The part of a segment on one triangle.
struct HWalkPiece
{
    size_t tri;
    std::array<exact::Hom, 2> bary;
};

/// The point `l` of triangle `t` in the coordinates of triangle `n`, if `n` has the corners it is
/// on: a point on a shared edge or corner, which keeps its coordinates and its exact zeros.
std::optional<exact::Hom> carry(const TriangleH& t, const exact::Hom& l, const TriangleH& n);
std::optional<Eigen::Vector3d>
carry(const TriangleH& t, const Eigen::Vector3d& l, const TriangleH& n);
std::optional<Eigen::Vector3d>
carry(const Triangle2& t, const Eigen::Vector3d& l, const Triangle2& n);

/// The first triangle of `tris` holding the point `p`, and p's coordinates there (normalized()):
/// with exact zeros on an edge or a corner. None if no triangle holds it.
std::optional<HWalkPoint> locate(const std::vector<TriangleH>& tris, const exact::Point& p);

/**
 * @brief The straight segment between two points of a triangulation, cut at its edges.
 *
 * Every decision -- which edge or corner the segment leaves a triangle through, whether it runs
 * along an edge -- is taken by exact predicates on the line through the two ends (exact::), so it
 * needs no tolerance. A crossing is given by two weights on the ends of the edge it crosses, both
 * positive, so it is strictly inside the edge; a piece through a corner or along an edge has exact
 * zeros. The pieces follow one another: the end of one is the start of the next, carried into its
 * triangle.
 *
 * Empty if both ends are the same point. None if the walk finds no way: a triangle that is not
 * positively oriented, or a segment that leaves the triangulation. Gives up (exact::undecided())
 * where approximate ends cannot decide.
 */
std::optional<std::vector<HWalkPiece>>
walk(const std::vector<TriangleH>& tris, const HWalkPoint& from, const HWalkPoint& to);
/// walk() between exact points, every value rounded to doubles.
std::optional<std::vector<WalkPiece>>
walk(const std::vector<TriangleH>& tris, const WalkPoint& from, const WalkPoint& to);
/// walk() on triangles of the plane.
std::optional<std::vector<WalkPiece>>
walk_segment(const std::vector<Triangle2>& tris, const WalkPoint& from, const WalkPoint& to);

} // namespace wmtk::components::tracking
