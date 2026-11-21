#include "SurfaceMap.hpp"

#include "scaffold2d.hpp"
#include "segment_walk.hpp"
#include "slim2d.hpp"

#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/predicates.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <bit>
#include <cfloat>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <set>
#include <unordered_map>

namespace wmtk::components::tracking {

using exact::Dyadic;
using history::Edit;
using history::EditKind;
using history::Replay;

namespace {

using Tri = std::array<uint64_t, 3>;
using UV = std::map<uint64_t, Eigen::Vector2d>;
using Positions = std::map<uint64_t, Eigen::Vector3d>;

constexpr uint64_t kNone = std::numeric_limits<uint64_t>::max();

/// Where a split's parent face is laid out: its corners, in stored order, on these points.
const std::array<Eigen::Vector2d, 3> kCanonical = {
    {Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0), Eigen::Vector2d(0, 1)}};

/// The barycentric coordinates of q in the triangle (a, b, c); not finite if it is degenerate.
Eigen::Vector3d barycentric(
    const Eigen::Vector2d& q,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    const Eigen::Vector2d ab = b - a, ac = c - a, aq = q - a;
    const double det = ab.x() * ac.y() - ab.y() * ac.x();
    const double l1 = (aq.x() * ac.y() - aq.y() * ac.x()) / det;
    const double l2 = (ab.x() * aq.y() - ab.y() * aq.x()) / det;
    return {1 - l1 - l2, l1, l2};
}

/// The position the edit gave `vid`, or `fallback` if it did not move it.
Eigen::Vector3d position_after(const Edit& e, const uint64_t vid, const Eigen::Vector3d& fallback)
{
    for (const auto& m : e.moves) {
        if (m.vid == vid) return m.pos;
    }
    return fallback;
}

/// Barycentric coordinates of p, projected onto the plane of the 3D triangle (a, b, c); not
/// finite if it is degenerate. Taken against the triangle's normal: the normal equations'
/// Gram determinant would square the conditioning, losing eight digits on a sliver of aspect
/// 1e4 -- which a tet surface has after insertion -- and a point mapped there and back would
/// drift by that much.
Eigen::Vector3d barycentric3(const Eigen::Vector3d& p, const std::array<Eigen::Vector3d, 3>& t)
{
    const Eigen::Vector3d e0 = t[1] - t[0], e1 = t[2] - t[0], ep = p - t[0];
    const Eigen::Vector3d n = e0.cross(e1);
    const double nn = n.squaredNorm();
    const double l1 = ep.cross(e1).dot(n) / nn, l2 = e0.cross(ep).dot(n) / nn;
    return {1 - l1 - l2, l1, l2};
}

/// Barycentric coordinates of p in the 3D triangle (a, b, c), kept strictly inside it.
Eigen::Vector3d inside_barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c)
{
    const Eigen::Vector3d e0 = b - a, e1 = c - a, ep = p - a;
    const Eigen::Vector3d n = e0.cross(e1);
    const double nn = n.squaredNorm();
    Eigen::Vector3d l(1. / 3, 1. / 3, 1. / 3);
    if (nn > 0) {
        l[1] = ep.cross(e1).dot(n) / nn;
        l[2] = e0.cross(ep).dot(n) / nn;
        l[0] = 1 - l[1] - l[2];
    }
    for (int k = 0; k < 3; ++k) l[k] = std::max(l[k], 1e-6);
    return l / l.sum();
}

/// Tutte embedding with uniform weights: the vertices of `tris` not in `fixed` go to the average
/// of their neighbours. Returns every vertex's position.
UV tutte(const std::vector<Tri>& tris, const UV& fixed)
{
    std::map<uint64_t, std::set<uint64_t>> nbr;
    for (const Tri& t : tris) {
        for (int i = 0; i < 3; ++i) {
            nbr[t[i]].insert(t[(i + 1) % 3]);
            nbr[t[(i + 1) % 3]].insert(t[i]);
        }
    }
    std::map<uint64_t, int> unknown;
    for (const auto& [v, ns] : nbr) {
        if (!fixed.count(v)) unknown.emplace(v, int(unknown.size()));
    }
    UV uv = fixed;
    if (unknown.empty()) return uv;
    const int n = int(unknown.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd b = Eigen::MatrixXd::Zero(n, 2);
    for (const auto& [v, i] : unknown) {
        A(i, i) = double(nbr[v].size());
        for (const uint64_t w : nbr[v]) {
            if (const auto it = unknown.find(w); it != unknown.end()) {
                A(i, it->second) -= 1;
            } else {
                b.row(i) += fixed.at(w).transpose();
            }
        }
    }
    const Eigen::MatrixXd x = A.partialPivLu().solve(b);
    for (const auto& [v, i] : unknown) uv[v] = x.row(i).transpose();
    return uv;
}

/// The boundary loop of an oriented triangle patch, in the direction of its triangles, if the
/// patch is a disk.
std::optional<std::vector<uint64_t>> disk_boundary(const std::vector<Tri>& tris)
{
    std::set<std::array<uint64_t, 2>> dir;
    std::set<uint64_t> verts;
    for (const Tri& t : tris) {
        for (int c = 0; c < 3; ++c) {
            const std::array<uint64_t, 2> e{{t[c], t[(c + 1) % 3]}};
            if (!dir.insert(e).second) return std::nullopt; // misoriented
            verts.insert(t[c]);
        }
    }
    std::map<uint64_t, uint64_t> next;
    size_t half_edges = 0;
    for (const auto& e : dir) {
        const bool twin = dir.count({{e[1], e[0]}}) > 0;
        half_edges += twin ? 1 : 2;
        if (twin) continue;
        if (!next.emplace(e[0], e[1]).second) return std::nullopt; // pinched at e[0]
    }
    if (next.size() < 3) return std::nullopt;
    std::vector<uint64_t> loop;
    uint64_t cur = next.begin()->first;
    do {
        loop.push_back(cur);
        const auto it = next.find(cur);
        if (it == next.end() || loop.size() > next.size()) return std::nullopt;
        cur = it->second;
    } while (cur != loop.front());
    if (loop.size() != next.size()) return std::nullopt; // more than one loop
    const long n_edges = long(half_edges / 2);
    if (long(verts.size()) - n_edges + long(tris.size()) != 1) return std::nullopt;
    return loop;
}

/// Coordinates of the barycentric point `b` of a face in the piece of it with corners `sub`.
Eigen::Vector3d local_coordinates(
    const std::array<Eigen::Vector3d, 3>& sub,
    const Eigen::Vector3d& b)
{
    Eigen::Matrix3d M;
    for (int k = 0; k < 3; ++k) M.col(k) = sub[k];
    return M.partialPivLu().solve(b);
}

bool positive(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    return utils::predicates::orient2d(a, b, c) == utils::predicates::Orientation::POSITIVE;
}

/// The piece of a face holding a point, among those from `first` to `last`, and the point in it.
std::pair<size_t, Eigen::Vector3d> piece_of(
    const Eigen::Vector3d& bary,
    const std::vector<SurfaceMap::Face>& pieces,
    const size_t first,
    const size_t last)
{
    size_t piece = first;
    Eigen::Vector3d l = bary;
    if (last - first > 1) {
        double best = -std::numeric_limits<double>::infinity();
        for (size_t k = first; k < last; ++k) {
            const Eigen::Vector3d lk = local_coordinates(pieces[k].sub, bary);
            if (lk.minCoeff() > best) {
                best = lk.minCoeff();
                piece = k;
                l = lk;
            }
        }
    }
    return {piece, l};
}

/// Where a point of the domain lands among a cell's pieces.
struct Landing
{
    size_t piece = 0;
    Eigen::Vector3d l = Eigen::Vector3d::Constant(1. / 3);
    /// False if it fell outside them all; l is then clamped onto the nearest.
    bool inside = true;
};

Landing land(const Eigen::Vector2d& q, const std::vector<SurfaceMap::Face>& to)
{
    double best = -std::numeric_limits<double>::infinity();
    Landing at;
    for (size_t k = 0; k < to.size(); ++k) {
        const Eigen::Vector3d lk = barycentric(q, to[k].uv[0], to[k].uv[1], to[k].uv[2]);
        const double score = lk.minCoeff();
        if (std::isfinite(score) && score > best) {
            best = score;
            at.piece = k;
            at.l = lk;
        }
    }
    if (!(best >= -1e-9)) {
        at.inside = false;
        at.l = at.l.cwiseMax(0.);
        if (at.l.sum() > 0) {
            at.l /= at.l.sum();
        } else {
            at.l = Eigen::Vector3d(1. / 3, 1. / 3, 1. / 3);
        }
    }
    return at;
}

/// A face's own corners, in its barycentric coordinates.
const TriangleH kFaceCorners = {
    {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()}};

using exact::Hom;
using exact::Value;

/// Points are mapped to within this, of their coordinates' sum: an accuracy, which decides when a
/// value is computed exactly, not where a point goes.
constexpr double kPointAccuracy = 0x1p-44;
/// A point of a curve is kept to within this fraction of the shortest piece it ends (and within
/// kPointAccuracy): the precision it needs so that no piece turns over, vanishes or swaps with its
/// neighbour when the point is stored. A point a few units in the last place from the next one
/// is so kept to two or more terms.
constexpr double kCurveSafety = 0x1p-20;
constexpr size_t kNoNode = std::numeric_limits<size_t>::max();
/// A point carried from step to step in double-doubles is computed, at each step, this much more
/// precisely than it is stored at the end: hundreds of steps add up to less than the storing.
constexpr double kStepShare = 0x1p-16;

/// The sensitivity of a point: how far the point of the starting mesh it stands for moves, at
/// most, per unit of error in its coordinates (relative to their sum). K takes a change of the
/// point's coordinates, scaled to sum 1, to the displacement in space of the point it came from;
/// such a change sums to zero, so only K's columns about their mean count, and an error e in each
/// coordinate changes the scaled coordinates by at most 4 e each.
double sensitivity(const Eigen::Matrix3d& K)
{
    const Eigen::Vector3d m = (K.col(0) + K.col(1) + K.col(2)) / 3;
    double s = 0;
    for (int k = 0; k < 3; ++k) s += (K.col(k) - m).norm();
    return std::isfinite(s) ? 4 * s : std::numeric_limits<double>::infinity();
}
/// The error (relative to the coordinates' sum) a point of sensitivity `s` may have for the point
/// of the starting mesh to be within `allowed` (a distance) of where it is.
double source_tolerance(const double s, const double allowed)
{
    return s > 0 ? allowed / s : std::numeric_limits<double>::infinity();
}
/// The corners of face f, as columns: coordinates on the face to a point in space.
Eigen::Matrix3d corner_matrix(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<uint64_t, 3>>& faces,
    const uint64_t f)
{
    Eigen::Matrix3d P;
    for (int k = 0; k < 3; ++k) P.col(k) = vertices[faces[f][k]];
    return P;
}
/// The bounding box diagonal of the vertices of the live faces.
double diagonal_of(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<uint64_t, 3>>& faces,
    const std::vector<uint8_t>& alive)
{
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d hi = -lo;
    for (size_t f = 0; f < faces.size(); ++f) {
        if (f >= alive.size() || !alive[f]) continue;
        for (const uint64_t v : faces[f]) {
            lo = lo.cwiseMin(vertices[v]);
            hi = hi.cwiseMax(vertices[v]);
        }
    }
    const double d = (hi - lo).norm();
    return std::isfinite(d) ? d : 0.;
}
/// The linear part of barycentric3 on triangle t: a displacement in space to the change of the
/// coordinates on t of the point it moves. barycentric3 is affine, so differences give it.
Eigen::Matrix3d barycentric_gradient(const std::array<Eigen::Vector3d, 3>& t)
{
    const Eigen::Vector3d c = (t[0] + t[1] + t[2]) / 3;
    const double h = std::max({(t[1] - t[0]).norm(), (t[2] - t[0]).norm(), (t[2] - t[1]).norm()});
    const Eigen::Vector3d at = barycentric3(c, t);
    Eigen::Matrix3d G = Eigen::Matrix3d::Zero();
    if (!(h > 0)) return G;
    for (int j = 0; j < 3; ++j) {
        G.col(j) = (barycentric3(c + h * Eigen::Vector3d::Unit(j), t) - at) / h;
    }
    return G;
}

/// h with its exact coordinates, if any, as double-doubles and the rest of them in the bound: what
/// exact::normalized divides by its sum at double-double precision (exact coordinates it only
/// scales by a power of two, which leaves points of different sums incomparable).
Hom as_double_double(const Hom& h)
{
    if (!h.x) return h;
    Hom in = h;
    for (int k = 0; k < 3; ++k) {
        const Dyadic& c = (*h.x)[k];
        in.v[k] = c.to_double();
        const Dyadic r = c - Dyadic(in.v[k]);
        in.lo[k] = r.to_double();
        in.e[k] = std::abs((r - Dyadic(in.lo[k])).to_double()) * (1 + 0x1p-50) + 0x1p-1000;
    }
    in.x.reset();
    return in;
}

/// A lower bound on the distance between two points of a face: the largest difference of their
/// coordinates scaled to sum 1. From the coordinates scaled in double-doubles and their bounds, or
/// exactly for points known exactly where those cannot tell.
double distance_below(const Hom& a, const Hom& b)
{
    const bool was_undecided = exact::undecided();
    Hom na, nb;
    {
        const exact::Precision dd(exact::kDoubleDouble);
        na = exact::normalized(as_double_double(a));
        nb = exact::normalized(as_double_double(b));
    }
    const bool scaled = !exact::undecided();
    if (!was_undecided) exact::clear_undecided();
    double d = 0, e = 0;
    for (int k = 0; scaled && k < 3; ++k) {
        // (na.v - nb.v) + (na.lo - nb.lo), the first difference exactly (Knuth's two-sum).
        const double s1 = na.v[k] - nb.v[k];
        const double bb = s1 - na.v[k];
        const double r1 = (na.v[k] - (s1 - bb)) + (-nb.v[k] - bb);
        d = std::max(d, std::abs(s1 + (r1 + (na.lo[k] - nb.lo[k]))));
        e = std::max(e, na.e[k] + nb.e[k]);
    }
    // The last sums are rounded: within a few units in the last place of d.
    e = e + 0x1p-50 * d + 0x1p-1000;
    if (scaled && d > 2 * e) return d - e;
    if (a.known() && b.known()) {
        const exact::Exact3 x = a.value(), y = b.value();
        const Dyadic s = x[0] + x[1] + x[2], t = y[0] + y[1] + y[2];
        // x_k / s - y_k / t = (x_k t - y_k s) / (s t): the numerator exactly, the rest in doubles.
        const double st = s.to_double() * t.to_double();
        double m = 0;
        for (int k = 0; k < 3; ++k) {
            m = std::max(m, std::abs((x[k] * t - y[k] * s).to_double()) / st);
        }
        return m * (1 - 0x1p-48);
    }
    return scaled ? std::max(0., d - e) : 0.;
}

/// Within this of a piece it does not end, a point's precision depends on how far it is from it;
/// farther, the precision kPointAccuracy asks for is enough already.
constexpr double kNear = kPointAccuracy / kCurveSafety;

/// A point of a face scaled to sum 1, its last two coordinates in double-doubles (the face's frame
/// in the plane), and a bound on their error; not `ok` if that could not be done.
struct Flat
{
    double x = 0, xl = 0, y = 0, yl = 0, e = 0;
    bool ok = false;
};
Flat flat(const Hom& h)
{
    const Hom in = as_double_double(h);
    const bool was_undecided = exact::undecided();
    Hom n;
    {
        const exact::Precision dd(exact::kDoubleDouble);
        n = exact::normalized(in);
    }
    Flat out{n.v[1], n.lo[1], n.v[2], n.lo[2], n.e.maxCoeff(), !exact::undecided()};
    if (!was_undecided) exact::clear_undecided();
    return out;
}
/// a - b for double-doubles, to within a unit in the last place of the result.
double difference(const double a, const double al, const double b, const double bl)
{
    const double s = a - b;
    const double bb = s - a;
    const double r = (a - (s - bb)) + (-b - bb);
    return s + (r + (al - bl));
}
/// The distance from p to the segment from a to b (all in one face's frame), from their values;
/// the points' errors are not in it.
double point_segment(const Flat& p, const Flat& a, const Flat& b)
{
    const double abx = difference(b.x, b.xl, a.x, a.xl), aby = difference(b.y, b.yl, a.y, a.yl);
    const double apx = difference(p.x, p.xl, a.x, a.xl), apy = difference(p.y, p.yl, a.y, a.yl);
    const double bpx = difference(p.x, p.xl, b.x, b.xl), bpy = difference(p.y, p.yl, b.y, b.yl);
    const double l2 = abx * abx + aby * aby, t = abx * apx + aby * apy;
    double d;
    if (l2 == 0 || t <= 0) {
        d = std::hypot(apx, apy);
    } else if (t >= l2) {
        d = std::hypot(bpx, bpy);
    } else {
        d = std::abs(abx * apy - aby * apx) / std::sqrt(l2);
    }
    return d * (1 - 0x1p-48); // the rounding above: a few units in the last place of d
}

/// Coordinates as a curve holds them: the doubles, and their further terms if any -- one further
/// term is the low part of a double-double, more are known exactly.
Hom stored_of(const Eigen::Vector3d& v, const BaryTail* tail)
{
    Hom h = Hom::of(v);
    h.stored = true;
    if (!tail || tail->empty()) return h;
    h.lo = tail->front();
    if (tail->size() == 1) return h;
    exact::Exact3 x = h.value();
    for (size_t i = 1; i < tail->size(); ++i) {
        for (int k = 0; k < 3; ++k) {
            const double t = (*tail)[i][k];
            if (t == 0) continue;
            x[k] = x[k] + Dyadic(t);
            h.e[k] += std::abs(t);
        }
    }
    h.e *= 1 + 0x1p-50;
    h.x = std::make_shared<const exact::Exact3>(std::move(x));
    return h;
}

/// The terms a point is kept to: doubles whose sum is within `tol` of its exact coordinates
/// (relative to their sum), each coordinate on its own, as few as that takes (one for most
/// points) -- or, for coordinates as a curve held them (Hom::stored), exactly the terms they had.
/// None if the point is known only approximately and its bound is above tol: it must be computed
/// exactly first.
std::optional<std::pair<Eigen::Vector3d, BaryTail>> terms_of(const Hom& h, const double tol)
{
    if (h.zeros() == 2 && !h.stored) {
        // A corner of the face: one representation for it, whatever its scale.
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        c[h.v[0] != 0 ? 0 : (h.v[1] != 0 ? 1 : 2)] = 1;
        return std::pair{c, BaryTail{}};
    }
    if (!h.x) {
        const double t = tol * h.v.sum();
        const bool has_lo = (h.lo.array() != 0).any();
        if (h.stored || h.e.maxCoeff() <= t) {
            // A double-double: its low part is a term of its own if the high part alone is not
            // within tol (or if the curve held it so).
            if (has_lo && (h.stored || h.e.maxCoeff() + h.lo.cwiseAbs().maxCoeff() > t)) {
                return std::pair{h.v, BaryTail{h.lo}};
            }
            if (!has_lo || h.e.maxCoeff() + h.lo.cwiseAbs().maxCoeff() <= t) {
                return std::pair{h.v, BaryTail{}};
            }
        }
        return std::nullopt;
    }
    // Truncate term by term: each remainder has the sign of the coordinate and is below the last
    // place of the term before, so the terms do not overlap, and coordinates stored this way give
    // back the same terms.
    exact::Exact3 r = *h.x;
    std::pair<Eigen::Vector3d, BaryTail> out;
    // A remainder too small for a double (never, for coordinates near 1 and a point that is not
    // within 2^-1000 of another) ends the terms.
    bool representable = true;
    const auto next = [&]() {
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        for (int k = 0; k < 3; ++k) {
            if (r[k].sign() == 0) continue;
            t[k] = r[k].to_double();
            if (std::abs(t[k]) < DBL_MIN) {
                representable = false;
                t[k] = 0;
                continue;
            }
            r[k] = r[k] - Dyadic(t[k]);
        }
        return t;
    };
    const auto rest = [&]() {
        double m = 0;
        for (int k = 0; k < 3; ++k) m = std::max(m, std::abs(r[k].to_double()));
        return m;
    };
    out.first = next();
    // A remainder below the smallest normal double has no term to hold it (never, for
    // coordinates near 1): at most 20 terms.
    while (representable && out.second.size() < 20 &&
           (h.stored ? (r[0].sign() || r[1].sign() || r[2].sign()) : rest() > tol * h.v.sum())) {
        out.second.push_back(next());
    }
    if (!out.second.empty() && (out.second.back().array() == 0).all()) out.second.pop_back();
    return out;
}

/// The point l of a piece in the face's barycentric coordinates. A point on an edge or a corner
/// of the face keeps its zeros: the corners of a piece on an edge are zero there. Not rescaled: a
/// point of a face that is not in pieces is l itself, so one carried onto the cell's outline
/// keeps its terms as they are, as the face on the other side of the outline, which the edit
/// leaves alone, has them.
Hom in_face(const SurfaceMap::Face& g, const Hom& l)
{
    if (g.sub[0] == kFaceCorners[0] && g.sub[1] == kFaceCorners[1] && g.sub[2] == kFaceCorners[2]) {
        return l;
    }
    // sum_k l_k sub_k, at the current precision; a coordinate with a single term by 1 is l's own.
    std::array<Value, 3> out;
    bool copied = true;
    for (int i = 0; i < 3; ++i) {
        int terms = 0;
        for (int k = 0; k < 3; ++k) {
            const double s = g.sub[k][i];
            if (l.v[k] == 0 || s == 0) continue;
            out[i] = exact::muladd(out[i], exact::coordinate(l, k), s);
            copied = copied && s == 1 && ++terms == 1;
        }
    }
    Hom b = exact::hom(out[0], out[1], out[2]);
    b.stored = l.stored && copied;
    return b;
}

/// The geometry of one cell, one way: its faces (or pieces) before and after laid out in its
/// domain, lifted to homogeneous coordinates for exact predicates.
struct CellGeometry
{
    const SurfaceMap::Cell& cell;
    const std::vector<SurfaceMap::Face>& from;
    const std::vector<SurfaceMap::Face>& to;
    bool forward;
    std::vector<TriangleH> from_uv, to_uv;
    /// back() of the pairs asked for so far.
    mutable std::unordered_map<uint64_t, Eigen::Matrix3d> backs;

    /// The linear map taking a change of a point's coordinates in the face of piece `tau`
    /// (after) to the change of its coordinates in the face of piece `sigma` (before), both
    /// scaled to sum 1, for a point the cell maps from sigma to tau: face to piece, piece to
    /// domain, domain to piece, piece to face, backwards. In doubles: it sets how precisely the
    /// point is kept (sensitivity()), nothing else. Not finite where a piece is flat in doubles.
    const Eigen::Matrix3d& back(const size_t sigma, const size_t tau) const
    {
        const auto [slot, fresh] = backs.try_emplace(uint64_t(sigma) * 65536 + tau);
        if (fresh) {
            const auto lifted = [](const SurfaceMap::Face& f) {
                Eigen::Matrix3d T;
                for (int k = 0; k < 3; ++k) T.col(k) << f.uv[k].x(), f.uv[k].y(), 1.;
                return T;
            };
            const auto pieces = [](const SurfaceMap::Face& f) {
                Eigen::Matrix3d S;
                for (int k = 0; k < 3; ++k) S.col(k) = f.sub[k];
                return S;
            };
            slot->second = pieces(from[sigma]) * lifted(from[sigma]).inverse() * lifted(to[tau]) *
                           pieces(to[tau]).inverse();
        }
        return slot->second;
    }

    CellGeometry(const SurfaceMap::Cell& c, const bool fw)
        : cell(c)
        , from(fw ? c.before : c.after)
        , to(fw ? c.after : c.before)
        , forward(fw)
    {
        for (const auto& f : from) from_uv.push_back(lift(f.uv));
        for (const auto& g : to) to_uv.push_back(lift(g.uv));
    }

    /// The piece among first..last (the pieces of one face, before) holding the point b of the
    /// face, and the point's coordinates in it. Exact; a point on an edge of the face gets the
    /// same coordinates whichever face of the edge it is given in (lin2 is antisymmetric), and a
    /// corner or an edge a piece shares with the face keeps its coordinates as they are.
    std::pair<size_t, Hom> piece_of(const Hom& b, const size_t first, const size_t last) const
    {
        if (last - first == 1) return {first, b};
        for (size_t k = first; k < last; ++k) {
            if (const auto l = carry(kFaceCorners, b, from[k].sub)) return {k, *l};
        }
        if (b.zeros() == 1) {
            // On an edge of the face that is cut at a virtual vertex: between two corners of a
            // piece on that edge, by the two coordinates along it.
            const int m = b.v[0] == 0 ? 0 : (b.v[1] == 0 ? 1 : 2);
            const int i = (m + 1) % 3, j = (m + 2) % 3;
            const Value bi = exact::coordinate(b, i), bj = exact::coordinate(b, j);
            for (size_t k = first; k < last; ++k) {
                const auto& s = from[k].sub;
                for (int x = 0; x < 3; ++x) {
                    for (int y = x + 1; y < 3; ++y) {
                        if (s[x][m] != 0 || s[y][m] != 0) continue;
                        const Eigen::Vector3d &X = s[x], &Y = s[y];
                        const Value D = exact::lin2(Value{X[i]}, Y[j], Value{X[j]}, Y[i]);
                        if (D.sign() == 0) continue;
                        Value al = exact::lin2(bi, Y[j], bj, Y[i]);
                        Value be = exact::lin2(bj, X[i], bi, X[j]);
                        if (D.sign() < 0) {
                            al = exact::negated(al);
                            be = exact::negated(be);
                        }
                        if (al.sign() < 0 || be.sign() < 0 || (al.sign() == 0 && be.sign() == 0)) {
                            continue;
                        }
                        std::array<Value, 3> l;
                        l[x] = al;
                        l[y] = be;
                        return {k, exact::normalized(exact::hom(l[0], l[1], l[2]))};
                    }
                }
            }
        }
        std::vector<TriangleH> pieces;
        for (size_t k = first; k < last; ++k) pieces.push_back(from[k].sub);
        if (const auto at = locate(pieces, exact::Point{&kFaceCorners, b})) {
            return {first + at->tri, at->bary};
        }
        const auto [k, l] = tracking::piece_of(b.v, from, first, last);
        return {k, Hom::of(l)};
    }

    /// An edge split's rule for a point on the split edge (forward) or on one of the halves
    /// (backward): by t0 and t1 alone. Exact, and not rescaled: the weights are products and sums
    /// of doubles, kept as the finite expansions they are (Hom::stored), so both cells of the edge
    /// -- each with its own pieces next to the point -- give the point the same terms.
    std::optional<HWalkPoint> by_split(const TriangleH& tf, const Hom& l) const
    {
        const auto& S = *cell.split;
        const Eigen::Vector3d A(S.a.x(), S.a.y(), 1), B(S.b.x(), S.b.y(), 1),
            W(S.w.x(), S.w.y(), 1);
        int ia = -1, ib = -1, iw = -1;
        for (int k = 0; k < 3; ++k) {
            if (l.v[k] == 0) continue;
            if (tf[k] == A) {
                ia = k;
            } else if (tf[k] == B) {
                ib = k;
            } else if (tf[k] == W) {
                iw = k;
            } else {
                return std::nullopt; // not on the edge
            }
        }
        if (forward ? (ia < 0 || ib < 0 || iw >= 0) : iw < 0) return std::nullopt;
        if (!l.known()) {
            exact::give_up();
            return std::nullopt;
        }
        const exact::Exact3 x = l.value();
        const Dyadic zero(0.);
        const Dyadic ca = ia >= 0 ? x[ia] : zero, cb = ib >= 0 ? x[ib] : zero,
                     cw = iw >= 0 ? x[iw] : zero;
        // The weights on (A, B, W) the point has after.
        Dyadic wa = zero, wb = zero, ww = zero;
        if (forward) {
            // ca A + cb B against W = t0 A + t1 B: on A..W if cb t0 < ca t1.
            const Dyadic s = cb * Dyadic(S.t0) - ca * Dyadic(S.t1);
            if (s.sign() < 0) {
                wa = -s;
                ww = cb;
            } else if (s.sign() > 0) {
                ww = ca;
                wb = s;
            } else {
                ww = Dyadic(1.);
            }
        } else {
            wa = ca + cw * Dyadic(S.t0);
            wb = cb + cw * Dyadic(S.t1);
        }
        for (size_t m = 0; m < to_uv.size(); ++m) {
            exact::Exact3 out;
            bool ok = true;
            for (const auto& [c, w] :
                 {std::pair{&A, &wa}, std::pair{&B, &wb}, std::pair{&W, &ww}}) {
                if (w->sign() == 0) continue;
                const int k =
                    int(std::find(to_uv[m].begin(), to_uv[m].end(), *c) - to_uv[m].begin());
                if (k == 3) {
                    ok = false;
                    break;
                }
                out[k] = *w;
            }
            if (!ok) continue;
            Hom h;
            h.stored = true;
            bool exact_doubles = true;
            for (int k = 0; k < 3; ++k) {
                const int sign = out[k].sign();
                if (sign == 0) continue;
                h.v[k] = out[k].to_double();
                if (h.v[k] == 0) h.v[k] = sign * DBL_MIN;
                exact_doubles = exact_doubles && Dyadic(h.v[k]) == out[k];
                h.e[k] = std::max(0x1p-52 * std::abs(h.v[k]), DBL_MIN);
            }
            if (exact_doubles) {
                h.e.setZero();
            } else {
                h.x = std::make_shared<const exact::Exact3>(std::move(out));
            }
            return HWalkPoint{m, h};
        }
        return std::nullopt;
    }

    /// Where the point l of piece `piece` (before) lands after: on the same corner or edge if a
    /// piece after has it (the cell's outline, or an edge the edit kept), coordinates and zeros
    /// as they are; by the split's rule on a split edge; otherwise located exactly. `inside` is
    /// set false if no piece after holds it (not for a valid cell): it is then clamped onto the
    /// nearest.
    HWalkPoint land(const size_t piece, const Hom& l, bool& inside) const
    {
        inside = true;
        const TriangleH& tf = from_uv[piece];
        if (l.zeros() > 0) {
            for (size_t m = 0; m < to_uv.size(); ++m) {
                if (const auto c = carry(tf, l, to_uv[m])) return {m, *c};
            }
        }
        if (cell.split) {
            if (const auto s = by_split(tf, l)) return *s;
        }
        if (const auto at = locate(to_uv, exact::Point{&tf, l})) return *at;
        const auto& f = from[piece];
        const Eigen::Vector3d n = l.v / l.v.sum();
        const Landing at = tracking::land(n[0] * f.uv[0] + n[1] * f.uv[1] + n[2] * f.uv[2], to);
        inside = false;
        return {at.piece, Hom::of(at.l)};
    }

    /// The real corners of the face whose pieces are first..last, as laid out (lifted).
    TriangleH face_corners(const size_t first, const size_t last) const
    {
        if (last - first == 1) return from_uv[first];
        TriangleH out = from_uv[first];
        for (size_t k = first; k < last; ++k) {
            for (int j = 0; j < 3; ++j) {
                for (int c = 0; c < 3; ++c) {
                    if (from[k].sub[j] == kFaceCorners[c]) out[c] = from_uv[k][j];
                }
            }
        }
        return out;
    }
};

/// A point of a face before, by its corners' places in the cell's domain and its terms there:
/// the same for a point on an edge or a corner whichever face it is given in. The corners with a
/// non-zero coordinate, sorted, each with its place and the coordinate's terms.
struct PointKey
{
    std::array<uint64_t, 10> v{};
    std::vector<uint64_t> tail; // the further terms, if any
    bool operator==(const PointKey& o) const { return v == o.v && tail == o.tail; }
};
struct PointKeyHash
{
    size_t operator()(const PointKey& k) const
    {
        uint64_t h = 1469598103934665603ull;
        for (const uint64_t x : k.v) h ^= x + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        for (const uint64_t x : k.tail) h ^= x + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        return size_t(h);
    }
};
PointKey key_of(const TriangleH& corners, const Eigen::Vector3d& b, const BaryTail* tail)
{
    std::array<std::array<uint64_t, 3>, 3> e{};
    std::array<int, 3> which{};
    int n = 0;
    for (int k = 0; k < 3; ++k) {
        if (b[k] == 0) continue;
        e[n] = {
            std::bit_cast<uint64_t>(corners[k][0]),
            std::bit_cast<uint64_t>(corners[k][1]),
            std::bit_cast<uint64_t>(b[k])};
        which[n++] = k;
    }
    std::array<int, 3> order = {0, 1, 2};
    std::sort(order.begin(), order.begin() + n, [&](int i, int j) { return e[i] < e[j]; });
    PointKey key;
    key.v[0] = uint64_t(n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < 3; ++j) key.v[1 + 3 * i + j] = e[order[i]][j];
    }
    if (tail && !tail->empty()) {
        for (int i = 0; i < n; ++i) {
            for (const Eigen::Vector3d& t : *tail) {
                key.tail.push_back(std::bit_cast<uint64_t>(t[which[order[i]]]));
            }
        }
    }
    return key;
}

/// A face, or a piece of one, in a local patch: its corners -- vertex ids, or a virtual vertex on
/// one of its edges -- and where they sit in the face, in barycentric coordinates.
struct Piece
{
    uint64_t fid;
    Tri t;
    std::array<Eigen::Vector3d, 3> sub = {
        {Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitZ()}};
};

/// The faces of a patch before or after an edit, and the 3D positions of their corners.
struct Patch
{
    std::vector<Piece> pieces;
    Positions P;

    Patch(const std::vector<uint64_t>& fs, const std::vector<Tri>& ts, Positions positions)
        : P(std::move(positions))
    {
        for (size_t i = 0; i < fs.size(); ++i) pieces.push_back({fs[i], ts[i]});
    }

    std::vector<Tri> tris() const
    {
        std::vector<Tri> ts;
        for (const Piece& p : pieces) ts.push_back(p.t);
        return ts;
    }

    /// Splits the piece holding the edge (a, b), virtually, at w = (1 - tau) a + tau b: into the
    /// piece with w for b and the one with w for a, consecutive. False if no piece holds it.
    bool split(const uint64_t a, const uint64_t b, const double tau, const uint64_t w)
    {
        for (size_t k = 0; k < pieces.size(); ++k) {
            const Tri& t = pieces[k].t;
            const size_t ia = size_t(std::find(t.begin(), t.end(), a) - t.begin());
            const size_t ib = size_t(std::find(t.begin(), t.end(), b) - t.begin());
            if (ia == 3 || ib == 3) continue;
            Piece with_a = pieces[k], with_b = pieces[k];
            const Eigen::Vector3d sub_w = (1 - tau) * pieces[k].sub[ia] + tau * pieces[k].sub[ib];
            with_a.t[ib] = w;
            with_a.sub[ib] = sub_w;
            with_b.t[ia] = w;
            with_b.sub[ia] = sub_w;
            pieces[k] = with_a;
            pieces.insert(pieces.begin() + long(k) + 1, with_b);
            P[w] = (1 - tau) * P.at(a) + tau * P.at(b);
            return true;
        }
        return false;
    }
};

/// Whether `loop` is `ids` up to where it starts.
bool same_loop(const std::vector<uint64_t>& loop, const std::vector<uint64_t>& ids)
{
    if (loop.size() != ids.size() || loop.empty()) return false;
    const auto it = std::find(loop.begin(), loop.end(), ids.front());
    if (it == loop.end()) return false;
    const size_t s = size_t(it - loop.begin());
    for (size_t i = 0; i < ids.size(); ++i) {
        if (loop[(s + i) % loop.size()] != ids[i]) return false;
    }
    return true;
}

} // namespace

SurfaceMap::SurfaceMap(const history::SurfaceHistory& h, const SurfaceMapOptions& options)
    : m_in_vertices(h.vertices)
    , m_in_faces(h.faces)
    , m_in_face_alive(h.face_alive)
{
    Replay r(h);
    // While a collapse of a vertex on a curve into one on none tries the removed vertex's star
    // alone (CollapsePatch::removed_star).
    bool curve_into_sheet_try = false;
    const auto layout = [](const Tri& t, const UV& uv) {
        return std::array<Eigen::Vector2d, 3>{{uv.at(t[0]), uv.at(t[1]), uv.at(t[2])}};
    };
    const auto corners_of = [&r](const std::vector<uint64_t>& fs) {
        std::vector<Tri> ts;
        for (const uint64_t f : fs) ts.push_back(r.faces()[f]);
        return ts;
    };
    const auto positions_of = [&r](const std::vector<Tri>& ts) {
        Positions P;
        for (const Tri& t : ts) {
            for (const uint64_t v : t) P[v] = r.vertices()[v];
        }
        return P;
    };

    // A virtual vertex: a point on an edge of a patch that stands in for a vertex the other patch
    // has there.
    constexpr uint64_t kVirtual = kNone - 1;
    using Slots = std::vector<std::pair<uint64_t, uint64_t>>;

    // Lowers the distortion of a joint embedding of `before` and `after`, keeping it one: SLIM
    // (Rabinovich et al. 2017) moves the vertices towards the 3D shapes of their triangles, and
    // a scaffold (Jiang et al. 2017) shared by both -- a ring of triangles from the outline out
    // to a fixed circle -- keeps the outline from folding over, since the line search keeps
    // every triangle, the scaffold's included, positive. The outline `slots` is free but shared:
    // each slot is one point, a corner of `before` and one of `after`. The embedding must be on
    // a circle about the origin, as the Tutte ones are; it is rescaled to the 3D area.
    const auto refine = [&](const Slots& slots,
                            const Patch& before,
                            const Patch& after,
                            UV& uv_b,
                            UV& uv_a) {
        const std::vector<Tri> before_t = before.tris(), after_t = after.tris();
        double area_3d = 0, area_2d = 0;
        for (const Tri& t : before_t) {
            const auto& P = before.P;
            area_3d += 0.5 * (P.at(t[1]) - P.at(t[0])).cross(P.at(t[2]) - P.at(t[0])).norm();
            const Eigen::Vector2d a = uv_b.at(t[1]) - uv_b.at(t[0]);
            const Eigen::Vector2d b = uv_b.at(t[2]) - uv_b.at(t[0]);
            area_2d += 0.5 * (a.x() * b.y() - a.y() * b.x());
        }
        if (!(area_3d > 0) || !(area_2d > 0)) return;
        const double scale = std::sqrt(area_3d / area_2d);

        std::vector<Eigen::Vector2d> x;
        std::vector<char> fixed;
        std::map<uint64_t, int> var_b, var_a;
        for (const auto& [b, a] : slots) {
            var_b[b] = var_a[a] = int(x.size());
            x.push_back(scale * uv_b.at(b));
            fixed.push_back(0);
        }
        const auto var = [&](std::map<uint64_t, int>& vars, const UV& uv, uint64_t v) {
            const auto [it, inserted] = vars.emplace(v, int(x.size()));
            if (inserted) {
                x.push_back(scale * uv.at(v));
                fixed.push_back(0);
            }
            return it->second;
        };
        std::vector<SlimElement> elements;
        const auto add = [&](const Patch& p, const std::vector<Tri>& ts, auto& vars, const UV& uv) {
            for (const Tri& t : ts) {
                elements.push_back(slim_element(
                    {{var(vars, uv, t[0]), var(vars, uv, t[1]), var(vars, uv, t[2])}},
                    p.P.at(t[0]),
                    p.P.at(t[1]),
                    p.P.at(t[2]),
                    1.));
            }
        };
        add(before, before_t, var_b, uv_b);
        add(after, after_t, var_a, uv_a);
        // The scaffold keeps the shared outline -- slots 0 to n - 1 -- a simple polygon, so the
        // two patches lay out the same polygon whatever SLIM does; weighted so it gives way to
        // them. See Scaffold.
        const double scaffold_weight = 1e-2 * area_3d / double(before_t.size());
        const size_t n = slots.size();
        if (options.scaffold == Scaffold::ring) {
            std::vector<int> outer(n);
            for (size_t i = 0; i < n; ++i) {
                outer[i] = int(x.size());
                x.push_back(3 * x[i]);
                fixed.push_back(1);
            }
            for (size_t i = 0; i < n; ++i) {
                const int a = int(i), b = int((i + 1) % n), c = outer[i], d = outer[(i + 1) % n];
                elements.push_back(slim_element({{a, c, d}}, x[a], x[c], x[d], scaffold_weight));
                elements.push_back(slim_element({{a, d, b}}, x[a], x[d], x[b], scaffold_weight));
            }
            slim(x, fixed, elements, 10);
        } else {
            // Rebuilt around the patch before every iteration, so SLIM's stopping rule -- an
            // iteration lowering the energy by less than 1e-4 of it -- is applied across the
            // rebuilds. box_scaffold refuses only an outline that is not simple, which the
            // scaffold itself prevents.
            const size_t n_vars = x.size(), n_patch = elements.size();
            std::vector<Eigen::Vector2d> polygon(n), corners;
            std::vector<std::array<int, 3>> tris;
            const auto var = [&](const int i) {
                return i < int(n) ? i : int(n_vars) + (i - int(n));
            };
            for (int it = 0; it < 10; ++it) {
                x.resize(n_vars);
                fixed.resize(n_vars);
                elements.resize(n_patch);
                Eigen::Vector2d lo = x[0], hi = x[0];
                for (const Eigen::Vector2d& p : x) {
                    lo = lo.cwiseMin(p);
                    hi = hi.cwiseMax(p);
                }
                const Eigen::Vector2d mid = (lo + hi) / 2, half = 1.5 * (hi - lo);
                std::copy(x.begin(), x.begin() + long(n), polygon.begin());
                if (!box_scaffold(polygon, mid - half, mid + half, corners, tris)) break;
                for (const Eigen::Vector2d& c : corners) {
                    x.push_back(c);
                    fixed.push_back(1);
                }
                for (const auto& t : tris) {
                    const std::array<int, 3> v = {{var(t[0]), var(t[1]), var(t[2])}};
                    elements.push_back(slim_element(v, x[v[0]], x[v[1]], x[v[2]], scaffold_weight));
                }
                const double before = slim_energy(x, elements);
                const double after = slim(x, fixed, elements, 1);
                if (!(before - after > 1e-4 * before)) break;
            }
        }
        for (auto& [v, uv] : uv_b) {
            if (const auto it = var_b.find(v); it != var_b.end()) uv = x[it->second];
        }
        for (auto& [v, uv] : uv_a) {
            if (const auto it = var_a.find(v); it != var_a.end()) uv = x[it->second];
        }
    };

    // The cell of a joint embedding of `before` and `after` in one planar domain, whose outline
    // both share: `slots`, in the order of before's outline, each a corner of `before` and one
    // of `after` at the same point. Tutte embeddings on the unit circle, outline spaced by 3D
    // length, start it; refine() lowers its distortion (the joint flattening of Liu et al. 2021,
    // with a shared scaffold). False if a patch's outline is not its slots.
    const auto joint = [&](const Patch& before,
                           const Patch& after,
                           const Slots& slots,
                           Cell& cell) {
        std::vector<uint64_t> outline_b, outline_a;
        for (const auto& [b, a] : slots) {
            outline_b.push_back(b);
            outline_a.push_back(a);
        }
        const std::vector<Tri> before_t = before.tris(), after_t = after.tris();
        const auto loop_b = disk_boundary(before_t), loop_a = disk_boundary(after_t);
        if (!loop_b || !loop_a || !same_loop(*loop_b, outline_b) ||
            !same_loop(*loop_a, outline_a)) {
            return false;
        }
        const size_t n = slots.size();
        std::vector<double> len(n), s(n + 1, 0.);
        for (size_t i = 0; i < n; ++i) {
            len[i] = (before.P.at(outline_b[i]) - before.P.at(outline_b[(i + 1) % n])).norm();
        }
        // A slot apart from the next by a fraction of the mean, at least, so the start is not
        // flat where the outline has a short edge; SLIM brings it back to its length.
        const double floor = 1e-2 * std::accumulate(len.begin(), len.end(), 0.) / double(n);
        for (size_t i = 0; i < n; ++i) s[i + 1] = s[i] + std::max({len[i], floor, 1e-300});
        UV fixed_b, fixed_a;
        for (size_t i = 0; i < n; ++i) {
            const double th = 2 * M_PI * s[i] / s.back();
            fixed_b[outline_b[i]] = fixed_a[outline_a[i]] =
                Eigen::Vector2d(std::cos(th), std::sin(th));
        }
        UV uv_b = tutte(before_t, fixed_b), uv_a = tutte(after_t, fixed_a);
        refine(slots, before, after, uv_b, uv_a);
        // A patch folded over in 3D -- a triangle facing against the patch -- cannot be laid flat
        // as it is: the map, still a bijection, stretches there.
        const auto folded = [](const std::vector<Tri>& ts, const Positions& Q) {
            const auto normal = [&Q](const Tri& t) {
                return Eigen::Vector3d((Q.at(t[1]) - Q.at(t[0])).cross(Q.at(t[2]) - Q.at(t[0])));
            };
            Eigen::Vector3d n = Eigen::Vector3d::Zero();
            for (const Tri& t : ts) n += normal(t);
            return std::any_of(ts.begin(), ts.end(), [&](const Tri& t) {
                return normal(t).dot(n) < 0;
            });
        };
        if (folded(before_t, before.P) || folded(after_t, after.P)) ++m_folded_cells;
        bool ok = true;
        const auto add = [&](std::vector<Face>& to, const Patch& p, const UV& uv) {
            for (const Piece& piece : p.pieces) {
                to.push_back({piece.fid, layout(piece.t, uv), piece.sub});
                ok = positive(to.back().uv[0], to.back().uv[1], to.back().uv[2]) && ok;
            }
        };
        add(cell.before, before, uv_b);
        add(cell.after, after, uv_a);
        if (!ok) ++m_build_warnings.inverted;
        return true;
    };

    // Ids of a cell's own: the pieces of a vertex a cut makes, and virtual vertices. Counted down
    // from kVirtual, far above any vertex id.
    struct LocalIds
    {
        uint64_t next = kVirtual;
        std::map<uint64_t, uint64_t> vertex; // of each, the vertex it stands for
        std::map<std::pair<uint64_t, uint64_t>, uint64_t> names; // of the pieces of vertices
        uint64_t fresh(const uint64_t v)
        {
            vertex[next] = v;
            return next--;
        }
        uint64_t vertex_of(const uint64_t id) const
        {
            const auto it = vertex.find(id);
            return it == vertex.end() ? id : it->second;
        }
    };

    // The feature edges among the edges of `ts`, now.
    const auto features_in = [&r](const std::vector<Tri>& ts) {
        std::set<std::array<uint64_t, 2>> fs;
        for (const Tri& t : ts) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = t[k], b = t[(k + 1) % 3];
                if (a > b) std::swap(a, b);
                if (r.is_feature({{a, b}})) fs.insert({{a, b}});
            }
        }
        return fs;
    };

    // Cuts the patch `ts` along the feature edges inside it -- a curve that ends inside -- so it
    // is a disk with every curve on its outline, which a joint embedding then keeps: each vertex
    // splits into its wedges, the fans of its triangles joined across edges that are not feature
    // edges, and a vertex with more than one is a corner of the outline once per wedge, each
    // with an id of its own (`P` gets their positions). A wedge is named by its vertex and the
    // vertex before it on the outline, `removed` read as `kept`, so the patches before and after
    // a collapse name a corner alike. `features` are the patch's feature edges. The patch as it
    // is if no curve ends inside it; nothing if the cut is not a disk.
    const auto cut = [](const std::vector<Tri>& ts,
                        const std::set<std::array<uint64_t, 2>>& features,
                        Positions& P,
                        LocalIds& ids,
                        const uint64_t removed,
                        const uint64_t kept) -> std::optional<std::vector<Tri>> {
        std::vector<size_t> root(3 * ts.size());
        for (size_t i = 0; i < root.size(); ++i) root[i] = i;
        const auto find = [&root](size_t i) {
            while (root[i] != i) i = root[i] = root[root[i]];
            return i;
        };
        const auto corner = [&ts](size_t f, uint64_t v) {
            return 3 * f + size_t(std::find(ts[f].begin(), ts[f].end(), v) - ts[f].begin());
        };
        std::map<std::array<uint64_t, 2>, std::vector<size_t>> edge_faces;
        for (size_t i = 0; i < ts.size(); ++i) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = ts[i][k], b = ts[i][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_faces[{{a, b}}].push_back(i);
            }
        }
        for (const auto& [e, fs] : edge_faces) {
            if (fs.size() != 2 || features.count(e)) continue;
            for (const uint64_t v : e) root[find(corner(fs[0], v))] = find(corner(fs[1], v));
        }
        std::map<uint64_t, std::set<size_t>> wedges;
        for (size_t c = 0; c < root.size(); ++c) wedges[ts[c / 3][c % 3]].insert(find(c));
        std::map<size_t, uint64_t> wedge_id; // temporary, for vertices with more than one
        LocalIds temp;
        for (const auto& [v, ws] : wedges) {
            if (ws.size() < 2) continue;
            for (const size_t w : ws) wedge_id[w] = temp.fresh(v);
        }
        if (wedge_id.empty()) return ts;
        std::vector<Tri> cut_ts = ts;
        for (size_t c = 0; c < root.size(); ++c) {
            if (const auto it = wedge_id.find(find(c)); it != wedge_id.end()) {
                cut_ts[c / 3][c % 3] = it->second;
            }
        }
        const auto loop = disk_boundary(cut_ts);
        if (!loop) return std::nullopt;
        std::map<uint64_t, uint64_t> final_id;
        const size_t n = loop->size();
        for (size_t i = 0; i < n; ++i) {
            const uint64_t t = (*loop)[i];
            if (!temp.vertex.count(t)) continue;
            const uint64_t v = temp.vertex_of(t);
            uint64_t before = temp.vertex_of((*loop)[(i + n - 1) % n]);
            if (before == removed) before = kept;
            const auto [it, inserted] = ids.names.emplace(std::make_pair(v, before), 0);
            if (inserted) it->second = ids.fresh(v);
            final_id[t] = it->second;
            P[it->second] = P.at(v);
        }
        for (Tri& t : cut_ts) {
            for (uint64_t& v : t) {
                if (temp.vertex.count(v)) {
                    const auto it = final_id.find(v);
                    if (it == final_id.end()) return std::nullopt; // a wedge off the outline
                    v = it->second;
                }
            }
        }
        return cut_ts;
    };

    // The cell of a collapse or swap: the patch before and after it, cut along the curves inside
    // them, on the outline of the patch before. A `removed` vertex on that outline -- on the
    // boundary or a curve, maybe more than once -- stays on it, and the triangle after the edit
    // that holds the edge replacing its two is split virtually at the same spot, so the outline
    // is the same polygon after it. `before_features` are the feature edges of the patch before
    // the edit, which the replay has already applied. False if the patch is not a disk.
    const auto flatten = [&](const std::vector<uint64_t>& before_f,
                             const std::vector<Tri>& before_t_uncut,
                             const std::set<std::array<uint64_t, 2>>& before_features,
                             const std::vector<uint64_t>& after_f,
                             const std::vector<Tri>& after_t_uncut,
                             const uint64_t removed,
                             const uint64_t kept,
                             const Positions& P_uncut,
                             const Positions& Pa_uncut,
                             Cell& cell) -> bool {
        LocalIds ids;
        Positions P = P_uncut, Pa = Pa_uncut;
        const auto before_t = cut(before_t_uncut, before_features, P, ids, removed, kept);
        const auto after_t = cut(after_t_uncut, features_in(after_t_uncut), Pa, ids, kNone, kNone);
        if (!before_t || !after_t) return false;
        const auto loop = disk_boundary(*before_t);
        if (!loop) return false;
        const Patch before(before_f, *before_t, P);
        Patch after(after_f, *after_t, Pa);
        Slots slots;
        for (const uint64_t v : *loop) slots.emplace_back(v, v);
        std::vector<size_t> on_outline;
        for (size_t i = 0; i < loop->size(); ++i) {
            if (ids.vertex_of((*loop)[i]) == removed) on_outline.push_back(i);
        }
        if (on_outline.empty()) return joint(before, after, slots, cell);

        ++m_curve_collapses;
        const auto length = [&P](uint64_t a, uint64_t b) { return (P.at(a) - P.at(b)).norm(); };
        const auto neighbours = [&loop](size_t i) {
            const size_t n = loop->size();
            return std::make_pair((*loop)[(i + n - 1) % n], (*loop)[(i + 1) % n]);
        };
        // The outline keeps the removed vertex. Where the survivor was inside -- a boundary vertex
        // collapsed into one inside -- the survivor comes out onto the outline in its place and
        // stands for it there, and that is decided first: every edge the collapse makes holds
        // the survivor, so an edge (prev, next) the patch has after was there before, the third
        // side of the face that survives a survivor of three faces, and splitting it would cut
        // the outline. Tried the other way round, that case was refused as not a disk: 20 of 20
        // not_a_disk warnings of tetwild on the open bunny (2026-11), python_tracking_
        // correspondences' intermittent CI failure. Otherwise a virtual vertex takes its place,
        // on the edge (prev, next) at its parameter along the two edges it replaced.
        const bool kept_inside = std::none_of(loop->begin(), loop->end(), [&](uint64_t v) {
            return ids.vertex_of(v) == kept;
        });
        bool split_all = true, kept_placed = false;
        for (const size_t i : on_outline) {
            const auto [prev, next] = neighbours(i);
            const double a = length(prev, (*loop)[i]), b = length((*loop)[i], next);
            const double tau = std::clamp(a + b > 0 ? a / (a + b) : 0.5, 0.01, 0.99);
            const uint64_t w = ids.fresh(removed);
            if (kept_inside && !kept_placed) {
                slots[i].second = kept;
                kept_placed = true;
            } else if (after.split(prev, next, tau, w)) {
                slots[i].second = w;
            } else {
                split_all = false;
            }
        }
        if (split_all) return joint(before, after, slots, cell);
        if (on_outline.size() > 1 || !ids.names.empty()) return false;
        // The removed star of a curve vertex collapsed into a vertex of a sheet that is on its
        // outline: the faces after do not cover the outline before, and the chord layout below
        // would not be a bijection. Both stars map it.
        if (curve_into_sheet_try) return false;
        const size_t i_rm = on_outline.front();
        const auto [prev, next] = neighbours(i_rm);
        const double a = length(prev, removed), b = length(removed, next);
        const double tau = std::clamp(a + b > 0 ? a / (a + b) : 0.5, 0.01, 0.99);
        const std::vector<Tri>& before_tris = *before_t;
        const std::vector<Tri>& after_tris = *after_t;

        // Nothing to split: flatten the outline over a chord instead, where the triangles with
        // all three corners on it go flat.
        std::vector<uint64_t> ring = *loop;
        ring.erase(ring.begin() + long(i_rm));
        std::vector<double> s(ring.size() + 1, 0.);
        for (size_t i = 0; i < ring.size(); ++i) {
            s[i + 1] = s[i] + std::max(length(ring[i], ring[(i + 1) % ring.size()]), 1e-300);
        }
        UV fixed;
        for (size_t i = 0; i < ring.size(); ++i) {
            const double th = 2 * M_PI * s[i] / s.back();
            fixed[ring[i]] = Eigen::Vector2d(std::cos(th), std::sin(th));
        }
        UV fixed_before = fixed;
        fixed_before[removed] = (1 - tau) * fixed.at(prev) + tau * fixed.at(next);
        for (const Tri& t : before_tris) {
            int on = 0;
            for (const uint64_t v : t) on += (v == prev || v == removed || v == next);
            if (on == 3) {
                ++m_build_warnings.flat_triangle;
                break;
            }
        }
        const UV uv_before = tutte(before_tris, fixed_before);
        const UV uv_after = tutte(after_tris, fixed);
        bool ok = true;
        for (size_t i = 0; i < before_f.size(); ++i) {
            cell.before.push_back({before_f[i], layout(before_tris[i], uv_before)});
            ok = positive(
                     cell.before.back().uv[0],
                     cell.before.back().uv[1],
                     cell.before.back().uv[2]) &&
                 ok;
        }
        for (size_t i = 0; i < after_f.size(); ++i) {
            cell.after.push_back({after_f[i], layout(after_tris[i], uv_after)});
            ok = positive(
                     cell.after.back().uv[0],
                     cell.after.back().uv[1],
                     cell.after.back().uv[2]) &&
                 ok;
        }
        if (!ok) ++m_build_warnings.inverted;
        return true;
    };

    // The sides of the feature curves through a patch: its triangles joined across an edge that
    // is not a feature edge, as index groups. Each side is flattened on its own, so a curve maps
    // onto itself and a face stays on its side.
    const auto sides_of = [&r](const std::vector<Tri>& ts) {
        std::vector<size_t> root(ts.size());
        for (size_t i = 0; i < root.size(); ++i) root[i] = i;
        const auto find = [&root](size_t i) {
            while (root[i] != i) i = root[i] = root[root[i]];
            return i;
        };
        std::map<std::array<uint64_t, 2>, std::vector<size_t>> edge_faces;
        for (size_t i = 0; i < ts.size(); ++i) {
            for (int k = 0; k < 3; ++k) {
                uint64_t a = ts[i][k], b = ts[i][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                edge_faces[{{a, b}}].push_back(i);
            }
        }
        for (const auto& [edge, fs] : edge_faces) {
            if (fs.size() == 2 && !r.is_feature({{edge[0], edge[1]}}))
                root[find(fs[0])] = find(fs[1]);
        }
        std::map<size_t, std::vector<size_t>> groups;
        for (size_t i = 0; i < ts.size(); ++i) groups[find(i)].push_back(i);
        std::vector<std::vector<size_t>> sides;
        for (auto& [g, members] : groups) sides.push_back(std::move(members));
        return sides;
    };

    // The cells of moving vertex v to `to` (Smoothing::flatten): one per side of its star, the
    // same faces before and after, so the points stay where they were on the surface as far as
    // the joint embedding can, instead of riding along with the vertex as they do in the paper's
    // smoothing (Smoothing::barycentric). On a curve or the boundary v keeps to the outline: both
    // its places are on it, each a virtual vertex on the edge of the patch it is not a corner of,
    // matched by arc length along the curve -- the same on both sides. False where v is on more
    // than one curve: the points then keep their barycentric coordinates, which is a bijection
    // too.
    const auto move_cells =
        [&](const uint64_t v, const Eigen::Vector3d& to, std::vector<Cell>& cells) {
            const std::vector<uint64_t> star = r.faces_of(v);
            if (star.empty()) return false;
            const std::vector<Tri> star_t = corners_of(star);
            const Positions P = positions_of(star_t);
            Positions Pa = P;
            Pa[v] = to;
            std::set<std::pair<uint64_t, uint64_t>> curve; // v's outline neighbours, per side
            for (const auto& members : sides_of(star_t)) {
                std::vector<uint64_t> fs;
                std::vector<Tri> ts;
                for (const size_t i : members) {
                    fs.push_back(star[i]);
                    ts.push_back(star_t[i]);
                }
                const auto loop = disk_boundary(ts);
                if (!loop) return false;
                Patch before(fs, ts, P), after(fs, ts, Pa);
                Slots slots;
                for (const uint64_t u : *loop) slots.emplace_back(u, u);
                const auto it = std::find(loop->begin(), loop->end(), v);
                if (it != loop->end()) {
                    const size_t i = size_t(it - loop->begin()), n = loop->size();
                    const uint64_t prev = (*loop)[(i + n - 1) % n], next = (*loop)[(i + 1) % n];
                    curve.insert(std::minmax(prev, next));
                    if (curve.size() > 1) return false;
                    // Measured from the same end on both sides of the curve, so that rounding
                    // cannot have one side see a move the other does not: a move of no length
                    // split on one side alone would part them by the clamp below.
                    const uint64_t first = std::min(prev, next), second = std::max(prev, next);
                    const auto fraction = [&](const Positions& Q) {
                        const double a = (Q.at(first) - Q.at(v)).norm(),
                                     b = (Q.at(v) - Q.at(second)).norm();
                        return a + b > 0 ? a / (a + b) : -1.;
                    };
                    const double from_first_old = fraction(P), from_first_new = fraction(Pa);
                    if (!(from_first_old > 0 && from_first_old < 1 && from_first_new > 0 &&
                          from_first_new < 1)) {
                        return false;
                    }
                    const bool towards_first = from_first_new < from_first_old;
                    const bool moved = from_first_new != from_first_old;
                    const bool towards_prev = prev == first ? towards_first : !towards_first;
                    const double f_old = prev == first ? from_first_old : 1 - from_first_old;
                    const double f_new = prev == first ? from_first_new : 1 - from_first_new;
                    // Kept off the edge's ends, as a collapse's: a split there would be a piece
                    // flat in the face but not in the domain, which the map could not invert. The
                    // bounds are symmetric, so the other side of the curve, measuring from its
                    // other end, splits the edge at the same point.
                    const auto clamped = [](double tau) { return std::clamp(tau, 0.01, 0.99); };
                    if (moved && towards_prev) {
                        before.split(prev, v, clamped(f_new / f_old), kVirtual);
                        after.split(v, next, clamped((f_old - f_new) / (1 - f_new)), kVirtual);
                        slots[i] = {kVirtual, v};
                        slots.insert(slots.begin() + long(i) + 1, {v, kVirtual});
                    } else if (moved) {
                        before.split(v, next, clamped((f_new - f_old) / (1 - f_old)), kVirtual);
                        after.split(prev, v, clamped(f_old / f_new), kVirtual);
                        slots[i] = {v, kVirtual};
                        slots.insert(slots.begin() + long(i) + 1, {kVirtual, v});
                    }
                }
                Cell cell;
                if (!joint(before, after, slots, cell)) return false;
                cells.push_back(std::move(cell));
            }
            return true;
        };

    // Where flatten() cannot go: each surviving face maps onto itself by its corners, which is
    // what relabelling `removed` to the survivor does, and a face that vanished takes its points
    // with it.
    const auto relabel = [&](const std::vector<uint64_t>& before_f,
                             const std::vector<uint64_t>& after_f,
                             Step& step) {
        ++m_build_warnings.not_a_disk;
        for (const uint64_t f : before_f) {
            if (std::find(after_f.begin(), after_f.end(), f) == after_f.end()) {
                step.dead.push_back(f);
                continue;
            }
            Cell cell;
            cell.before.push_back({f, kCanonical});
            cell.after.push_back({f, kCanonical});
            step.cells.push_back(std::move(cell));
        }
    };

    for (size_t edit_index = 0; edit_index < h.edits.size(); ++edit_index) {
        const Edit& e = h.edits[edit_index];
        const size_t warnings_before = m_build_warnings.total();
        Step step{e.kind, {}, {}, {}, {}};
        switch (e.kind) {
        case EditKind::split_edge: {
            const uint64_t v0 = e.v[0], v1 = e.v[1], vn = e.v[2];
            const Eigen::Vector3d p0 = r.vertices()[v0], p1 = r.vertices()[v1];
            const Eigen::Vector3d pn = position_after(e, vn, (p0 + p1) / 2);
            const double l2 = (p1 - p0).squaredNorm();
            const double t = l2 > 0 ? std::clamp((pn - p0).dot(p1 - p0) / l2, 1e-9, 1 - 1e-9) : 0.5;
            // The new vertex at t0 v0 + t1 v1 with t0 + t1 = 1 exactly (Sterbenz: one of 1 - t and
            // 1 - t0 is exact), so in every layout below -- corners at 0 and 1 -- it is exactly on
            // the edge, and both faces of the edge put it at the same place along it.
            const double t0 = 1 - t, t1 = 1 - t0;
            std::vector<Tri> parents;
            for (size_t i = 0; i + 1 < e.faces.size(); i += 2) {
                parents.push_back(r.faces()[e.faces[i]]);
            }
            r.apply(e);
            for (size_t i = 0; i + 1 < e.faces.size(); i += 2) {
                const Tri& parent = parents[i / 2];
                UV uv;
                for (int k = 0; k < 3; ++k) uv[parent[k]] = kCanonical[k];
                uv[vn] = t0 * uv.at(v0) + t1 * uv.at(v1);
                Cell cell;
                cell.split = Cell::EdgeSplit{uv.at(v0), uv.at(v1), uv.at(vn), t0, t1};
                cell.before.push_back({e.faces[i], kCanonical});
                cell.after.push_back({e.faces[i], layout(r.faces()[e.faces[i]], uv)});
                cell.after.push_back({e.faces[i + 1], layout(r.faces()[e.faces[i + 1]], uv)});
                step.cells.push_back(std::move(cell));
            }
            break;
        }
        case EditKind::split_face: {
            const uint64_t f = e.faces[0], vn = e.v[3];
            const Tri parent = r.faces()[f];
            const auto& P = r.vertices();
            const Eigen::Vector3d centroid = (P[parent[0]] + P[parent[1]] + P[parent[2]]) / 3;
            const Eigen::Vector3d l = inside_barycentric(
                position_after(e, vn, centroid),
                P[parent[0]],
                P[parent[1]],
                P[parent[2]]);
            r.apply(e);
            UV uv;
            for (int k = 0; k < 3; ++k) uv[parent[k]] = kCanonical[k];
            uv[vn] = l[0] * kCanonical[0] + l[1] * kCanonical[1] + l[2] * kCanonical[2];
            Cell cell;
            cell.before.push_back({f, kCanonical});
            for (const uint64_t g : e.faces) cell.after.push_back({g, layout(r.faces()[g], uv)});
            step.cells.push_back(std::move(cell));
            break;
        }
        case EditKind::collapse_edge: {
            const uint64_t removed = e.v[0], kept = e.v[1];
            // The faces a collapse maps, read before the replay applies it.
            struct Faces
            {
                std::vector<uint64_t> f;
                std::vector<Tri> t;
                Positions P;
                std::vector<std::vector<size_t>> sides;
                std::set<std::array<uint64_t, 2>> features;
            };
            const auto faces = [&](std::vector<uint64_t> fs) {
                Faces b{std::move(fs), {}, {}, {}, {}};
                b.t = corners_of(b.f);
                b.P = positions_of(b.t);
                b.sides = sides_of(b.t);
                b.features = features_in(b.t);
                return b;
            };
            std::vector<uint64_t> union_f;
            std::set_union(
                r.faces_of(removed).begin(),
                r.faces_of(removed).end(),
                r.faces_of(kept).begin(),
                r.faces_of(kept).end(),
                std::back_inserter(union_f));
            const Faces both = faces(union_f);
            const std::optional<Faces> star = options.collapse_patch == CollapsePatch::removed_star
                                                  ? std::optional<Faces>(faces(r.faces_of(removed)))
                                                  : std::nullopt;
            size_t n_spanning = 0; // faces with both endpoints, which the collapse removes
            for (const Tri& t : both.t) {
                if (std::count(t.begin(), t.end(), removed) &&
                    std::count(t.begin(), t.end(), kept)) {
                    ++n_spanning;
                }
            }
            // On a curve of the complex: on an edge of one face, of three or more, or a feature
            // edge -- the edges sides_of() cuts at. Read before the replay applies the collapse,
            // which puts the survivor on the removed vertex's curves.
            const auto on_curve = [&r](const uint64_t v) {
                std::map<uint64_t, int> n;
                for (const uint64_t f : r.faces_of(v)) {
                    for (const uint64_t x : r.faces()[f]) {
                        if (x != v) ++n[x];
                    }
                }
                return std::any_of(n.begin(), n.end(), [&](const auto& xn) {
                    return xn.second != 2 || r.is_feature({{v, xn.first}});
                });
            };
            const bool curve_into_sheet = star && on_curve(removed) && !on_curve(kept);
            r.apply(e);
            const std::vector<uint64_t> after_f = r.faces_of(kept);
            if (after_f.size() + n_spanning < both.f.size()) {
                m_build_warnings.merged_faces += both.f.size() - n_spanning - after_f.size();
            }
            // One cell per side of the curves through `b`; false if a side had to be relabelled.
            const auto map_sides = [&](const Faces& b, Step& to) {
                bool all = true;
                for (const auto& members : b.sides) {
                    std::vector<uint64_t> side_before_f;
                    std::vector<Tri> side_before_t;
                    bool has_removed = false;
                    for (const size_t i : members) {
                        side_before_f.push_back(b.f[i]);
                        side_before_t.push_back(b.t[i]);
                        has_removed =
                            has_removed || std::count(b.t[i].begin(), b.t[i].end(), removed);
                    }
                    if (!has_removed) continue; // nothing on this side changes
                    std::vector<uint64_t> side_after_f;
                    for (const uint64_t f : after_f) {
                        if (std::find(side_before_f.begin(), side_before_f.end(), f) !=
                            side_before_f.end()) {
                            side_after_f.push_back(f);
                        }
                    }
                    // A side the collapse removes whole -- faces on the edge cut off by
                    // non-manifold or feature edges, faces merged into another side's -- has
                    // nothing to map onto.
                    if (side_after_f.empty()) {
                        relabel(side_before_f, side_after_f, to);
                        all = false;
                        continue;
                    }
                    Cell cell;
                    const std::vector<Tri> side_after_t = corners_of(side_after_f);
                    if (flatten(
                            side_before_f,
                            side_before_t,
                            b.features,
                            side_after_f,
                            side_after_t,
                            removed,
                            kept,
                            b.P,
                            positions_of(side_after_t),
                            cell)) {
                        to.cells.push_back(std::move(cell));
                    } else {
                        relabel(side_before_f, side_after_f, to);
                        all = false;
                    }
                }
                return all;
            };
            if (star) {
                const MapWarnings warnings = m_build_warnings;
                const size_t curve_collapses = m_curve_collapses, folded = m_folded_cells;
                Step tried{e.kind, {}, {}, {}, {}};
                curve_into_sheet_try = curve_into_sheet;
                const bool star_ok = map_sides(*star, tried);
                curve_into_sheet_try = false;
                if (star_ok) {
                    step = std::move(tried);
                    break;
                }
                m_build_warnings = warnings;
                m_curve_collapses = curve_collapses;
                m_folded_cells = folded;
            }
            map_sides(both, step);
            break;
        }
        case EditKind::swap_edge: {
            const std::vector<uint64_t> fs = e.faces;
            const std::vector<Tri> before_t = corners_of(fs);
            const Positions P = positions_of(before_t);
            const auto before_features = features_in(before_t);
            r.apply(e);
            Cell cell;
            if (flatten(
                    fs,
                    before_t,
                    before_features,
                    fs,
                    corners_of(fs),
                    kNone,
                    kNone,
                    P,
                    P,
                    cell)) {
                step.cells.push_back(std::move(cell));
            } else {
                relabel(fs, fs, step);
            }
            break;
        }
        case EditKind::replace_faces: {
            const size_t n_added = e.v.size() / 3, n_removed = e.faces.size() - n_added;
            step.dead.assign(e.faces.begin(), e.faces.begin() + long(n_removed));
            step.born.assign(e.faces.begin() + long(n_removed), e.faces.end());
            m_build_warnings.replaced += e.faces.size();
            r.apply(e);
            break;
        }
        case EditKind::relabel: {
            // Faces keep their corners, so their points keep their coordinates: a renumbering,
            // with the faces a weld leaves degenerate gone.
            step.kind = EditKind::consolidate;
            step.old_to_new.assign(r.faces().size(), kNone);
            const auto renamed = [&e](uint64_t v) { return v < e.v.size() ? e.v[v] : kNone; };
            for (uint64_t f = 0; f < r.faces().size(); ++f) {
                if (!r.face_alive()[f]) continue;
                std::array<uint64_t, 3> t = r.faces()[f];
                for (uint64_t& v : t) v = renamed(v);
                if (t[0] == kNone || t[1] == kNone || t[2] == kNone || t[0] == t[1] ||
                    t[1] == t[2] || t[0] == t[2]) {
                    continue;
                }
                const uint64_t g = e.faces.empty()      ? step.new_to_old.size()
                                   : f < e.faces.size() ? e.faces[f]
                                                        : kNone;
                if (g == kNone) continue;
                step.old_to_new[f] = g;
                if (g >= step.new_to_old.size()) step.new_to_old.resize(g + 1, kNone);
                step.new_to_old[g] = f;
            }
            r.apply(e);
            break;
        }
        case EditKind::embed: {
            Embedding& em = step.embedding;
            const auto& P = r.vertices();
            em.before.resize(r.faces().size());
            em.to_after.assign(r.faces().size(), {});
            for (uint64_t f = 0; f < r.faces().size(); ++f) {
                if (!r.face_alive()[f]) continue;
                for (int k = 0; k < 3; ++k) em.before[f][k] = P[r.faces()[f][k]];
            }
            std::vector<Eigen::Vector3d> Q;
            for (const auto& m : e.moves) {
                if (m.vid >= Q.size()) Q.resize(m.vid + 1, Eigen::Vector3d::Zero());
                Q[m.vid] = m.pos;
            }
            const size_t nf = e.v.size() / 3;
            em.after.resize(nf);
            em.to_before.resize(nf);
            for (size_t f = 0, i = 0; f < nf; ++f) {
                for (int k = 0; k < 3; ++k) em.after[f][k] = Q[e.v[3 * f + k]];
                const uint64_t n = e.faces[i++];
                for (uint64_t j = 0; j < n; ++j) {
                    const uint64_t g = e.faces[i++];
                    if (g >= em.to_after.size() || !r.face_alive()[g]) continue;
                    em.to_before[f].push_back(g);
                    em.to_after[g].push_back(f);
                }
            }
            step.kind = EditKind::embed;
            r.apply(e);
            break;
        }
        case EditKind::consolidate: {
            step.old_to_new.assign(r.faces().size(), kNone);
            for (uint64_t f = 0; f < r.faces().size(); ++f) {
                if (!r.face_alive()[f]) continue;
                step.old_to_new[f] = step.new_to_old.size();
                step.new_to_old.push_back(f);
            }
            r.apply(e);
            break;
        }
        case EditKind::move_vertices: {
            // One vertex at a time, each its own step.
            for (const history::VertexMove& m : e.moves) {
                std::vector<Cell> cells;
                const bool mapped = m.vid < r.vertex_alive().size() && r.vertex_alive()[m.vid] &&
                                    r.vertices()[m.vid] != m.pos &&
                                    options.smoothing == Smoothing::flatten &&
                                    move_cells(m.vid, m.pos, cells);
                r.apply(Edit{EditKind::move_vertices, {}, {}, {m}});
                if (!mapped) {
                    ++m_kept_moves;
                    continue;
                }
                ++m_cells_by_kind[size_t(EditKind::move_vertices)];
                m_steps.push_back({EditKind::move_vertices, std::move(cells), {}, {}, {}});
                m_steps.back().edit = edit_index;
            }
            break;
        }
        default: r.apply(e); break;
        }
        if (m_build_warnings.total() > warnings_before) m_warned_edits.push_back(edit_index);
        if (step.cells.empty() && step.dead.empty() && step.born.empty() &&
            step.kind != EditKind::consolidate && step.kind != EditKind::embed) {
            continue;
        }
        ++m_cells_by_kind[size_t(e.kind) % m_cells_by_kind.size()];
        m_steps.push_back(std::move(step));
        m_steps.back().edit = edit_index;
    }

    m_out_vertices = r.vertices();
    m_out_faces = r.faces();
    m_out_face_alive = r.face_alive();
    m_out_features = r.features();
    // An embed maps through space: a bijection only as far as the faces before and after are
    // coplanar, and computed in doubles. A point through one need not come back to where it
    // started, whatever its precision, so there is nothing to certify it against.
    m_source_accuracy = options.source_accuracy;
    m_certify = options.certify && std::none_of(m_steps.begin(), m_steps.end(), [](const Step& s) {
                    return s.kind == EditKind::embed;
                });
    m_in_scale = diagonal_of(m_in_vertices, m_in_faces, m_in_face_alive);
    m_out_scale = diagonal_of(m_out_vertices, m_out_faces, m_out_face_alive);
}

std::vector<SurfacePoint> SurfaceMap::to_input(
    std::vector<SurfacePoint> points,
    MapWarnings* warnings) const
{
    return map(std::move(points), false, warnings);
}

std::vector<SurfacePoint> SurfaceMap::to_output(
    std::vector<SurfacePoint> points,
    MapWarnings* warnings) const
{
    return map(std::move(points), true, warnings);
}

/// Points mapped, before they are stored: each scaled to sum 1 as a double-double (bary + lo) if
/// it was computed at all, with its sensitivity matrix (see sensitivity()).
struct SurfaceMap::Raw
{
    std::vector<SurfacePoint> pts;
    std::vector<char> computed;
    std::vector<Eigen::Matrix3d> K;
};

SurfaceMap::Raw SurfaceMap::map_raw(
    std::vector<SurfacePoint> pts,
    const bool forward,
    MapWarnings* warnings,
    const std::vector<double>* share) const
{
    // Counted for this call alone: the map itself is never written, so several threads may map
    // through it at once.
    MapWarnings w;
    std::unordered_map<uint64_t, std::vector<size_t>> on_face;
    // A point must start on a live face of its side: any other id would be carried along by
    // whichever face a later edit gives that slot.
    const auto& alive = forward ? m_in_face_alive : m_out_face_alive;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (pts[i].face < alive.size() && alive[pts[i].face]) {
            on_face[pts[i].face].push_back(i);
        } else {
            pts[i].face = kNone;
            ++w.unlocated;
        }
    }
    std::vector<std::pair<uint64_t, size_t>> landed;
    // Between steps a point is kept in double-doubles, bary + lo: the rounding of each step stays
    // a few units in the 106th bit, and the point is rounded to doubles once, at the end.
    std::vector<Eigen::Vector3d> lo(pts.size(), Eigen::Vector3d::Zero());
    for (size_t i = 0; i < pts.size(); ++i) {
        lo[i] = pts[i].lo;
        pts[i].lo.setZero();
    }
    // Whether a point was computed anew (not only carried as it was): such a point is scaled to
    // sum 1 at the end; one never computed keeps its doubles.
    std::vector<char> computed(pts.size(), 0);
    // Each point's sensitivity matrix (see sensitivity()): a change of its coordinates to the
    // displacement of the point of the starting mesh it stands for. The corners of its face at
    // first; each step multiplies it by the step's local map, backwards.
    const auto& start_vertices = forward ? m_in_vertices : m_out_vertices;
    const auto& start_faces = forward ? m_in_faces : m_out_faces;
    const double scale = forward ? m_in_scale : m_out_scale;
    std::vector<Eigen::Matrix3d> K(pts.size(), Eigen::Matrix3d::Zero());
    for (size_t i = 0; i < pts.size(); ++i) {
        if (pts[i].face != kNone) K[i] = corner_matrix(start_vertices, start_faces, pts[i].face);
    }

    const auto visit = [&](const Step& s) {
        if (s.kind == EditKind::consolidate) {
            const auto& ids = forward ? s.old_to_new : s.new_to_old;
            std::unordered_map<uint64_t, std::vector<size_t>> renamed;
            for (auto& [f, on] : on_face) {
                const uint64_t g = f < ids.size() ? ids[f] : kNone;
                if (g == kNone) w.unlocated += on.size();
                for (const size_t i : on) pts[i].face = g;
                if (g != kNone) renamed[g] = std::move(on);
            }
            on_face = std::move(renamed);
            return;
        }
        if (s.kind == EditKind::embed) {
            // Each point stays where it is in space, on whichever face of the other side that
            // lies on its own holds it best.
            const Embedding& em = s.embedding;
            const auto& from = forward ? em.before : em.after;
            const auto& to = forward ? em.after : em.before;
            const auto& candidates = forward ? em.to_after : em.to_before;
            std::unordered_map<uint64_t, std::vector<size_t>> moved;
            for (auto& [f, on] : on_face) {
                for (const size_t i : on) {
                    if (f >= candidates.size() || candidates[f].empty()) {
                        ++w.unlocated;
                        pts[i].face = kNone;
                        continue;
                    }
                    const auto& c = from[f];
                    const Eigen::Vector3d p =
                        pts[i].bary[0] * c[0] + pts[i].bary[1] * c[1] + pts[i].bary[2] * c[2];
                    double best = -std::numeric_limits<double>::infinity();
                    uint64_t best_g = candidates[f].front();
                    Eigen::Vector3d best_l(1. / 3, 1. / 3, 1. / 3);
                    for (const uint64_t g : candidates[f]) {
                        const Eigen::Vector3d l = barycentric3(p, to[g]);
                        if (std::isfinite(l.minCoeff()) && l.minCoeff() > best) {
                            best = l.minCoeff();
                            best_g = g;
                            best_l = l;
                        }
                    }
                    if (!(best >= -1e-9)) ++w.unlocated;
                    best_l = best_l.cwiseMax(0.);
                    best_l = best_l.sum() > 0 ? Eigen::Vector3d(best_l / best_l.sum())
                                              : Eigen::Vector3d(1. / 3, 1. / 3, 1. / 3);
                    // The point keeps its place in space: coordinates after, to space, to
                    // coordinates before.
                    Eigen::Matrix3d after;
                    for (int k = 0; k < 3; ++k) after.col(k) = to[best_g][k];
                    K[i] = K[i] * barycentric_gradient(c) * after;
                    pts[i] = {best_g, best_l};
                    lo[i].setZero();
                    computed[i] = 1;
                    moved[best_g].push_back(i);
                }
            }
            on_face = std::move(moved);
            return;
        }
        landed.clear();
        for (const Cell& c : s.cells) {
            const auto& from = forward ? c.before : c.after;
            const auto& to = forward ? c.after : c.before;
            std::optional<CellGeometry> g;
            // The pieces of one face are consecutive.
            for (size_t first = 0; first < from.size();) {
                size_t last = first + 1;
                while (last < from.size() && from[last].fid == from[first].fid) ++last;
                const auto it = on_face.find(from[first].fid);
                if (it != on_face.end()) {
                    if (!g) g.emplace(c, forward);
                    for (const size_t i : it->second) {
                        // The piece of the face holding the point, where it lands after, and its
                        // coordinates there to within kPointAccuracy -- in doubles if their bound
                        // allows, exactly otherwise.
                        const BaryTail tail = {lo[i]};
                        const Hom b =
                            stored_of(pts[i].bary, (lo[i].array() != 0).any() ? &tail : nullptr);
                        bool inside = true, fresh = false;
                        Eigen::Matrix3d Kn = K[i];
                        // The point after, as a double-double, if its bound is within
                        // kPointAccuracy and within its share of what the point it stands for
                        // allows.
                        const auto image =
                            [&]() -> std::optional<std::pair<SurfacePoint, Eigen::Vector3d>> {
                            const auto [piece, l] = g->piece_of(b, first, last);
                            const HWalkPoint at = g->land(piece, l, inside);
                            const Hom h = in_face(to[at.tri], at.bary);
                            Kn = K[i] * g->back(piece, at.tri);
                            const double tol = std::min(
                                kPointAccuracy,
                                (share ? (*share)[i] : 1.) * kStepShare *
                                    source_tolerance(sensitivity(Kn), m_source_accuracy * scale));
                            Eigen::Vector3d v = h.v, low = h.lo;
                            fresh = !h.stored;
                            if (h.x) {
                                for (int k = 0; k < 3; ++k) {
                                    v[k] = (*h.x)[k].to_double();
                                    low[k] = ((*h.x)[k] - Dyadic(v[k])).to_double();
                                }
                            } else if (!(h.e.maxCoeff() <= tol * h.v.sum())) {
                                return std::nullopt;
                            }
                            return std::pair{SurfacePoint{to[at.tri].fid, v}, low};
                        };
                        // In double-doubles, else exactly.
                        std::optional<std::pair<SurfacePoint, Eigen::Vector3d>> p;
                        for (int level = exact::kDoubleDouble; level <= exact::kExact && !p;
                             ++level) {
                            exact::clear_undecided();
                            const exact::Precision precision{exact::Level(level)};
                            const auto q = image();
                            if (!exact::undecided()) p = q;
                        }
                        exact::clear_undecided();
                        if (!inside) ++w.unlocated;
                        pts[i] = p->first;
                        lo[i] = p->second;
                        K[i] = Kn;
                        computed[i] = computed[i] || fresh;
                        landed.emplace_back(p->first.face, i);
                    }
                    on_face.erase(it);
                }
                first = last;
            }
        }
        for (const uint64_t f : forward ? s.dead : s.born) {
            const auto it = on_face.find(f);
            if (it == on_face.end()) continue;
            w.unlocated += it->second.size();
            for (const size_t i : it->second) pts[i].face = kNone;
            on_face.erase(it);
        }
        for (const auto& [f, i] : landed) on_face[f].push_back(i);
    };

    if (forward) {
        for (const Step& s : m_steps) visit(s);
    } else {
        for (auto it = m_steps.rbegin(); it != m_steps.rend(); ++it) visit(*it);
    }
    // Scaled to sum 1, in double-doubles.
    for (size_t i = 0; i < pts.size(); ++i) {
        if (pts[i].face == kNone) continue;
        if (!computed[i]) {
            // Never computed: as given.
            pts[i].lo = lo[i];
            continue;
        }
        Hom h = Hom::of(pts[i].bary);
        h.lo = lo[i];
        const bool was_undecided = exact::undecided();
        Hom n;
        {
            const exact::Precision dd(exact::kDoubleDouble);
            n = exact::normalized(h);
        }
        const bool scaled = !exact::undecided();
        if (!was_undecided) exact::clear_undecided();
        if (scaled) {
            for (int k = 0; k < 3; ++k) {
                if (pts[i].bary[k] == 0) n.v[k] = n.lo[k] = 0;
            }
            pts[i].bary = n.v;
            pts[i].lo = n.lo;
            continue;
        }
        const double sum = (pts[i].bary.sum() + lo[i].sum());
        for (int k = 0; k < 3; ++k) {
            pts[i].bary[k] = pts[i].bary[k] == 0 ? 0. : (pts[i].bary[k] / sum + lo[i][k] / sum);
        }
    }
    if (warnings) *warnings = w;
    return Raw{std::move(pts), std::move(computed), std::move(K)};
}

namespace {

/// A point of a mesh in space, exactly, times the sum of its coordinates: sum_k (bary + lo)_k p_k
/// and that sum.
struct ExactPosition
{
    std::array<Dyadic, 3> x;
    Dyadic sum;
};
ExactPosition exact_position(
    const std::vector<Eigen::Vector3d>& vertices,
    const std::array<uint64_t, 3>& face,
    const Eigen::Vector3d& bary,
    const Eigen::Vector3d& lo)
{
    ExactPosition out;
    for (int k = 0; k < 3; ++k) {
        if (bary[k] == 0 && lo[k] == 0) continue;
        const Dyadic c = Dyadic(bary[k]) + Dyadic(lo[k]);
        out.sum = out.sum + c;
        for (int d = 0; d < 3; ++d) out.x[d] = out.x[d] + c * Dyadic(vertices[face[k]][d]);
    }
    return out;
}
/// b - a for two points: the differences exact, then rounded once each, so the result is as
/// accurate as doubles are however close the points.
Eigen::Vector3d exact_difference(const ExactPosition& a, const ExactPosition& b)
{
    const double scale = a.sum.to_double() * b.sum.to_double();
    Eigen::Vector3d out;
    for (int d = 0; d < 3; ++d) out[d] = (b.x[d] * a.sum - a.x[d] * b.sum).to_double() / scale;
    return out;
}
/// The distance from q to the segment from a to b, to a few units in its own last place however
/// small it is: which part of the segment is nearest, and the cross product, exactly.
double exact_distance(const ExactPosition& q, const ExactPosition& a, const ExactPosition& b)
{
    // U = (q - a) sa sq and W = (b - a) sa sb, exactly.
    std::array<Dyadic, 3> U, W;
    for (int d = 0; d < 3; ++d) {
        U[d] = q.x[d] * a.sum - a.x[d] * q.sum;
        W[d] = b.x[d] * a.sum - a.x[d] * b.sum;
    }
    const Dyadic dot = U[0] * W[0] + U[1] * W[1] + U[2] * W[2];
    const Dyadic ww = W[0] * W[0] + W[1] * W[1] + W[2] * W[2];
    if (ww.sign() == 0 || dot.sign() <= 0) return exact_difference(a, q).norm();
    if ((dot - ww).sign() >= 0) return exact_difference(b, q).norm();
    // |U x W| / |W| = dist sa sq.
    const Dyadic c0 = U[1] * W[2] - U[2] * W[1], c1 = U[2] * W[0] - U[0] * W[2],
                 c2 = U[0] * W[1] - U[1] * W[0];
    const double cross = Eigen::Vector3d(c0.to_double(), c1.to_double(), c2.to_double()).norm();
    const double w = Eigen::Vector3d(W[0].to_double(), W[1].to_double(), W[2].to_double()).norm();
    return cross / w / (a.sum.to_double() * q.sum.to_double());
}

/// The rounds of raising precision a point or a curve gets before it is given up on.
constexpr int kCertifyRounds = 6;

} // namespace

std::vector<SurfacePoint>
SurfaceMap::map(std::vector<SurfacePoint> pts, const bool forward, MapWarnings* warnings) const
{
    MapWarnings w;
    const std::vector<SurfacePoint> given = pts;
    Raw raw = map_raw(std::move(pts), forward, &w, nullptr);
    const size_t n = given.size();
    const double scale = forward ? m_in_scale : m_out_scale;
    const double allowed = m_source_accuracy * scale;
    // A point is stored in doubles where they tell the point of the starting mesh it stands for
    // to within `allowed`, and with its low part as a further term (lo) where they do not.
    // First by its sensitivity: an estimate, right on the pieces of the local maps the point
    // went through but not on their neighbours (a vertex of the starting mesh lands inside a face:
    // around its image are the images of all the faces around it, each with a sensitivity of its
    // own). So what is stored is then mapped back and compared with the point it started as.
    std::vector<SurfacePoint> out = raw.pts;
    std::vector<size_t> check;
    for (size_t i = 0; i < n; ++i) {
        if (out[i].face == kNone || !raw.computed[i]) continue;
        const double tol = source_tolerance(sensitivity(raw.K[i]), allowed) / 4;
        if (out[i].lo.cwiseAbs().maxCoeff() <= tol * out[i].bary.sum()) out[i].lo.setZero();
        check.push_back(i);
    }
    if (!m_certify || !(allowed > 0)) {
        if (warnings) *warnings = w;
        return out;
    }
    const auto& vertices = forward ? m_in_vertices : m_out_vertices;
    const auto& faces = forward ? m_in_faces : m_out_faces;
    std::vector<double> share(n, 1.);
    for (int round = 0; !check.empty(); ++round) {
        std::vector<SurfacePoint> stored;
        for (const size_t i : check) stored.push_back(out[i]);
        const Raw back = map_raw(std::move(stored), !forward, nullptr, nullptr);
        std::vector<size_t> failed;
        for (size_t j = 0; j < check.size(); ++j) {
            const size_t i = check[j];
            const SurfacePoint& b = back.pts[j];
            bool ok = b.face != kNone;
            if (ok) {
                const ExactPosition x =
                    exact_position(vertices, faces[given[i].face], given[i].bary, given[i].lo);
                const ExactPosition y = exact_position(vertices, faces[b.face], b.bary, b.lo);
                ok = exact_difference(x, y).norm() <= allowed;
            }
            if (!ok) failed.push_back(i);
        }
        if (failed.empty()) break;
        if (round + 1 == kCertifyRounds) {
            w.imprecise += failed.size();
            break;
        }
        // With its further term, if it was stored without; else computed again, each step more
        // precisely.
        std::vector<size_t> again;
        for (const size_t i : failed) {
            if ((out[i].lo.array() == 0).all() && (raw.pts[i].lo.array() != 0).any()) {
                out[i].lo = raw.pts[i].lo;
            } else {
                share[i] *= 0x1p-24;
                again.push_back(i);
            }
        }
        if (!again.empty()) {
            std::vector<SurfacePoint> start;
            std::vector<double> shares;
            for (const size_t i : again) {
                start.push_back(given[i]);
                shares.push_back(share[i]);
            }
            const Raw redone = map_raw(std::move(start), forward, nullptr, &shares);
            for (size_t j = 0; j < again.size(); ++j) {
                raw.pts[again[j]] = redone.pts[j];
                out[again[j]] = redone.pts[j];
            }
        }
        check = std::move(failed);
    }
    if (warnings) *warnings = w;
    return out;
}

std::vector<std::vector<SurfaceCurve>> SurfaceMap::curves_to_output(
    const std::vector<SurfaceCurve>& curves,
    MapWarnings* warnings) const
{
    return map_curves(curves, true, warnings);
}

std::vector<std::vector<SurfaceCurve>> SurfaceMap::curves_to_input(
    const std::vector<SurfaceCurve>& curves,
    MapWarnings* warnings) const
{
    return map_curves(curves, false, warnings);
}

std::vector<std::vector<SurfaceCurve>> SurfaceMap::map_curves(
    const std::vector<SurfaceCurve>& curves,
    const bool forward,
    MapWarnings* warnings) const
{
    // The segments given, numbered through the curves.
    std::vector<const SurfaceSegment*> given;
    for (const SurfaceCurve& c : curves) {
        for (const SurfaceSegment& seg : c) given.push_back(&seg);
    }
    const double scale = forward ? m_in_scale : m_out_scale;
    const double allowed = m_source_accuracy * scale;
    const auto& vertices = forward ? m_in_vertices : m_out_vertices;
    const auto& faces = forward ? m_in_faces : m_out_faces;
    const auto position = [&](const SurfaceSegment& seg, const int end) {
        ExactPosition x =
            exact_position(vertices, faces[seg.face], seg.bary[end], Eigen::Vector3d::Zero());
        if (seg.tail) {
            for (const Eigen::Vector3d& t : (*seg.tail)[end]) {
                const ExactPosition more =
                    exact_position(vertices, faces[seg.face], t, Eigen::Vector3d::Zero());
                x.sum = x.sum + more.sum;
                for (int d = 0; d < 3; ++d) x.x[d] = x.x[d] + more.x[d];
            }
        }
        return x;
    };
    // Each given segment's share of the accuracy its points are kept to: 1 at first, less for
    // one whose image, mapped back, does not return to it.
    std::vector<double> accuracy(given.size(), 1.);
    for (int round = 0;; ++round) {
        MapWarnings w;
        std::vector<std::vector<std::vector<size_t>>> origins;
        auto out = map_curves_once(curves, forward, &w, accuracy, &origins);
        if (!m_certify || !(allowed > 0)) {
            if (warnings) *warnings = w;
            return out;
        }
        // Every point of the image is mapped back as a point, and must be on the segment it came
        // from, to within `allowed`. The estimate behind what a point is kept to cannot
        // promise that -- a curve along an edge has another face on its other side, mapped
        // otherwise -- so it is checked, and the segments that fail are mapped again, their points
        // kept more precisely.
        std::vector<SurfacePoint> pts;
        std::vector<size_t> of;
        const auto point = [&](const SurfaceSegment& seg, const int end, const size_t origin) {
            SurfacePoint p{seg.face, seg.bary[end]};
            if (seg.tail) {
                for (const Eigen::Vector3d& t : (*seg.tail)[end]) p.lo += t;
            }
            pts.push_back(p);
            of.push_back(origin);
        };
        for (size_t c = 0; c < out.size(); ++c) {
            for (size_t part = 0; part < out[c].size(); ++part) {
                const SurfaceCurve& pieces = out[c][part];
                for (size_t k = 0; k < pieces.size(); ++k) {
                    point(pieces[k], 0, origins[c][part][k]);
                    if (k + 1 == pieces.size()) point(pieces[k], 1, origins[c][part][k]);
                }
            }
        }
        const Raw back = map_raw(std::move(pts), !forward, nullptr, nullptr);
        std::unordered_map<size_t, std::pair<ExactPosition, ExactPosition>> ends;
        std::unordered_map<size_t, double> worst; // per failing segment, how far off (of allowed)
        size_t failing = 0;
        for (size_t j = 0; j < of.size(); ++j) {
            const SurfacePoint& b = back.pts[j];
            double off = std::numeric_limits<double>::infinity();
            if (b.face != kNone) {
                const auto [slot, fresh] = ends.try_emplace(of[j]);
                if (fresh) slot->second = {position(*given[of[j]], 0), position(*given[of[j]], 1)};
                off = exact_distance(
                          exact_position(vertices, faces[b.face], b.bary, b.lo),
                          slot->second.first,
                          slot->second.second) /
                      allowed;
            }
            if (off <= 1) continue;
            ++failing;
            double& worse = worst[of[j]];
            worse = std::max(worse, off);
        }
        if (failing == 0 || round + 1 == kCertifyRounds) {
            w.imprecise += failing;
            if (warnings) *warnings = w;
            return out;
        }
        for (const auto& [origin, off] : worst) {
            accuracy[origin] *= std::isfinite(off) ? std::min(0x1p-8, 1 / (8 * off)) : 0x1p-24;
        }
    }
}

std::vector<std::vector<SurfaceCurve>> SurfaceMap::map_curves_once(
    const std::vector<SurfaceCurve>& curves,
    const bool forward,
    MapWarnings* warnings,
    const std::vector<double>& accuracy,
    std::vector<std::vector<std::vector<size_t>>>* origins) const
{
    MapWarnings w;
    // The curves as linked lists of segments, since a step replaces one by several anywhere. A
    // segment with no image stays in its list, with no face, to mark where a part ends. Each
    // knows the segment given it is a piece of the image of.
    struct Link
    {
        SurfaceSegment s;
        size_t next;
        size_t origin;
    };
    constexpr size_t kEnd = std::numeric_limits<size_t>::max();
    std::vector<Link> links;
    std::vector<size_t> heads;
    std::unordered_map<uint64_t, std::vector<size_t>> on_face;
    const auto& alive = forward ? m_in_face_alive : m_out_face_alive;
    for (const SurfaceCurve& c : curves) {
        heads.push_back(c.empty() ? kEnd : links.size());
        for (size_t i = 0; i < c.size(); ++i) {
            const size_t id = links.size();
            links.push_back({c[i], i + 1 < c.size() ? id + 1 : kEnd, id});
            if (c[i].face < alive.size() && alive[c[i].face]) {
                on_face[c[i].face].push_back(id);
            } else {
                links[id].s.face = kNone;
                ++w.broken_segments;
            }
        }
    }
    std::vector<std::pair<uint64_t, size_t>> landed;

    const auto drop = [&](const size_t id) {
        links[id].s.face = kNone;
        ++w.broken_segments;
    };
    // The segments `id` becomes, in order, the first in its place.
    const auto replace = [&](const size_t id, const std::vector<SurfaceSegment>& pieces) {
        const size_t next = links[id].next;
        size_t last = id;
        links[id].s = pieces.front();
        landed.emplace_back(pieces.front().face, id);
        for (size_t k = 1; k < pieces.size(); ++k) {
            links.push_back({pieces[k], next, links[id].origin});
            links[last].next = links.size() - 1;
            last = links.size() - 1;
            landed.emplace_back(pieces[k].face, last);
        }
    };
    const auto visit = [&](const Step& s) {
        if (s.kind == EditKind::consolidate) {
            const auto& ids = forward ? s.old_to_new : s.new_to_old;
            std::unordered_map<uint64_t, std::vector<size_t>> renamed;
            for (auto& [f, on] : on_face) {
                const uint64_t g = f < ids.size() ? ids[f] : kNone;
                for (const size_t id : on) {
                    if (g == kNone) {
                        drop(id);
                    } else {
                        links[id].s.face = g;
                    }
                }
                if (g != kNone) renamed[g] = std::move(on);
            }
            on_face = std::move(renamed);
            return;
        }
        landed.clear();
        if (s.kind == EditKind::embed) {
            // A segment keeps its place in space: it is walked over the faces lying on its face,
            // laid out in that face's plane.
            const Embedding& em = s.embedding;
            const auto& from = forward ? em.before : em.after;
            const auto& to = forward ? em.after : em.before;
            const auto& candidates = forward ? em.to_after : em.to_before;
            for (auto& [f, on] : on_face) {
                if (f >= candidates.size() || candidates[f].empty()) {
                    for (const size_t id : on) drop(id);
                    continue;
                }
                const auto& c = from[f];
                const Eigen::Vector3d x = (c[1] - c[0]).normalized();
                const Eigen::Vector3d n = (c[1] - c[0]).cross(c[2] - c[0]).normalized();
                const Eigen::Vector3d y = n.cross(x);
                const auto flat = [&](const Eigen::Vector3d& p) {
                    return Eigen::Vector2d((p - c[0]).dot(x), (p - c[0]).dot(y));
                };
                std::vector<Triangle2> tris;
                std::vector<uint64_t> faces;
                for (const uint64_t g : candidates[f]) {
                    const Triangle2 t = {{flat(to[g][0]), flat(to[g][1]), flat(to[g][2])}};
                    if (!positive(t[0], t[1], t[2])) continue;
                    tris.push_back(t);
                    faces.push_back(g);
                }
                const auto end = [&](const Eigen::Vector3d& b) -> std::optional<WalkPoint> {
                    const Eigen::Vector3d p = b[0] * c[0] + b[1] * c[1] + b[2] * c[2];
                    double best = -std::numeric_limits<double>::infinity();
                    std::optional<WalkPoint> at;
                    for (size_t k = 0; k < faces.size(); ++k) {
                        const Eigen::Vector3d l = barycentric3(p, to[faces[k]]);
                        if (std::isfinite(l.minCoeff()) && l.minCoeff() > best) {
                            best = l.minCoeff();
                            at = WalkPoint{k, l};
                        }
                    }
                    if (at) {
                        at->bary = at->bary.cwiseMax(0.);
                        at->bary /= at->bary.sum();
                    }
                    return at;
                };
                for (const size_t id : on) {
                    const SurfaceSegment seg = links[id].s;
                    const auto a = end(seg.bary[0]), b = end(seg.bary[1]);
                    const auto walked = a && b ? walk_segment(tris, *a, *b) : std::nullopt;
                    if (!walked) {
                        drop(id);
                        continue;
                    }
                    std::vector<SurfaceSegment> pieces;
                    for (const WalkPiece& p : *walked) pieces.push_back({faces[p.tri], p.bary});
                    if (pieces.empty()) pieces.push_back({faces[a->tri], {a->bary, a->bary}});
                    replace(id, pieces);
                }
            }
            on_face.clear();
            for (const auto& [f, id] : landed) on_face[f].push_back(id);
            return;
        }
        for (const Cell& c : s.cells) {
            const auto& from = forward ? c.before : c.after;
            const auto& to = forward ? c.after : c.before;
            std::optional<CellGeometry> g;
            // Every point this step makes: the triangle after it is computed in, and its
            // coordinates there. A point ends the pieces on either side of it, and the end of a
            // segment ends the segments on either side of it too, maybe from the face on the other
            // side of an edge: each such point is computed once, keyed by where it is before.
            struct Node
            {
                size_t tri;
                Hom h;
                int level = exact::kDouble; // the precision it was computed at
            };
            std::vector<Node> nodes;
            std::unordered_map<PointKey, size_t, PointKeyHash> ends;
            // Each segment's image: its points in order, and the triangle of each piece between
            // two of them; a segment of no length has one point and no piece.
            struct Chain
            {
                size_t id, first, last;
                std::vector<size_t> nodes, tris;
                int level = exact::kDouble; // the precision it was traced at
                size_t unlocated = 0;
            };
            std::vector<Chain> chains;

            const auto end_of = [&](const SurfaceSegment& seg, const int k) {
                return stored_of(seg.bary[k], seg.tail ? &(*seg.tail)[k] : nullptr);
            };
            // The image of segment `id`, of the face whose pieces are first..last; none if it has
            // none. Gives up (exact::undecided()) where approximate values cannot decide.
            const auto trace = [&](const size_t id,
                                   const size_t first,
                                   const size_t last) -> std::optional<Chain> {
                const SurfaceSegment& seg = links[id].s;
                size_t unlocated = 0;
                const TriangleH corners = g->face_corners(first, last);
                const Hom b0 = end_of(seg, 0), b1 = end_of(seg, 1);
                // The node of an end: landed once per point. A node landed at a lower precision
                // before (and not exact) is landed again, at this one, in place.
                const auto end_node = [&](const Hom& b, const int k) -> size_t {
                    const PointKey key =
                        key_of(corners, seg.bary[k], seg.tail ? &(*seg.tail)[k] : nullptr);
                    const auto found = ends.find(key);
                    if (found != ends.end() && (nodes[found->second].level >= exact::precision() ||
                                                nodes[found->second].h.known())) {
                        return found->second;
                    }
                    const auto [piece, l] = g->piece_of(b, first, last);
                    bool inside = true;
                    const HWalkPoint at = g->land(piece, l, inside);
                    if (exact::undecided()) return kNoNode;
                    if (found != ends.end()) {
                        nodes[found->second] = {at.tri, at.bary, exact::precision()};
                        return found->second;
                    }
                    if (!inside) ++unlocated;
                    nodes.push_back({at.tri, at.bary, exact::precision()});
                    ends.emplace(key, nodes.size() - 1);
                    return nodes.size() - 1;
                };
                // A face in pieces bends along their edges: cut the segment there first, in the
                // face, exactly.
                std::vector<HWalkPiece> parts = {{0, {b0, b1}}};
                if (last - first > 1) {
                    std::vector<TriangleH> sub_tris;
                    for (size_t k = first; k < last; ++k) sub_tris.push_back(from[k].sub);
                    const auto [a, la] = g->piece_of(b0, first, last);
                    const auto [b, lb] = g->piece_of(b1, first, last);
                    const auto cut =
                        walk(sub_tris, HWalkPoint{a - first, la}, HWalkPoint{b - first, lb});
                    if (!cut || exact::undecided()) return std::nullopt;
                    parts = *cut;
                }
                Chain ch{id, first, last, {}, {}};
                size_t start = end_node(b0, 0);
                if (start == kNoNode) return std::nullopt;
                ch.nodes.push_back(start);
                for (size_t p = 0; p < parts.size(); ++p) {
                    size_t stop;
                    if (p + 1 == parts.size()) {
                        stop = end_node(b1, 1);
                        if (stop == kNoNode) return std::nullopt;
                    } else {
                        bool inside = true;
                        const HWalkPoint at =
                            g->land(first + parts[p].tri, parts[p].bary[1], inside);
                        if (exact::undecided()) return std::nullopt;
                        if (!inside) ++unlocated;
                        nodes.push_back({at.tri, at.bary, exact::precision()});
                        stop = nodes.size() - 1;
                    }
                    if (stop == start) continue; // the same point: no piece
                    const HWalkPoint from_here{nodes[start].tri, nodes[start].h};
                    const HWalkPoint to_there{nodes[stop].tri, nodes[stop].h};
                    const auto walked = walk(g->to_uv, from_here, to_there);
                    if (!walked || exact::undecided()) return std::nullopt;
                    for (size_t q = 0; q < walked->size(); ++q) {
                        ch.tris.push_back((*walked)[q].tri);
                        if (q + 1 < walked->size()) {
                            nodes.push_back(
                                {(*walked)[q].tri, (*walked)[q].bary[1], exact::precision()});
                            ch.nodes.push_back(nodes.size() - 1);
                        }
                    }
                    if (!walked->empty()) ch.nodes.push_back(stop);
                    start = stop;
                }
                ch.unlocated = unlocated;
                return ch;
            };
            // At the lowest precision from `from` on that can decide; none if the segment has no
            // image.
            const auto trace_any = [&](const size_t id,
                                       const size_t first,
                                       const size_t last,
                                       const int from) -> std::optional<Chain> {
                for (int level = from; level <= exact::kExact; ++level) {
                    exact::clear_undecided();
                    const exact::Precision precision{exact::Level(level)};
                    auto r = trace(id, first, last);
                    if (exact::undecided()) continue;
                    if (r) r->level = level;
                    return r;
                }
                exact::clear_undecided();
                return std::nullopt;
            };

            for (size_t first = 0; first < from.size();) {
                size_t last = first + 1;
                while (last < from.size() && from[last].fid == from[first].fid) ++last;
                const auto it = on_face.find(from[first].fid);
                if (it != on_face.end()) {
                    if (!g) g.emplace(c, forward);
                    for (const size_t id : it->second) {
                        if (auto ch = trace_any(id, first, last, exact::kDoubleDouble)) {
                            w.unlocated += ch->unlocated;
                            chains.push_back(std::move(*ch));
                        } else {
                            drop(id);
                        }
                    }
                    on_face.erase(it);
                }
                first = last;
            }
            if (chains.empty()) continue;

            // Each point is kept to as many terms as the pieces next to it need: its error below
            // kCurveSafety of the shortest of them (so no piece turns over, shrinks to nothing or
            // swaps with its neighbour) and below kPointAccuracy. Its coordinates in the face of
            // each piece it ends are computed from where it is computed, so every copy of it
            // agrees.
            std::unordered_map<uint64_t, Hom> version;
            const auto version_of = [&](const size_t n, const size_t t) -> Hom& {
                const auto [slot, fresh] = version.try_emplace(uint64_t(n) * 4096 + t);
                if (fresh) {
                    const auto l = carry(g->to_uv[nodes[n].tri], nodes[n].h, g->to_uv[t]);
                    slot->second = in_face(to[t], l ? *l : nodes[n].h);
                }
                return slot->second;
            };
            std::vector<double> shortest, share;
            const auto tolerance = [&](const size_t n) {
                return std::min(kCurveSafety * shortest[n], kPointAccuracy * share[n]);
            };
            for (int round = 0;; ++round) {
                shortest.assign(nodes.size(), std::numeric_limits<double>::infinity());
                // A point is kept as precisely as the segments it is on ask (`accuracy`).
                share.assign(nodes.size(), 1.);
                for (const Chain& ch : chains) {
                    const double a = accuracy[links[ch.id].origin];
                    for (const size_t n : ch.nodes) share[n] = std::min(share[n], a);
                }
                // The pieces a point ends, and those of every curve near it in the same triangle
                // (other curves may run within a few units in the last place of it, and a point is
                // kept so as not to touch them either).
                struct PieceRef
                {
                    uint64_t face;
                    size_t t, a, b;
                };
                std::vector<PieceRef> all;
                for (const Chain& ch : chains) {
                    for (size_t i = 0; i < ch.tris.size(); ++i) {
                        const size_t a = ch.nodes[i], b = ch.nodes[i + 1];
                        const double d =
                            distance_below(version_of(a, ch.tris[i]), version_of(b, ch.tris[i]));
                        shortest[a] = std::min(shortest[a], d);
                        shortest[b] = std::min(shortest[b], d);
                        all.push_back({to[ch.tris[i]].fid, ch.tris[i], a, b});
                    }
                }
                // By face: the pieces of a face after are in its coordinates, whichever of its
                // pieces (a face may be split virtually) they were walked in.
                std::sort(all.begin(), all.end(), [](const PieceRef& p, const PieceRef& q) {
                    return p.face < q.face;
                });
                struct Box
                {
                    double x0, x1, y0, y1;
                    size_t k;
                };
                std::vector<Box> boxes;
                std::vector<Eigen::Vector2d>
                    ends2; // per piece in the group: its two ends, in doubles
                std::vector<std::optional<Flat>> flats;
                for (size_t g0 = 0; g0 < all.size();) {
                    size_t g1 = g0 + 1;
                    while (g1 < all.size() && all[g1].face == all[g0].face) ++g1;
                    const size_t k = g1 - g0;
                    if (k < 2) {
                        g0 = g1;
                        continue;
                    }
                    ends2.resize(2 * k);
                    flats.assign(2 * k, std::nullopt);
                    boxes.clear();
                    for (size_t i = 0; i < k; ++i) {
                        for (int e = 0; e < 2; ++e) {
                            const Hom& h =
                                version_of(e == 0 ? all[g0 + i].a : all[g0 + i].b, all[g0 + i].t);
                            const double sum = h.v.sum();
                            ends2[2 * i + e] = Eigen::Vector2d(h.v[1] / sum, h.v[2] / sum);
                        }
                        const Eigen::Vector2d &a = ends2[2 * i], &b = ends2[2 * i + 1];
                        boxes.push_back(
                            {std::min(a.x(), b.x()) - kNear,
                             std::max(a.x(), b.x()) + kNear,
                             std::min(a.y(), b.y()) - kNear,
                             std::max(a.y(), b.y()) + kNear,
                             i});
                    }
                    std::sort(boxes.begin(), boxes.end(), [](const Box& p, const Box& q) {
                        return p.x0 < q.x0;
                    });
                    const auto flat_at = [&](const size_t slot) -> const Flat& {
                        if (!flats[slot]) {
                            const PieceRef& r = all[g0 + slot / 2];
                            flats[slot] = flat(version_of(slot % 2 == 0 ? r.a : r.b, r.t));
                        }
                        return *flats[slot];
                    };
                    // End e of piece i against piece j.
                    const auto near = [&](const size_t i, const int e, const size_t j) {
                        const size_t n = e == 0 ? all[g0 + i].a : all[g0 + i].b;
                        const PieceRef& q = all[g0 + j];
                        if (n == q.a || n == q.b) return;
                        // In doubles first: a point farther than 2 kNear needs nothing more.
                        const Eigen::Vector2d &p = ends2[2 * i + e], &a = ends2[2 * j],
                                              &b = ends2[2 * j + 1];
                        const Eigen::Vector2d ab = b - a, ap = p - a;
                        const double l2 = ab.squaredNorm();
                        const double tt = l2 > 0 ? std::clamp(ab.dot(ap) / l2, 0., 1.) : 0.;
                        if ((ap - tt * ab).norm() > 2 * kNear) return;
                        const Flat &fp = flat_at(2 * i + e), &fa = flat_at(2 * j),
                                   &fb = flat_at(2 * j + 1);
                        const double d = fp.ok && fa.ok && fb.ok ? point_segment(fp, fa, fb) : 0.;
                        // Each point is within 2 e of where its value says (scaled to sum 1).
                        const double err = 2 * (fp.e + fa.e + fb.e);
                        if (d > 2 * err || !(fp.ok && fa.ok && fb.ok)) {
                            // The distance is known to within half of itself: every point there
                            // is kept to within a fraction of it.
                            const double below = d - err;
                            if (!(below < kNear)) return;
                            for (const size_t m : {n, q.a, q.b})
                                shortest[m] = std::min(shortest[m], std::max(below, 0.));
                            return;
                        }
                        // Not known from the values. If all three points are known exactly,
                        // exactly: a point on the piece is the same point as one of the piece's
                        // (two curves crossing there), which asks nothing of their precision; off
                        // it, its
                        // distance is |det[p; a; b]| / (sums |a - b|), the determinant exact.
                        const Hom &hp = version_of(n, all[g0 + i].t), &ha = version_of(q.a, q.t),
                                  &hb = version_of(q.b, q.t);
                        if (hp.known() && ha.known() && hb.known()) {
                            const exact::Exact3 x = hp.value(), y = ha.value(), z = hb.value();
                            const Dyadic D = x[0] * (y[1] * z[2] - y[2] * z[1]) -
                                             x[1] * (y[0] * z[2] - y[2] * z[0]) +
                                             x[2] * (y[0] * z[1] - y[1] * z[0]);
                            if (D.sign() == 0) {
                                // On the line through the piece: on the piece itself, or as far
                                // as the nearer of its ends.
                                const double abx = difference(fb.x, fb.xl, fa.x, fa.xl),
                                             aby = difference(fb.y, fb.yl, fa.y, fa.yl);
                                const double apx = difference(fp.x, fp.xl, fa.x, fa.xl),
                                             apy = difference(fp.y, fp.yl, fa.y, fa.yl);
                                const double along = abx * apx + aby * apy,
                                             l2 = abx * abx + aby * aby;
                                if (along > 0 && along < l2) return;
                                const double ends =
                                    std::min(distance_below(hp, ha), distance_below(hp, hb));
                                if (ends < kNear) {
                                    for (const size_t m : {n, q.a, q.b})
                                        shortest[m] = std::min(shortest[m], ends);
                                }
                                return;
                            }
                            const double sums = (x[0] + x[1] + x[2]).to_double() *
                                                (y[0] + y[1] + y[2]).to_double() *
                                                (z[0] + z[1] + z[2]).to_double();
                            const double ab = std::hypot(
                                difference(fb.x, fb.xl, fa.x, fa.xl),
                                difference(fb.y, fb.yl, fa.y, fa.yl));
                            const double exact_d = std::abs(D.to_double()) /
                                                   (sums * std::max(ab, 0x1p-1000)) * (1 - 0x1p-40);
                            if (exact_d < kNear) {
                                for (const size_t m : {n, q.a, q.b})
                                    shortest[m] = std::min(shortest[m], exact_d);
                            }
                            return;
                        }
                        // Otherwise the points whose error makes it unknown are computed more
                        // precisely first (their distance to nothing is known: 0), and asked again.
                        const double emax = std::max({fp.e, fa.e, fb.e});
                        const std::array<std::pair<size_t, double>, 3> who = {
                            {{n, fp.e}, {q.a, fa.e}, {q.b, fb.e}}};
                        for (const auto& [m, em] : who) {
                            if (em >= emax / 4) shortest[m] = 0;
                        }
                    };
                    for (size_t i = 0; i < boxes.size(); ++i) {
                        for (size_t j = i + 1; j < boxes.size() && boxes[j].x0 <= boxes[i].x1;
                             ++j) {
                            if (boxes[j].y0 > boxes[i].y1 || boxes[i].y0 > boxes[j].y1) continue;
                            const size_t P = boxes[i].k, Q = boxes[j].k;
                            for (int e = 0; e < 2; ++e) {
                                near(P, e, Q);
                                near(Q, e, P);
                            }
                        }
                    }
                    g0 = g1;
                }
                // A point whose bound is too large is computed exactly: its copy in a face, if the
                // point is known exactly; the whole segment otherwise.
                std::vector<size_t> again;
                for (size_t ci = 0; ci < chains.size(); ++ci) {
                    const Chain& ch = chains[ci];
                    bool redo = false;
                    for (size_t i = 0; i < ch.nodes.size() && !redo; ++i) {
                        const size_t n = ch.nodes[i];
                        const double tol = tolerance(n);
                        for (const size_t t :
                             {i > 0 ? ch.tris[i - 1] : nodes[n].tri,
                              i < ch.tris.size() ? ch.tris[i] : nodes[n].tri}) {
                            Hom& h = version_of(n, t);
                            if (h.x || h.e.maxCoeff() <= tol * h.v.sum()) continue;
                            if (nodes[n].h.known()) {
                                const auto l =
                                    carry(g->to_uv[nodes[n].tri], nodes[n].h, g->to_uv[t]);
                                for (const exact::Level level :
                                     {exact::kDoubleDouble, exact::kExact}) {
                                    const exact::Precision precision{level};
                                    h = in_face(to[t], l ? *l : nodes[n].h);
                                    if (h.x || h.e.maxCoeff() <= tol * h.v.sum()) break;
                                }
                            } else {
                                redo = true;
                                break;
                            }
                        }
                    }
                    if (redo) again.push_back(ci);
                }
                // Each segment again, one precision up, until none needs it: at most twice each,
                // since an exact one needs nothing more.
                bool raised = false;
                for (const size_t ci : again) {
                    Chain& ch = chains[ci];
                    if (ch.level == exact::kExact) continue;
                    if (auto again_chain = trace_any(ch.id, ch.first, ch.last, ch.level + 1)) {
                        ch = std::move(*again_chain);
                        raised = true;
                    }
                }
                if (!raised) break;
                version.clear();
            }

            for (const Chain& ch : chains) {
                // At least as precisely as a double-double: the doubles of a point are within a
                // unit in their last place of it, but a later step may amplify that a millionfold
                // (a needle squeezed into a sliver of its domain), so a point computed more
                // precisely keeps its next term too. Points are carried so from step to step, as
                // SurfaceMap::map carries them, and stored so at the end.
                const auto terms = [&](const size_t n, const size_t t) {
                    const Hom& h = version_of(n, t);
                    auto out = terms_of(h, tolerance(n));
                    if (!out) {
                        // Not reached once every segment is exact; else as precise as it is.
                        out = std::pair{h.v, BaryTail{}};
                    }
                    if (!h.stored && out->second.empty() && h.zeros() < 2) {
                        Eigen::Vector3d next = Eigen::Vector3d::Zero();
                        if (h.x) {
                            for (int k = 0; k < 3; ++k) {
                                if (out->first[k] != 0)
                                    next[k] = ((*h.x)[k] - Dyadic(out->first[k])).to_double();
                            }
                        } else if (out->first == h.v) {
                            next = h.lo;
                        }
                        if ((next.array() != 0).any()) out->second.push_back(next);
                    }
                    return std::move(*out);
                };
                const auto segment = [&](const size_t t, const size_t a, const size_t b) {
                    auto ta = terms(a, t), tb = terms(b, t);
                    SurfaceSegment out{to[t].fid, {ta.first, tb.first}};
                    if (!ta.second.empty() || !tb.second.empty()) {
                        out.tail = std::make_shared<const std::array<BaryTail, 2>>(
                            std::array<BaryTail, 2>{std::move(ta.second), std::move(tb.second)});
                    }
                    return out;
                };
                std::vector<SurfaceSegment> pieces;
                if (ch.tris.empty()) {
                    // A segment of no length stays one.
                    const size_t n = ch.nodes.front();
                    pieces.push_back(segment(nodes[n].tri, n, n));
                }
                for (size_t i = 0; i < ch.tris.size(); ++i) {
                    pieces.push_back(segment(ch.tris[i], ch.nodes[i], ch.nodes[i + 1]));
                }
                replace(ch.id, pieces);
            }
        }
        for (const uint64_t f : forward ? s.dead : s.born) {
            const auto it = on_face.find(f);
            if (it == on_face.end()) continue;
            for (const size_t id : it->second) drop(id);
            on_face.erase(it);
        }
        for (const auto& [f, id] : landed) on_face[f].push_back(id);
    };

    if (forward) {
        for (const Step& s : m_steps) visit(s);
    } else {
        for (auto it = m_steps.rbegin(); it != m_steps.rend(); ++it) visit(*it);
    }

    std::vector<std::vector<SurfaceCurve>> out(curves.size());
    if (origins) origins->assign(curves.size(), {});
    for (size_t c = 0; c < curves.size(); ++c) {
        SurfaceCurve part;
        std::vector<size_t> from;
        const auto close = [&]() {
            if (part.empty()) return;
            out[c].push_back(std::move(part));
            if (origins) (*origins)[c].push_back(std::move(from));
            part.clear();
            from.clear();
        };
        for (size_t id = heads[c]; id != kEnd; id = links[id].next) {
            if (links[id].s.face == kNone) {
                close();
            } else {
                part.push_back(links[id].s);
                from.push_back(links[id].origin);
            }
        }
        close();
    }
    if (warnings) *warnings = w;
    return out;
}

std::vector<const SurfaceMap::Cell*> SurfaceMap::cells_of(const size_t edit) const
{
    std::vector<const Cell*> out;
    auto it = std::lower_bound(m_steps.begin(), m_steps.end(), edit, [](const Step& s, size_t e) {
        return s.edit < e;
    });
    for (; it != m_steps.end() && it->edit == edit; ++it) {
        for (const Cell& c : it->cells) out.push_back(&c);
    }
    return out;
}

} // namespace wmtk::components::tracking
