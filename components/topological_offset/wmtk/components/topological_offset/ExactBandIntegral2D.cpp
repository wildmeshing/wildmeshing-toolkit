#include "ExactBandIntegral2D.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <string>

// Every function below is the function of the same name in ExactBandIntegral.cpp, one dimension
// down, with its constants and tolerances; a comment says where the plane differs.

namespace wmtk::components::topological_offset {

namespace {

using Vec2 = Eigen::Vector2d;
using Stats = ExactBandIntegral2D::Stats;

constexpr double EPS = std::numeric_limits<double>::epsilon();

double maxabs(const Vec2& p)
{
    return p.cwiseAbs().maxCoeff();
}

double cross2(const Vec2& a, const Vec2& b)
{
    return a.x() * b.y() - a.y() * b.x();
}

/// a rotated clockwise: the outward normal direction of an edge along a of a counter-clockwise
/// polygon
Vec2 rot_cw(const Vec2& a)
{
    return Vec2(a.y(), -a.x());
}

/// The partition's tolerance budget and rounding scale (3D's Ctx, without its timers), and an edge
/// partition's direction and cap (3D's face partition's ring_n and ring_cap; seg_tol()).
struct Ctx
{
    double* tol_cut = nullptr;
    double* tol_any = nullptr;
    double scale = 0.;
    const Eigen::Vector2d* seg_t = nullptr;
    double seg_cap = 0.;
};

// ------------------------------------------------------------------------------------------------
// Convex polygons: V counter-clockwise, edge i from V[i] to V[i + 1] with outward unit normal N[i],
// the normal of the line it was cut by (t's edge or a clip line). 3D's Poly keeps each face's
// plane normal for the same use: the derivative recognises a piece's edge on an edge of t by its
// normal, and takes a cut point's normal from the neighbouring edge, where a normal recomputed
// from a short edge's ends would be off by eps / its length.
// ------------------------------------------------------------------------------------------------

struct Poly
{
    std::vector<Vec2> V, N;
    int size() const { return int(V.size()); }
    void clear()
    {
        V.clear();
        N.clear();
    }
    double max_abs() const
    {
        double m = 0.;
        for (const Vec2& p : V) m = std::max(m, maxabs(p));
        return m;
    }
};

/// tet_polytope: t counter-clockwise with outward edge normals; false if flat.
bool tri_polygon(const std::array<Vec2, 3>& T, Poly& out)
{
    double L2 = 0.;
    for (int i = 0; i < 3; ++i)
        L2 = std::max(L2, (T[size_t(i)] - T[size_t((i + 1) % 3)]).squaredNorm());
    const double a2 = cross2(T[1] - T[0], T[2] - T[0]);
    if (L2 == 0. || std::abs(a2) <= 64 * EPS * L2) return false;
    out.clear();
    for (int i = 0; i < 3; ++i) out.V.push_back(T[size_t(a2 > 0. ? i : (3 - i) % 3)]);
    for (int i = 0; i < 3; ++i) {
        out.N.push_back(rot_cw(out.V[size_t((i + 1) % 3)] - out.V[size_t(i)]).normalized());
    }
    return true;
}

enum class Cut { None, Same, New };

/// clip: Q intersected with {x : g.x <= h}, |g| = 1. Vertices within rounding distance of the
/// line are on it. None: no interior; Same: Q entirely inside (`out` untouched); New: the result
/// in `out`. Each vertex is put on, inside or outside the line once and each crossing computed
/// once, from its inside end, as 3D's clip() does; a polygon's edge is shared by no other face, so
/// the result is closed by construction. Edges no longer than tol (max norm) are collapsed, the
/// later vertex merged into the earlier, as 3D's.
Cut clip(const Poly& Q, const Vec2& g, const double h, Poly& out, const Ctx& cx)
{
    const double tol = 16 * EPS * (std::abs(h) + std::max(Q.max_abs(), cx.scale));
    const int n = Q.size();
    thread_local std::vector<double> sv; // per-thread scratch, as 3D's (clip is not re-entrant)
    sv.resize(size_t(n));
    double smax = -std::numeric_limits<double>::infinity(), smin = -smax;
    for (int i = 0; i < n; ++i) {
        const double si = Q.V[size_t(i)].dot(g) - h;
        smax = std::max(smax, si);
        smin = std::min(smin, si);
        sv[size_t(i)] = std::abs(si) <= tol ? 0. : si;
    }
    if (cx.tol_any) *cx.tol_any += tol;
    if (smax <= tol) return Cut::Same; // entirely inside
    if (smin >= -tol) return Cut::None; // entirely outside (or flat on the line)
    out.clear();
    for (int i = 0; i < n; ++i) {
        const int j = (i + 1) % n;
        const double si = sv[size_t(i)], sj = sv[size_t(j)];
        const Vec2& p = Q.V[size_t(i)];
        const Vec2& q = Q.V[size_t(j)];
        if (si < 0.) {
            out.V.push_back(p);
            out.N.push_back(Q.N[size_t(i)]);
            if (sj > 0.) { // leaves: the crossing, then the cut
                out.V.push_back(p + (si / (si - sj)) * (q - p));
                out.N.push_back(g);
            }
        } else if (si == 0.) {
            out.V.push_back(p);
            out.N.push_back(sj > 0. ? g : Q.N[size_t(i)]);
        } else if (sj < 0.) { // enters: the crossing, from its inside end q
            out.V.push_back(q + (sj / (sj - si)) * (p - q));
            out.N.push_back(Q.N[size_t(i)]);
        }
    }
    size_t m = 0;
    for (size_t k = 0; k < out.V.size(); ++k) {
        if (m > 0 && maxabs(out.V[k] - out.V[m - 1]) <= tol) {
            out.N[m - 1] = out.N[k]; // the edge leaving the merged vertex
            continue;
        }
        out.V[m] = out.V[k];
        out.N[m] = out.N[k];
        ++m;
    }
    while (m > 1 && maxabs(out.V[m - 1] - out.V[0]) <= tol) --m;
    out.V.resize(m);
    out.N.resize(m);
    if (cx.tol_cut) *cx.tol_cut += tol;
    return m >= 3 ? Cut::New : Cut::None;
}

/// R = clip(R, g, h) in place; false (R emptied) if None.
bool clip_inplace(Poly& R, const Vec2& g, const double h, Poly& tmp, const Ctx& cx)
{
    const Cut c = clip(R, g, h, tmp, cx);
    if (c == Cut::None) {
        R.clear();
        return false;
    }
    if (c == Cut::New) std::swap(R, tmp);
    return true;
}

/// _clip_affine: Q intersected with {w.x <= rhs}, in place; false if empty.
bool clip_affine(Poly& R, const Vec2& w, const double rhs, Poly& tmp, const Ctx& cx)
{
    const double nw = w.norm();
    return clip_inplace(R, w / nw, rhs / nw, tmp, cx);
}

/// volume_centroid: area and centroid of a convex polygon (fan from one vertex).
std::pair<double, Vec2> area_centroid(const Poly& Q)
{
    const Vec2 o = Q.V[0];
    double a2 = 0.;
    Vec2 mom = Vec2::Zero();
    for (int i = 1; i + 1 < Q.size(); ++i) {
        const Vec2 p = Q.V[size_t(i)] - o, q = Q.V[size_t(i + 1)] - o;
        const double d = cross2(p, q);
        a2 += d;
        mom += d * (p + q);
    }
    if (a2 <= 0.) return {0., o};
    return {a2 / 2., o + mom / (3. * a2)};
}

// ------------------------------------------------------------------------------------------------
// One-feature integrals. A 2D edge's distance is affine on each side of its line (3D's face);
// a point's by Euler's theorem, int_Q f = 1/3 sum_e h_e int_e f ds for f homogeneous of degree 1
// about v (3D: 1/4 over faces), each edge integral 3D's face_point() edge term at height 0.
// ------------------------------------------------------------------------------------------------

/// int_{u1}^{u1+ell} sqrt(u^2 + k^2) du, ell > 0, k >= 0 (3D's, verbatim: rationalized where the
/// two ends have the same sign, so a short or far interval does not cancel).
double int_sqrt(double u1, const double ell, const double k)
{
    double u2 = u1 + ell;
    if (k == 0.) return 0.5 * (u2 * std::abs(u2) - u1 * std::abs(u1));
    if (u1 * u2 <= 0.) { // interval contains u = 0: no cancellation
        const auto F = [k](double u) {
            return 0.5 * (u * std::hypot(u, k) + k * k * std::asinh(u / k));
        };
        return F(u2) - F(u1);
    }
    if (u2 < 0.) { // reflect to the positive side
        const double a = -u2, b = -u1;
        u1 = a;
        u2 = b;
    }
    const double R1 = std::hypot(u1, k), R2 = std::hypot(u2, k);
    const double A = ell * (u1 + u2) * (u1 * u1 + u2 * u2 + k * k) / (u2 * R2 + u1 * R1);
    const double B = std::asinh(ell * (u1 + u2) / (u2 * R1 + u1 * R2));
    return 0.5 * (A + k * k * B);
}

/// int over the segment [x1, x2] of |x - v| ds.
double edge_point(const Vec2& x1, const Vec2& x2, const Vec2& v)
{
    const Vec2 d = x2 - x1;
    const double ell = d.norm();
    if (ell == 0.) return 0.;
    const Vec2 t = d / ell;
    const Vec2 r = x1 - v;
    return int_sqrt(r.dot(t), ell, std::abs(cross2(t, r)));
}

/// _through: the edge line passes through o up to rounding; its Euler term is rounding noise.
bool through(const double h, const Vec2& x1, const Vec2& x2, const Vec2& o)
{
    return std::abs(h) <= 16 * EPS * (std::max(maxabs(x1), maxabs(x2)) + maxabs(o));
}

/// vol_point: int_Q |x - v| dA.
double area_point(const Poly& Q, const Vec2& v)
{
    double tot = 0.;
    const int n = Q.size();
    for (int i = 0; i < n; ++i) {
        const Vec2& x1 = Q.V[size_t(i)];
        const Vec2& x2 = Q.V[size_t((i + 1) % n)];
        const double h = Q.N[size_t(i)].dot(x1 - v);
        if (!through(h, x1, x2, v)) tot += h * edge_point(x1, x2, v);
    }
    return tot / 3.;
}

/// vol_plane: int_Q |n.(x - a0)| dA, split by the line, affine on each side.
double area_line(const Poly& Q, const Vec2& n, const Vec2& a0, const Ctx& cx)
{
    double tot = 0.;
    Poly tmp;
    for (const double sg : {1., -1.}) { // keep sg n.(x - a0) >= 0
        const Cut c = clip(Q, -sg * n, -sg * n.dot(a0), tmp, cx);
        if (c == Cut::None) continue;
        const auto [a, cen] = area_centroid(c == Cut::Same ? Q : tmp);
        tot += a * (sg * n.dot(cen) - sg * n.dot(a0));
    }
    return tot;
}

// ------------------------------------------------------------------------------------------------
// The partition (3D's Loc, d_feat, bounds, classify, split, prune_members).
// ------------------------------------------------------------------------------------------------

struct Loc
{
    int kind; // 1 edge, 2 vertex (3D's numbering)
    Vec2 p, dir; // edge: (a, unit normal); vertex: (v, -)
    int h_begin, h_end; // into the local half-planes
};

double int_g(const Loc& g, const Poly& Q, const Ctx& cx)
{
    if (g.kind == 1) return area_line(Q, g.dir, g.p, cx);
    return area_point(Q, g.p);
}

/// _d_feat: d_g at one point.
double d_feat(const Loc& g, const Vec2& y)
{
    if (g.kind == 1) return std::abs((y - g.p).dot(g.dir));
    return (y - g.p).norm();
}

/// _bounds: (lower, upper) bound of d_g over the convex hull of the points Vq. Upper: the max at
/// the points (d_g convex). Lower: exact for a line (affine per side, 3D's face), else d_g(c) - max
/// |x_i - c| (1-Lipschitz).
std::pair<double, double> bounds(const Loc& g, const std::vector<Vec2>& Vq)
{
    double up = 0.;
    for (const Vec2& y : Vq) up = std::max(up, d_feat(g, y));
    double lo;
    if (g.kind == 1) {
        double smin = std::numeric_limits<double>::infinity(), smax = -smin, amin = smin;
        for (const Vec2& y : Vq) {
            const double s = (y - g.p).dot(g.dir);
            smin = std::min(smin, s);
            smax = std::max(smax, s);
            amin = std::min(amin, std::abs(s));
        }
        lo = (smin < 0. && 0. < smax) ? 0. : amin;
    } else {
        Vec2 c = Vec2::Zero();
        for (const Vec2& y : Vq) c += y;
        c /= double(Vq.size());
        double r = 0.;
        for (const Vec2& y : Vq) r = std::max(r, (y - c).norm());
        lo = std::max(0., d_feat(g, c) - r);
    }
    return {lo, up};
}

enum class Side { In, Out, Split };

/// _classify: 'in' if the piece lies in all half-planes, 'out' if outside one, else 'split'.
Side classify(
    const Poly& Q,
    const std::pair<Vec2, double>* H,
    const int nh,
    double* tol_any,
    const double scale)
{
    const double ma = std::max(Q.max_abs(), scale);
    bool s_in = true;
    for (int i = 0; i < nh; ++i) {
        const Vec2& g = H[i].first;
        const double h = H[i].second;
        double smin = std::numeric_limits<double>::infinity(), smax = -smin;
        for (const Vec2& p : Q.V) {
            const double s = p.dot(g) - h;
            smin = std::min(smin, s);
            smax = std::max(smax, s);
        }
        const double tol = 16 * EPS * (std::abs(h) + ma);
        if (tol_any) *tol_any += tol;
        if (smin >= -tol) return Side::Out;
        if (smax > tol) s_in = false;
    }
    return s_in ? Side::In : Side::Split;
}

/// _split: Q inside all of H (R; emptied if None), and the convex parts of Q outside.
void split(
    const Poly& Q,
    const std::pair<Vec2, double>* H,
    const int nh,
    Poly& R,
    std::vector<Poly>& outside,
    const Ctx& cx)
{
    R = Q;
    Poly tmp;
    for (int i = 0; i < nh; ++i) {
        const Vec2& g = H[i].first;
        const double h = H[i].second;
        const Cut c = clip(R, -g, -h, tmp, cx);
        if (c == Cut::Same) outside.push_back(R);
        if (c == Cut::New) outside.push_back(std::move(tmp));
        if (!clip_inplace(R, g, h, tmp, cx)) break;
    }
}

/// _prune_points: drop g if its lower bound over the points Vq is >= another member's upper bound
/// (it cannot attain the minimum on their hull); changes no value.
std::vector<int>
prune_points(const std::vector<Vec2>& Vq, const std::vector<int>& S, const std::vector<Loc>& loc)
{
    std::vector<std::pair<double, double>> b;
    b.reserve(S.size());
    for (const int k : S) b.push_back(bounds(loc[size_t(k)], Vq));
    double ubmin = std::numeric_limits<double>::infinity();
    for (const auto& lu : b) ubmin = std::min(ubmin, lu.second);
    std::vector<int> kept;
    for (size_t i = 0; i < S.size(); ++i) {
        if (b[i].first < ubmin || b[i].second == ubmin) kept.push_back(S[i]);
    }
    return kept;
}

/// _prune_members: prune_points() over the piece's vertices.
std::vector<int>
prune_members(const Poly& Q, const std::vector<int>& S, const std::vector<Loc>& loc)
{
    return prune_points(Q.V, S, loc);
}

// ------------------------------------------------------------------------------------------------
// Exact pieces of several members (3D's faces_min_pieces, vertices_min_pieces).
// ------------------------------------------------------------------------------------------------

/// parallel_face_loses: whether edge i is nowhere the min against edge j of the same slope W in a
/// sign cell (parallel lines); within rounding the two are one function and the lower index wins.
/// In the plane this holds for the collinear segments of a straight run of the input as well.
bool parallel_edge_loses(
    const double rhs,
    const double ci,
    const double cj,
    const size_t i,
    const size_t j,
    const Ctx& cx)
{
    const double tie = 64 * EPS * (std::abs(ci) + std::abs(cj) + cx.scale);
    return rhs < -tie || (rhs <= tie && j < i);
}

/// faces_min_pieces with the edges' lines for the faces' planes: Q split so that min_i |n_i.(x -
/// a_i)| is one affine function W.x - c per sub-piece (split by the lines, each |.| then affine,
/// then by the bisectors of the affine distances); `piece(R, W, c, i, sg)` for each sub-piece.
template <typename F>
void edges_min_pieces(const Poly& Q, const std::vector<const Loc*>& edges, const Ctx& cx, F&& piece)
{
    struct Cell
    {
        Poly C;
        std::vector<double> sg;
    };
    std::vector<Cell> cells(1);
    cells[0].C = Q;
    Poly tmp;
    for (const Loc* f : edges) {
        const Vec2& n = f->dir;
        const Vec2& a0 = f->p;
        std::vector<Cell> next;
        for (const Cell& cl : cells) {
            for (const double s : {1., -1.}) { // keep s n.(x - a0) >= 0
                const Cut c = clip(cl.C, -s * n, -s * n.dot(a0), tmp, cx);
                if (c == Cut::None) continue;
                Cell nc;
                nc.C = c == Cut::Same ? cl.C : std::move(tmp);
                nc.sg = cl.sg;
                nc.sg.push_back(s);
                next.push_back(std::move(nc));
            }
        }
        cells = std::move(next);
    }
    const size_t nf = edges.size();
    std::vector<Vec2> W(nf);
    std::vector<double> cc(nf);
    Poly R;
    for (const Cell& cl : cells) {
        for (size_t i = 0; i < nf; ++i) { // d_i = W_i . x - c_i
            W[i] = cl.sg[i] * edges[i]->dir;
            cc[i] = cl.sg[i] * edges[i]->dir.dot(edges[i]->p);
        }
        for (size_t i = 0; i < nf; ++i) {
            R = cl.C;
            bool alive = true;
            for (size_t j = 0; j < nf && alive; ++j) {
                if (j == i) continue;
                const Vec2 w = W[i] - W[j];
                const double rhs = cc[i] - cc[j];
                if (w.norm() <= 1e-14) { // parallel lines: the nearer wins (the lower on a tie)
                    if (parallel_edge_loses(rhs, cc[i], cc[j], i, j, cx)) alive = false;
                    continue;
                }
                alive = clip_affine(R, w, rhs, tmp, cx);
            }
            if (alive) piece(R, W[i], cc[i], int(i), cl.sg[i]);
        }
    }
}

/// _vertices_min_pieces: split Q by the perpendicular bisectors of the points; `piece(R, v, i)`
/// for each sub-piece with its nearest vertex v.
template <typename F>
void vertices_min_pieces(
    const Poly& Q,
    const std::vector<const Loc*>& verts,
    const Ctx& cx,
    F&& piece)
{
    Poly R, tmp;
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec2& vi = verts[i]->p;
        R = Q;
        bool alive = true;
        for (size_t j = 0; j < verts.size() && alive; ++j) {
            if (j == i) continue;
            const Vec2& vj = verts[j]->p;
            alive = clip_affine(R, 2. * (vj - vi), vj.dot(vj) - vi.dot(vi), tmp, cx);
        }
        if (alive) piece(R, vi, int(i));
    }
}

// ------------------------------------------------------------------------------------------------
// The offset side (3D's SidePlane, side_clip, side_split, nearest_member).
// ------------------------------------------------------------------------------------------------

/// A member's side line in t's frame: unit normal N, a point ql on it; has false: none.
struct SidePlane
{
    bool has = false;
    Vec2 N, ql;
};

/// _side_clip: Q cap {sigma N.(x - q) >= 0}; Same where the line is none (every point counts).
Cut side_clip(const Poly& Q, const SidePlane& sp, const int sigma, Poly& out, const Ctx& cx)
{
    if (!sp.has) return Cut::Same;
    return clip(Q, -double(sigma) * sp.N, -double(sigma) * sp.N.dot(sp.ql), out, cx);
}

/// _side_split: Q split by the members' lines: parts with, per member, whether the part is on that
/// line's offset side; the offset-side part of a cut first.
struct SidePart
{
    Poly R;
    std::vector<char> on;
};
void side_split(
    const Poly& Q,
    const std::vector<SidePlane>& planes,
    const int sigma,
    std::vector<SidePart>& parts,
    const Ctx& cx)
{
    parts.assign(1, SidePart());
    parts[0].R = Q;
    std::vector<SidePart> next;
    Poly in, out;
    for (const SidePlane& sp : planes) {
        next.clear();
        for (SidePart& c : parts) {
            const Cut k = side_clip(c.R, sp, sigma, in, cx);
            if (k == Cut::Same || k == Cut::None) {
                c.on.push_back(k == Cut::Same);
                next.push_back(std::move(c));
                continue;
            }
            SidePart a;
            a.R = in;
            a.on = c.on;
            a.on.push_back(1);
            const Cut ko =
                clip(c.R, double(sigma) * sp.N, double(sigma) * sp.N.dot(sp.ql), out, cx);
            next.push_back(std::move(a));
            if (ko == Cut::None) continue;
            SidePart b;
            b.R = ko == Cut::Same ? c.R : out;
            b.on = std::move(c.on);
            b.on.push_back(0);
            next.push_back(std::move(b));
        }
        std::swap(parts, next);
    }
}

/// The member nearest at the centroid of the part R: the side of a part where the members' lines
/// disagree is that member's (3D's nearest_member(), the pseudonormal theorem in the plane).
size_t nearest_member(const Poly& R, const std::vector<const Loc*>& members)
{
    const Vec2 c = area_centroid(R).second;
    size_t best = 0;
    double dmin = std::numeric_limits<double>::infinity();
    for (size_t m = 0; m < members.size(); ++m) {
        const double dm = d_feat(*members[m], c);
        if (dm < dmin) dmin = dm, best = m;
    }
    return best;
}

double point_seg_dist(const Vec2& p, const Vec2& A, const Vec2& B)
{
    const Vec2 d = B - A;
    const double dd = d.dot(d);
    const double s = dd > 0. ? std::clamp((p - A).dot(d) / dd, 0., 1.) : 0.;
    return (p - A - s * d).norm();
}

double box_dist(const Vec2& lo, const Vec2& hi, const Vec2& tlo, const Vec2& thi)
{
    double s = 0.;
    for (int k = 0; k < 2; ++k) {
        const double g = std::max(0., std::max(lo[k] - thi[k], tlo[k] - hi[k]));
        s += g * g;
    }
    return std::sqrt(s);
}

struct Piece
{
    Poly Q;
    std::vector<int> S; // member candidates
};

/// t's corners and delta in hex floats, bit exact, for the error messages.
std::string cell_string(const std::array<Vec2, 3>& T, const double delta)
{
    std::string s = "t =";
    char buf[96];
    for (const Vec2& p : T) {
        std::snprintf(buf, sizeof(buf), " (%a, %a)", p[0], p[1]);
        s += buf;
    }
    std::snprintf(buf, sizeof(buf), ", delta = %a", delta);
    return s + buf;
}

// ------------------------------------------------------------------------------------------------
// The elementary pieces and their boundary integrals (3D's Elem, piece_value, moments,
// cut_integral). Kind 0: phi = dir.x - c (affine); kind 2: phi = |x - p|.
// ------------------------------------------------------------------------------------------------

struct Elem
{
    Poly Q;
    int kind = 0;
    Vec2 p = Vec2::Zero(), dir = Vec2::Zero();
    double c = 0.;
};

double piece_value(const Elem& e)
{
    if (e.kind == 0) {
        const auto [a, cen] = area_centroid(e.Q);
        return a * (e.dir.dot(cen) - e.c);
    }
    return area_point(e.Q, e.p);
}

constexpr double SHORT = 0.5; ///< 3D's: a segment is "short" if ell <= SHORT |mid - v|
constexpr int K_SEG = 30; ///< 3D's Taylor order on short segments (ratio <= SHORT/2 = 1/4)
constexpr int NJ = K_SEG + 4; ///< the longest series: K_SEG + the numerator's 4 coefficients

double binom(const int j, const int i)
{
    double r = 1.;
    for (int m = 1; m <= i; ++m) r = r * double(j - i + m) / double(m);
    return r;
}

/// _pshift: coefficients (ascending) of P(tau + c), P of length n.
void pshift(const double* P, const int n, const double c, double* out)
{
    for (int i = 0; i < n; ++i) out[i] = 0.;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i <= j; ++i) out[i] += P[j] * binom(j, i) * std::pow(c, double(j - i));
    }
}

/// _pmul: the product polynomial; returns its length.
int pmul(const double* a, const int na, const double* b, const int nb, double* c)
{
    const int n = na + nb - 1;
    for (int k = 0; k < n; ++k) {
        double s = 0.;
        for (int j = std::max(0, k - nb + 1); j <= std::min(k, na - 1); ++j) s += a[j] * b[k - j];
        c[k] = s;
    }
    return n;
}

/// _M_closed: [int u^i (u^2 + k^2)^(power/2) du for i = 0..imax] (indefinite).
void M_closed(const double u, const double k, const int imax, const int power, double* M)
{
    const double S = std::hypot(u, k);
    if (power == 1) {
        const double S3 = std::pow(S, 3.);
        M[0] = 0.5 * (u * S + (k > 0 ? k * k * std::asinh(u / k) : 0.));
        if (imax >= 1) M[1] = S3 / 3.;
        for (int i = 2; i <= imax; ++i) {
            M[i] = (std::pow(u, double(i - 1)) * S3 - (i - 1) * k * k * M[i - 2]) / (i + 2);
        }
    } else {
        M[0] = std::asinh(u / k);
        if (imax >= 1) M[1] = S;
        for (int i = 2; i <= imax; ++i) {
            M[i] = (std::pow(u, double(i - 1)) * S - (i - 1) * k * k * M[i - 2]) / i;
        }
    }
}

void jsqrt(const double* a, const int n, double* c)
{
    c[0] = std::sqrt(a[0]);
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        for (int j = 1; j < k; ++j) s += c[j] * c[k - j];
        c[k] = (a[k] - s) / (2. * c[0]);
    }
}

void jdiv(const double* a, const double* b, const int n, double* c)
{
    c[0] = a[0] / b[0];
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        for (int j = 1; j <= k; ++j) s += b[j] * c[k - j];
        c[k] = (a[k] - s) / b[0];
    }
}

/// seg_int: int_0^ell P(sigma) S(sigma)^power d sigma, S(sigma) = |w0 + sigma t|, t a unit vector,
/// power = +1 or -1, P ascending in sigma (length np <= 4). With power = -1 the caller guarantees
/// P/S is bounded (P vanishes where S does). 3D's seg_int() for a point (its tp = t, sa = 1), the
/// same branches: short against the distance to the complex zeros of S^2 (ell <= SHORT S(mid)),
/// the Taylor series of the integrand about the midpoint, ratio <= SHORT/2, K_SEG + np terms;
/// otherwise closed forms in u (S = sqrt(u^2 + k^2)), binomial sums bounded by (1 + 2/SHORT)^deg.
double seg_int(
    const Vec2& w0,
    const Vec2& t,
    const double ell,
    const double* P,
    const int np,
    const int power)
{
    const Vec2 wm = w0 + 0.5 * ell * t;
    const double Sm = wm.norm();
    double Pc[4];
    pshift(P, np, 0.5 * ell, Pc); // in tau = sigma - ell/2
    if (Sm > 0 && ell <= SHORT * Sm) {
        // Taylor series in tau' = tau / (ell/2) in [-1, 1] (scaled, so no coefficient overflows)
        const int K = K_SEG + np;
        const double hl = 0.5 * ell;
        double S2[NJ] = {}, S[NJ], Si[NJ], g[NJ];
        S2[0] = Sm * Sm;
        S2[1] = 2 * wm.dot(t) * hl;
        S2[2] = hl * hl;
        jsqrt(S2, K, S);
        const double* Sp = S;
        if (power == -1) {
            double one[NJ] = {};
            one[0] = 1.;
            jdiv(one, S, K, Si);
            Sp = Si;
        }
        double Ps[4];
        for (int j = 0; j < np; ++j) Ps[j] = Pc[j] * std::pow(hl, double(j));
        for (int k = 0; k < K; ++k) {
            double sk = 0.;
            for (int j = 0; j <= std::min(k, np - 1); ++j) sk += Ps[j] * Sp[k - j];
            g[k] = sk;
        }
        double sum = 0.;
        for (int i = 0; i < K; i += 2) sum += g[i] * 2. / double(i + 1);
        return hl * sum;
    }
    const double um = wm.dot(t);
    const double k = std::abs(cross2(wm, t));
    const double u1 = um - 0.5 * ell, u2 = um + 0.5 * ell;
    const int deg = np - 1;
    if (power == -1 && k <= 16 * EPS * (std::abs(um) + ell)) {
        // the segment meets the zero of S at u = 0: P/S = Q(tau) sign(u) with P = (tau - tau0) Q
        const double tau0 = -um;
        double out[4];
        double acc = 0.;
        for (int j = deg; j >= 0; --j) { // synthetic division by (tau - tau0), descending
            acc = acc * tau0 + Pc[j];
            out[deg - j] = acc;
        }
        double Q[4]; // quotient, ascending: out[:-1] reversed
        for (int j = 0; j < deg; ++j) Q[j] = out[deg - 1 - j];
        const auto F = [&](const double x) {
            double s = 0.;
            for (int j = 0; j < deg; ++j) s += Q[j] * std::pow(x, double(j + 1)) / double(j + 1);
            return s;
        };
        const double lo = -0.5 * ell, hi = 0.5 * ell;
        const double t0 = std::min(std::max(tau0, lo), hi);
        return (F(hi) - F(t0)) - (F(t0) - F(lo));
    }
    double M1[4], M2[4];
    M_closed(u1, k, deg, power, M1);
    M_closed(u2, k, deg, power, M2);
    double tot = 0.;
    for (int j = 0; j <= deg; ++j) {
        if (Pc[j] == 0.) continue;
        double sj = 0.;
        for (int i = 0; i <= j; ++i) {
            sj += binom(j, i) * std::pow(-um, double(j - i)) * (M2[i] - M1[i]);
        }
        tot += Pc[j] * sj;
    }
    return tot;
}

/// _moments on the trace [y1, y2] of a piece on an edge of t: (P1, P2) = (int lam phi ds,
/// int lam^2 grad phi ds), lam = l0 + g0.y. Affine phi: Simpson, exact for the quadratics.
void moments(
    const Vec2& y1,
    const Vec2& y2,
    const Elem& e,
    const double l0,
    const Vec2& g0,
    double& P1,
    Vec2& P2)
{
    const Vec2 d = y2 - y1;
    const double ell = d.norm();
    const auto lam = [&](const Vec2& y) { return l0 + g0.dot(y); };
    if (e.kind == 0) {
        const Vec2 ym = 0.5 * (y1 + y2);
        const auto f = [&](const Vec2& y) { return lam(y) * (e.dir.dot(y) - e.c); };
        const double l1 = lam(y1), lm = lam(ym), l2 = lam(y2);
        P1 = ell * (f(y1) + 4 * f(ym) + f(y2)) / 6.;
        P2 = e.dir * (ell * (l1 * l1 + 4 * lm * lm + l2 * l2) / 6.);
        return;
    }
    // grad |y - p| = (y - p) / |y - p| = (w0 + sigma t) / S along the trace
    const Vec2 t = d / ell;
    const Vec2 w0 = y1 - e.p;
    const double lp[2] = {lam(y1), g0.dot(t)};
    double l2p[3];
    pmul(lp, 2, lp, 2, l2p);
    P1 = seg_int(w0, t, ell, lp, 2, 1);
    for (int k = 0; k < 2; ++k) {
        const double wk[2] = {w0[k], t[k]};
        double Pk[4];
        pmul(l2p, 3, wk, 2, Pk);
        P2[k] = seg_int(w0, t, ell, Pk, 4, -1);
    }
}

/// _cut_integral at a cut point y, 3D's integral over a cut edge one dimension down: lam(y)^2
/// phi(y).
double cut_value(const Vec2& y, const Elem& e, const double l0, const Vec2& g0)
{
    const double l = l0 + g0.dot(y);
    const double phi = e.kind == 0 ? e.dir.dot(y) - e.c : (y - e.p).norm();
    return l * l * phi;
}

// ------------------------------------------------------------------------------------------------
// An edge's partition (3D's face partition one dimension down: ring_tol(), pclip(), pclassify(),
// psplit() and label_face()'s helpers), for front_edge_derivs(). A part of f is a Span: an
// interval of f, and per end the unit normal of the line that cut f there, out of the part (zero
// at an end of f). 3D's rings keep no normals and find a cut edge's plane among the partition's
// recorded planes (Ctx::planes); a span's ends are made by its own clips, which name the line.
// ------------------------------------------------------------------------------------------------

struct Span
{
    std::array<Vec2, 2> y; ///< the ends, in f's direction
    std::array<Vec2, 2> g{{Vec2::Zero(), Vec2::Zero()}}; ///< per end its cut line's normal
    double max_abs() const { return std::max(maxabs(y[0]), maxabs(y[1])); }
};

/// ring_tol(): what a decision of tolerance tol against the line of unit normal g adds to an edge
/// partition's budget: tol / |g.t|, t f's unit direction (Ctx::seg_t) -- the strip |g.x - h| <= tol
/// meets f's line in an interval 2 tol / |g.t| long -- at most Ctx::seg_cap and never below tol;
/// tol itself outside an edge partition.
double seg_tol(const Ctx& cx, const Vec2& g, const double tol)
{
    if (!cx.seg_t) return tol;
    return std::max(tol, std::min(tol / std::abs(g.dot(*cx.seg_t)), cx.seg_cap));
}

/// pclip(): S cut by {g.x <= h}, |g| = 1: Same (S inside up to tol, `out` untouched), None
/// (nothing inside, or a part no longer than tol: clean_ring()'s), or New (in `out`, the cut
/// end's normal g), the crossing computed from the inside end.
Cut sclip(const Span& S, const Vec2& g, const double h, Span& out, const Ctx& cx)
{
    const double tol = 16 * EPS * (std::abs(h) + std::max(S.max_abs(), cx.scale));
    const double budget = seg_tol(cx, g, tol);
    if (cx.tol_any) *cx.tol_any += budget;
    const double s[2] = {S.y[0].dot(g) - h, S.y[1].dot(g) - h};
    if (std::max(s[0], s[1]) <= tol) return Cut::Same;
    if (std::min(s[0], s[1]) >= -tol) return Cut::None;
    const int a = s[0] < 0. ? 0 : 1, b = 1 - a;
    const Vec2 yc = S.y[size_t(a)] + (s[a] / (s[a] - s[b])) * (S.y[size_t(b)] - S.y[size_t(a)]);
    if (cx.tol_cut) *cx.tol_cut += budget;
    if (maxabs(yc - S.y[size_t(a)]) <= tol) return Cut::None;
    out = S;
    out.y[size_t(b)] = yc;
    out.g[size_t(b)] = g;
    return Cut::New;
}

/// Whether S lies on the line g.x = h, up to sclip()'s tol: inside both of its half-planes.
bool on_line(const Span& S, const Vec2& g, const double h, const Ctx& cx)
{
    const double tol = 16 * EPS * (std::abs(h) + std::max(S.max_abs(), cx.scale));
    return std::abs(S.y[0].dot(g) - h) <= tol && std::abs(S.y[1].dot(g) - h) <= tol;
}

/// pclassify(): as classify(), the inside tested first (a span on a region's line is inside it).
Side sclassify(const Span& S, const std::pair<Vec2, double>* H, const int nh, const Ctx& cx)
{
    const double ma = std::max(S.max_abs(), cx.scale);
    bool s_in = true;
    for (int i = 0; i < nh; ++i) {
        const Vec2& g = H[i].first;
        const double h = H[i].second;
        const double s0 = S.y[0].dot(g) - h, s1 = S.y[1].dot(g) - h;
        const double tol = 16 * EPS * (std::abs(h) + ma);
        if (cx.tol_any) *cx.tol_any += seg_tol(cx, g, tol);
        if (std::max(s0, s1) <= tol) continue;
        if (std::min(s0, s1) >= -tol) return Side::Out;
        s_in = false;
    }
    return s_in ? Side::In : Side::Split;
}

/// psplit(): the part of S inside all of H (R; false if none) and the parts outside.
bool ssplit(
    const Span& S,
    const std::pair<Vec2, double>* H,
    const int nh,
    Span& R,
    std::vector<Span>& outside,
    const Ctx& cx)
{
    R = S;
    Span tmp;
    for (int i = 0; i < nh; ++i) {
        const Vec2& g = H[i].first;
        const double h = H[i].second;
        const double tol = 16 * EPS * (std::abs(h) + std::max(R.max_abs(), cx.scale));
        if (std::max(R.y[0].dot(g) - h, R.y[1].dot(g) - h) <= tol) continue; // inside, on it too
        const Cut c = sclip(R, -g, -h, tmp, cx);
        if (c == Cut::Same) outside.push_back(R);
        if (c == Cut::New) outside.push_back(tmp);
        const Cut k = sclip(R, g, h, tmp, cx);
        if (k == Cut::None) return false;
        if (k == Cut::New) R = tmp;
    }
    return true;
}

/// A labelled part of f: its span and its integrand d, an Elem without its polygon (kind 0 the
/// affine W.x - c of an edge member on one side of its line, kind 2 |x - p| of a vertex member);
/// `far`: an edge member's part kept as the offset side of its line (3D's Field::far).
struct SpanPart
{
    Span S;
    Elem e;
    bool far = false;
};

/// labeled(): the parts of R on which g's distance is one smooth function, on the offset side. An
/// edge member is split by its line into its two affine sides, of which only the offset side is
/// kept when it has a side line (sp), both otherwise; a span lying on the line (inside both clips
/// up to tol) is one part, not two (3D MEASURED on its rings: counted twice, a front face on the
/// input doubled int (d - delta) phi). A vertex part is clipped by its side line.
void labeled_span(
    const Span& R,
    const Loc& g,
    const SidePlane& sp,
    const int sigma,
    const Ctx& cx,
    std::vector<SpanPart>& out)
{
    const bool side = sigma != 0 && sp.has;
    Span C;
    if (g.kind == 1) {
        const Vec2& n = g.dir;
        const double hn = n.dot(g.p);
        for (const double s : {1., -1.}) { // keep s n.(x - a0) >= 0
            if (side && !(double(sigma) * s * n.dot(sp.N) > 0.)) continue; // the far side
            const Cut c = sclip(R, -s * n, -s * hn, C, cx);
            if (c == Cut::None) continue;
            SpanPart p;
            p.S = c == Cut::Same ? R : C;
            p.e.kind = 0;
            p.e.dir = s * n;
            p.e.c = s * hn;
            p.far = side;
            out.push_back(std::move(p));
            if (c == Cut::Same) break;
        }
        return;
    }
    SpanPart p;
    p.e.kind = 2;
    p.e.p = g.p;
    if (!side) {
        p.S = R;
        out.push_back(std::move(p));
        return;
    }
    const Cut c = sclip(R, -double(sigma) * sp.N, -double(sigma) * sp.N.dot(sp.ql), C, cx);
    if (c == Cut::None) return;
    p.S = c == Cut::Same ? R : C;
    out.push_back(std::move(p));
}

/// edges_min_pieces() on a span: split by the edges' lines (each |.| then affine; a span on a
/// line is one sign cell, as labeled_span() keeps it), then by the bisectors of the affine
/// distances; `piece(R, W, c, i, sg)` per sub-span. A span on the bisector of two members goes to
/// the lower one, parallel_edge_loses()'s tie.
template <typename F>
void edges_min_spans(const Span& Q, const std::vector<const Loc*>& edges, const Ctx& cx, F&& piece)
{
    struct Cell
    {
        Span C;
        std::vector<double> sg;
    };
    std::vector<Cell> cells(1);
    cells[0].C = Q;
    Span tmp;
    for (const Loc* f : edges) {
        const Vec2& n = f->dir;
        const Vec2& a0 = f->p;
        std::vector<Cell> next;
        for (const Cell& cl : cells) {
            for (const double s : {1., -1.}) { // keep s n.(x - a0) >= 0
                const Cut c = sclip(cl.C, -s * n, -s * n.dot(a0), tmp, cx);
                if (c == Cut::None) continue;
                Cell nc;
                nc.C = c == Cut::Same ? cl.C : tmp;
                nc.sg = cl.sg;
                nc.sg.push_back(s);
                next.push_back(std::move(nc));
                if (c == Cut::Same) break;
            }
        }
        cells = std::move(next);
    }
    const size_t nf = edges.size();
    std::vector<Vec2> W(nf);
    std::vector<double> cc(nf);
    Span R;
    for (const Cell& cl : cells) {
        for (size_t i = 0; i < nf; ++i) { // d_i = W_i . x - c_i
            W[i] = cl.sg[i] * edges[i]->dir;
            cc[i] = cl.sg[i] * edges[i]->dir.dot(edges[i]->p);
        }
        for (size_t i = 0; i < nf; ++i) {
            R = cl.C;
            bool alive = true;
            for (size_t j = 0; j < nf && alive; ++j) {
                if (j == i) continue;
                const Vec2 w = W[i] - W[j];
                const double rhs = cc[i] - cc[j];
                const double nw = w.norm();
                if (nw <= 1e-14) { // parallel lines: the nearer wins (the lower on a tie)
                    if (parallel_edge_loses(rhs, cc[i], cc[j], i, j, cx)) alive = false;
                    continue;
                }
                const Cut c = sclip(R, w / nw, rhs / nw, tmp, cx);
                if (c == Cut::None) {
                    alive = false;
                } else if (c == Cut::New) {
                    R = tmp;
                } else if (j < i && on_line(R, w / nw, rhs / nw, cx)) {
                    alive = false;
                }
            }
            if (alive) piece(R, W[i], cc[i], int(i), cl.sg[i]);
        }
    }
}

/// vertices_min_pieces() on a span: split by the perpendicular bisectors of the points; `piece(R,
/// v, i)` per sub-span with its nearest vertex v. A span on a bisector goes to the lower vertex.
template <typename F>
void vertices_min_spans(
    const Span& Q,
    const std::vector<const Loc*>& verts,
    const Ctx& cx,
    F&& piece)
{
    Span R, tmp;
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec2& vi = verts[i]->p;
        R = Q;
        bool alive = true;
        for (size_t j = 0; j < verts.size() && alive; ++j) {
            if (j == i) continue;
            const Vec2& vj = verts[j]->p;
            const Vec2 w = 2. * (vj - vi);
            const double nw = w.norm();
            const double rhs = (vj.dot(vj) - vi.dot(vi)) / nw;
            const Cut c = sclip(R, w / nw, rhs, tmp, cx);
            if (c == Cut::None) {
                alive = false;
            } else if (c == Cut::New) {
                R = tmp;
            } else if (j < i && on_line(R, w / nw, rhs, cx)) {
                alive = false;
            }
        }
        if (alive) piece(R, vi, int(i));
    }
}

/// side_split() on a span: parts with, per member, whether the part is on that member's side
/// line's offset side; the offset-side part of a cut first.
struct SpanSidePart
{
    Span R;
    std::vector<char> on;
};
void span_side_split(
    const Span& Q,
    const std::vector<SidePlane>& planes,
    const int sigma,
    std::vector<SpanSidePart>& parts,
    const Ctx& cx)
{
    parts.assign(1, SpanSidePart());
    parts[0].R = Q;
    std::vector<SpanSidePart> next;
    Span in, out;
    for (const SidePlane& sp : planes) {
        next.clear();
        for (SpanSidePart& c : parts) {
            const Cut k =
                sp.has ? sclip(c.R, -double(sigma) * sp.N, -double(sigma) * sp.N.dot(sp.ql), in, cx)
                       : Cut::Same; // side_clip(): every point counts
            if (k == Cut::Same || k == Cut::None) {
                c.on.push_back(k == Cut::Same);
                next.push_back(std::move(c));
                continue;
            }
            SpanSidePart a;
            a.R = in;
            a.on = c.on;
            a.on.push_back(1);
            const Cut ko =
                sclip(c.R, double(sigma) * sp.N, double(sigma) * sp.N.dot(sp.ql), out, cx);
            next.push_back(std::move(a));
            if (ko == Cut::None) continue;
            SpanSidePart b;
            b.R = ko == Cut::Same ? c.R : out;
            b.on = std::move(c.on);
            b.on.push_back(0);
            next.push_back(std::move(b));
        }
        std::swap(parts, next);
    }
}

/// int over the span R of d_g lam ds, lam = l0 + g0.x: rule (a)'s integral over the thin swept
/// triangle's part above R, in the limit and up to its height (3D's label_face() int_d_phi). An
/// edge member over its two affine sides.
double span_int_d_lam(const Span& R, const Loc& g, const double l0, const Vec2& g0, const Ctx& cx)
{
    Elem e;
    double P1 = 0.;
    Vec2 P2;
    if (!(maxabs(R.y[1] - R.y[0]) > 0.)) return 0.;
    if (g.kind == 2) {
        e.kind = 2;
        e.p = g.p;
        moments(R.y[0], R.y[1], e, l0, g0, P1, P2);
        return P1;
    }
    double s = 0.;
    Span C;
    for (const double sg : {1., -1.}) {
        const Cut c = sclip(R, -sg * g.dir, -sg * g.dir.dot(g.p), C, cx);
        if (c == Cut::None) continue;
        const Span& P = c == Cut::Same ? R : C;
        e.kind = 0;
        e.dir = sg * g.dir;
        e.c = sg * g.dir.dot(g.p);
        moments(P.y[0], P.y[1], e, l0, g0, P1, P2);
        s += P1;
        if (c == Cut::Same) break;
    }
    return s;
}

/// nearest_member() for a rule-(a) part over the span R whose members' side lines disagree: the
/// member nearest at the centroid of the thin swept triangle's part above R, in the limit -- R's
/// points weighted by the part's height, the hat function lam = l0 + g0.x of the moving corner.
size_t nearest_member_span(
    const Span& R,
    const std::vector<const Loc*>& members,
    const double l0,
    const Vec2& g0)
{
    const double l1 = l0 + g0.dot(R.y[0]), l2 = l0 + g0.dot(R.y[1]);
    const double tau = l1 + l2 > 0. ? (l1 + 2. * l2) / (3. * (l1 + l2)) : 0.5;
    const Vec2 c = R.y[0] + tau * (R.y[1] - R.y[0]);
    size_t best = 0;
    double dmin = std::numeric_limits<double>::infinity();
    for (size_t m = 0; m < members.size(); ++m) {
        const double dm = d_feat(*members[m], c);
        if (dm < dmin) dmin = dm, best = m;
    }
    return best;
}

/// f's corners in hex floats, bit exact, for the error messages (3D's face_string()).
std::string edge_string(const std::array<Vec2, 2>& f)
{
    std::string s = "f =";
    char buf[96];
    for (const Vec2& p : f) {
        std::snprintf(buf, sizeof(buf), " (%a, %a)", p[0], p[1]);
        s += buf;
    }
    return s;
}

} // namespace

// ------------------------------------------------------------------------------------------------
// ExactBandIntegral2D
// ------------------------------------------------------------------------------------------------

ExactBandIntegral2D::ExactBandIntegral2D(
    const Eigen::MatrixXd& Vin,
    const Eigen::MatrixXi& Ein,
    const std::vector<int>& Pin,
    const std::vector<int8_t>& segment_side,
    const int sigma)
{
    const int ns = int(Ein.rows()), np = int(Pin.size());
    if (sigma != 0 && int(segment_side.size()) != ns) {
        log_and_throw_error(
            "ExactBandIntegral2D: {} segment sides for {} segments",
            segment_side.size(),
            ns);
    }
    // The referenced vertices, merged by exact coordinates and sorted lexicographically (3D's
    // numpy.unique), the segments' ends before the points.
    std::vector<Vec2> corner(size_t(2 * ns + np));
    for (int s = 0; s < ns; ++s) {
        for (int c = 0; c < 2; ++c) {
            corner[size_t(2 * s + c)] = Vin.row(Ein(s, c)).head<2>().transpose();
        }
    }
    const size_t pt0 = size_t(2 * ns);
    for (int p = 0; p < np; ++p)
        corner[pt0 + size_t(p)] = Vin.row(Pin[size_t(p)]).head<2>().transpose();
    if (corner.empty()) log_and_throw_error("ExactBandIntegral2D: an empty input");
    std::vector<int> ord(corner.size());
    std::iota(ord.begin(), ord.end(), 0);
    const auto lex = [&](int a, int b) {
        const Vec2 &p = corner[size_t(a)], &q = corner[size_t(b)];
        if (p[0] != q[0]) return p[0] < q[0];
        return p[1] < q[1];
    };
    std::sort(ord.begin(), ord.end(), lex);
    std::vector<Vec2> V;
    std::vector<int> inv(corner.size());
    for (size_t i = 0; i < ord.size(); ++i) {
        if (i == 0 || lex(ord[i - 1], ord[i])) V.push_back(corner[size_t(ord[i])]);
        inv[size_t(ord[i])] = int(V.size()) - 1;
    }

    // The edges (segments whose ends did not merge) in row order, and the primitives for d_P:
    // the segments, then the points.
    struct Edge
    {
        int a, b, side;
    };
    std::vector<Edge> edges;
    for (int s = 0; s < ns; ++s) {
        const int a = inv[size_t(2 * s)], b = inv[size_t(2 * s + 1)];
        m_SA.push_back(V[size_t(a)]);
        m_SB.push_back(V[size_t(b)]);
        if (a != b) edges.push_back({a, b, sigma != 0 ? int(segment_side[size_t(s)]) : 0});
    }
    for (int p = 0; p < np; ++p) {
        m_SA.push_back(V[size_t(inv[pt0 + size_t(p)])]);
        m_SB.push_back(V[size_t(inv[pt0 + size_t(p)])]);
    }
    m_nE = int(edges.size());
    std::vector<std::vector<int>> vedges(V.size()); // per vertex its edges
    for (size_t i = 0; i < edges.size(); ++i) {
        vedges[size_t(edges[i].a)].push_back(int(i));
        vedges[size_t(edges[i].b)].push_back(int(i));
    }

    // features: (kind, data, half-planes); region = {x : g.(x - q) <= 0 for all (g, q)}
    const auto unit = [](const Vec2& v) -> Vec2 { return v / v.norm(); };
    for (const Edge& ed : edges) {
        const Vec2 &a = V[size_t(ed.a)], &b = V[size_t(ed.b)];
        const Vec2 u = unit(b - a);
        Feature ft;
        ft.kind = 1;
        ft.p = a;
        ft.dir = Vec2(-u.y(), u.x());
        ft.h_begin = int(m_hs.size());
        m_hs.push_back({-u, a});
        m_hs.push_back({u, b});
        ft.h_end = int(m_hs.size());
        ft.lo = a.cwiseMin(b);
        ft.hi = a.cwiseMax(b);
        m_feat.push_back(ft);
    }
    for (size_t iv = 0; iv < V.size(); ++iv) {
        const Vec2& v = V[iv];
        Feature ft;
        ft.kind = 2;
        ft.p = v;
        ft.dir.setZero();
        ft.h_begin = int(m_hs.size());
        for (const int i : vedges[iv]) {
            const int w = edges[size_t(i)].a == int(iv) ? edges[size_t(i)].b : edges[size_t(i)].a;
            m_hs.push_back({unit(V[size_t(w)] - v), v});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = ft.hi = v;
        m_feat.push_back(ft);
    }

    // The offset side (class doc): per edge its line, the normal out of the solid by its side; per
    // vertex of exactly two sided edges the pseudonormal, their normals' sum.
    if (sigma != 0) {
        m_sigma = sigma > 0 ? 1 : -1;
        const size_t ng = m_feat.size();
        m_side_has.assign(ng, 0);
        m_side_n.assign(ng, Vec2::Zero());
        m_side_q.assign(ng, Vec2::Zero());
        for (size_t i = 0; i < edges.size(); ++i) {
            if (edges[i].side == 0) continue;
            m_side_has[i] = 1;
            m_side_n[i] = -double(edges[i].side) * m_feat[i].dir; // (b - a) rotated clockwise
            m_side_q[i] = m_feat[i].p;
        }
        for (size_t iv = 0; iv < V.size(); ++iv) {
            const std::vector<int>& ve = vedges[iv];
            if (ve.size() != 2 || !m_side_has[size_t(ve[0])] || !m_side_has[size_t(ve[1])])
                continue;
            const Vec2 m = m_side_n[size_t(ve[0])] + m_side_n[size_t(ve[1])];
            const double nm = m.norm();
            if (!(nm > 0.)) continue;
            const size_t g = size_t(m_nE) + iv;
            m_side_has[g] = 1;
            m_side_n[g] = m / nm;
            m_side_q[g] = V[iv];
        }
    }
}

double ExactBandIntegral2D::upper_bound(const Eigen::Vector2d* t, const int n) const
{
    double m = std::numeric_limits<double>::infinity();
    for (size_t f = 0; f < m_SA.size(); ++f) {
        double s = 0.;
        for (int i = 0; i < n; ++i) {
            s = std::max(s, point_seg_dist(t[i], m_SA[f], m_SB[f]));
            if (s >= m) break; // this primitive cannot lower the min: the result is unchanged
        }
        m = std::min(m, s);
    }
    return m;
}

double ExactBandIntegral2D::min_mean_distance(const Eigen::Vector2d* t, const int n, int64_t& best)
    const
{
    double m = std::numeric_limits<double>::infinity();
    best = -1;
    for (size_t f = 0; f < m_SA.size(); ++f) {
        double s = 0.;
        for (int i = 0; i < n; ++i) s += point_seg_dist(t[i], m_SA[f], m_SB[f]);
        s /= n;
        // increasing index, so a tie keeps the lowest
        if (s < m) m = s, best = int64_t(f);
    }
    return m;
}

double ExactBandIntegral2D::primitive_distance(
    const int64_t P,
    const Eigen::Vector2d& x,
    Eigen::Vector2d* grad,
    Eigen::Matrix2d* hess) const
{
    // See the declaration. The value and the foot are point_seg_dist()'s; the foot's kind is its
    // clamp's.
    const Vec2& A = m_SA[size_t(P)];
    const Vec2& B = m_SB[size_t(P)];
    const double d = point_seg_dist(x, A, B);
    if (!grad && !hess) return d;
    const Vec2 ab = B - A;
    const double dd = ab.dot(ab);
    const double s = dd > 0. ? (x - A).dot(ab) / dd : 0.;
    const bool interior = dd > 0. && s > 0. && s < 1.;
    const Vec2 foot = A + std::clamp(s, 0., 1.) * ab;
    const Vec2 u = d > 0. ? Vec2((x - foot) / d) : Vec2::Zero();
    if (grad) *grad = u;
    if (hess) {
        hess->setZero();
        if (d > 0. && !interior) *hess = (Eigen::Matrix2d::Identity() - u * u.transpose()) / d;
    }
    return d;
}

struct ExactBandIntegral2D::Work
{
    Vec2 c0; ///< the local origin: t's centroid
    Poly Q0; ///< t in local coordinates
    std::vector<Loc> loc; ///< the candidate features, local
    std::vector<int> cand; ///< their feature indices
    std::vector<std::pair<Vec2, double>> hs; ///< their half-planes, local
    double U = 0.;
    double tol_cut = 0., tol_any = 0.; ///< the partition's tolerance budget (Ctx)
    double scale = 0.; ///< the rounding scale (Ctx::scale)
    std::vector<Piece> pieces; ///< the partition of t with member sets
};

bool ExactBandIntegral2D::partition(const std::array<Eigen::Vector2d, 3>& T, Work& w, Stats* stats)
    const
{
    Ctx cx;
    w.tol_cut = w.tol_any = 0.;
    cx.tol_cut = &w.tol_cut;
    cx.tol_any = &w.tol_any;
    // step 0: candidates in local coordinates (origin c0 = centroid of t; 3D's localize())
    w.c0 = ((T[0] + T[1]) + T[2]) / 3.;
    const Vec2& c0 = w.c0;
    std::array<Vec2, 3> Tl;
    for (int i = 0; i < 3; ++i) Tl[size_t(i)] = T[size_t(i)] - c0;
    if (!tri_polygon(Tl, w.Q0)) return false;
    Vec2 tlo = T[0], thi = T[0];
    for (const Vec2& p : T) {
        tlo = tlo.cwiseMin(p);
        thi = thi.cwiseMax(p);
    }
    w.U = upper_bound(T.data(), 3);
    const double r = w.U * (1 + 1e-12);
    w.cand.clear();
    w.loc.clear();
    w.hs.clear();
    w.scale = 0.;
    for (size_t gi = 0; gi < m_feat.size(); ++gi) {
        const Feature& ft = m_feat[gi];
        if (!(box_dist(ft.lo, ft.hi, tlo, thi) <= r)) continue;
        w.cand.push_back(int(gi));
        Loc l;
        l.kind = ft.kind;
        l.p = ft.p - c0;
        l.dir = ft.dir;
        l.h_begin = int(w.hs.size());
        for (int i = ft.h_begin; i < ft.h_end; ++i) {
            const auto& [g, q] = m_hs[size_t(i)];
            w.hs.push_back({g, g.dot(q - c0)}); // in t's frame (3D's m_hs)
            w.scale = std::max(w.scale, maxabs(q - c0));
        }
        l.h_end = int(w.hs.size());
        w.loc.push_back(l);
    }
    if (stats) stats->candidates += int64_t(w.cand.size());
    // the rounding scale (3D's localize()): the corners and the candidates' half-plane points
    for (const Vec2& p : T) w.scale = std::max(w.scale, maxabs(p - c0));
    cx.scale = w.scale;

    // step 1: the convex pieces of t with constant member sets
    std::vector<Piece>& pieces = w.pieces;
    std::vector<Piece> next;
    pieces.assign(1, Piece());
    pieces[0].Q = w.Q0;
    std::vector<Poly> outs;
    for (int k = 0; k < int(w.loc.size()); ++k) {
        const std::pair<Vec2, double>* H = w.hs.data() + w.loc[size_t(k)].h_begin;
        const int nh = w.loc[size_t(k)].h_end - w.loc[size_t(k)].h_begin;
        next.clear();
        for (Piece& pc : pieces) {
            const Side st = classify(pc.Q, H, nh, cx.tol_any, cx.scale);
            if (st == Side::In) {
                pc.S.push_back(k);
                next.push_back(std::move(pc));
            } else if (st == Side::Out) {
                next.push_back(std::move(pc));
            } else {
                Piece in;
                outs.clear();
                split(pc.Q, H, nh, in.Q, outs, cx);
                if (in.Q.size() > 0) {
                    in.S = pc.S;
                    in.S.push_back(k);
                    next.push_back(std::move(in));
                }
                for (Poly& qo : outs) {
                    Piece o;
                    o.Q = std::move(qo);
                    o.S = pc.S;
                    next.push_back(std::move(o));
                }
            }
        }
        std::swap(pieces, next);
    }
    return true;
}

double ExactBandIntegral2D::evaluate(
    const std::array<Eigen::Vector2d, 3>& T,
    const double delta,
    const int kv,
    Eigen::Vector2d* G,
    Eigen::Matrix2d* H,
    const bool want_value,
    Stats* stats) const
{
    if (G) G->setZero();
    if (H) H->setZero();
    Work w;
    if (!partition(T, w, stats)) return want_value ? 0. : std::numeric_limits<double>::quiet_NaN();
    Ctx cx;
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    const double A0 = area_centroid(w.Q0).first;

    // 3D's check_cell_pieces(), the bounds one dimension down. A cutting clip snaps vertices
    // within tol of the line and collapses edges at tol (max norm), so each part differs from the
    // exact one only in the strip |g.x - h| <= sqrt(2) tol, whose chord of R is at most P(R)/2 <=
    // P(t)/2 (perimeters of nested convex sets are monotone): the area moves by at most sqrt(2)
    // tol P(t) per cutting clip. A piece with no member exists only because a decision taken with
    // a tolerance put a strip of half-width tol on the wrong side: at most tol P(t) of area per
    // decision. Each area is a fan sum of m cross products of vectors no longer than D, t's
    // longest edge: 3D's rounding bound (13 m + m^2) eps D^3 / 6 with D^2 / 2, an over-bound in
    // the plane (fewer operations per term).
    {
        double area_sum = 0., empty = 0., Pt = 0., D = 0.;
        for (int i = 0; i < 3; ++i) {
            const double l = (T[size_t(i)] - T[size_t((i + 1) % 3)]).norm();
            Pt += l;
            D = std::max(D, l);
        }
        const auto rho = [&](const Poly& Q) {
            const double m = double(Q.size() - 2);
            return (13. * m + m * m) * EPS * D * D / 2.;
        };
        double rho_all = rho(w.Q0), rho_empty = 0.;
        for (const Piece& pc : w.pieces) {
            const double a = area_centroid(pc.Q).first;
            area_sum += a;
            rho_all += rho(pc.Q);
            if (pc.S.empty()) {
                empty += a;
                rho_empty += rho(pc.Q);
            }
        }
        if (stats) {
            ++stats->cells;
            stats->pieces += int64_t(w.pieces.size());
            stats->area += A0;
            stats->empty_area += empty;
        }
        const double tile_bound = std::sqrt(2.) * Pt * w.tol_cut + rho_all;
        const double empty_bound = Pt * (w.tol_any + std::sqrt(2.) * w.tol_cut) + rho_empty;
        if (!(empty <= empty_bound)) {
            log_and_throw_error(
                "ExactBandIntegral2D: pieces outside every local region, area {} of {} (bound "
                "{}) | {}",
                empty,
                A0,
                empty_bound,
                cell_string(T, delta));
        }
        if (!(std::abs(area_sum - A0) <= tile_bound)) {
            log_and_throw_error(
                "ExactBandIntegral2D: pieces do not tile t: {} vs {} (bound {}) | {}",
                area_sum,
                A0,
                tile_bound,
                cell_string(T, delta));
        }
    }

    // The elementary pieces (3D's value_grad_hess_impl() build): pieces on which the integrand is
    // ONE smooth function. An edge member is split by its line (affine per side); a rule-(a) piece
    // carries the member of smallest integral (lowest candidate on a tie), so its derivative is
    // that of the BOUND. With the offset side: only the offset-side parts, a rule-(a) part where
    // the members' lines disagree on the side of the member nearest at its centroid.
    const bool side = m_sigma != 0;
    const auto side_plane = [&](const int k) {
        SidePlane sp;
        if (!side) return sp;
        const size_t gi = size_t(w.cand[size_t(k)]);
        sp.has = m_side_has[gi];
        sp.N = m_side_n[gi];
        sp.ql = m_side_q[gi] - w.c0;
        return sp;
    };
    bool clipped = false;
    std::vector<Elem> pieces;
    Poly tmp, Rc;
    const auto add = [&](const Poly& Q, const Loc& g, const SidePlane& plane) {
        if (g.kind == 1) {
            for (const double sg : {1., -1.}) {
                const Cut c = clip(Q, -sg * g.dir, -sg * g.dir.dot(g.p), tmp, cx);
                if (c == Cut::None) continue;
                if (plane.has && double(m_sigma) * sg * g.dir.dot(plane.N) <= 0.) { // the far side
                    clipped = true;
                    continue;
                }
                Elem e;
                e.Q = c == Cut::Same ? Q : tmp;
                e.kind = 0;
                e.dir = sg * g.dir;
                e.c = sg * g.dir.dot(g.p);
                pieces.push_back(std::move(e));
            }
            return;
        }
        const Poly* Qp = &Q;
        if (plane.has) {
            const Cut ck = side_clip(Q, plane, m_sigma, Rc, cx);
            if (ck != Cut::Same) clipped = true;
            if (ck == Cut::None) return;
            if (ck == Cut::New) Qp = &Rc;
        }
        Elem e;
        e.Q = *Qp;
        e.kind = 2;
        e.p = g.p;
        pieces.push_back(std::move(e));
    };
    std::vector<const Loc*> members;
    std::vector<SidePlane> planes;
    std::vector<SidePart> parts;
    for (const Piece& pc : w.pieces) {
        if (pc.S.empty()) continue;
        const std::vector<int> S = pc.S.size() > 1 ? prune_members(pc.Q, pc.S, loc) : pc.S;
        bool all_edges = true, all_verts = true;
        members.clear();
        planes.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            planes.push_back(side_plane(k));
            all_edges = all_edges && loc[size_t(k)].kind == 1;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
        }
        if (S.size() == 1) {
            add(pc.Q, loc[size_t(S[0])], planes[0]);
        } else if (all_edges) {
            edges_min_pieces(
                pc.Q,
                members,
                cx,
                [&](const Poly& R, const Vec2& W, const double c, const int i, const double sg) {
                    const SidePlane& p = planes[size_t(i)];
                    if (p.has && double(m_sigma) * sg * members[size_t(i)]->dir.dot(p.N) <= 0.) {
                        clipped = true;
                        return;
                    }
                    Elem e;
                    e.Q = R;
                    e.kind = 0;
                    e.dir = W;
                    e.c = c;
                    pieces.push_back(std::move(e));
                });
        } else if (all_verts) {
            vertices_min_pieces(pc.Q, members, cx, [&](const Poly& R, const Vec2& v, const int i) {
                const Poly* Rp = &R;
                const SidePlane& p = planes[size_t(i)];
                if (p.has) {
                    const Cut ck = side_clip(R, p, m_sigma, Rc, cx);
                    if (ck != Cut::Same) clipped = true;
                    if (ck == Cut::None) return;
                    if (ck == Cut::New) Rp = &Rc;
                }
                Elem e;
                e.Q = *Rp;
                e.kind = 2;
                e.p = v;
                pieces.push_back(std::move(e));
            });
        } else {
            // rule (a): min over g of int d_g >= int min_g d_g, an upper bound
            if (stats) {
                ++stats->rule_a_pieces;
                stats->rule_a_area += area_centroid(pc.Q).first;
            }
            if (side) {
                side_split(pc.Q, planes, m_sigma, parts, cx);
            } else {
                parts.assign(1, SidePart());
                parts[0].R = pc.Q;
                parts[0].on.assign(S.size(), 1);
            }
            bool all_on = parts.size() == 1;
            for (const char o : parts[0].on) all_on = all_on && o;
            if (!all_on) clipped = true;
            for (const SidePart& sp : parts) {
                bool all = true, any = false;
                for (const char o : sp.on) {
                    all = all && o;
                    any = any || o;
                }
                if (!all && !any) continue;
                // the members' lines disagree: the side of the member nearest at the part's
                // centroid; on the offset side, the rule-(a) part over the members that put it
                // there
                if (!all && !sp.on[nearest_member(sp.R, members)]) continue;
                double best = std::numeric_limits<double>::infinity();
                int bk = -1;
                for (size_t q = 0; q < S.size(); ++q) { // ascending: the lowest on a tie
                    if (!sp.on[q]) continue;
                    const double v = int_g(loc[size_t(S[q])], sp.R, cx);
                    if (v < best) best = v, bk = S[q];
                }
                add(sp.R, loc[size_t(bk)], SidePlane());
            }
        }
    }
    if (clipped) { // phi = d - delta on every kept piece
        std::vector<Elem> sh;
        sh.reserve(2 * pieces.size());
        for (Elem& e : pieces) {
            if (e.kind == 0) {
                e.c += delta;
                sh.push_back(std::move(e));
            } else {
                Elem f;
                f.Q = e.Q;
                f.kind = 0;
                f.c = delta;
                sh.push_back(std::move(e));
                sh.push_back(std::move(f));
            }
        }
        pieces = std::move(sh);
    }
    if (stats && clipped) ++stats->side_clipped;

    double J = std::numeric_limits<double>::quiet_NaN();
    if (want_value) {
        double Jd = 0.;
        for (const Elem& e : pieces) Jd += piece_value(e);
        // nothing clipped: int_t d - delta area(t), the triangle's own area, as 3D keeps
        J = clipped ? Jd : Jd - delta * A0;
    }
    if (kv < 0) return J;

    // The derivatives in x = t[kv] (3D's accumulate()). lambda = l0 + g0.y: 1 at x, 0 on the
    // opposite edge (a, b).
    std::array<Vec2, 3> Tl;
    for (int i = 0; i < 3; ++i) Tl[size_t(i)] = T[size_t(i)] - w.c0;
    const Vec2 x = Tl[size_t(kv)];
    const Vec2& a = Tl[size_t((kv + 1) % 3)];
    const Vec2& b = Tl[size_t((kv + 2) % 3)];
    const Vec2 no = rot_cw(b - a);
    const double den = no.dot(x - a);
    const Vec2 g0 = no / den;
    const double l0 = -no.dot(a) / den;
    double scale = 0.;
    for (const Vec2& p : Tl) scale = std::max(scale, maxabs(p));
    const double tol = 64 * EPS * scale;
    // the two edges of t through x: outward unit normal nu, N = |N| nu with |N| the edge's length,
    // S = dN/dx, and the line nu.y = h
    struct TEdge
    {
        Vec2 nu;
        Eigen::Matrix2d S;
        double nN, h;
    };
    std::array<TEdge, 2> te;
    Eigen::Matrix2d Rcw;
    Rcw << 0., 1., -1., 0.; // Rcw v = rot_cw(v)
    Vec2 gradA = Vec2::Zero();
    for (int i = 0; i < 2; ++i) {
        const Vec2& o = i == 0 ? a : b; // the edge's other corner
        const Vec2& third = i == 0 ? b : a;
        double sg = 1.;
        Vec2 N = rot_cw(o - x);
        if (N.dot(third - x) > 0.) { // outward: away from the third corner
            N = -N;
            sg = -1.;
        }
        te[size_t(i)].nN = N.norm();
        te[size_t(i)].nu = N / te[size_t(i)].nN;
        te[size_t(i)].S = -sg * Rcw; // N = sg Rcw (o - x)
        te[size_t(i)].h = te[size_t(i)].nu.dot(x);
        gradA += te[size_t(i)].nu * (0.5 * te[size_t(i)].nN); // int_e lambda ds = |e| / 2
    }
    const auto corner = [&](const Vec2& y) {
        for (const Vec2& p : Tl) {
            if (maxabs(y - p) <= tol) return true;
        }
        return false;
    };
    Vec2& Ga = *G;
    Eigen::Matrix2d& Ha = *H;
    for (const Elem& e : pieces) {
        const Poly& Q = e.Q;
        const int n = Q.size();
        // Rounding sliver (width = area / longest edge at rounding level): skipped, as 3D's. Its
        // neighbours' cut terms pair up across it.
        double lmax = 0.;
        for (int f = 0; f < n; ++f) {
            lmax = std::max(lmax, (Q.V[size_t((f + 1) % n)] - Q.V[size_t(f)]).norm());
        }
        if (area_centroid(Q).first <= 1e3 * tol * lmax) continue;
        std::array<bool, 2> on_tedge = {false, false}; // Q has an edge on te[i]
        for (int f = 0; f < n; ++f) {
            const Vec2& y1 = Q.V[size_t(f)];
            const Vec2& y2 = Q.V[size_t((f + 1) % n)];
            const Vec2& nf = Q.N[size_t(f)];
            for (int i = 0; i < 2; ++i) {
                const TEdge& tf = te[size_t(i)];
                if (nf.dot(tf.nu) < 1 - 1e-12) continue;
                if (std::max(std::abs(y1.dot(tf.nu) - tf.h), std::abs(y2.dot(tf.nu) - tf.h)) >
                    1e3 * tol) {
                    continue; // the sliver test's scale
                }
                on_tedge[size_t(i)] = true;
                double P1;
                Vec2 P2;
                moments(y1, y2, e, l0, g0, P1, P2);
                Ga += tf.nu * P1;
                Ha += tf.nu * P2.transpose() + tf.S * (P1 / tf.nN);
                // The cut points: each end of the trace that is not a corner of t (a corner moves
                // with t, or is fixed), with the normal of Q's edge on the other side of it, the
                // fixed line that cut the trace there. A neighbour along t's edge (collinear
                // vertices) does not slide over it.
                const std::array<std::pair<const Vec2*, int>, 2> ends = {
                    {{&y1, (f + n - 1) % n}, {&y2, (f + 1) % n}}};
                for (const auto& [y, nb] : ends) {
                    if (corner(*y)) continue;
                    const Vec2& gc = Q.N[size_t(nb)];
                    const Vec2 gp = gc - gc.dot(tf.nu) * tf.nu;
                    if (gp.norm() <= 1e-12) continue;
                    Ha -= tf.nu * gc.transpose() * (cut_value(*y, e, l0, g0) / gp.norm());
                }
                break; // an edge lies on at most one edge of t
            }
        }
        // Q touching an edge of t through x at a vertex only (Q has no edge on it): 3D's contact
        // term one dimension down. Moving x into t moves the edge into Q, whose trace on it is then
        // a segment between Q's two edges through that vertex, of length O(|dx|) and zero at x;
        // both of its ends slide, and each of Q's edges through the vertex adds its cut term.
        // MEASURED (test exact-band-integral-2d-hessian-contact, t's edge through the meeting
        // point of three fixed lines in the L's notch): without this term the Hessian at both
        // corners on that edge came 0.82 (units S / L^2) off its central differences, with it
        // 3.8e-11; the random triangles of the other tests never reach it.
        for (int i = 0; i < 2; ++i) {
            if (on_tedge[size_t(i)]) continue;
            const TEdge& tf = te[size_t(i)];
            for (int f = 0; f < n; ++f) {
                const Vec2& g = Q.N[size_t(f)];
                const Vec2 gp = g - g.dot(tf.nu) * tf.nu;
                if (gp.norm() <= 1e-12) continue; // a line parallel to the edge does not slide
                for (const Vec2* y : {&Q.V[size_t(f)], &Q.V[size_t((f + 1) % n)]}) {
                    if (std::abs(y->dot(tf.nu) - tf.h) > 1e3 * tol) continue;
                    if (corner(*y)) continue;
                    Ha -= tf.nu * g.transpose() * (cut_value(*y, e, l0, g0) / gp.norm());
                }
            }
        }
    }
    if (!clipped) Ga -= delta * gradA;
    return J;
}

double ExactBandIntegral2D::value(
    const std::array<Eigen::Vector2d, 3>& t,
    const double delta,
    Stats* stats) const
{
    return evaluate(t, delta, -1, nullptr, nullptr, true, stats);
}

double ExactBandIntegral2D::value_grad_hess(
    const std::array<Eigen::Vector2d, 3>& t,
    const int kv,
    const double delta,
    Eigen::Vector2d& grad,
    Eigen::Matrix2d& hess,
    Stats* stats) const
{
    return evaluate(t, delta, kv, &grad, &hess, true, stats);
}

void ExactBandIntegral2D::grad_hess(
    const std::array<Eigen::Vector2d, 3>& t,
    const int kv,
    const double delta,
    Eigen::Vector2d& grad,
    Eigen::Matrix2d& hess) const
{
    evaluate(t, delta, kv, &grad, &hess, false, nullptr);
}

struct ExactBandIntegral2D::EdgeWork
{
    Vec2 c0; ///< the local origin: f's midpoint
    std::array<Vec2, 2> F; ///< f in local coordinates
    std::vector<Loc> loc; ///< the candidate features, local
    std::vector<int> cand; ///< their feature indices
    std::vector<std::pair<Vec2, double>> hs; ///< their half-planes, local
    double U = 0.;
    double tol_cut = 0., tol_any = 0.; ///< the partition's tolerance budget (Ctx)
    double scale = 0.; ///< the rounding scale (Ctx::scale)
    Vec2 n = Vec2::Zero(); ///< f's unit normal rot_cw(F[1] - F[0]) / |.|; zero for a point
    std::vector<SpanPart> parts; ///< the labelled parts on the offset side
    int rule_a = 0; ///< parts labelled by rule (a)
};

void ExactBandIntegral2D::label_edge(
    const std::array<Eigen::Vector2d, 2>& f,
    const int kv,
    EdgeWork& w) const
{
    w.parts.clear();
    w.rule_a = 0;
    w.n.setZero();
    // step 0: candidates in local coordinates, origin c0 = f's midpoint (partition()'s; 3D's
    // tri_partition() takes the face's centroid)
    w.c0 = 0.5 * (f[0] + f[1]);
    const Vec2& c0 = w.c0;
    w.F = {{Vec2(f[0] - c0), Vec2(f[1] - c0)}};
    const double L = (f[1] - f[0]).norm();
    if (!(L > 0.)) return; // a point: no length
    const Vec2 flo = f[0].cwiseMin(f[1]), fhi = f[0].cwiseMax(f[1]);
    w.U = upper_bound(f.data(), 2);
    const double r = w.U * (1 + 1e-12);
    w.cand.clear();
    w.loc.clear();
    w.hs.clear();
    w.scale = 0.;
    for (size_t gi = 0; gi < m_feat.size(); ++gi) {
        const Feature& ft = m_feat[gi];
        if (!(box_dist(ft.lo, ft.hi, flo, fhi) <= r)) continue;
        w.cand.push_back(int(gi));
        Loc l;
        l.kind = ft.kind;
        l.p = ft.p - c0;
        l.dir = ft.dir;
        l.h_begin = int(w.hs.size());
        for (int i = ft.h_begin; i < ft.h_end; ++i) {
            const auto& [g, q] = m_hs[size_t(i)];
            w.hs.push_back({g, g.dot(q - c0)});
            w.scale = std::max(w.scale, maxabs(q - c0));
        }
        l.h_end = int(w.hs.size());
        w.loc.push_back(l);
    }
    for (const Vec2& p : f) w.scale = std::max(w.scale, maxabs(p - c0));
    const Vec2 t = (w.F[1] - w.F[0]) / L;
    w.n = rot_cw(t);
    // The cap of one decision's share of the budget (seg_tol()): L / 2, so that a capped decision
    // is charged f's length in the checks below (3D: A / (2 D), the face's area there).
    Ctx cx;
    w.tol_cut = w.tol_any = 0.;
    cx.tol_cut = &w.tol_cut;
    cx.tol_any = &w.tol_any;
    cx.scale = w.scale;
    cx.seg_t = &t;
    cx.seg_cap = 0.5 * L;

    // step 1: the spans of f with constant member sets (tri_partition()'s)
    std::vector<std::pair<Span, std::vector<int>>> pieces(1), next;
    pieces[0].first.y = w.F;
    std::vector<Span> outs;
    for (int k = 0; k < int(w.loc.size()); ++k) {
        const std::pair<Vec2, double>* H = w.hs.data() + w.loc[size_t(k)].h_begin;
        const int nh = w.loc[size_t(k)].h_end - w.loc[size_t(k)].h_begin;
        next.clear();
        for (auto& pc : pieces) {
            const Side st = sclassify(pc.first, H, nh, cx);
            if (st == Side::In) {
                pc.second.push_back(k);
                next.push_back(std::move(pc));
            } else if (st == Side::Out) {
                next.push_back(std::move(pc));
            } else {
                Span in;
                outs.clear();
                if (ssplit(pc.first, H, nh, in, outs, cx)) {
                    std::vector<int> S = pc.second;
                    S.push_back(k);
                    next.push_back({in, std::move(S)});
                }
                for (const Span& so : outs) next.push_back({so, pc.second});
            }
        }
        std::swap(pieces, next);
    }

    // 3D's check_face_pieces() on lengths. A cutting clip moves a part's end only within the strip
    // |g.x - h| <= sqrt(2) tol, a decision taken with a tolerance misplaces at most the strip of
    // half-width tol, and a strip of half-width s meets f's line in 2 s / |g.t| of it: tol_cut and
    // tol_any sum tol / |g.t| (seg_tol(), capped). A length rounds by at most 4 eps L.
    {
        double len_sum = 0., empty = 0., rho_all = 4. * EPS * L, rho_empty = 0.;
        for (const auto& [S, mem] : pieces) {
            const double len = (S.y[1] - S.y[0]).norm();
            len_sum += len;
            rho_all += 4. * EPS * L;
            if (mem.empty()) {
                empty += len;
                rho_empty += 4. * EPS * L;
            }
        }
        const double tile_bound = 2. * std::sqrt(2.) * w.tol_cut + rho_all;
        const double empty_bound = 2. * (w.tol_any + std::sqrt(2.) * w.tol_cut) + rho_empty;
        if (!(empty <= empty_bound)) {
            log_and_throw_error(
                "ExactBandIntegral2D::front_edge_derivs: parts outside every local region, "
                "length {} of {} (bound {}) | {}",
                empty,
                L,
                empty_bound,
                edge_string(f));
        }
        if (!(std::abs(len_sum - L) <= tile_bound)) {
            log_and_throw_error(
                "ExactBandIntegral2D::front_edge_derivs: parts do not tile f: {} vs {} (bound {}) "
                "| {}",
                len_sum,
                L,
                tile_bound,
                edge_string(f));
        }
    }

    // The labelled parts (label_face(); value()'s classification of a piece's members, on the
    // span). The sub-spans' clips take no tolerance budget, as 3D's.
    Ctx lc;
    lc.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    const Vec2& xk = w.F[size_t(kv)];
    const Vec2& bk = w.F[size_t(1 - kv)];
    const Vec2 g0 = (xk - bk) / (xk - bk).squaredNorm(); // lam = l0 + g0.y: 1 at x, 0 at the end
    const double l0 = -g0.dot(bk);
    const bool side = m_sigma != 0;
    const auto side_plane = [&](const int k) {
        SidePlane sp;
        if (!side) return sp;
        const size_t gi = size_t(w.cand[size_t(k)]);
        sp.has = m_side_has[gi];
        sp.N = m_side_n[gi];
        sp.ql = m_side_q[gi] - c0;
        return sp;
    };
    std::vector<const Loc*> members;
    std::vector<SidePlane> planes;
    std::vector<SpanSidePart> sparts;
    for (const auto& [R, S0] : pieces) {
        if (S0.empty()) continue;
        const std::vector<int> S = S0.size() > 1 ? prune_points({R.y[0], R.y[1]}, S0, loc) : S0;
        bool all_edges = true, all_verts = true;
        members.clear();
        planes.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            planes.push_back(side_plane(k));
            all_edges = all_edges && loc[size_t(k)].kind == 1;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
        }
        if (S.size() == 1) {
            labeled_span(R, loc[size_t(S[0])], planes[0], m_sigma, lc, w.parts);
        } else if (all_edges) {
            edges_min_spans(
                R,
                members,
                lc,
                [&](const Span& Rs, const Vec2& W, const double c, const int i, const double sg) {
                    const SidePlane& p = planes[size_t(i)];
                    const bool sided = side && p.has;
                    if (sided && double(m_sigma) * sg * members[size_t(i)]->dir.dot(p.N) <= 0.) {
                        return; // the far side
                    }
                    SpanPart sp;
                    sp.S = Rs;
                    sp.e.kind = 0;
                    sp.e.dir = W;
                    sp.e.c = c;
                    sp.far = sided;
                    w.parts.push_back(std::move(sp));
                });
        } else if (all_verts) {
            vertices_min_spans(R, members, lc, [&](const Span& Rs, const Vec2&, const int i) {
                labeled_span(Rs, *members[size_t(i)], planes[size_t(i)], m_sigma, lc, w.parts);
            });
        } else {
            // rule (a), as value() takes it on the thin swept triangle over R: split by the
            // members' side lines, each part the member of smallest integral among those putting
            // it on the offset side (the first on a tie), a part where they disagree the side of
            // the member nearest at its centroid
            if (side) {
                span_side_split(R, planes, m_sigma, sparts, lc);
            } else {
                sparts.assign(1, SpanSidePart());
                sparts[0].R = R;
                sparts[0].on.assign(S.size(), 1);
            }
            for (const SpanSidePart& sp : sparts) {
                bool all = true, any = false;
                for (const char o : sp.on) {
                    all = all && o;
                    any = any || o;
                }
                if (!all && !any) continue;
                if (!all && !sp.on[nearest_member_span(sp.R, members, l0, g0)]) continue;
                double best = std::numeric_limits<double>::infinity();
                int bkk = -1;
                for (size_t q = 0; q < S.size(); ++q) { // ascending: the lowest on a tie
                    if (!sp.on[q]) continue;
                    const double v = span_int_d_lam(sp.R, loc[size_t(S[q])], l0, g0, lc);
                    if (v < best) best = v, bkk = S[q];
                }
                if (bkk < 0) {
                    log_and_throw_error(
                        "ExactBandIntegral2D::front_edge_derivs: rule (a) on a curved piece: no "
                        "member's integral is a number | {}",
                        edge_string(f));
                }
                labeled_span(sp.R, loc[size_t(bkk)], SidePlane(), m_sigma, lc, w.parts);
                ++w.rule_a;
            }
        }
    }
}

void ExactBandIntegral2D::front_edge_derivs(
    const std::array<Eigen::Vector2d, 2>& f,
    const double delta,
    const int kv,
    Eigen::Vector2d& grad,
    Eigen::Matrix2d& hess,
    int* rule_a,
    Eigen::Matrix2d* hess_swept) const
{
    // See the declaration; 3D's front_face_derivs() one dimension down.
    grad.setZero();
    hess.setZero();
    if (hess_swept) hess_swept->setZero();
    EdgeWork w;
    label_edge(f, kv, w);
    if (rule_a) *rule_a = w.rule_a;
    const Vec2& n = w.n;
    if (n.isZero(0.)) return; // a point: no length, every term 0
    // lam = l0 + g0.y, the hat function of x = F[kv] along f: 1 there, 0 at the other end b.
    // N = rot_cw(F1 - F0), |N| = |b - x|, dN/dx = -Rcw for kv = 0, +Rcw for kv = 1 (Rcw v =
    // rot_cw(v)).
    const Vec2& x = w.F[size_t(kv)];
    const Vec2& b = w.F[size_t(1 - kv)];
    const Vec2 g0 = (x - b) / (x - b).squaredNorm();
    const double l0 = -g0.dot(b);
    const double nN = (b - x).norm();
    Eigen::Matrix2d Rcw;
    Rcw << 0., 1., -1., 0.;
    const Eigen::Matrix2d dN = kv == 0 ? Eigen::Matrix2d(-Rcw) : Rcw;
    const auto lam = [&](const Vec2& y) { return l0 + g0.dot(y); };
    // a part on the input (d = 0 at both ends): grad d from the band side (declaration)
    const double tolS = 64 * EPS * (std::max(maxabs(w.F[0]), maxabs(w.F[1])) + maxabs(w.c0));
    double gi = 0.;
    Vec2 P2 = Vec2::Zero();
    Eigen::Matrix2d Hcut = Eigen::Matrix2d::Zero();
    Vec2 Qcut =
        Vec2::Zero(); // hess_swept's: - sum g_c lam^2 (d - delta) / |g_c,f| (Hcut = n (x) Qcut)
    for (const SpanPart& pt : w.parts) {
        const Vec2& y1 = pt.S.y[0];
        const Vec2& y2 = pt.S.y[1];
        const double len = (y2 - y1).norm();
        if (!(len > 0.)) continue;
        const Elem& e = pt.e;
        const double int_lam = 0.5 * len * (lam(y1) + lam(y2));
        double p1 = 0.;
        Vec2 p2 = Vec2::Zero();
        moments(y1, y2, e, l0, g0, p1, p2);
        gi += p1 - delta * int_lam;
        if (e.kind == 0) {
            // Without a side both affine sides are parts, and a part lying on the line takes the
            // side labeled_span() met first; grad d is then the band side's, -n (n out of the
            // band). With a side the part IS the offset side, the side the band is on: its own W.
            const double on =
                std::max(std::abs(e.dir.dot(y1) - e.c), std::abs(e.dir.dot(y2) - e.c));
            if (!pt.far && on <= tolS) p2 = -n * (p2.dot(e.dir));
        }
        P2 += p2;
        // The cut terms (value_grad_hess()'s cut points, front_face_derivs()' cut edges): an end
        // of a part that is not an end of f lies on a line that cut f there, fixed in space, so it
        // slides along f as x moves, and the part's chi (d - delta) there adds
        //     - n (x) g_c / |g_c,f| lam^2 (d - delta),
        // g_c the line's normal out of the part. Where the part across the end has the same
        // integrand there (d continuous, both on the offset side) the two terms cancel; where it
        // jumps they leave the jump's: the offset side's clip and the ends of a part labelled by
        // rule (a). A line parallel to f does not slide along it; a rounding sliver of a part is
        // skipped (its neighbours' terms pair up across it), as 3D's.
        double ma = w.scale;
        ma = std::max(ma, std::max(maxabs(y1), maxabs(y2)));
        const double tolc = 64 * EPS * ma;
        if (len <= 1e3 * tolc) continue;
        for (int k = 0; k < 2; ++k) {
            const Vec2& gc = pt.S.g[size_t(k)];
            if (gc.isZero(0.)) continue; // an end of f
            const Vec2 gp = gc - gc.dot(n) * n;
            if (gp.norm() <= 1e-12) continue;
            const Vec2& y = pt.S.y[size_t(k)];
            const double l = lam(y);
            const double P3 = cut_value(y, e, l0, g0) - delta * l * l;
            Hcut -= n * gc.transpose() * (P3 / gp.norm());
            Qcut -= gc * (P3 / gp.norm());
        }
    }
    grad = gi * n;
    hess = n * P2.transpose() + (gi / nN) * dN + Hcut;
    if (hess_swept) {
        const Vec2 Q = P2 + Qcut;
        *hess_swept = 0.5 * (n * Q.transpose() + Q * n.transpose());
    }
}

} // namespace wmtk::components::topological_offset
