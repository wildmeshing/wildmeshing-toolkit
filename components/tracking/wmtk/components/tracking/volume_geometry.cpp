#include "volume_geometry.hpp"

#include "slim2d.hpp"

#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/predicates.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numeric>

namespace wmtk::components::tracking::volume {

int orient(
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
    // Shewchuk's sign is that of det(a - d, b - d, c - d), the opposite of det(b - a, c - a, d -
    // a).
    return -int(utils::predicates::orient3d(a, b, c, d));
}

bool inside(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
    const int s = orient(a, b, c, d);
    if (s == 0) return false;
    return orient(p, b, c, d) * s >= 0 && orient(a, p, c, d) * s >= 0 &&
           orient(a, b, p, d) * s >= 0 && orient(a, b, c, p) * s >= 0;
}

Eigen::Vector4d barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
    // A backward stable solve: in a sliver the coordinates themselves are ill-conditioned, but
    // they put the point back where it was to rounding, which is what the maps need.
    Eigen::Matrix3d M;
    M << b - a, c - a, d - a;
    const Eigen::Vector3d l = M.fullPivLu().solve(p - a);
    if (!l.allFinite()) return Eigen::Vector4d::Constant(-std::numeric_limits<double>::infinity());
    return Eigen::Vector4d(1 - l.sum(), l[0], l[1], l[2]);
}

Outside outside_of(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
    Outside o;
    const int s = orient(a, b, c, d);
    if (s == 0) return {15, std::numeric_limits<double>::infinity()};
    const std::array<Eigen::Vector3d, 4> v = {{a, b, c, d}};
    for (int k = 0; k < 4; ++k) {
        std::array<Eigen::Vector3d, 4> w = v;
        w[k] = p;
        if (orient(w[0], w[1], w[2], w[3]) * s >= 0) continue;
        o.faces |= 1 << k;
        const Eigen::Vector3d &x = v[(k + 1) % 4], &y = v[(k + 2) % 4], &z = v[(k + 3) % 4];
        const Eigen::Vector3d n = (y - x).cross(z - x);
        const double l = n.norm();
        o.distance = std::max(
            o.distance,
            l > 0 ? std::abs(n.dot(p - x)) / l : std::numeric_limits<double>::infinity());
    }
    return o;
}

Eigen::Vector4d exact_barycentric(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    const Eigen::Vector3d& d)
{
    // a rational of a NaN or an infinity is a floating point exception in GMP
    if (!(p.allFinite() && a.allFinite() && b.allFinite() && c.allFinite() && d.allFinite())) {
        return Eigen::Vector4d::Constant(-std::numeric_limits<double>::infinity());
    }
    using Q = wmtk::Rational;
    using V = std::array<Q, 3>;
    const auto q = [](const Eigen::Vector3d& x) { return V{{Q(x[0]), Q(x[1]), Q(x[2])}}; };
    const auto vol = [](const V& w, const V& x, const V& y, const V& z) {
        const Q x0 = x[0] - w[0], x1 = x[1] - w[1], x2 = x[2] - w[2];
        const Q y0 = y[0] - w[0], y1 = y[1] - w[1], y2 = y[2] - w[2];
        const Q z0 = z[0] - w[0], z1 = z[1] - w[1], z2 = z[2] - w[2];
        return x0 * (y1 * z2 - y2 * z1) - x1 * (y0 * z2 - y2 * z0) + x2 * (y0 * z1 - y1 * z0);
    };
    const V P = q(p), A = q(a), B = q(b), C = q(c), D = q(d);
    const Q v = vol(A, B, C, D);
    if (v == Q(0.)) return Eigen::Vector4d::Constant(-std::numeric_limits<double>::infinity());
    return Eigen::Vector4d(
        (vol(P, B, C, D) / v).to_double(),
        (vol(A, P, C, D) / v).to_double(),
        (vol(A, B, P, D) / v).to_double(),
        (vol(A, B, C, P) / v).to_double());
}

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

std::set<Vid> vertices_of(const Tets& T)
{
    std::set<Vid> vs;
    for (const Tet& t : T) vs.insert(t.begin(), t.end());
    return vs;
}

/// The vertex every tet of T shares, if exactly one.
std::optional<Vid> common_vertex(const Tets& T)
{
    if (T.empty()) return std::nullopt;
    std::set<Vid> common(T[0].begin(), T[0].end());
    for (const Tet& t : T) {
        std::set<Vid> next;
        for (const Vid v : t) {
            if (common.count(v)) next.insert(v);
        }
        common = std::move(next);
    }
    if (common.empty()) return std::nullopt;
    return *common.begin();
}

} // namespace

std::optional<StarPatch> StarPatch::make(const Tets& T, const Positions& P)
{
    StarPatch S;
    S.m_T = T;
    S.m_P = P;
    std::map<Tri, std::vector<Vid>> opp;
    for (const Tet& t : T) {
        for (int k = 0; k < 4; ++k) {
            Tri f = {{t[(k + 1) % 4], t[(k + 2) % 4], t[(k + 3) % 4]}};
            opp[sorted(f)].push_back(t[k]);
        }
    }
    int sign = 0;
    for (size_t k = 0; k < T.size(); ++k) {
        const Tet& t = T[k];
        const int o = orient(P[t[0]], P[t[1]], P[t[2]], P[t[3]]);
        if (o == 0 || (sign != 0 && o != sign)) return std::nullopt;
        sign = o;
        S.m_index.emplace_back(sorted(t), uint32_t(k));
    }
    std::sort(S.m_index.begin(), S.m_index.end());
    for (const auto& [f, o] : opp) {
        if (o.size() > 2) return std::nullopt;
        if (o.size() != 1) continue;
        S.m_F.push_back(f);
        Tri g = f;
        if (orient(P[f[0]], P[f[1]], P[f[2]], P[o[0]]) > 0) std::swap(g[1], g[2]);
        S.m_Fo.push_back(g);
        S.corners.push_back(P[g[0]]);
        S.normals.push_back((P[g[1]] - P[g[0]]).cross(P[g[2]] - P[g[0]]));
    }
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(kInf), hi = -lo;
    for (const Vid v : vertices_of(T)) {
        lo = lo.cwiseMin(P[v]);
        hi = hi.cwiseMax(P[v]);
    }
    S.m_scale = (hi - lo).norm();
    return S;
}

size_t StarPatch::bytes() const
{
    return sizeof(*this) + m_T.capacity() * sizeof(Tet) +
           (m_P.capacity() + normals.capacity() + corners.capacity()) * sizeof(Eigen::Vector3d) +
           (m_F.capacity() + m_Fo.capacity()) * sizeof(Tri) +
           m_index.capacity() * sizeof(m_index[0]);
}

std::optional<Vid> StarPatch::boundary_apex() const
{
    const auto c = common_vertex(m_T);
    if (!c) return std::nullopt;
    for (const Tri& f : m_F) {
        if (contains(f, *c)) return c;
    }
    return std::nullopt;
}

std::optional<Vid> StarPatch::interior_apex() const
{
    const auto c = common_vertex(m_T);
    if (!c) return std::nullopt;
    for (const Tri& f : m_F) {
        if (contains(f, *c)) return std::nullopt;
    }
    return c;
}

bool StarPatch::sees_all(const Eigen::Vector3d& s) const
{
    for (const Tri& g : m_Fo) {
        if (orient(m_P[g[0]], m_P[g[1]], m_P[g[2]], s) >= 0) return false;
    }
    return true;
}

bool StarPatch::set_centre(const Eigen::Vector3d& s)
{
    if (!sees_all(s)) return false;
    m_s = s;
    return true;
}

double StarPatch::margin() const
{
    double m = kInf;
    for (size_t k = 0; k < normals.size(); ++k) {
        m = std::min(m, normals[k].dot(corners[k] - m_s) / normals[k].norm());
    }
    return m / m_scale;
}

StarPatch::Exit StarPatch::exit(const Eigen::Vector3d& x) const
{
    const Eigen::Vector3d d = x - m_s;
    Exit best{0, Eigen::Vector3d::Constant(1. / 3), 0};
    double best_min = -kInf;
    for (size_t k = 0; k < m_Fo.size(); ++k) {
        const Eigen::Vector3d& n = normals[k];
        const double den = n.dot(d);
        if (!(den > 0)) continue;
        const double lam = n.dot(corners[k] - m_s) / den;
        const Eigen::Vector3d q = m_s + lam * d;
        const Tri& g = m_Fo[k];
        const Eigen::Vector3d &a = m_P[g[0]], &b = m_P[g[1]], &c = m_P[g[2]];
        const double nn = n.squaredNorm();
        const double w0 = (b - q).cross(c - q).dot(n) / nn;
        const double w1 = (c - q).cross(a - q).dot(n) / nn;
        const Eigen::Vector3d w(w0, w1, 1 - w0 - w1);
        if (w.minCoeff() <= best_min) continue;
        best_min = w.minCoeff();
        // in the face's sorted corners
        const Tri& f = m_F[k];
        Eigen::Vector3d ws;
        for (int i = 0; i < 3; ++i) ws[int(std::find(f.begin(), f.end(), g[i]) - f.begin())] = w[i];
        ws = ws.cwiseMax(0.);
        ws /= ws.sum();
        best = {k, ws, lam > 0 ? 1 / lam : 0};
    }
    return best;
}

StarPatch::Located StarPatch::locate(const Eigen::Vector3d& y) const
{
    if (const auto l = locate_in(m_T, m_P, y, [](size_t) { return true; })) return {l->tet, l->b};
    // further out than a rounding: the tet whose smallest coordinate is largest, exactly
    Located best{0, Eigen::Vector4d::Constant(0.25)};
    double best_min = -kInf;
    for (size_t k = 0; k < m_T.size(); ++k) {
        const auto& t = m_T[k];
        const Eigen::Vector4d b = barycentric(y, m_P[t[0]], m_P[t[1]], m_P[t[2]], m_P[t[3]]);
        if (b.minCoeff() > best_min) {
            best_min = b.minCoeff();
            best = {k, b};
        }
    }
    const auto& t = m_T[best.tet];
    best.b = exact_barycentric(y, m_P[t[0]], m_P[t[1]], m_P[t[2]], m_P[t[3]]).cwiseMax(0.);
    best.b /= best.b.sum();
    return best;
}

std::optional<size_t> StarPatch::index_of(const Tet& t) const
{
    const Tet k = sorted(t);
    const auto it = std::lower_bound(
        m_index.begin(),
        m_index.end(),
        k,
        [](const std::pair<Tet, uint32_t>& a, const Tet& b) { return a.first < b; });
    if (it == m_index.end() || it->first != k) return std::nullopt;
    return size_t(it->second);
}

namespace {

/// max c.z subject to A z <= b, z >= 0, for b >= 0 (the origin is feasible): a dense tableau,
/// Bland's rule. Nothing if unbounded or out of iterations.
std::optional<Eigen::VectorXd>
simplex_max(const Eigen::VectorXd& c, const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    const int m = int(A.rows()), n = int(A.cols());
    Eigen::MatrixXd tab = Eigen::MatrixXd::Zero(m + 1, n + m + 1);
    tab.topLeftCorner(m, n) = A;
    tab.block(0, n, m, m) = Eigen::MatrixXd::Identity(m, m);
    tab.col(n + m).head(m) = b;
    tab.row(m).head(n) = -c.transpose();
    std::vector<int> basis(m);
    std::iota(basis.begin(), basis.end(), n);
    const double eps = 1e-12;
    for (int it = 0; it < 50 * (n + m); ++it) {
        int enter = -1;
        for (int j = 0; j < n + m; ++j) {
            if (tab(m, j) < -eps) {
                enter = j;
                break;
            }
        }
        if (enter < 0) {
            Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
            for (int i = 0; i < m; ++i) {
                if (basis[i] < n) z[basis[i]] = tab(i, n + m);
            }
            return z;
        }
        int leave = -1;
        double best = kInf;
        for (int i = 0; i < m; ++i) {
            if (tab(i, enter) > eps) {
                const double r = tab(i, n + m) / tab(i, enter);
                if (r < best - eps || (r < best + eps && leave >= 0 && basis[i] < basis[leave])) {
                    best = r;
                    leave = i;
                }
            }
        }
        if (leave < 0) return std::nullopt;
        tab.row(leave) /= tab(leave, enter);
        for (int i = 0; i <= m; ++i) {
            if (i != leave && tab(i, enter) != 0) tab.row(i) -= tab(i, enter) * tab.row(leave);
        }
        basis[leave] = enter;
    }
    return std::nullopt;
}

} // namespace

std::optional<std::pair<Eigen::Vector3d, double>> chebyshev(
    const std::vector<Eigen::Vector3d>& N,
    const std::vector<double>& b,
    const Eigen::Vector3d& x0,
    const double L,
    const double t_max)
{
    const int m = int(N.size());
    double t0 = t_max;
    for (int i = 0; i < m; ++i) t0 = std::min(t0, b[i] - N[i].dot(x0));
    if (!std::isfinite(t0)) return std::nullopt;
    // z = (d+, d-, tau): x = x0 + d+ - d-, t = t0 + tau
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(m + 7, 7);
    Eigen::VectorXd rhs(m + 7);
    for (int i = 0; i < m; ++i) {
        A.block<1, 3>(i, 0) = N[i].transpose();
        A.block<1, 3>(i, 3) = -N[i].transpose();
        A(i, 6) = 1;
        rhs[i] = std::max(0., b[i] - N[i].dot(x0) - t0);
    }
    for (int k = 0; k < 6; ++k) {
        A(m + k, k) = 1;
        rhs[m + k] = L;
    }
    A(m + 6, 6) = 1;
    rhs[m + 6] = std::max(0., t_max - t0);
    Eigen::VectorXd c = Eigen::VectorXd::Zero(7);
    c[6] = 1;
    const auto z = simplex_max(c, A, rhs);
    if (!z) return std::nullopt;
    return std::make_pair(Eigen::Vector3d(x0 + z->head<3>() - z->segment<3>(3)), t0 + (*z)[6]);
}

std::optional<Eigen::Vector3d> kernel(const std::vector<const StarPatch*>& patches)
{
    std::vector<Eigen::Vector3d> N;
    std::vector<double> b;
    double scale = 0;
    Eigen::Vector3d x0 = Eigen::Vector3d::Zero();
    size_t nv = 0;
    for (const StarPatch* S : patches) {
        scale = std::max(scale, S->scale());
        for (size_t k = 0; k < S->normals.size(); ++k) {
            const double l = S->normals[k].norm();
            if (!(l > 1e-300) || !S->corners[k].allFinite()) return std::nullopt;
            N.push_back(S->normals[k] / l);
            b.push_back(N.back().dot(S->corners[k]));
        }
    }
    for (const Vid v : vertices_of(patches[0]->tets())) {
        x0 += patches[0]->positions()[v];
        ++nv;
    }
    x0 /= double(nv);
    const auto r = chebyshev(N, b, x0, scale, scale);
    if (!r || !(r->second > 0)) return std::nullopt;
    for (const StarPatch* S : patches) {
        if (!S->sees_all(r->first)) return std::nullopt;
    }
    return r->first;
}

std::optional<Eigen::Vector3d> apex_centre(const StarPatch& S)
{
    const auto c = S.boundary_apex();
    if (!c) return std::nullopt;
    std::vector<Eigen::Vector3d> N;
    for (size_t k = 0; k < S.faces().size(); ++k) {
        if (!contains(S.faces()[k], *c)) continue;
        const double l = S.normals[k].norm();
        if (!(l > 0)) return std::nullopt;
        N.push_back(S.normals[k] / l);
    }
    const auto r = chebyshev(N, std::vector<double>(N.size(), 0.), Eigen::Vector3d::Zero(), 1, 1);
    if (!r || !(r->second > 0)) return std::nullopt;
    const Eigen::Vector3d pc = S.positions()[*c];
    for (double eps = S.scale(); eps > std::ldexp(S.scale(), -60); eps /= 2) {
        const Eigen::Vector3d s = pc + eps * r->first;
        if (S.sees_all(s)) return s;
    }
    return std::nullopt;
}

std::optional<Eigen::Vector3d> centre_of(const StarPatch& S, bool* thin)
{
    if (thin) *thin = false;
    if (auto s = kernel({&S})) return s;
    if (thin) *thin = true;
    if (auto s = apex_centre(S)) return s;
    if (const auto a = S.interior_apex()) {
        if (S.sees_all(S.positions()[*a])) return S.positions()[*a];
    }
    return std::nullopt;
}

namespace {

/// Tutte's embedding: `fixed` vertices where given, every other at the mean of its neighbours.
std::map<Vid, Eigen::Vector2d> tutte(
    const std::set<Vid>& verts,
    const std::set<std::array<Vid, 2>>& edges,
    const std::map<Vid, Eigen::Vector2d>& fixed)
{
    std::map<Vid, std::set<Vid>> nb;
    for (const auto& e : edges) {
        nb[e[0]].insert(e[1]);
        nb[e[1]].insert(e[0]);
    }
    std::vector<Vid> free;
    std::map<Vid, int> fx;
    for (const Vid v : verts) {
        if (!fixed.count(v)) {
            fx[v] = int(free.size());
            free.push_back(v);
        }
    }
    std::map<Vid, Eigen::Vector2d> out = fixed;
    if (free.empty()) return out;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(int(free.size()), int(free.size()));
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(int(free.size()), 2);
    for (const Vid v : free) {
        const int r = fx[v];
        A(r, r) = double(nb[v].size());
        for (const Vid u : nb[v]) {
            if (fx.count(u)) {
                A(r, fx[u]) -= 1;
            } else {
                B.row(r) += fixed.at(u).transpose();
            }
        }
    }
    const Eigen::MatrixXd X = A.partialPivLu().solve(B);
    for (const Vid v : free) out[v] = X.row(fx[v]).transpose();
    return out;
}

/// Heights making the triangles `F` of a planar layout one convex surface (Maxwell-Cremona,
/// from a unit stress on every edge): each face's plane differs from its neighbour's by the
/// edge's perpendicular. Nothing if the heights disagree.
std::optional<std::map<Vid, double>> lift(
    const std::vector<Tri>& F,
    const std::map<Vid, Eigen::Vector2d>& P2)
{
    if (F.empty()) return std::nullopt;
    std::map<std::array<Vid, 2>, std::vector<size_t>> ef;
    for (size_t k = 0; k < F.size(); ++k) {
        for (int i = 0; i < 3; ++i) {
            Vid a = F[k][i], b = F[k][(i + 1) % 3];
            if (a > b) std::swap(a, b);
            ef[{{a, b}}].push_back(k);
        }
    }
    std::vector<std::optional<std::pair<Eigen::Vector2d, double>>> plane(F.size());
    plane[0] = std::make_pair(Eigen::Vector2d::Zero(), 0.);
    std::vector<size_t> todo = {0};
    while (!todo.empty()) {
        const size_t k = todo.back();
        todo.pop_back();
        const auto [a, d] = *plane[k];
        for (int i = 0; i < 3; ++i) {
            const Vid u = F[k][i], v = F[k][(i + 1) % 3];
            for (const size_t g : ef[{{std::min(u, v), std::max(u, v)}}]) {
                if (plane[g]) continue;
                Vid t = 0;
                for (const Vid x : F[g]) {
                    if (x != u && x != v) t = x;
                }
                const Eigen::Vector2d e = P2.at(v) - P2.at(u);
                Eigen::Vector2d perp(-e.y(), e.x());
                if (perp.dot(P2.at(t) - P2.at(u)) < 0) perp = -perp;
                const Eigen::Vector2d bb = a + perp;
                plane[g] = std::make_pair(bb, d + (a - bb).dot(P2.at(u)));
                todo.push_back(g);
            }
        }
    }
    std::map<Vid, double> z;
    for (size_t k = 0; k < F.size(); ++k) {
        if (!plane[k]) return std::nullopt;
        for (const Vid v : F[k]) {
            const double h = plane[k]->first.dot(P2.at(v)) + plane[k]->second;
            const auto it = z.find(v);
            if (it != z.end() && std::abs(it->second - h) > 1e-6 * (1 + std::abs(h))) {
                return std::nullopt;
            }
            z[v] = h;
        }
    }
    return z;
}

/// The size of the patch of T in P, to scale constructions to.
double size_of(const Tets& T, const Positions& P)
{
    Eigen::Vector3d lo = Eigen::Vector3d::Constant(kInf), hi = -lo;
    for (const Vid v : vertices_of(T)) {
        if (v >= P.size() || !P[v].allFinite()) continue;
        lo = lo.cwiseMin(P[v]);
        hi = hi.cwiseMax(P[v]);
    }
    const double s = (hi - lo).norm();
    return std::isfinite(s) && s > 0 ? s : 1.;
}

/// The link triangles of apex c, oriented alike (each traverses a shared edge the other way
/// round its neighbour), and as a whole the way most real tets are: positive with c in front.
std::vector<Tri> oriented_link(const Tets& T, const Positions& P, const Vid c)
{
    const Eigen::Vector3d pc = P[c];
    std::vector<Tri> L;
    for (const Tet& t : T) {
        Tri f;
        int k = 0;
        for (const Vid v : t) {
            if (v != c) f[k++] = v;
        }
        L.push_back(f);
    }
    std::map<std::array<Vid, 2>, std::vector<size_t>> ef;
    for (size_t k = 0; k < L.size(); ++k) {
        for (int i = 0; i < 3; ++i) {
            const Vid a = L[k][i], b = L[k][(i + 1) % 3];
            ef[{{std::min(a, b), std::max(a, b)}}].push_back(k);
        }
    }
    std::vector<std::optional<Tri>> out(L.size());
    for (size_t s0 = 0; s0 < L.size(); ++s0) {
        if (out[s0]) continue;
        out[s0] = L[s0];
        std::vector<size_t> todo = {s0};
        while (!todo.empty()) {
            const size_t k = todo.back();
            todo.pop_back();
            const Tri f = *out[k];
            for (int i = 0; i < 3; ++i) {
                const Vid a = f[i], b = f[(i + 1) % 3];
                for (const size_t k2 : ef[{{std::min(a, b), std::max(a, b)}}]) {
                    if (k2 == k || out[k2]) continue;
                    Tri g = L[k2];
                    const int ia = int(std::find(g.begin(), g.end(), a) - g.begin());
                    const int ib = int(std::find(g.begin(), g.end(), b) - g.begin());
                    if ((ib + 1) % 3 != ia) std::swap(g[0], g[1]);
                    out[k2] = g;
                    todo.push_back(k2);
                }
            }
        }
    }
    std::vector<Tri> res;
    double agree = 0;
    for (const auto& f : out) {
        res.push_back(*f);
        const double det = (P[(*f)[0]] - pc).dot((P[(*f)[1]] - pc).cross(P[(*f)[2]] - pc));
        agree += det > 0 ? 1 : (det < 0 ? -1 : 0);
    }
    if (agree < 0) {
        for (Tri& f : res) std::swap(f[0], f[1]);
    }
    return res;
}

double orient2(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c)
{
    return (b - a).x() * (c - a).y() - (b - a).y() * (c - a).x();
}

/// L-BFGS, the line search refusing steps `valid` rejects: minimises f from x.
void lbfgs(
    Eigen::VectorXd& x,
    const std::function<double(const Eigen::VectorXd&, Eigen::VectorXd*)>& f,
    const std::function<bool(const Eigen::VectorXd&)>& valid,
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>& first_step,
    const int iterations)
{
    Eigen::VectorXd g(x.size());
    double E = f(x, &g);
    if (!std::isfinite(E)) return;
    std::vector<Eigen::VectorXd> S, Y;
    for (int it = 0; it < iterations; ++it) {
        Eigen::VectorXd q = g;
        std::vector<double> al;
        for (int k = int(S.size()) - 1; k >= 0; --k) {
            const double a = S[k].dot(q) / Y[k].dot(S[k]);
            al.push_back(a);
            q -= a * Y[k];
        }
        if (!S.empty()) q *= S.back().dot(Y.back()) / Y.back().dot(Y.back());
        for (size_t k = 0; k < S.size(); ++k) {
            const double b = Y[k].dot(q) / Y[k].dot(S[k]);
            q += (al[S.size() - 1 - k] - b) * S[k];
        }
        Eigen::VectorXd d = -q;
        if (d.dot(g) >= 0) {
            d = -g;
            S.clear();
            Y.clear();
        }
        double t = first_step(x, d);
        Eigen::VectorXd xn;
        double En = kInf;
        while (t > 1e-20) {
            xn = x + t * d;
            if (valid(xn)) {
                En = f(xn, nullptr);
                if (En <= E + 1e-4 * t * g.dot(d)) break;
            }
            t *= 0.5;
        }
        if (!(t > 1e-20)) break;
        Eigen::VectorXd gn(x.size());
        En = f(xn, &gn);
        const Eigen::VectorXd s = xn - x, y = gn - g;
        if (s.dot(y) > 1e-300) {
            S.push_back(s);
            Y.push_back(y);
            if (S.size() > 10) {
                S.erase(S.begin());
                Y.erase(Y.begin());
            }
        }
        const double dec = E - En;
        x = xn;
        E = En;
        g = gn;
        if (dec < 1e-10 * E) break;
    }
}

} // namespace

std::optional<Positions> spherical(const Tets& T, const Positions& P, const double fat)
{
    const auto c_opt = common_vertex(T);
    if (!c_opt) return std::nullopt;
    const Vid c = *c_opt;
    const auto B = boundary(T);
    if (!B) return std::nullopt;
    if (std::none_of(B->begin(), B->end(), [c](const Tri& f) { return contains(f, c); })) {
        return std::nullopt;
    }
    const Eigen::Vector3d pc = P[c];
    const std::vector<Tri> L = oriented_link(T, P, c);
    std::vector<Vid> vs;
    for (const Vid v : vertices_of(T)) {
        if (v != c) vs.push_back(v);
    }
    std::map<Vid, int> ix;
    for (size_t k = 0; k < vs.size(); ++k) ix[vs[k]] = int(k);
    const int n = int(vs.size());
    // the directed outline: link edges on one triangle, the triangle on their left
    std::map<std::array<Vid, 2>, int> cnt;
    for (const Tri& f : L) {
        for (int i = 0; i < 3; ++i) {
            const Vid a = f[i], b = f[(i + 1) % 3];
            ++cnt[{{std::min(a, b), std::max(a, b)}}];
        }
    }
    std::vector<std::pair<Vid, Vid>> out;
    for (const Tri& f : L) {
        for (int i = 0; i < 3; ++i) {
            const Vid a = f[i], b = f[(i + 1) % 3];
            if (cnt[{{std::min(a, b), std::max(a, b)}}] == 1) out.emplace_back(a, b);
        }
    }
    std::map<Vid, Vid> nxt;
    for (const auto& [a, b] : out) {
        if (!nxt.emplace(a, b).second) return std::nullopt;
    }
    if (out.empty()) return std::nullopt;
    std::vector<Vid> loop = {out[0].first};
    while (true) {
        const auto it = nxt.find(loop.back());
        if (it == nxt.end()) return std::nullopt;
        if (it->second == loop[0]) break;
        loop.push_back(it->second);
        if (loop.size() > out.size()) return std::nullopt;
    }
    if (loop.size() != out.size()) return std::nullopt;

    std::vector<Eigen::Vector3d> X(n), U(n);
    std::vector<double> rho(n);
    for (int k = 0; k < n; ++k) {
        X[k] = P[vs[k]] - pc;
        rho[k] = X[k].norm();
        if (!(rho[k] > 0)) return std::nullopt;
        U[k] = X[k] / rho[k];
    }
    // d: the least violated inward direction of the faces at c
    std::vector<Eigen::Vector3d> Nat;
    for (const auto& [a, b] : out) {
        const Eigen::Vector3d nin = (P[a] - pc).cross(P[b] - pc);
        const double l = nin.norm();
        if (!(l > 0)) return std::nullopt;
        Nat.push_back(-nin / l);
    }
    Eigen::Vector3d d = Eigen::Vector3d::Zero();
    if (const auto r =
            chebyshev(Nat, std::vector<double>(Nat.size(), 0.), Eigen::Vector3d::Zero(), 1, 1)) {
        d = r->first;
    }
    if (!(d.norm() > 1e-9)) {
        for (const auto& u : U) d += u;
    }
    if (!(d.norm() > 1e-12)) return std::nullopt;
    d.normalize();
    const Eigen::Vector3d h =
        std::abs(d.x()) < 0.9 ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
    const Eigen::Vector3d e1 = d.cross(h).normalized(), e2 = d.cross(e1);

    // Tutte start: the outline on a circle, spaced by the angles between real directions
    std::vector<double> s(loop.size() + 1, 0.), angles;
    for (size_t k = 0; k < loop.size(); ++k) {
        const double ang =
            std::acos(std::clamp(U[ix[loop[k]]].dot(U[ix[loop[(k + 1) % loop.size()]]]), -1., 1.));
        s[k + 1] = s[k] + ang + 1e-3;
        angles.push_back(std::acos(std::clamp(U[ix[loop[k]]].dot(d), -1., 1.)));
    }
    std::nth_element(angles.begin(), angles.begin() + long(angles.size() / 2), angles.end());
    const double rad = std::tan(std::clamp(angles[angles.size() / 2], 0.3, 1.1));
    std::map<Vid, Eigen::Vector2d> fixed;
    for (size_t k = 0; k < loop.size(); ++k) {
        const double th = 2 * M_PI * s[k] / s.back();
        fixed[loop[k]] = rad * Eigen::Vector2d(std::cos(th), std::sin(th));
    }
    std::set<std::array<Vid, 2>> edges;
    for (const Tri& f : L) {
        for (int i = 0; i < 3; ++i) {
            const Vid a = f[i], b = f[(i + 1) % 3];
            edges.insert({{std::min(a, b), std::max(a, b)}});
        }
    }
    const auto p2 = tutte(std::set<Vid>(vs.begin(), vs.end()), edges, fixed);
    // layout vertices: 0..n-1 the link's, n the origin
    std::vector<Eigen::Vector2d> x(n + 1, Eigen::Vector2d::Zero());
    for (int k = 0; k < n; ++k) x[k] = p2.at(vs[k]);
    bool all_negative = true;
    for (const Tri& f : L)
        all_negative = all_negative && orient2(x[ix[f[0]]], x[ix[f[1]]], x[ix[f[2]]]) < 0;
    if (all_negative) {
        for (auto& p : x) p.y() = -p.y();
    }
    // targets: the real spherical (chordal) link triangles, slivers opened to `fat`
    std::vector<double> vol(L.size());
    double vol_sum = 0;
    for (size_t k = 0; k < L.size(); ++k) {
        const auto& f = L[k];
        vol[k] = std::abs(X[ix[f[0]]].dot(X[ix[f[1]]].cross(X[ix[f[2]]])));
        vol_sum += vol[k];
    }
    std::vector<SlimElement> elements;
    for (size_t k = 0; k < L.size(); ++k) {
        const auto& f = L[k];
        const Eigen::Vector3d a = U[ix[f[0]]], b = U[ix[f[1]]], e = U[ix[f[2]]];
        const Eigen::Vector3d u = b - a, v = e - a;
        const double lu = u.norm();
        if (!(lu > 0)) return std::nullopt;
        const Eigen::Vector3d xu = u / lu;
        const double vx = v.dot(xu);
        double ly = (v - vx * xu).norm();
        const double longest = std::max({lu, v.norm(), (e - b).norm()});
        ly = std::max(ly, fat * longest);
        const double w = 0.5 * (vol_sum > 0 ? vol[k] / vol_sum : 0) + 0.5 / double(L.size());
        elements.push_back(slim_element(
            {{ix[f[0]], ix[f[1]], ix[f[2]]}},
            Eigen::Vector2d(0, 0),
            Eigen::Vector2d(lu, 0),
            Eigen::Vector2d(vx, ly),
            w));
    }
    for (const auto& [a, b] : out) {
        elements.push_back(
            slim_element({{n, ix[a], ix[b]}}, x[n], x[ix[a]], x[ix[b]], 1e-2 / double(out.size())));
    }
    if (!std::isfinite(slim_energy(x, elements))) return std::nullopt; // the start is not valid
    std::vector<char> fixed_flags(n + 1, 0);
    fixed_flags[n] = 1;
    slim(x, fixed_flags, elements, 200);

    // 3D polish: z = (gnomonic p, log radius) per vertex, symmetric Dirichlet of the tets
    std::vector<Eigen::Matrix3d> Rinv;
    std::vector<double> w3;
    std::vector<std::array<int, 3>> LI;
    bool polish = true;
    for (size_t k = 0; k < L.size(); ++k) {
        const auto& f = L[k];
        LI.push_back({{ix[f[0]], ix[f[1]], ix[f[2]]}});
        Eigen::Vector3d A = X[LI[k][0]], Bv = X[LI[k][1]], C = X[LI[k][2]];
        Eigen::Vector3d nrm = (Bv - A).cross(C - A);
        if (!(A.dot(Bv.cross(C)) > 0) || !(nrm.norm() > 0)) {
            polish = false;
            break;
        }
        nrm.normalize();
        const double hgt = A.dot(nrm);
        const double longest = std::max(
            {A.norm(), Bv.norm(), C.norm(), (Bv - A).norm(), (C - A).norm(), (C - Bv).norm()});
        const Eigen::Vector3d lift = std::max(fat * longest - hgt, 0.) * nrm;
        Eigen::Matrix3d R;
        R << A + lift, Bv + lift, C + lift;
        Rinv.push_back(R.inverse());
        w3.push_back(0.5 * (vol_sum > 0 ? vol[k] / vol_sum : 0) + 0.5 / double(L.size()));
    }
    const auto lifted = [&](const Eigen::VectorXd& z,
                            std::vector<Eigen::Vector3d>& Y,
                            std::vector<Eigen::Vector3d>& Uq,
                            std::vector<double>& nw) {
        Y.resize(n);
        Uq.resize(n);
        nw.resize(n);
        for (int k = 0; k < n; ++k) {
            const Eigen::Vector3d Wv = z[3 * k] * e1 + z[3 * k + 1] * e2 + d;
            nw[k] = Wv.norm();
            Uq[k] = Wv / nw[k];
            Y[k] = rho[k] * std::exp(z[3 * k + 2]) * Uq[k];
        }
    };
    const auto valid2d = [&](const Eigen::VectorXd& z) {
        const auto p = [&](int k) { return Eigen::Vector2d(z[3 * k], z[3 * k + 1]); };
        for (const auto& t : LI) {
            if (!(orient2(p(t[0]), p(t[1]), p(t[2])) > 0)) return false;
        }
        for (const auto& [a, b] : out) {
            if (!(orient2(Eigen::Vector2d::Zero(), p(ix[a]), p(ix[b])) > 0)) return false;
        }
        return true;
    };
    std::vector<SlimElement> fan(elements.end() - long(out.size()), elements.end());
    Eigen::VectorXd z(3 * n);
    for (int k = 0; k < n; ++k) {
        z[3 * k] = x[k].x();
        z[3 * k + 1] = x[k].y();
        z[3 * k + 2] = 0;
    }
    if (polish && valid2d(z)) {
        const auto energy = [&](const Eigen::VectorXd& zz, Eigen::VectorXd* g) -> double {
            std::vector<Eigen::Vector3d> Y, Uq;
            std::vector<double> nw;
            lifted(zz, Y, Uq, nw);
            double E = 0;
            std::vector<Eigen::Vector3d> gY(n, Eigen::Vector3d::Zero());
            for (size_t k = 0; k < LI.size(); ++k) {
                Eigen::Matrix3d E3;
                E3 << Y[LI[k][0]], Y[LI[k][1]], Y[LI[k][2]];
                const Eigen::Matrix3d J = E3 * Rinv[k];
                if (!(J.determinant() > 0)) return kInf;
                const Eigen::Matrix3d Ji = J.inverse();
                E += w3[k] * (J.squaredNorm() + Ji.squaredNorm());
                if (g) {
                    const Eigen::Matrix3d dJ =
                        w3[k] * (2 * J - 2 * Ji.transpose() * Ji * Ji.transpose());
                    const Eigen::Matrix3d dE3 = dJ * Rinv[k].transpose();
                    for (int c3 = 0; c3 < 3; ++c3) gY[LI[k][c3]] += dE3.col(c3);
                }
            }
            // the fan scaffold, in the gnomonic plane
            std::vector<Eigen::Vector2d> xp(n + 1, Eigen::Vector2d::Zero());
            for (int k = 0; k < n; ++k) xp[k] = Eigen::Vector2d(zz[3 * k], zz[3 * k + 1]);
            std::vector<Eigen::Vector2d> gp(n + 1, Eigen::Vector2d::Zero());
            for (const SlimElement& e : fan) {
                Eigen::Matrix2d Xe;
                Xe << xp[e.v[1]] - xp[e.v[0]], xp[e.v[2]] - xp[e.v[0]];
                const Eigen::Matrix2d J = Xe * e.rest_inv;
                const double dt = J.determinant();
                if (!(dt > 0)) return kInf;
                const double fro = J.squaredNorm();
                E += e.weight * fro * (1 + 1 / (dt * dt));
                if (g) {
                    Eigen::Matrix2d cof;
                    cof << J(1, 1), -J(1, 0), -J(0, 1), J(0, 0);
                    const Eigen::Matrix2d dJ =
                        e.weight * (2 * J * (1 + 1 / (dt * dt)) - 2 * fro / (dt * dt * dt) * cof);
                    const Eigen::Matrix2d G = dJ * e.rest_inv.transpose();
                    gp[e.v[1]] += G.col(0);
                    gp[e.v[2]] += G.col(1);
                    gp[e.v[0]] -= G.col(0) + G.col(1);
                }
            }
            if (g) {
                g->setZero(3 * n);
                for (int k = 0; k < n; ++k) {
                    const double rr = rho[k] * std::exp(zz[3 * k + 2]);
                    (*g)[3 * k + 2] = gY[k].dot(Y[k]);
                    const Eigen::Vector3d gU = gY[k] * rr;
                    const Eigen::Vector3d gW = (gU - gU.dot(Uq[k]) * Uq[k]) / nw[k];
                    (*g)[3 * k] = gW.dot(e1) + gp[k].x();
                    (*g)[3 * k + 1] = gW.dot(e2) + gp[k].y();
                }
            }
            return E;
        };
        const auto first_step = [](const Eigen::VectorXd& zz, const Eigen::VectorXd& dd) {
            double mx = 1e-3, md = 0;
            for (int k = 0; k < zz.size(); k += 3) {
                mx = std::max({mx, std::abs(zz[k]), std::abs(zz[k + 1])});
            }
            md = dd.cwiseAbs().maxCoeff();
            return std::min(1., 0.25 * mx / std::max(md, 1e-300));
        };
        Eigen::VectorXd z0 = z;
        lbfgs(z, energy, valid2d, first_step, 2000);
        if (!valid2d(z)) z = z0;
    }
    std::vector<Eigen::Vector3d> Y, Uq;
    std::vector<double> nw;
    lifted(z, Y, Uq, nw);
    Positions out_P = P;
    for (int k = 0; k < n; ++k) out_P[vs[k]] = pc + Y[k];
    return out_P;
}

std::optional<Positions> bowl(const Tets& T, const Positions& P)
{
    const auto c_opt = common_vertex(T);
    const auto B = boundary(T);
    if (!c_opt || !B) return std::nullopt;
    const Vid c = *c_opt;
    std::vector<Tri> D;
    for (const Tri& f : *B) {
        if (!contains(f, c)) D.push_back(f);
    }
    const auto cyc = disk_cycle(Faces(D.begin(), D.end()));
    if (!cyc || cyc->empty()) return std::nullopt;
    std::set<Vid> vs;
    std::set<std::array<Vid, 2>> edges;
    for (const Tri& f : D) {
        for (int i = 0; i < 3; ++i) {
            vs.insert(f[i]);
            const Vid a = f[i], b = f[(i + 1) % 3];
            edges.insert({{std::min(a, b), std::max(a, b)}});
        }
    }
    std::map<Vid, Eigen::Vector2d> fixed;
    for (size_t k = 0; k < cyc->size(); ++k) {
        const double th = 2 * M_PI * double(k) / double(cyc->size());
        fixed[(*cyc)[k]] = Eigen::Vector2d(std::cos(th), std::sin(th));
    }
    const auto P2 = tutte(vs, edges, fixed);
    const auto z = lift(D, P2);
    if (!z) return std::nullopt;
    double zmin = kInf, zmax = -kInf;
    for (const auto& [v, h] : *z) {
        zmin = std::min(zmin, h);
        zmax = std::max(zmax, h);
    }
    // every face plane's height above the centre, then above the bowl's rim
    double top = -kInf;
    for (const Tri& f : D) {
        Eigen::Matrix3d M;
        Eigen::Vector3d r;
        for (int i = 0; i < 3; ++i) {
            M.row(i) << P2.at(f[i]).x(), P2.at(f[i]).y(), 1;
            r[i] = z->at(f[i]);
        }
        top = std::max(top, M.partialPivLu().solve(r)[2]);
    }
    const double H = top + (zmax - zmin) + 1.0;
    const double scale = size_of(T, P) / 2;
    Positions out = P;
    for (const Vid v : vs) out[v] = scale * Eigen::Vector3d(P2.at(v).x(), P2.at(v).y(), z->at(v));
    out[c] = scale * Eigen::Vector3d(0, 0, H);
    return out;
}

std::optional<Positions> convex(const Tets& T, const Positions& P)
{
    const auto B = boundary(T);
    if (!B || B->size() < 4) return std::nullopt;
    std::vector<Tri> F(B->begin(), B->end());
    std::set<Vid> vs;
    std::set<std::array<Vid, 2>> edges;
    for (const Tri& f : F) {
        for (int i = 0; i < 3; ++i) {
            vs.insert(f[i]);
            const Vid a = f[i], b = f[(i + 1) % 3];
            edges.insert({{std::min(a, b), std::max(a, b)}});
        }
    }
    const Tri f0 = F[0];
    const std::map<Vid, Eigen::Vector2d> fixed = {
        {f0[0], Eigen::Vector2d(0, 0)},
        {f0[1], Eigen::Vector2d(1, 0)},
        {f0[2], Eigen::Vector2d(0, 1)}};
    const auto P2 = tutte(vs, edges, fixed);
    const std::vector<Tri> rest(F.begin() + 1, F.end());
    const auto z = lift(rest, P2);
    if (!z) return std::nullopt;
    const double scale = size_of(T, P);
    Positions out = P;
    for (const Vid v : vs) {
        if (!z->count(v)) return std::nullopt;
        out[v] = scale * Eigen::Vector3d(P2.at(v).x(), P2.at(v).y(), z->at(v));
    }
    std::set<Vid> inner;
    for (const Vid v : vertices_of(T)) {
        if (!vs.count(v)) inner.insert(v);
    }
    if (inner.size() > 1) return std::nullopt;
    if (inner.size() == 1) {
        Eigen::Vector3d m = Eigen::Vector3d::Zero();
        for (const Vid v : vs) m += out[v];
        out[*inner.begin()] = m / double(vs.size());
    }
    return out;
}

} // namespace wmtk::components::tracking::volume
