#include "ExactBandIntegral.hpp"

#include <wmtk/utils/Logger.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <numeric>
#include <optional>
#include <string>

// Every function below is the Python function of the same name (exact_tri.py, exact_mesh.py,
// scratchpad exact_integral/), with its operation order kept: the reference's numbers are the
// spec, and the tests compare to them at rounding level.

namespace wmtk::components::topological_offset {

namespace {

using Vec3 = Eigen::Vector3d;
using Stats = ExactBandIntegral::Stats;

constexpr double EPS = std::numeric_limits<double>::epsilon();

// exact_tri.py, near-parallel branch. MEASURED there on 400 random faces, max relative error for
// eps_F in [0,1e-3) [1e-3,1e-2) [1e-2,0.1) [0.1,0.3) [0.3,1):
//   radial branch forced  3.1e-11  1.9e-12  1.3e-13  1.9e-14  7.8e-15
//   series branch forced  4.1e-15  5.4e-15  5.8e-15  4.6e-15  diverges (1.8e+01)
// so the switch sits at 0.1, with margin below the divergence. The long-edge binomial sums
// amplify rounding by about ((1 + 2/SHORT) eps_F / 2)^k, which must stay below 1.
constexpr double EPS_PAR = 0.1; ///< eps_F below which a face uses the near-parallel series
constexpr int K_LONG = 12; ///< truncation: (EPS_PAR / 2)^(K_LONG + 1) ~ 1e-17
constexpr double SHORT = 0.5; ///< edge "short" if its (eta, zeta) length <= SHORT * rho
constexpr int K_SHORT = 28; ///< even terms up to K_SHORT: (SHORT / 2)^(K_SHORT + 2) ~ 1e-18
// Each series stops at the actual ratio of its edge (short: q = |P2 - P1| / (2 rho) <= SHORT/2;
// long: |dz| / (2 zm) < EPS_PAR/2), at the order whose tail meets the same bound as the fixed
// orders above, which stay the cap (exact_tri._order_short / _order_long, adopted in both
// 2026-10-12). MEASURED (mac-mini): t100026 11.2 -> 6.8 us per cell, series edges 41% -> 14% of
// the time, J moved by <= 3.4e-14 relative, 99.9% of 22180 band cells bit-identical.

/// Highest even order kept for ratio q: q^(order + 2) <= (SHORT/2)^(K_SHORT + 2).
int order_short(const double q)
{
    const int k =
        q > 0. ? int(std::ceil((K_SHORT + 2) * std::log(SHORT / 2) / std::log(q))) - 2 : 0;
    return std::clamp(k + (k & 1), 0, K_SHORT);
}

/// Highest order kept for ratio r: r^(order + 1) <= (EPS_PAR/2)^(K_LONG + 1).
int order_long(const double r)
{
    const int k =
        r > 0. ? int(std::ceil((K_LONG + 1) * std::log(EPS_PAR / 2) / std::log(r))) - 1 : 0;
    return std::clamp(k, 0, K_LONG);
}

double maxabs(const Vec3& p)
{
    return p.cwiseAbs().maxCoeff();
}

// ------------------------------------------------------------------------------------------------
// Timing and counting (Stats); a null Stats costs one branch per call site.
// ------------------------------------------------------------------------------------------------

struct Ctx
{
    Stats* st = nullptr;
    bool timing = false;
    /// The partition's tolerance budget (ExactBandIntegral::partition(), the checks in value()):
    /// the sum of tol over the clips that cut (tol_cut), and over every decision taken with a
    /// tolerance, clips and region tests alike (tol_any). Null outside the partition.
    double* tol_cut = nullptr;
    double* tol_any = nullptr;
    /// The cell's rounding scale (ExactBandIntegral::partition()): every clip and region test of
    /// the cell takes its tolerance on it, never below the polytope's own coordinates.
    double scale = 0.;
};

struct Timer
{
    double* acc = nullptr;
    std::chrono::steady_clock::time_point t0;
    Timer(const Ctx& cx, double Stats::* field)
    {
        if (cx.timing) {
            acc = &(cx.st->*field);
            t0 = std::chrono::steady_clock::now();
        }
    }
    ~Timer()
    {
        if (acc) {
            *acc += std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        }
    }
};

// ------------------------------------------------------------------------------------------------
// Convex polytopes (exact_tri: a list of faces (V, n), V counter-clockwise about the unit outward
// normal n). Face f is the ring P[off[f]] .. P[off[f + 1] - 1]. No faces = the Python's None.
// ------------------------------------------------------------------------------------------------

struct Poly
{
    std::vector<Vec3> P;
    std::vector<int> off{0};
    std::vector<Vec3> N;
    /// Per ring point: its vertex, 0 .. nv - 1, numbered in order of first appearance. The
    /// polytope's vertices are stored once: every ring point with the same id has the same
    /// coordinates (clip() writes each vertex once).
    std::vector<int> id;
    int nv = 0;
    /// The vertices, V[v] the coordinates of every ring point with id v (add_face() appends each at
    /// its first appearance). A min or max over the ring points is the one over V, which holds
    /// the same coordinates without the repeats: a convex polytope's vertex lies on 3 or more
    /// faces, so the ring points are 3 or more times as many. MEASURED (Geneva base, inward):
    /// partition()'s own time, classify() inlined in it and looping over the ring points, was 27%
    /// of the collapse pass's busy time once the value table answered its before-sums.
    std::vector<Vec3> V;
    int faces() const { return int(N.size()); }
    int size(int f) const { return off[size_t(f) + 1] - off[size_t(f)]; }
    const Vec3* ring(int f) const { return P.data() + off[size_t(f)]; }
    const int* ring_id(int f) const { return id.data() + off[size_t(f)]; }
    void clear()
    {
        P.clear();
        off.assign(1, 0);
        N.clear();
        id.clear();
        nv = 0;
        V.clear();
    }
    void add_face(const Vec3* pts, const int* ids, int k, const Vec3& n)
    {
        P.insert(P.end(), pts, pts + k);
        id.insert(id.end(), ids, ids + k);
        off.push_back(int(P.size()));
        N.push_back(n);
        for (int i = 0; i < k; ++i) {
            if (ids[i] == int(V.size())) V.push_back(pts[i]);
        }
    }
    /// np.max(np.abs(vstack of the rings)), over V
    double max_abs() const
    {
        double m = 0.;
        for (const Vec3& p : V) m = std::max(m, maxabs(p));
        return m;
    }
};

/// tet_polytope: the faces of T, outward; false if flat.
bool tet_polytope(const std::array<Vec3, 4>& T, Poly& out)
{
    double L = 0.;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < i; ++j) L = std::max(L, (T[size_t(i)] - T[size_t(j)]).norm());
    }
    const double vol6 = (T[1] - T[0]).dot((T[2] - T[0]).cross(T[3] - T[0]));
    if (L == 0. || std::abs(vol6) <= 64 * EPS * (L * L * L)) return false;
    out.clear();
    out.P.reserve(12); // four triangles (see clip(): reserved, not regrown)
    out.id.reserve(12);
    out.V.reserve(4);
    out.off.reserve(5);
    out.N.reserve(4);
    int renum[4] = {-1, -1, -1, -1};
    for (int i = 0; i < 4; ++i) {
        Vec3 V[3];
        int ids[3];
        int c = 0;
        for (int j = 0; j < 4; ++j) {
            if (j != i) {
                ids[c] = j;
                V[c++] = T[size_t(j)];
            }
        }
        Vec3 n = (V[1] - V[0]).cross(V[2] - V[0]);
        if (n.dot(V[0] - T[size_t(i)]) < 0) { // outward = away from the 4th vertex
            std::swap(V[0], V[2]);
            std::swap(ids[0], ids[2]);
            n = -n;
        }
        for (int& v : ids) { // corners numbered in order of first appearance
            if (renum[v] < 0) renum[v] = out.nv++;
            v = renum[v];
        }
        out.add_face(V, ids, 3, n / n.norm());
    }
    return true;
}

std::pair<Vec3, Vec3> plane_basis(const Vec3& g)
{
    const Vec3 a = std::abs(g[0]) < 0.9 ? Vec3(1., 0., 0.) : Vec3(0., 1., 0.);
    Vec3 u = g.cross(a);
    u /= u.norm();
    return {u, g.cross(u)}; // u x w = g
}

/// _clean_ring: drop consecutive near-duplicate vertices; false if fewer than 3 remain.
bool clean_ring(const std::vector<Vec3>& P, const double tol, std::vector<Vec3>& Q)
{
    Q.clear();
    for (const Vec3& p : P) {
        if (Q.empty() || maxabs(p - Q.back()) > tol) Q.push_back(p);
    }
    while (Q.size() > 1 && maxabs(Q.front() - Q.back()) <= tol) Q.pop_back();
    return Q.size() >= 3;
}

enum class Cut { None, Same, New };

/// clip: Q intersected with {x : g.x <= h}, |g| = 1. Vertices within rounding distance of the
/// plane are on it. None: no interior; Same: Q entirely inside (the result IS Q, `out` untouched);
/// New: the result in `out`.
///
/// Q's vertices are taken once (Poly::id): each vertex is put on, inside or outside the plane once,
/// each edge's crossing is computed once (from its inside end) and shared by the two faces through
/// the edge, and the cap is the circuit of the edges the cut opened, so the result is a closed
/// surface by construction (exact_tri.clip). Before, every face kept its own copy of each vertex,
/// and copies made by different cuts drifted apart. MEASURED (Geneva base, inward, exact rule):
/// copies of one vertex 1.4 tol apart, made by two cuts, became 8 tol apart at the next cut nearly
/// parallel to their edge, then an open sliver 0.84 long and 5.9e-9 wide; a piece came 1.7e-10
/// off its volume, the cell 1.6e-11 over (5 times the tiling bound), and the run aborted in turn 1.
/// Earlier forms of the same defect: a cap sorted by angle that skipped a vertex lying tol / theta
/// inside it (the cut crossing a face at the tiny angle theta), and a cap added where the kept
/// faces already closed up.
Cut clip(const Poly& Q, const Vec3& g, const double h, Poly& out, const Ctx& cx)
{
    Timer tm(cx, &Stats::t_clip);
    if (cx.st) ++cx.st->clips;
    const double tol = 16 * EPS * (std::abs(h) + std::max(Q.max_abs(), cx.scale));
    double smax = -std::numeric_limits<double>::infinity(), smin = -smax;
    for (const Vec3& p : Q.V) {
        const double s = p.dot(g) - h;
        smax = std::max(smax, s);
        smin = std::min(smin, s);
    }
    if (cx.tol_any) *cx.tol_any += tol;
    if (smax <= tol) return Cut::Same; // entirely inside
    if (smin >= -tol) return Cut::None; // entirely outside (or flat on the plane)
    out.clear();
    // per-thread scratch (clip is not re-entrant): MEASURED, its temporaries per call were a
    // visible part of the cost
    thread_local std::vector<Vec3> X;
    thread_local std::vector<double> sv, s;
    thread_local std::vector<char> seen;
    thread_local std::vector<int> rings, roff, capP, ring, pos, renum, circuit, stack, used, path,
        loop;
    thread_local std::vector<Vec3> rN, pts;
    thread_local std::vector<std::array<int, 3>> cross; // (lo, hi, vertex)
    thread_local std::vector<std::pair<int, int>> E;
    thread_local std::vector<std::vector<int>> nxt;
    thread_local std::vector<double> ang;
    // one position and one plane value per vertex
    X.resize(size_t(Q.nv));
    sv.resize(size_t(Q.nv));
    for (int v = 0; v < Q.nv; ++v) {
        X[size_t(v)] = Q.V[size_t(v)];
        const double si = Q.V[size_t(v)].dot(g) - h;
        sv[size_t(v)] = std::abs(si) <= tol ? 0. : si;
    }
    seen.assign(size_t(Q.nv), 0);
    rings.clear();
    roff.assign(1, 0);
    rN.clear();
    capP.clear();
    cross.clear();
    const auto cap_pt = [&](const int v) {
        if (size_t(v) >= seen.size()) seen.resize(size_t(v) + 1, 0);
        if (seen[size_t(v)]) return;
        seen[size_t(v)] = 1;
        capP.push_back(v);
    };
    for (int f = 0; f < Q.faces(); ++f) {
        const int* r = Q.ring_id(f);
        const int k = Q.size(f);
        s.resize(size_t(k));
        bool all_le = true, all_ge = true, on_plane = true;
        for (int i = 0; i < k; ++i) {
            const double si = sv[size_t(r[i])];
            s[size_t(i)] = si;
            all_le = all_le && si <= 0.;
            all_ge = all_ge && si >= 0.;
            on_plane = on_plane && si == 0.;
        }
        // A face of Q whose outward normal is g up to rounding lies in the cut plane: a convex
        // polytope has one supporting plane per outward normal, and the cap is the result's face
        // with normal g. So the face is part of the cap and not kept itself (also when every vertex
        // is within tol of the plane). Kept as well, it counted twice in the volume. MEASURED: band
        // cells of the sphere and the prism came out 2.6e-6, 3.4% and 9% over their volume, with
        // faces in the plane up to 1.3 tol (vertices made at a larger scale, or by a coincident
        // plane of another feature; normal equal to g or 1 ulp off).
        const Vec3& nf = Q.N[size_t(f)];
        const bool in_cap = nf.dot(g) > 0. && (on_plane || nf.cross(g).norm() <= 16 * EPS);
        if (all_le) {
            if (in_cap) {
                for (int i = 0; i < k; ++i) cap_pt(r[i]);
            } else {
                rings.insert(rings.end(), r, r + k);
                roff.push_back(int(rings.size()));
                rN.push_back(nf);
                for (int i = 0; i < k; ++i) {
                    if (s[size_t(i)] == 0.) cap_pt(r[i]);
                }
            }
            continue;
        }
        if (all_ge) {
            for (int i = 0; i < k; ++i) {
                if (s[size_t(i)] == 0.) cap_pt(r[i]);
            }
            continue;
        }
        ring.clear();
        for (int i = 0; i < k; ++i) {
            const int j = (i + 1) % k;
            const double si = s[size_t(i)], sj = s[size_t(j)];
            if (si <= 0.) ring.push_back(r[i]);
            if (si == 0.) cap_pt(r[i]);
            if ((si < 0. && 0. < sj) || (sj < 0. && 0. < si)) {
                const int lo = std::min(r[i], r[j]), hi = std::max(r[i], r[j]);
                int x = -1;
                for (const auto& c : cross) {
                    if (c[0] == lo && c[1] == hi) {
                        x = c[2];
                        break;
                    }
                }
                if (x < 0) {
                    const int a = si < 0. ? r[i] : r[j],
                              b = si < 0. ? r[j] : r[i]; // from the inside end
                    const double sa = sv[size_t(a)], sb = sv[size_t(b)];
                    x = int(X.size());
                    X.push_back(X[size_t(a)] + (sa / (sa - sb)) * (X[size_t(b)] - X[size_t(a)]));
                    sv.push_back(0.);
                    cross.push_back({lo, hi, x});
                }
                ring.push_back(x);
                cap_pt(x);
            }
        }
        if (in_cap) {
            for (const int v : ring) cap_pt(v);
            continue;
        }
        rings.insert(rings.end(), ring.begin(), ring.end());
        roff.push_back(int(rings.size()));
        rN.push_back(Q.N[size_t(f)]);
    }
    const int nV = int(X.size());
    // The cap: the free edges of the kept rings (a directed edge (a, b) with no reversed partner
    // borders the cap, which runs it as (b, a)), traced as Euler circuits, one cap face per circuit
    // (a circuit may touch itself at a vertex). Each circuit starts at its smallest angle about the
    // mean of the cap points (capP, in the order the cut met them; ties by that order), and at a
    // vertex the unused edge to the earliest cap point is taken first: a convex cap is the ring the
    // angle sort gave. The kept rings bound a 2-chain, so its free edges form closed walks.
    E.clear();
    for (size_t r = 0; r + 1 < roff.size(); ++r) {
        const int b = roff[r], k = roff[r + 1] - b;
        for (int i = 0; i < k; ++i)
            E.push_back({rings[size_t(b + i)], rings[size_t(b + (i + 1) % k)]});
    }
    std::sort(E.begin(), E.end());
    nxt.resize(size_t(nV));
    for (auto& t : nxt) t.clear();
    int n_free = 0;
    for (size_t i = 0; i < E.size();) {
        size_t j = i;
        while (j < E.size() && E[j] == E[i]) ++j;
        const auto [a, b] = E[i];
        const auto rev = std::equal_range(E.begin(), E.end(), std::pair<int, int>{b, a});
        for (int m = int(j - i) - int(rev.second - rev.first); m > 0; --m) {
            nxt[size_t(b)].push_back(a); // the cap runs b -> a
            ++n_free;
        }
        i = j;
    }
    int n_caps = 0;
    if (n_free > 0) {
        pos.assign(size_t(nV), -1);
        int np = 0;
        for (const int v : capP) pos[size_t(v)] = np++;
        for (const int v :
             rings) { // free-edge ends the cut did not meet: after them, by first appearance
            if (pos[size_t(v)] < 0 && !nxt[size_t(v)].empty()) pos[size_t(v)] = np++;
        }
        for (int v = 0; v < nV; ++v) {
            for (const int w : nxt[size_t(v)]) {
                if (pos[size_t(w)] < 0) pos[size_t(w)] = np++;
            }
        }
        const auto [u, w] = plane_basis(g);
        Vec3 c = Vec3::Zero();
        for (const int v : capP) c += X[size_t(v)];
        c /= double(capP.size());
        ang.assign(size_t(nV), 0.);
        for (int v = 0; v < nV; ++v) {
            if (pos[size_t(v)] >= 0) {
                ang[size_t(v)] = std::atan2((X[size_t(v)] - c).dot(w), (X[size_t(v)] - c).dot(u));
            }
            std::sort(nxt[size_t(v)].begin(), nxt[size_t(v)].end(), [&](int a, int b) {
                return pos[size_t(a)] < pos[size_t(b)];
            });
        }
        used.assign(size_t(nV), 0);
        while (true) {
            int start = -1;
            for (int v = 0; v < nV; ++v) {
                if (used[size_t(v)] >= int(nxt[size_t(v)].size())) continue;
                if (start < 0 || ang[size_t(v)] < ang[size_t(start)] ||
                    (ang[size_t(v)] == ang[size_t(start)] && pos[size_t(v)] < pos[size_t(start)])) {
                    start = v;
                }
            }
            if (start < 0) break;
            stack.assign(1, start);
            circuit.clear();
            while (!stack.empty()) {
                const int v = stack.back();
                if (used[size_t(v)] < int(nxt[size_t(v)].size())) {
                    stack.push_back(nxt[size_t(v)][size_t(used[size_t(v)]++)]);
                } else {
                    circuit.push_back(v);
                    stack.pop_back();
                }
            }
            std::reverse(circuit.begin(), circuit.end());
            rings.insert(rings.end(), circuit.begin(), circuit.end() - 1);
            roff.push_back(int(rings.size()));
            rN.push_back(g);
            ++n_caps;
        }
    }
    // Edges no longer than tol (max norm) are collapsed, in every face at once (a vertex merges
    // into the one that appears first), and faces left with fewer than 3 vertices are dropped: the
    // surface stays closed (exact_tri.clip). Per-face cleaning (before) merged a vertex in one face
    // and kept it in the next; and two crossings can round to the same point (MEASURED, t100026:
    // two edges ending 1e1 tol above the plane, 4e9 tol below at their other ends).
    renum.assign(size_t(nV), -1); // here: rank, the order of first appearance
    int nr = 0;
    for (const int v : rings) {
        if (renum[size_t(v)] < 0) renum[size_t(v)] = nr++;
    }
    used.assign(size_t(nV), 0); // here: union-find parent
    for (int v = 0; v < nV; ++v) used[size_t(v)] = v;
    const auto find = [&](int v) {
        while (used[size_t(v)] != v) v = used[size_t(v)];
        return v;
    };
    for (size_t r = 0; r + 1 < roff.size(); ++r) {
        const int b = roff[r], k = roff[r + 1] - b;
        for (int i = 0; i < k; ++i) {
            int x = find(rings[size_t(b + i)]), y = find(rings[size_t(b + (i + 1) % k)]);
            if (x == y || maxabs(X[size_t(x)] - X[size_t(y)]) > tol) continue;
            if (renum[size_t(y)] < renum[size_t(x)]) std::swap(x, y);
            used[size_t(y)] = x;
        }
    }
    // the faces, their vertices renumbered in the order they appear
    for (int& v : renum) v = -1;
    int nv_out = 0;
    // Room for the result, reserved once: its faces are the kept rings and the caps, each emitted
    // as one or more loops of its points, so the rings' points and faces bound it except where a
    // ring splits into several loops. MEASURED (Geneva base, inward, collapse pass, sampled):
    // add_face(), the vectors regrowing face by face, was 25% of the busy time.
    out.P.reserve(rings.size());
    out.id.reserve(rings.size());
    out.V.reserve(size_t(nV));
    out.off.reserve(roff.size());
    out.N.reserve(roff.size() - 1);
    for (size_t r = 0; r + 1 < roff.size(); ++r) {
        const int b = roff[r], k = roff[r + 1] - b;
        ring.clear();
        for (int i = 0; i < k; ++i) {
            const int v = find(rings[size_t(b + i)]);
            if (ring.empty() || ring.back() != v) ring.push_back(v);
        }
        while (ring.size() > 1 && ring.front() == ring.back()) ring.pop_back();
        // A ring that visits a vertex twice (a cap circuit touching itself, or two vertices merged
        // above) is split there into simple loops, each a face with the same normal; loops of
        // fewer than 3 vertices (an edge run there and back) are dropped. The edges are the same,
        // so the surface stays closed, and every face is a simple polygon: no edge in more than two
        // faces at the next cut. MEASURED (Geneva): a ring through one vertex twice, cut again,
        // left an edge 9 tol long in four faces.
        const auto emit = [&](const int* L, const int k) {
            if (k < 3) return;
            pts.resize(size_t(k));
            loop.resize(size_t(k));
            for (int i = 0; i < k; ++i) {
                const int v = L[i];
                pts[size_t(i)] = X[size_t(v)];
                if (renum[size_t(v)] < 0) renum[size_t(v)] = nv_out++;
                loop[size_t(i)] = renum[size_t(v)];
            }
            out.add_face(pts.data(), loop.data(), k, rN[r]);
        };
        path.clear();
        for (const int v : ring) {
            int i = -1;
            for (size_t j = 0; j < path.size(); ++j) {
                if (path[j] == v) {
                    i = int(j);
                    break;
                }
            }
            if (i >= 0) {
                emit(path.data() + i, int(path.size()) - i);
                path.resize(size_t(i) + 1);
            } else {
                path.push_back(v);
            }
        }
        emit(path.data(), int(path.size()));
    }
    out.nv = nv_out;
    if (cx.st) {
        if (n_caps == 0) ++cx.st->clips_closed_up;
        if (n_caps > 1) ++cx.st->caps_split;
    }
    if (cx.tol_cut) *cx.tol_cut += tol;
    if (cx.st && cx.st->check_closed && out.faces() >= 4) {
        // every directed edge exactly once, and its reverse exactly once
        E.clear();
        for (int f = 0; f < out.faces(); ++f) {
            const int* r = out.ring_id(f);
            const int k = out.size(f);
            for (int i = 0; i < k; ++i) E.push_back({r[i], r[(i + 1) % k]});
        }
        std::sort(E.begin(), E.end());
        bool closed = std::adjacent_find(E.begin(), E.end()) == E.end();
        for (size_t i = 0; i < E.size() && closed; ++i) {
            closed = std::binary_search(
                E.begin(),
                E.end(),
                std::pair<int, int>{E[i].second, E[i].first});
        }
        ++cx.st->closed_checked;
        if (!closed) ++cx.st->open_results;
    }
    return out.faces() >= 4 ? Cut::New : Cut::None;
}

/// R = clip(R, g, h) in place; false (R emptied) if None.
bool clip_inplace(Poly& R, const Vec3& g, const double h, Poly& tmp, const Ctx& cx)
{
    const Cut c = clip(R, g, h, tmp, cx);
    if (c == Cut::None) {
        R.clear();
        return false;
    }
    if (c == Cut::New) std::swap(R, tmp);
    return true;
}

/// volume_centroid: volume and centroid of a convex polytope (fan of tets from one vertex).
std::pair<double, Vec3> volume_centroid(const Poly& Q)
{
    const Vec3 o = Q.P[0];
    double vol6 = 0.;
    Vec3 mom = Vec3::Zero();
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const Vec3 a = V[0] - o;
        for (int i = 1; i < Q.size(f) - 1; ++i) {
            const Vec3 b = V[i] - o, c = V[i + 1] - o;
            const double d = a.dot(b.cross(c));
            vol6 += d;
            mom += d * (a + b + c);
        }
    }
    if (vol6 <= 0.) return {0., o};
    return {vol6 / 6., o + mom / (4. * vol6)};
}

// ------------------------------------------------------------------------------------------------
// One-dimensional closed forms (exact_tri). Differences of antiderivatives in rationalized form
// where the two ends have the same sign, so a short or far interval does not cancel.
// ------------------------------------------------------------------------------------------------

/// int_{u1}^{u1+ell} sqrt(u^2 + k^2) du, ell > 0, k >= 0.
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

/// int over the segment [x1, x2] of the distance to the line {A + l e}.
double seg_line(const Vec3& x1, const Vec3& x2, const Vec3& A, const Vec3& e)
{
    const Vec3 d = x2 - x1;
    const double ell = d.norm();
    Vec3 w0 = x1 - A;
    w0 = w0 - w0.dot(e) * e; // perpendicular offset at x1
    Vec3 tp = d / ell;
    tp = tp - tp.dot(e) * e; // perpendicular part of the direction
    const double sa = tp.norm();
    if (sa <= EPS) return ell * w0.norm(); // segment parallel to the line
    const Vec3 t = tp / sa;
    const double beta = w0.dot(t);
    const double k = w0.cross(t).norm();
    return int_sqrt(beta, sa * ell, k) / sa;
}

/// int_0^eta sqrt(t^2 + b^2) dt.
double G_(const double eta, const double b)
{
    if (b == 0.) return 0.5 * eta * std::abs(eta);
    return 0.5 * (eta * std::hypot(eta, b) + b * b * std::asinh(eta / b));
}

/// Mean of G_(., b) over the interval between e1 and e2.
double mean_G(const double e1, const double e2, const double b)
{
    if (e1 <= 0. && e2 <= 0.) {
        if (e1 == 0. && e2 == 0.) return 0.;
        return -mean_G(-e1, -e2, b); // G is odd
    }
    if (e1 < 0. || e2 < 0.) { // opposite signs: split at 0
        const double lo = std::min(e1, e2), hi = std::max(e1, e2);
        return (hi * mean_G(0., hi, b) - (-lo) * mean_G(0., -lo, b)) / (hi - lo);
    }
    if (e1 == e2) return G_(e1, b);
    if (b == 0.) return (e1 * e1 + e1 * e2 + e2 * e2) / 6.;
    const double R1 = std::hypot(e1, b), R2 = std::hypot(e2, b);
    const double dR = (e1 + e2) / (R1 + R2); // (R2 - R1) / (e2 - e1)
    const double t1 = dR * (R2 * R2 + R1 * R2 + R1 * R1) / 6.;
    const double den = e2 * R1 + e1 * R2;
    const double x = (e2 - e1) * (e1 + e2) / den; // asinh(e2/b) - asinh(e1/b) = asinh(x)
    const double ash = (e1 + e2) / den * (x != 0. ? std::asinh(x) / x : 1.);
    return t1 + 0.5 * b * b * (e2 * ash + std::asinh(e1 / b) - dR);
}

// ------------------------------------------------------------------------------------------------
// Truncated power series ("jets"), exact_tri _jmul .. _jG. Length n <= JCAP.
// ------------------------------------------------------------------------------------------------

/// A jet's (or, in the derivative code, a polynomial's) capacity: the longest is seg_int()'s
/// short-segment series, K_SEG + 4 = 34 terms.
constexpr int JCAP = 48;
using Jet = std::array<double, JCAP>;

void jdiv(const double* a, const double* b, const int n, double* c)
{
    c[0] = a[0] / b[0];
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        for (int j = 1; j <= k; ++j) s += b[j] * c[k - j];
        c[k] = (a[k] - s) / b[0];
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

/// The length of a without its trailing exact zeros (+0 or -0).
int jlen(const double* a, int n)
{
    while (n > 0 && a[n - 1] == 0.) --n;
    return n;
}

/// The product of a vanishing beyond its first na coefficients and b beyond its first nb (exact
/// zeros, jlen()), to length n: c[k] = sum over j of a[j] b[k - j], added from +0 in ascending j
/// (exact_tri _jmul), the zero terms left out. The skipped terms lie at the two ends of each sum
/// (j < k - nb + 1 or j >= na), and adding a zero product to a sum started at +0 changes nothing,
/// so the value is the full sum's, bit for bit. The sums are taken JB at a time, k0 .. k0 + JB - 1,
/// in registers, one term each per j over the j any of them needs: a term a sum does not need
/// (b[k - j] with k - j < 0 or >= nb) reads a zero of the padded copy of b, a zero product, which
/// changes nothing for the same reason. Why: the front form's series multiply jets of length up to
/// KB, most of them polynomials of low degree (the weights, the powers of eta, the jets of
/// eta = 0), and a sum taken alone is a chain of dependent additions. MEASURED (cube_out band, the
/// bench's front_flux evaluations, sampled): jasinh_w(), jXi() and edge_np_w() were 43%, 25% and
/// 15% of the time, self, with the dense products one sum after another; (mac-mini, the build's
/// flags, n = 31) a dense product 0.31 us that way, 0.081 us in blocks of 8, the same bits.
void jmul_s(const double* a, const int na, const double* b, const int nb, const int n, double* c)
{
    constexpr int JB = 8;
    double bp[JB + JCAP + JB]; // bp[JB + i] = b[i], i < min(nb, n); zeros around, as far as read
    const int mb = std::max(0, std::min(nb, n));
    std::fill(bp, bp + JB, 0.);
    std::memcpy(bp + JB, b, size_t(mb) * sizeof(double));
    std::fill(bp + JB + mb, bp + JB + n + JB, 0.);
    for (int k0 = 0; k0 < n; k0 += JB) {
        double s[JB] = {};
        const int jlo = std::max(0, k0 - nb + 1), jhi = std::min(na - 1, k0 + JB - 1);
        for (int j = jlo; j <= jhi; ++j) {
            const double aj = a[j];
            const double* bj = bp + JB + k0 - j; // bj[i] = b[k0 + i - j]
            for (int i = 0; i < JB; ++i) s[i] += aj * bj[i];
        }
        for (int i = 0; i < JB && k0 + i < n; ++i) c[k0 + i] = s[i];
    }
}

void jmul(const double* a, const double* b, const int n, double* c)
{
    jmul_s(a, n, b, n, n, c);
}

/// Jets of asinh(u) and w = sqrt(1 + u^2). u = 0 (every coefficient +0, as jdiv2() makes it from
/// eta = 0): A = 0 and w = 1, the values the recurrences below return there, bit for bit (every
/// sum and quotient of +0 terms is +0, sqrt(1) = 1).
void jasinh_w(const double* u, const int n, double* A, double* w)
{
    bool zero = true;
    for (int i = 0; i < n && zero; ++i) zero = u[i] == 0. && !std::signbit(u[i]);
    if (zero) {
        for (int i = 0; i < n; ++i) A[i] = w[i] = 0.;
        w[0] = 1.;
        return;
    }
    Jet t, du, q;
    jmul(u, u, n, t.data());
    t[0] += 1.;
    jsqrt(t.data(), n, w);
    for (int i = 0; i + 1 < n; ++i) du[size_t(i)] = double(i + 1) * u[i + 1];
    jdiv(du.data(), w, n - 1, q.data()); // d asinh(u) = du / w
    A[0] = std::asinh(u[0]);
    for (int i = 0; i + 1 < n; ++i) A[i + 1] = q[size_t(i)] / double(i + 1);
}

/// jmul(a, b) for a vanishing beyond its first 3 coefficients: the same sum in the same order,
/// the zero terms left out, so the same value.
void jmul3(const double* a, const double* b, const int n, double* c)
{
    jmul_s(a, 3, b, n, n, c); // a[0] b[k] + a[1] b[k - 1] + a[2] b[k - 2], in that order
}

/// jdiv(a, b) for b vanishing beyond its first 2 coefficients: the same sum, zero terms left out.
void jdiv2(const double* a, const double* b, const int n, double* c)
{
    c[0] = a[0] / b[0];
    for (int k = 1; k < n; ++k) {
        double s = 0.;
        s += b[1] * c[k - 1];
        c[k] = (a[k] - s) / b[0];
    }
}

/// _jG for the two jets both series use, eta = e0 + e1 l and zeta = z0 + z1 l (linear; z0 > 0):
/// jets of G(eta, zeta) = int_0^eta sqrt(t^2 + zeta^2) dt and, if R is given, of
/// R = sqrt(eta^2 + zeta^2). The products with a linear factor skip its zero coefficients
/// (jmul3, jdiv2), which halves the dense jet products and changes no value (MEASURED: every
/// band cell of the five stored cases bit-identical to the dense _jG).
void jG_linear(
    const double e0,
    const double e1,
    const double z0,
    const double z1,
    const int n,
    double* G,
    double* R)
{
    Jet eta{}, zeta{}, u, A, w, z2, uw;
    eta[0] = e0;
    eta[1] = e1;
    zeta[0] = z0;
    zeta[1] = z1;
    jdiv2(eta.data(), zeta.data(), n, u.data());
    jasinh_w(u.data(), n, A.data(), w.data());
    jmul3(zeta.data(), zeta.data(), n, z2.data());
    jmul(u.data(), w.data(), n, uw.data());
    for (int i = 0; i < n; ++i) uw[size_t(i)] += A[size_t(i)];
    jmul3(z2.data(), uw.data(), n, G);
    for (int i = 0; i < n; ++i) G[i] *= 0.5;
    if (R) jmul3(zeta.data(), w.data(), n, R);
}

double binom_loop(const int k, const int j)
{
    double c = 1.;
    for (int i = 1; i <= j; ++i) c = c * double(k - j + i) / double(i);
    return std::round(c);
}

/// binom_loop(k, j), tabulated once for 0 <= j <= k < 64: the same doubles. Why: the series
/// call it in their innermost loops. MEASURED (coarse_r1S1 band, the bench's front_flux
/// evaluations, sampled): binom(), std::pow() and pshift() were 31%, 25% and 9% of the time, self,
/// in edge_np_w()'s long branch.
double binom(const int k, const int j)
{
    constexpr int N = 64;
    static const std::vector<double> tab = [] {
        std::vector<double> t(size_t(N * N), 0.);
        for (int a = 0; a < N; ++a) {
            for (int b = 0; b <= a; ++b) t[size_t(a * N + b)] = binom_loop(a, b);
        }
        return t;
    }();
    if (0 <= j && j <= k && k < N) return tab[size_t(k * N + j)];
    return binom_loop(k, j);
}

/// _edge_near_parallel: int_{xi1}^{xi2} G(eta(xi), zeta(xi)) dxi for one edge, dxi = xi2 - xi1.
/// G is analytic in zeta about the edge's mid value zm with radius zm; the series below converge
/// with ratio <= SHORT/2 (short edges, along the edge) or < EPS_PAR/2 (long edges, in eta).
double edge_near_parallel(
    const double e1,
    const double e2,
    const double z1,
    const double z2,
    const double dxi,
    const Ctx& cx)
{
    Timer tm(cx, &Stats::t_series);
    const double em = 0.5 * (e1 + e2), zm = 0.5 * (z1 + z2);
    const double de = e2 - e1, dz = z2 - z1;
    const double rho = std::hypot(em, zm);
    if (std::hypot(de, dz) <= SHORT * rho) {
        // Taylor series of lambda -> G(P_m + lambda (P2 - P1)), mean over [-1/2, 1/2]
        if (cx.st) ++cx.st->series_short;
        const int K = order_short(std::hypot(de, dz) / (2. * rho)) + 1;
        Jet g;
        jG_linear(em, de, zm, dz, K, g.data(), nullptr); // R is not needed here
        double sum = 0., p = 1.;
        for (int i = 0; i < K; i += 2) {
            sum += g[size_t(i)] * p / double(i + 1);
            p *= 0.25; // 0.5^i, exact
        }
        return dxi * sum;
    }
    // long edge: exact in eta, Taylor in delta = zeta - zm = mu (eta - em)
    if (cx.st) ++cx.st->series_long;
    const int K = order_long(std::abs(dz) / (2. * zm)) + 1;
    const int KJ = K + 1; // jet length
    const double mu = dz / de;
    Jet zj{}, z2j;
    zj[0] = zm;
    zj[1] = 1.;
    jmul3(zj.data(), zj.data(), KJ, z2j.data());
    std::array<std::array<Jet, K_LONG + 1>, 2> Aend;
    for (int s = 0; s < 2; ++s) {
        const double ea = s == 0 ? e1 : e2;
        Jet G, R, R2, R3, t;
        jG_linear(ea, 0., zm, 1., KJ, G.data(), R.data());
        jmul(R.data(), R.data(), KJ, R2.data());
        jmul(R.data(), R2.data(), KJ, R3.data());
        std::array<Jet, K_LONG + 2> Phi; // Phi_m = int eta^m R d eta
        Phi[0] = G;
        for (int i = 0; i < KJ; ++i) Phi[1][size_t(i)] = R3[size_t(i)] / 3.;
        for (int m = 2; m <= K; ++m) {
            jmul3(z2j.data(), Phi[size_t(m - 2)].data(), KJ, t.data());
            const double em1 = std::pow(ea, double(m - 1));
            for (int i = 0; i < KJ; ++i) {
                Phi[size_t(m)][size_t(i)] =
                    (em1 * R3[size_t(i)] - double(m - 1) * t[size_t(i)]) / double(m + 2);
            }
        }
        // A_j = int eta^j G d eta = (eta^{j+1} G - Phi_{j+1}) / (j + 1)
        for (int jj = 0; jj < K; ++jj) {
            const double ep = std::pow(ea, double(jj + 1));
            for (int i = 0; i < KJ; ++i) {
                Aend[size_t(s)][size_t(jj)][size_t(i)] =
                    (ep * G[size_t(i)] - Phi[size_t(jj + 1)][size_t(i)]) / double(jj + 1);
            }
        }
    }
    double tot = 0.;
    for (int k = 0; k < K; ++k) {
        double sk = 0.;
        for (int jj = 0; jj <= k; ++jj) {
            sk += binom(k, jj) * std::pow(-em, double(k - jj)) *
                  (Aend[1][size_t(jj)][size_t(k)] - Aend[0][size_t(jj)][size_t(k)]);
        }
        tot += std::pow(mu, double(k)) * sk;
    }
    return dxi / de * tot;
}

// ------------------------------------------------------------------------------------------------
// Face integrals over a planar convex polygon V[0..k), CCW about the unit normal n (exact_tri).
// _edges skips zero-length edges and gives t = unit direction, m = t x n (in-plane outward).
// ------------------------------------------------------------------------------------------------

/// int_F |x - v| dA.
double face_point(const Vec3* V, const int k, const Vec3& n, const Vec3& v, const Ctx& cx)
{
    Timer tm(cx, &Stats::t_closed);
    if (cx.st) ++cx.st->point_faces;
    const double z0 = n.dot(V[0] - v);
    const double c = std::abs(z0);
    const Vec3 q = v + z0 * n; // foot of v on the face plane
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (ell == 0.) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const double h = (x1 - q).dot(m); // signed distance from q to the edge line
        if (h == 0.) continue;
        if (cx.st) ++cx.st->closed_edges;
        const double s1 = (x1 - q).dot(t);
        const double s2 = s1 + ell;
        const double a = std::hypot(h, c);
        double term = h * int_sqrt(s1, ell, a);
        if (c > 0.) {
            const double R1 = std::hypot(s1, a), R2 = std::hypot(s2, a);
            double num, ash;
            if (s1 * s2 > 0.) {
                const double den = s2 * R1 + s1 * R2;
                num = a * a * ell * (s1 + s2) / den; // s2 R1 - s1 R2
                ash = std::asinh(ell * (s1 + s2) / den); // asinh(s2/a) - asinh(s1/a)
            } else {
                num = s2 * R1 - s1 * R2;
                ash = std::asinh(s2 / a) - std::asinh(s1 / a);
            }
            // c^3 [atan(s c/(h R)) - atan(s/h)] between the ends, as exact angle differences
            const double ang = std::atan2(c * num * h, h * h * R1 * R2 + s1 * s2 * c * c) -
                               std::atan2(ell * h, h * h + s1 * s2);
            term += h * c * c * ash + c * c * c * ang;
        }
        tot += term;
    }
    return tot / 3.;
}

/// _face_line_parallel: line exactly parallel to the plane, r = sqrt(eta^2 + b^2), field (G, 0).
double face_line_parallel(
    const Vec3* V,
    const int k,
    const Vec3& n,
    const Vec3& A,
    const Vec3& e,
    const Ctx& cx)
{
    if (cx.st) ++cx.st->line_faces_parallel;
    const Vec3 eh = n.cross(e); // in-plane, across the projected line
    const double b = std::abs(n.dot(V[0] - A));
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3& x2 = V[(i + 1) % k];
        const Vec3 d = x2 - x1;
        const double ell = d.norm();
        if (ell == 0.) continue;
        const Vec3 m = (d / ell).cross(n);
        const double mh = m.dot(eh);
        if (mh != 0.) {
            if (cx.st) ++cx.st->closed_edges;
            tot += mh * ell * mean_G((x1 - A).dot(eh), (x2 - A).dot(eh), b);
        }
    }
    return tot;
}

/// int_F r dA, r = distance to the line {A + l e}. Three branches (exact_tri.face_line): exactly
/// parallel (closed form), near-parallel eps_F < EPS_PAR (series), otherwise radial about the
/// crossing point q, 1/3 sum_e h_e int_e r ds.
double
face_line(const Vec3* V, const int k, const Vec3& n, const Vec3& A, const Vec3& e, const Ctx& cx)
{
    Timer tm(cx, &Stats::t_closed);
    const double sn = e.dot(n);
    if (sn == 0.) return face_line_parallel(V, k, n, A, e, cx);
    if (std::abs(sn) < 0.5) { // in-plane frame well defined; else q is near
        const Vec3 ep = e - sn * n;
        const double cth = ep.norm();
        const Vec3 exi = ep / cth;
        const Vec3 eeta = n.cross(exi); // (exi, eeta, n) right-handed
        Vec3 vm = Vec3::Zero();
        for (int i = 0; i < k; ++i) vm += V[i];
        vm /= double(k);
        const Vec3 Ac = A + (vm - A).dot(e) * e;
        const double H = n.dot(V[0] - Ac);
        const auto xi_of = [&](int i) { return (V[i] - Ac).dot(exi); };
        const auto eta_of = [&](int i) { return (V[i] - Ac).dot(eeta); };
        const auto zeta_of = [&](double xi) { return xi * sn - H * cth; };
        double ximin = std::numeric_limits<double>::infinity(), ximax = -ximin;
        double zmin = ximin, zmax = -ximin;
        for (int i = 0; i < k; ++i) {
            const double xi = xi_of(i), z = zeta_of(xi);
            ximin = std::min(ximin, xi);
            ximax = std::max(ximax, xi);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
        if (zmin > 0. || zmax < 0.) {
            const double sg = zeta_of(xi_of(0)) > 0 ? 1. : -1.;
            const double zamin = sg > 0 ? zmin : -zmax; // min of sg * zeta, G is even in zeta
            if (std::abs(sn) * (ximax - ximin) < EPS_PAR * zamin) {
                if (cx.st) ++cx.st->line_faces_series;
                double tot = 0.;
                for (int i = 0; i < k; ++i) {
                    const int j = (i + 1) % k;
                    const double xii = xi_of(i), xij = xi_of(j);
                    if (xij != xii) {
                        tot -= edge_near_parallel(
                            eta_of(i),
                            eta_of(j),
                            sg * zeta_of(xii),
                            sg * zeta_of(xij),
                            xij - xii,
                            cx);
                    }
                }
                return tot;
            }
        }
    }
    if (cx.st) ++cx.st->line_faces_radial;
    const Vec3 q = A + (n.dot(V[0] - A) / sn) * e; // line meets the face plane at q
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3& x2 = V[(i + 1) % k];
        const Vec3 d = x2 - x1;
        const double ell = d.norm();
        if (ell == 0.) continue;
        const Vec3 m = (d / ell).cross(n);
        const double h = (x1 - q).dot(m);
        if (h != 0.) {
            if (cx.st) ++cx.st->closed_edges;
            tot += h * seg_line(x1, x2, A, e);
        }
    }
    return tot / 3.;
}

// ------------------------------------------------------------------------------------------------
// Volume integrals over a convex polytope (exact_tri), Euler's theorem: int_Q f = 1/4 sum_F h_F
// int_F f for f homogeneous of degree 1 about o; a face through o (h_F = 0) drops out.
// ------------------------------------------------------------------------------------------------

/// _through: the face plane passes through o up to rounding; its Euler term is rounding noise.
bool through(const double h, const Vec3* V, const int k, const Vec3& o)
{
    double m = 0.;
    for (int i = 0; i < k; ++i) m = std::max(m, maxabs(V[i]));
    return std::abs(h) <= 16 * EPS * (m + maxabs(o));
}

/// int_Q |x - v| dV.
double vol_point(const Poly& Q, const Vec3& v, const Ctx& cx)
{
    double tot = 0.;
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const Vec3& n = Q.N[size_t(f)];
        const double h = n.dot(V[0] - v);
        if (!through(h, V, Q.size(f), v)) tot += h * face_point(V, Q.size(f), n, v, cx);
    }
    return tot / 4.;
}

/// int_Q r dV, Euler centre o on the line, nearest the mean of the ring vertices.
double vol_line(const Poly& Q, const Vec3& A, const Vec3& e, const Ctx& cx)
{
    Vec3 c = Vec3::Zero();
    for (const Vec3& p : Q.P) c += p;
    c /= double(Q.P.size());
    const Vec3 o = A + (c - A).dot(e) * e; // line point nearest the piece
    double tot = 0.;
    for (int f = 0; f < Q.faces(); ++f) {
        const Vec3* V = Q.ring(f);
        const Vec3& n = Q.N[size_t(f)];
        const double h = n.dot(V[0] - o);
        if (!through(h, V, Q.size(f), o)) tot += h * face_line(V, Q.size(f), n, o, e, cx);
    }
    return tot / 4.;
}

/// int_Q |n.(x - a0)| dV: split by the plane, affine on each side.
double vol_plane(const Poly& Q, const Vec3& n, const Vec3& a0, const Ctx& cx)
{
    double tot = 0.;
    Poly tmp;
    for (const double sgn : {1., -1.}) { // keep sgn * n.(x - a0) >= 0
        const Cut c = clip(Q, -sgn * n, -sgn * n.dot(a0), tmp, cx);
        if (c == Cut::None) continue;
        const auto [vol, cen] = volume_centroid(c == Cut::Same ? Q : tmp);
        tot += vol * sgn * n.dot(cen - a0);
    }
    return tot;
}

// ------------------------------------------------------------------------------------------------
// exact_mesh: features in local coordinates, pieces, member sets.
// ------------------------------------------------------------------------------------------------

/// A candidate feature in the cell's local coordinates (origin c0 = the cell's centroid).
struct Loc
{
    int kind; // 0 face, 1 edge, 2 vertex
    Vec3 p, dir; // face: (a0, n); edge: (a, e); vertex: (v, -)
    int h_begin, h_end; // into the local half-spaces
};

double int_g(const Loc& g, const Poly& Q, const Ctx& cx)
{
    if (g.kind == 0) return vol_plane(Q, g.dir, g.p, cx);
    if (g.kind == 1) return vol_line(Q, g.p, g.dir, cx);
    return vol_point(Q, g.p, cx);
}

/// _d_feat: d_g at one point.
double d_feat(const Loc& g, const Vec3& y)
{
    if (g.kind == 0) return std::abs((y - g.p).dot(g.dir));
    if (g.kind == 1) {
        const Vec3 w = y - g.p;
        return (w - w.dot(g.dir) * g.dir).norm();
    }
    return (y - g.p).norm();
}

/// _bounds: (lower, upper) bound of d_g over the convex hull of the points Vq. Upper: the max at
/// the points (d_g convex). Lower: exact for a plane (affine per side), else d_g(c) - max |x_i - c|
/// (1-Lipschitz).
std::pair<double, double> bounds(const Loc& g, const std::vector<Vec3>& Vq)
{
    double up = 0.;
    for (const Vec3& y : Vq) up = std::max(up, d_feat(g, y));
    double lo;
    if (g.kind == 0) {
        double smin = std::numeric_limits<double>::infinity(), smax = -smin, amin = smin;
        for (const Vec3& y : Vq) {
            const double s = (y - g.p).dot(g.dir);
            smin = std::min(smin, s);
            smax = std::max(smax, s);
            amin = std::min(amin, std::abs(s));
        }
        lo = (smin < 0. && 0. < smax) ? 0. : amin;
    } else {
        Vec3 c = Vec3::Zero();
        for (const Vec3& y : Vq) c += y;
        c /= double(Vq.size());
        double r = 0.;
        for (const Vec3& y : Vq) r = std::max(r, (y - c).norm());
        lo = std::max(0., d_feat(g, c) - r);
    }
    return {lo, up};
}

enum class Side { In, Out, Split };

/// _classify: 'in' if the piece lies in all half-spaces, 'out' if outside one, else 'split'.
Side classify(
    const Poly& Q,
    const std::pair<Vec3, double>* H,
    const int nh,
    double* tol_any,
    const double scale)
{
    const double ma = std::max(Q.max_abs(), scale);
    bool s_in = true;
    for (int i = 0; i < nh; ++i) {
        const Vec3& g = H[i].first;
        const double h = H[i].second;
        double smin = std::numeric_limits<double>::infinity(), smax = -smin;
        for (const Vec3& p : Q.V) {
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
    const std::pair<Vec3, double>* H,
    const int nh,
    Poly& R,
    std::vector<Poly>& outside,
    const Ctx& cx)
{
    R = Q;
    Poly tmp;
    for (int i = 0; i < nh; ++i) {
        const Vec3& g = H[i].first;
        const double h = H[i].second;
        const Cut c = clip(R, -g, -h, tmp, cx);
        if (c == Cut::Same) outside.push_back(R);
        if (c == Cut::New) outside.push_back(std::move(tmp));
        if (!clip_inplace(R, g, h, tmp, cx)) break;
    }
}

/// _clip_affine: Q intersected with {w.x <= rhs}, in place; false if empty.
bool clip_affine(Poly& R, const Vec3& w, const double rhs, Poly& tmp, const Ctx& cx)
{
    const double nw = w.norm();
    return clip_inplace(R, w / nw, rhs / nw, tmp, cx);
}

/// _faces_min_pieces: split Q so that min_i |n_i.(x - a_i)| is one affine function W.x - c per
/// sub-piece (split by the planes, each |.| then affine, then by the bisectors of the affine
/// distances); `piece(R, W, c)` for each sub-piece, in the Python's order.
template <typename F>
void faces_min_pieces(const Poly& Q, const std::vector<const Loc*>& faces, const Ctx& cx, F&& piece)
{
    struct Cell
    {
        Poly C;
        std::vector<double> sg;
    };
    std::vector<Cell> cells(1);
    cells[0].C = Q;
    Poly tmp;
    for (const Loc* f : faces) {
        const Vec3& n = f->dir;
        const Vec3& a0 = f->p;
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
    const size_t nf = faces.size();
    std::vector<Vec3> W(nf);
    std::vector<double> cc(nf);
    Poly R;
    for (const Cell& cl : cells) {
        for (size_t i = 0; i < nf; ++i) { // d_i = W_i . x - c_i
            W[i] = cl.sg[i] * faces[i]->dir;
            cc[i] = cl.sg[i] * faces[i]->dir.dot(faces[i]->p);
        }
        for (size_t i = 0; i < nf; ++i) {
            R = cl.C;
            bool alive = true;
            for (size_t j = 0; j < nf && alive; ++j) {
                if (j == i) continue;
                const Vec3 w = W[i] - W[j];
                const double rhs = cc[i] - cc[j];
                if (w.norm() <= 1e-14) { // identical function: lower index wins
                    if (j < i) alive = false;
                    continue;
                }
                alive = clip_affine(R, w, rhs, tmp, cx);
            }
            if (alive) piece(R, W[i], cc[i], int(i), cl.sg[i]);
        }
    }
}

/// _faces_min: exact int_Q min_i |n_i.(x - a_i)| dV.
double faces_min(const Poly& Q, const std::vector<const Loc*>& faces, const Ctx& cx)
{
    double tot = 0.;
    faces_min_pieces(Q, faces, cx, [&](const Poly& R, const Vec3& W, const double c, int, double) {
        const auto [vol, cen] = volume_centroid(R);
        tot += vol * (W.dot(cen) - c);
    });
    return tot;
}

/// _vertices_min_pieces: split Q by the bisector planes of the points; `piece(R, v)` for each
/// sub-piece with its nearest vertex v.
template <typename F>
void vertices_min_pieces(
    const Poly& Q,
    const std::vector<const Loc*>& verts,
    const Ctx& cx,
    F&& piece)
{
    Poly R, tmp;
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec3& vi = verts[i]->p;
        R = Q;
        bool alive = true;
        for (size_t j = 0; j < verts.size() && alive; ++j) {
            if (j == i) continue;
            const Vec3& vj = verts[j]->p;
            alive = clip_affine(R, 2. * (vj - vi), vj.dot(vj) - vi.dot(vi), tmp, cx);
        }
        if (alive) piece(R, vi, int(i));
    }
}

/// _vertices_min: exact int_Q min_i |x - v_i| dV (the bisectors of points are planes).
double vertices_min(const Poly& Q, const std::vector<const Loc*>& verts, const Ctx& cx)
{
    double tot = 0.;
    vertices_min_pieces(Q, verts, cx, [&](const Poly& R, const Vec3& v, int) {
        tot += vol_point(R, v, cx);
    });
    return tot;
}

// ------------------------------------------------------------------------------------------------
// The offset side (exact_mesh: _side_clip, _side_split, _d_bound)
// ------------------------------------------------------------------------------------------------

/// A member's side plane in the cell's frame: unit normal N, a point ql on it; has false: none.
struct SidePlane
{
    bool has = false;
    Vec3 N, ql;
};

/// _side_clip: Q cap {sigma N.(x - q) >= 0}; Same where the plane is none (every point counts).
Cut side_clip(const Poly& Q, const SidePlane& sp, const int sigma, Poly& out, const Ctx& cx)
{
    if (!sp.has) return Cut::Same;
    return clip(Q, -double(sigma) * sp.N, -double(sigma) * sp.N.dot(sp.ql), out, cx);
}

/// _side_split: Q split by the members' planes: parts with, per member, whether the part is on
/// that plane's offset side; the offset-side part of a cut first.
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

/// _d_bound: vol(R) max(0, min over members of (max over R's vertices of d_g) - delta); `mx_all`
/// gets the largest member distance at R's vertices.
double
d_bound(const Poly& R, const std::vector<const Loc*>& members, const double delta, double& mx_all)
{
    double mn = std::numeric_limits<double>::infinity();
    mx_all = 0.;
    for (const Loc* g : members) {
        double mx = 0.;
        for (const Vec3& y : R.P) mx = std::max(mx, d_feat(*g, y));
        mn = std::min(mn, mx);
        mx_all = std::max(mx_all, mx);
    }
    return volume_centroid(R).first * std::max(0., mn - delta);
}

/// _d_derivs: the gradient and Hessian of d_g at v (off the feature).
void d_derivs(const Loc& g, const Vec3& v, Vec3& grad, Eigen::Matrix3d& hess)
{
    if (g.kind == 0) {
        const double s = g.dir.dot(v - g.p);
        grad = (s > 0. ? 1. : (s < 0. ? -1. : 0.)) * g.dir;
        hess.setZero();
        return;
    }
    Vec3 w = v - g.p;
    if (g.kind == 1) w -= w.dot(g.dir) * g.dir;
    const double r = w.norm();
    const Vec3 u = w / r;
    grad = u;
    hess = (Eigen::Matrix3d::Identity() - u * u.transpose()) / r;
    if (g.kind == 1) hess -= g.dir * g.dir.transpose() / r;
}

/// point_tri_dist for one triangle: min over the three segments, and the plane distance when the
/// projection is inside.
double point_tri_dist(const Vec3& p, const Vec3& A, const Vec3& B, const Vec3& C)
{
    double best = std::numeric_limits<double>::infinity();
    for (const auto& [P, Q] : {std::pair{&A, &B}, std::pair{&B, &C}, std::pair{&C, &A}}) {
        const Vec3 d = *Q - *P;
        const double s = std::clamp((p - *P).dot(d) / d.dot(d), 0., 1.);
        best = std::min(best, (p - *P - s * d).norm());
    }
    const Vec3 v0 = B - A, v1 = C - A, w = p - A;
    const double d00 = v0.dot(v0), d01 = v0.dot(v1), d11 = v1.dot(v1);
    const double d20 = w.dot(v0), d21 = w.dot(v1);
    const double den = d00 * d11 - d01 * d01;
    const double l1 = (d11 * d20 - d01 * d21) / den;
    const double l2 = (d00 * d21 - d01 * d20) / den;
    if (l1 >= 0 && l2 >= 0 && l1 + l2 <= 1) {
        const Vec3 n = v0.cross(v1);
        best = std::min(best, std::abs(w.dot(n)) / n.norm());
    }
    return best;
}

/// box_dist: distance between the box [lo, hi] and t's box [tlo, thi].
double box_dist(const Vec3& lo, const Vec3& hi, const Vec3& tlo, const Vec3& thi)
{
    double s = 0.;
    for (int k = 0; k < 3; ++k) {
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

/// The cell's corners and delta in hex floats, bit exact, for the error messages: a failing cell
/// can be replayed offline.
std::string cell_string(const std::array<Vec3, 4>& T, const double delta)
{
    std::string s = "t =";
    char buf[128];
    for (const Vec3& p : T) {
        std::snprintf(buf, sizeof(buf), " (%a, %a, %a)", p[0], p[1], p[2]);
        s += buf;
    }
    std::snprintf(buf, sizeof(buf), ", delta = %a", delta);
    return s + buf;
}

/// _prune_members: drop g if its lower bound on the piece is >= another member's upper bound (it
/// cannot attain the minimum there); changes no value.
std::vector<int>
prune_members(const Poly& Q, const std::vector<int>& S, const std::vector<Loc>& loc)
{
    std::vector<std::pair<double, double>> b;
    b.reserve(S.size());
    for (const int k : S) b.push_back(bounds(loc[size_t(k)], Q.P));
    double ubmin = std::numeric_limits<double>::infinity();
    for (const auto& lu : b) ubmin = std::min(ubmin, lu.second);
    std::vector<int> kept;
    for (size_t i = 0; i < S.size(); ++i) {
        if (b[i].first < ubmin || b[i].second == ubmin) kept.push_back(S[i]);
    }
    return kept;
}

// ================================================================================================
// The exact max of d over a front triangle (exact_mesh.max_on_triangle; the settle loop's max
// test). The triangle is cut by the candidates' local regions exactly as a cell is, a planar ring
// in place of a polytope; every function below is the Python one of the same name. One difference
// from the cell: a ring can lie ON a region's boundary plane (every vertex within tol of it),
// which a solid piece cannot except as a flat sliver. A point of that plane belongs to both closed
// regions, so the ring counts as inside the half-space (pclassify, psplit test the inside first;
// the cell's classify tests the outside first). Duplicated pieces cannot lower a max, lost ones
// can.
// ================================================================================================

using Ring = std::vector<Vec3>;

double ring_max_abs(const Ring& V)
{
    double m = 0.;
    for (const Vec3& p : V) m = std::max(m, maxabs(p));
    return m;
}

/// _pclip: the ring V cut by {g.x <= h}, |g| = 1; Same (V inside up to tol, `out` untouched),
/// None (nothing inside) or New (in `out`): clip()'s rules on one face ring.
Cut pclip(const Ring& V, const Vec3& g, const double h, Ring& out, const Ctx& cx)
{
    const double tol = 16 * EPS * (std::abs(h) + std::max(ring_max_abs(V), cx.scale));
    if (cx.tol_any) *cx.tol_any += tol;
    const int k = int(V.size());
    thread_local std::vector<double> s;
    thread_local Ring P;
    s.resize(size_t(k));
    double smax = -std::numeric_limits<double>::infinity(), smin = -smax;
    for (int i = 0; i < k; ++i) {
        s[size_t(i)] = V[size_t(i)].dot(g) - h;
        smax = std::max(smax, s[size_t(i)]);
        smin = std::min(smin, s[size_t(i)]);
    }
    if (smax <= tol) return Cut::Same;
    if (smin >= -tol) return Cut::None;
    for (int i = 0; i < k; ++i) {
        if (std::abs(s[size_t(i)]) <= tol) s[size_t(i)] = 0.;
    }
    P.clear();
    for (int i = 0; i < k; ++i) {
        const int j = (i + 1) % k;
        const double si = s[size_t(i)], sj = s[size_t(j)];
        if (si <= 0.) P.push_back(V[size_t(i)]);
        if ((si < 0. && 0. < sj) || (sj < 0. && 0. < si)) {
            const int a = si < 0. ? i : j, b = si < 0. ? j : i; // from the inside end
            const double sa = s[size_t(a)], sb = s[size_t(b)];
            P.push_back(V[size_t(a)] + (sa / (sa - sb)) * (V[size_t(b)] - V[size_t(a)]));
        }
    }
    if (cx.tol_cut) *cx.tol_cut += tol;
    return clean_ring(P, tol, out) ? Cut::New : Cut::None;
}

/// R = pclip(R, g, h) in place (_pclip_keep); false (R emptied) if None.
bool pclip_inplace(Ring& R, const Vec3& g, const double h, Ring& tmp, const Ctx& cx)
{
    const Cut c = pclip(R, g, h, tmp, cx);
    if (c == Cut::None) {
        R.clear();
        return false;
    }
    if (c == Cut::New) std::swap(R, tmp);
    return true;
}

/// _pclassify: as classify, the inside tested first.
Side pclassify(const Ring& V, const std::pair<Vec3, double>* H, const int nh, const Ctx& cx)
{
    const double ma = std::max(ring_max_abs(V), cx.scale);
    bool s_in = true;
    for (int i = 0; i < nh; ++i) {
        const Vec3& g = H[i].first;
        const double h = H[i].second;
        double smin = std::numeric_limits<double>::infinity(), smax = -smin;
        for (const Vec3& p : V) {
            const double sv = p.dot(g) - h;
            smin = std::min(smin, sv);
            smax = std::max(smax, sv);
        }
        const double tol = 16 * EPS * (std::abs(h) + ma);
        if (cx.tol_any) *cx.tol_any += tol;
        if (smax <= tol) continue;
        if (smin >= -tol) return Side::Out;
        s_in = false;
    }
    return s_in ? Side::In : Side::Split;
}

/// _psplit: the part of V inside all of H (R; emptied if None) and the parts outside.
void psplit(
    const Ring& V,
    const std::pair<Vec3, double>* H,
    const int nh,
    Ring& R,
    std::vector<Ring>& outside,
    const Ctx& cx)
{
    R = V;
    Ring tmp;
    for (int i = 0; i < nh; ++i) {
        const Vec3& g = H[i].first;
        const double h = H[i].second;
        const double tol = 16 * EPS * (std::abs(h) + std::max(ring_max_abs(R), cx.scale));
        double smax = -std::numeric_limits<double>::infinity();
        for (const Vec3& p : R) smax = std::max(smax, p.dot(g) - h);
        if (smax <= tol) continue; // inside this half-space, on its plane included
        const Cut c = pclip(R, -g, -h, tmp, cx);
        if (c == Cut::Same) outside.push_back(R);
        if (c == Cut::New) outside.push_back(tmp);
        if (!pclip_inplace(R, g, h, tmp, cx)) break;
    }
}

/// _ring_area: the area of the planar ring V, signed along the unit normal n (fan sum).
double ring_area(const Ring& V, const Vec3& n)
{
    double a = 0.;
    for (size_t i = 1; i + 1 < V.size(); ++i) a += n.dot((V[i] - V[0]).cross(V[i + 1] - V[0]));
    return 0.5 * a;
}

/// _pfaces_min_pieces: sub-rings with one affine distance W.x - c each; `piece(R, W, c)`.
template <typename F>
void pfaces_min_pieces(
    const Ring& R0,
    const std::vector<const Loc*>& faces,
    const Ctx& cx,
    F&& piece)
{
    struct Cell
    {
        Ring C;
        std::vector<double> sg;
    };
    std::vector<Cell> cells(1);
    cells[0].C = R0;
    Ring tmp;
    for (const Loc* f : faces) {
        const Vec3& n = f->dir;
        const Vec3& a0 = f->p;
        std::vector<Cell> next;
        for (const Cell& cl : cells) {
            for (const double s : {1., -1.}) { // keep s n.(x - a0) >= 0
                const Cut c = pclip(cl.C, -s * n, -s * n.dot(a0), tmp, cx);
                if (c == Cut::None) continue;
                Cell nc;
                nc.C = c == Cut::Same ? cl.C : tmp;
                nc.sg = cl.sg;
                nc.sg.push_back(s);
                next.push_back(std::move(nc));
            }
        }
        cells = std::move(next);
    }
    const size_t nf = faces.size();
    std::vector<Vec3> W(nf);
    std::vector<double> cc(nf);
    Ring R;
    for (const Cell& cl : cells) {
        for (size_t i = 0; i < nf; ++i) {
            W[i] = cl.sg[i] * faces[i]->dir;
            cc[i] = cl.sg[i] * faces[i]->dir.dot(faces[i]->p);
        }
        for (size_t i = 0; i < nf; ++i) {
            R = cl.C;
            bool alive = true;
            for (size_t j = 0; j < nf && alive; ++j) {
                if (j == i) continue;
                const Vec3 w = W[i] - W[j];
                const double rhs = cc[i] - cc[j];
                if (w.norm() <= 1e-14) { // identical function: lower index wins
                    if (j < i) alive = false;
                    continue;
                }
                const double nw = w.norm();
                alive = pclip_inplace(R, w / nw, rhs / nw, tmp, cx);
            }
            if (alive) piece(R, W[i], cc[i]);
        }
    }
}

/// _pvertices_min_pieces: one nearest vertex per sub-ring; `piece(R, v)`.
template <typename F>
void pvertices_min_pieces(
    const Ring& R0,
    const std::vector<const Loc*>& verts,
    const Ctx& cx,
    F&& piece)
{
    Ring R, tmp;
    for (size_t i = 0; i < verts.size(); ++i) {
        const Vec3& vi = verts[i]->p;
        R = R0;
        bool alive = true;
        for (size_t j = 0; j < verts.size() && alive; ++j) {
            if (j == i) continue;
            const Vec3& vj = verts[j]->p;
            const Vec3 w = 2. * (vj - vi);
            const double nw = w.norm();
            alive = pclip_inplace(R, w / nw, (vj.dot(vj) - vi.dot(vi)) / nw, tmp, cx);
        }
        if (alive) piece(R, vi);
    }
}

/// _prune_points: _prune_members over the points Vq.
std::vector<int>
prune_points(const Ring& Vq, const std::vector<int>& S, const std::vector<Loc>& loc)
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

struct RingPiece
{
    Ring R;
    std::vector<int> S; // member candidates
};

/// The face's corners in hex floats, bit exact, for the error messages (as cell_string).
std::string face_string(const std::array<Vec3, 3>& f)
{
    std::string s = "f =";
    char buf[128];
    for (const Vec3& p : f) {
        std::snprintf(buf, sizeof(buf), " (%a, %a, %a)", p[0], p[1], p[2]);
        s += buf;
    }
    return s;
}

// ================================================================================================
// Exact gradient and Hessian with respect to one vertex x of t (exact_mesh.py, "Exact gradient and
// Hessian"; every function below is the Python one of the same name).
//
// J(x) = int_{t(x)} (d - delta) dV. Every point of t moves with velocity lambda0(y) per unit motion
// of x (lambda0 the barycentric coordinate of x), so by Reynolds' transport theorem, for a piece
// Q(x) = t(x) cap (fixed region) with a smooth integrand phi on Q:
//   grad int_Q phi = sum_F nu_F int_Pi phi lambda0 dA,
// F the three faces of t through x, Pi = Q cap F, nu_F the outward unit normal; and
//   hess int_Q phi = sum_F [ nu_F (x) int_Pi lambda0^2 grad phi dA + S_F int_Pi phi lambda0 dA / |N_F|
//                            - sum_{cut edges c of Pi} nu_F (x) g_c/|g_c,Pi| int_c phi lambda0^2 ds ],
// S_F the matrix of v -> v x (a - b) for F = {alpha x + beta a + gamma b}, N_F = (x - b) x (a - b)
// outward, and a cut edge an edge of Pi on a fixed plane with outward normal g_c: F tilts under
// that plane, so the edge slides over the material points of F. Summed over the pieces, the cut
// terms of two pieces sharing an edge cancel when phi is continuous across it (every exact
// piece); they remain on rule-(a) pieces, whose bound is not. A kink of d lying ON a face of t
// makes J one-sided C^2: the trace of grad phi from inside t is used, i.e. the Hessian for x
// moving into t.
// ================================================================================================

constexpr int K_SEG = 30; ///< Taylor order on short segments (ratio <= SHORT/2 = 1/4)
// Derivative integrals carry polynomial weights, which make the radial branch lose up to ~1e-12 at
// eps_F ~ 0.1-0.4 (Python, measured: radial forced 1.3e-9 max, default 1.5e-12); the series branch
// is therefore used up to EPS_PAR_D, with more terms: per-edge ratio <= EPS_PAR_D/2 = 0.125,
// 0.125^19 ~ 1e-17; rounding growth ((1 + 2/SHORT) EPS_PAR_D / 2)^k = 0.625^k still decays.
constexpr double EPS_PAR_D = 0.25;
constexpr int K_LONG_D = 18;

/// pshift() with cp[m] = std::pow(c, m) given, for P vanishing below j0: the same sums, their
/// zero leading terms left out (as jmul_s()), the same values.
void pshift_cp(const double* P, const int n, const double* cp, const int j0, double* out)
{
    for (int i = 0; i < n; ++i) out[i] = 0.;
    for (int j = j0; j < n; ++j) {
        for (int i = 0; i <= j; ++i) out[i] += P[j] * binom(j, i) * cp[j - i];
    }
}
/// _pshift: coefficients (ascending) of P(tau + c), P of length n.
void pshift(const double* P, const int n, const double c, double* out)
{
    double cp[JCAP];
    for (int m = 0; m < n; ++m) cp[m] = std::pow(c, double(m)); // c ** (j - i)
    pshift_cp(P, n, cp, 0, out);
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

/// The jet of S (power 1) or 1/S (power -1) about a segment's midpoint, S^2 = S2[0] + S2[1] tau' +
/// S2[2] tau'^2 (seg_int()), to length K. Coefficient k depends on coefficients 0..k of S2 only
/// (jsqrt, jdiv are recurrences in k), so the jet computed to length K_SEG + 4, the longest any
/// caller asks for, is in its first K coefficients the jet computed to length K, bit for bit. The
/// last jets of this thread are kept and read back for the same S2: the callers integrate up to
/// six polynomials over each segment (face_line_derivs(): one in its first loop over the edges,
/// five more in the radial branch's; face_point_derivs(): four). MEASURED (Geneva base, inward,
/// the smoothing pass after the split, sampled, the value table and Partition in place):
/// seg_int() was 16% of the busy time, self.
const Jet& seg_jet(const double* S2, const int K, const int power)
{
    constexpr int KMAX = K_SEG + 4;
    struct Entry
    {
        double key[3];
        bool full = false, inv = false;
        Jet S, Si;
    };
    constexpr int kEntries = 16; // a polygon's edges, between the two loops of face_line_derivs()
    thread_local std::array<Entry, kEntries> memo;
    thread_local int next = 0;
    if (K > KMAX) { // longer than any caller asks for: not kept
        thread_local Jet S, Si;
        jsqrt(S2, K, S.data());
        if (power == 1) return S;
        Jet e0{};
        e0[0] = 1.;
        jdiv(e0.data(), S.data(), K, Si.data());
        return Si;
    }
    Entry* e = nullptr;
    for (Entry& x : memo) {
        if (x.full && std::memcmp(x.key, S2, sizeof(x.key)) == 0) {
            e = &x;
            break;
        }
    }
    if (!e) {
        e = &memo[size_t(next)];
        next = (next + 1) % kEntries;
        e->full = false;
        jsqrt(S2, KMAX, e->S.data());
        std::memcpy(e->key, S2, sizeof(e->key));
        e->inv = false;
        e->full = true;
    }
    if (power == 1) return e->S;
    if (!e->inv) {
        Jet e0{};
        e0[0] = 1.;
        jdiv(e0.data(), e->S.data(), KMAX, e->Si.data());
        e->inv = true;
    }
    return e->Si;
}

/// seg_int: int_0^ell P(sigma) S(sigma)^power d sigma, S(sigma) = |w0 + sigma tp|, power = +1 or
/// -1, P ascending in sigma (length np). With power = -1 the caller guarantees P/S is bounded (P
/// vanishes where S does). Short against the distance to the complex zeros of S^2 (ell |tp| <=
/// SHORT S(mid)): Taylor series of the integrand about the midpoint, ratio <= SHORT/2, K_SEG + np
/// terms. Otherwise closed forms in u (S = sqrt(u^2 + k^2)), binomial sums bounded by
/// (1 + 2/SHORT)^deg.
double seg_int(
    const Vec3& w0,
    const Vec3& tp,
    const double ell,
    const double* P,
    const int np,
    const int power)
{
    const Vec3 wm = w0 + 0.5 * ell * tp;
    const double Sm = wm.norm();
    const double sa = tp.norm();
    double Pc[JCAP];
    pshift(P, np, 0.5 * ell, Pc); // in tau = sigma - ell/2
    const auto even_sum = [&]() {
        double s = 0.;
        for (int j = 0; j < np; j += 2) {
            s += Pc[j] * 2 * std::pow(0.5 * ell, double(j + 1)) / double(j + 1);
        }
        return s;
    };
    if (sa <= 1e-14 && Sm <= 1e-14 * ell) {
        // the segment lies on the zero set of S up to rounding (an edge on the input edge line or
        // through the vertex): S <= 2e-14 ell on it; for power -1 the caller's P vanishes as well
        return power == -1 ? 0. : Sm * even_sum();
    }
    if (sa == 0.) return (power == 1 ? Sm : 1. / Sm) * even_sum(); // S constant
    if (Sm > 0 && sa * ell <= SHORT * Sm) {
        // Taylor series in tau' = tau / (ell/2) in [-1, 1] (scaled, so no coefficient overflows)
        const int K = K_SEG + np;
        const double hl = 0.5 * ell;
        Jet S2{}, Ps{}, g;
        S2[0] = Sm * Sm;
        S2[1] = 2 * wm.dot(tp) * hl;
        S2[2] = (sa * hl) * (sa * hl);
        const Jet& S = seg_jet(S2.data(), K, power);
        for (int j = 0; j < np; ++j) Ps[size_t(j)] = Pc[j] * std::pow(hl, double(j));
        for (int k = 0; k < K;
             ++k) { // jmul(Ps, S), Ps vanishing beyond its first np: zeros skipped
            double sk = 0.;
            for (int j = 0; j <= std::min(k, np - 1); ++j) sk += Ps[size_t(j)] * S[size_t(k - j)];
            g[size_t(k)] = sk;
        }
        double sum = 0.;
        for (int i = 0; i < K; i += 2) sum += g[size_t(i)] * 2. / double(i + 1);
        return hl * sum;
    }
    const double um = wm.dot(tp) / sa;
    const double k = wm.cross(tp).norm() / sa;
    const double u1 = um - 0.5 * sa * ell, u2 = um + 0.5 * sa * ell;
    const int deg = np - 1;
    if (power == -1 && k <= 16 * EPS * (std::abs(um) + sa * ell)) {
        // the segment meets the zero of S at u = 0: P/S = Q(tau) sign(u) / sa with P = (tau - tau0) Q
        const double tau0 = -um / sa;
        double out[JCAP];
        double acc = 0.;
        for (int j = deg; j >= 0; --j) { // synthetic division by (tau - tau0), descending
            acc = acc * tau0 + Pc[j];
            out[deg - j] = acc;
        }
        double Q[JCAP]; // quotient, ascending: out[:-1] reversed
        for (int j = 0; j < deg; ++j) Q[j] = out[deg - 1 - j];
        const auto F = [&](const double t) {
            double s = 0.;
            for (int j = 0; j < deg; ++j) s += Q[j] * std::pow(t, double(j + 1)) / double(j + 1);
            return s;
        };
        const double lo = -0.5 * ell, hi = 0.5 * ell;
        const double t0 = std::min(std::max(tau0, lo), hi);
        return ((F(hi) - F(t0)) - (F(t0) - F(lo))) / sa;
    }
    double M1[JCAP], M2[JCAP];
    M_closed(u1, k, deg, power, M1);
    M_closed(u2, k, deg, power, M2);
    double tot = 0.;
    for (int j = 0; j <= deg; ++j) {
        if (Pc[j] == 0.) continue;
        double sj = 0.;
        for (int i = 0; i <= j; ++i) {
            sj += binom(j, i) * std::pow(-um, double(j - i)) * (M2[i] - M1[i]);
        }
        tot += Pc[j] / std::pow(sa, double(j + 1)) * sj;
    }
    return tot;
}

/// face_inv_point: int_F 1/|x - v| dA: radial field (R - |z|)/rho^2 (y - q), divergence 1/R; per
/// edge h [asinh(s2/a) - asinh(s1/a)] + |z| [atan(s c/(h R)) - atan(s/h)] (rationalized).
double face_inv_point(const Vec3* V, const int k, const Vec3& n, const Vec3& v)
{
    const double z0 = n.dot(V[0] - v);
    const double c = std::abs(z0);
    const Vec3 q = v + z0 * n;
    double tot = 0.;
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const double h = (x1 - q).dot(m);
        if (h == 0.) continue;
        const double s1 = (x1 - q).dot(t);
        const double s2 = s1 + ell;
        const double a = std::hypot(h, c);
        const double R1 = std::hypot(s1, a), R2 = std::hypot(s2, a);
        double num, ash;
        if (s1 * s2 > 0.) {
            const double den = s2 * R1 + s1 * R2;
            num = a * a * ell * (s1 + s2) / den;
            ash = std::asinh(ell * (s1 + s2) / den);
        } else {
            num = s2 * R1 - s1 * R2;
            ash = std::asinh(s2 / a) - std::asinh(s1 / a);
        }
        double term = h * ash;
        if (c > 0.) {
            term += c * (std::atan2(c * num * h, h * h * R1 * R2 + s1 * s2 * c * c) -
                         std::atan2(ell * h, h * h + s1 * s2));
        }
        tot += term;
    }
    return tot;
}

/// face_point_derivs: (P1, P2) = (int_F lam R dA, int_F lam^2 grad R dA), R = |y - v|,
/// lam = l0 + g0.y. Uses grad_Pi R = (y - q)/R, (y - q) R = grad_Pi R^3/3 and
/// a.grad[(a.(y - c)) R] = |a|^2 R + (a.(y - c))(a.(y - q))/R, expanded about the face centre c
/// (expanding about the foot q instead loses (|q - F|/diam F)^2).
void face_point_derivs(
    const Vec3* V,
    const int k,
    const Vec3& n,
    const Vec3& v,
    const double l0,
    const Vec3& g0,
    double& P1,
    Vec3& P2,
    const Ctx& cx)
{
    const double z0 = n.dot(V[0] - v);
    const Vec3 q = v + z0 * n;
    Vec3 c = Vec3::Zero();
    for (int i = 0; i < k; ++i) c += V[i];
    c /= double(k);
    const Vec3 a = g0 - g0.dot(n) * n; // in-plane gradient of lam
    const double lc = l0 + g0.dot(c);
    const double IR = face_point(V, k, n, v, cx);
    Vec3 E1 = Vec3::Zero(), E3 = Vec3::Zero(), El = Vec3::Zero();
    double Ea = 0.;
    const double one[1] = {1.};
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const Vec3 w0 = x1 - v;
        const double R2[3] = {w0.dot(w0), 2 * w0.dot(t), 1.}; // |y - v|^2 along the edge
        const double lam[2] = {l0 + g0.dot(x1), g0.dot(t)};
        const double La[2] = {a.dot(x1 - c), a.dot(t)};
        double lam2[3];
        pmul(lam, 2, lam, 2, lam2);
        E1 += m * seg_int(w0, t, ell, one, 1, 1);
        E3 += m * seg_int(w0, t, ell, R2, 3, 1);
        Ea += a.dot(m) * seg_int(w0, t, ell, La, 2, 1);
        El += m * seg_int(w0, t, ell, lam2, 3, 1);
    }
    const double aqc = a.dot(q - c);
    P1 = lc * IR + a.dot(E3) / 3. + aqc * IR;
    double lam2_over_R = 0.;
    if (z0 != 0.) {
        const double I1 = face_inv_point(V, k, n, v);
        const double B = a.dot(E1) + aqc * I1; // int a.(y - c) / R
        const double C = (Ea - a.dot(a) * IR) + aqc * B; // int (a.(y - c))^2 / R
        lam2_over_R = lc * lc * I1 + 2 * lc * B + C;
    }
    P2 = El - 2 * a * P1 + z0 * n * lam2_over_R;
}

/// _jXi: jets of Xi_m, m = 0..mmax, antiderivatives in eta at fixed zeta > 0:
/// kind 0 ('r'):  Xi_m = int eta^m r d eta,         Xi_0 = int_0^eta r = G;
/// kind 1 ('zr'): Xi_m = zeta int eta^m / r d eta,  Xi_0 = zeta asinh(eta/zeta);
/// r = sqrt(eta^2 + zeta^2). Recursions from d(eta^(m-1) r^3) and d(eta^(m-1) r). Xi has room for
/// max(mmax + 1, 2) jets of length n. eta and zeta are linear (every caller's: e0 + e1 l), so the
/// products with them, with zeta^2 and the quotient by zeta skip their zero coefficients (jmul3,
/// jdiv2): the same sums in the same order, the same values (MEASURED: bit-identical on every
/// stored band cell).
constexpr int MAXXI = 2 + K_LONG_D + 2 + 3; ///< mmax + 1 <= Aa + (K_LONG_D + degw) + 2 = 25
///
/// The jets u = eta / zeta, asinh(u), w = sqrt(1 + u^2), R = zeta w and zeta^2 (the base) depend
/// on eta and zeta alone, not on kind or mmax, and coefficient k of each on coefficients 0..k of
/// eta and zeta only (jmul, jsqrt, jdiv, jdiv2, jmul3 are recurrences in k, the same operations
/// for every length): so a base computed to length n' >= n is, in its first n coefficients, the
/// base computed to length n, bit for bit. jXi() is likewise a function of (eta, zeta, n, mmax,
/// kind) alone. The bases and the results with mmax <= 2 are kept per thread, in tables keyed by
/// the linear jets' four coefficients (a slot per hash of the inputs, taken over by any other
/// inputs), and read back for the same inputs: a value is the one the evaluation would return
/// again, a lost slot costs an evaluation. The base is computed to the length the caller names
/// (edge_np_w(): KB, the longest any weight asks for), so the weights of one edge share it.
/// Why: edge_np_w() calls jXi() for the same jets with both weights of an edge
/// (face_line_derivs(), its near-parallel branch), and the front form evaluates the same edges
/// twice at one x: its value (flux(), weight L) and then its derivatives (front_face_derivs(),
/// both weights), whose kind-0 jXi() is the value's. MEASURED (Geneva base, inward, the smoothing
/// pass after the split, sampled, the value table and Partition in place): jXi() was 25% of the
/// busy time, 16% in the base's jasinh_w(); (cube_out band, the bench's front_flux evaluations,
/// sampled, after jmul_s()): jasinh_w() 34% and jXi() 30%, self, with a table of the last 4 bases.
constexpr int KB = K_SHORT + 3; ///< edge_np_w(): K_SHORT + 1 + deg W, deg W <= 2
constexpr size_t kJetSlots = 256;
struct JetBase
{
    double key[4] = {}; ///< eta[0], eta[1], zeta[0], zeta[1]
    int n = 0; ///< length computed; 0: unset
    double u[KB], A[KB], w[KB], R[KB], z2[KB];
};
struct XiSlot
{
    double key[4] = {};
    int n = 0, mmax = -1, kind = -1; ///< n = 0: unset
    double Xi[3][KB];
};
/// The slot of the linear jets eta, zeta (and of n, mmax, kind for jXi()'s table).
size_t jet_slot(const double* key, const int extra)
{
    uint64_t h = 0x9e3779b97f4a7c15ull ^ uint64_t(extra);
    for (int i = 0; i < 4; ++i) {
        uint64_t b;
        std::memcpy(&b, key + i, sizeof(b));
        h = (h ^ b) * 0xff51afd7ed558ccdull;
        h ^= h >> 32;
    }
    return size_t(h % kJetSlots);
}
/// The base of eta, zeta to length >= n (computed to nb >= n when not kept); eta and zeta linear.
const JetBase&
jet_base(const double* key, const double* eta, const double* zeta, const int n, const int nb)
{
    thread_local std::vector<JetBase> bases(kJetSlots);
    JetBase& b = bases[jet_slot(key, 0)];
    if (b.n >= n && std::memcmp(b.key, key, sizeof(b.key)) == 0) return b;
    b.n = 0;
    std::memcpy(b.key, key, sizeof(b.key));
    jdiv2(eta, zeta, nb, b.u);
    jasinh_w(b.u, nb, b.A, b.w);
    jmul3(zeta, b.w, nb, b.R);
    jmul3(zeta, zeta, nb, b.z2);
    b.n = nb;
    return b;
}
/// jXi() as described above; nb: the base's length when it is computed (>= n, <= KB).
void jXi(
    const double* eta,
    const double* zeta,
    const int n,
    const int nb,
    const int mmax,
    const int kind,
    Jet* Xi)
{
    // eta and zeta are linear (jXi() above): beyond their first two coefficients, zeros
    const double key[4] = {eta[0], eta[1], zeta[0], zeta[1]};
    XiSlot* slot = nullptr;
    if (mmax <= 2 && n <= KB) {
        thread_local std::vector<XiSlot> table(kJetSlots);
        slot = &table[jet_slot(key, 1 + n + 64 * (mmax + 4 * kind))];
        if (slot->n == n && slot->mmax == mmax && slot->kind == kind &&
            std::memcmp(slot->key, key, sizeof(key)) == 0) {
            for (int m = 0; m <= mmax; ++m)
                std::memcpy(Xi[m].data(), slot->Xi[m], size_t(n) * sizeof(double));
            return;
        }
        slot->n = 0;
    }
    Jet t, t2;
    const JetBase& base = jet_base(key, eta, zeta, n, nb);
    const double *u = base.u, *A = base.A, *w = base.w, *R = base.R, *z2 = base.z2;
    const int ne = std::max(0, mmax - 1); // ep[m] = eta^m, m = 0..ne
    std::array<Jet, MAXXI> ep;
    ep[0].fill(0.);
    ep[0][0] = 1.;
    for (int m = 1; m <= ne; ++m) jmul3(eta, ep[size_t(m - 1)].data(), n, ep[size_t(m)].data());
    // the products by jmul_s(), zero terms skipped: the same values (jmul_s()); eta^m has at most
    // m (le - 1) + 1 terms, le = eta's
    const auto len = [n](const double* a) { return jlen(a, n); };
    const int le = jlen(eta, 2);
    const auto len_ep = [&](const int m) { return std::clamp(m * (le - 1) + 1, 0, n); };
    const int lR = len(R);
    if (kind == 0) {
        Jet R2, R3;
        jmul_s(R, lR, R, lR, n, R2.data());
        jmul_s(R, lR, R2.data(), len(R2.data()), n, R3.data());
        jmul_s(u, len(u), w, len(w), n, t.data());
        for (int i = 0; i < n; ++i) t[size_t(i)] += A[i];
        jmul3(z2, t.data(), n, Xi[0].data());
        for (int i = 0; i < n; ++i) Xi[0][size_t(i)] *= 0.5;
        for (int i = 0; i < n; ++i) Xi[1][size_t(i)] = R3[size_t(i)] / 3.;
        const int lR3 = len(R3.data());
        for (int m = 2; m <= mmax; ++m) {
            jmul_s(ep[size_t(m - 1)].data(), len_ep(m - 1), R3.data(), lR3, n, t.data());
            jmul3(z2, Xi[m - 2].data(), n, t2.data());
            for (int i = 0; i < n; ++i) {
                Xi[m][size_t(i)] = (t[size_t(i)] - (m - 1) * t2[size_t(i)]) / (m + 2);
            }
        }
    } else {
        std::array<Jet, MAXXI> Y;
        std::memcpy(Y[0].data(), A, size_t(n) * sizeof(double));
        std::memcpy(Y[1].data(), R, size_t(n) * sizeof(double));
        for (int m = 2; m <= mmax; ++m) {
            jmul_s(ep[size_t(m - 1)].data(), len_ep(m - 1), R, lR, n, t.data());
            jmul3(z2, Y[size_t(m - 2)].data(), n, t2.data());
            for (int i = 0; i < n; ++i) {
                Y[size_t(m)][size_t(i)] = (t[size_t(i)] - (m - 1) * t2[size_t(i)]) / m;
            }
        }
        for (int m = 0; m <= mmax; ++m) jmul3(zeta, Y[size_t(m)].data(), n, Xi[m].data());
    }
    if (slot) {
        for (int m = 0; m <= mmax; ++m)
            std::memcpy(slot->Xi[m], Xi[m].data(), size_t(n) * sizeof(double));
        std::memcpy(slot->key, key, sizeof(key));
        slot->mmax = mmax;
        slot->kind = kind;
        slot->n = n;
    }
}

/// The weights of _edge_np_w: W[a] ascending coefficients in xi, a = 0..na-1, each of length len[a].
struct Weights
{
    int na = 0;
    int len[3] = {0, 0, 0};
    double c[3][3] = {};
};

/// _edge_np_w: int_{xi1}^{xi2} sum_a W[a](xi) Phi_a(eta(xi), zeta(xi)) d xi along one edge, with
/// Phi_a = int_0^eta t^a Psi(t, zeta) dt, Psi = r (kind 0) or zeta / r (kind 1), zeta > 0. The same
/// two paths as edge_near_parallel(), at the fixed orders (K_SHORT + 1 + deg W, K_LONG_D + 1).
double edge_np_w(
    const double e1,
    const double e2,
    const double z1,
    const double z2,
    const double x1,
    const double x2,
    const Weights& W,
    const int kind)
{
    const double em = 0.5 * (e1 + e2), zm = 0.5 * (z1 + z2), xm = 0.5 * (x1 + x2);
    const double de = e2 - e1, dz = z2 - z1, dx = x2 - x1;
    const int Aa = W.na - 1;
    int degw = 0;
    for (int a = 0; a < W.na; ++a) degw = std::max(degw, W.len[a] - 1);
    if (std::hypot(de, dz) <= SHORT * std::hypot(em, zm)) {
        const int K = K_SHORT + 1 + degw;
        Jet ej{}, zj{}, xj{}, zero{};
        ej[0] = em;
        ej[1] = de;
        zj[0] = zm;
        zj[1] = dz;
        xj[0] = xm;
        xj[1] = dx;
        std::array<Jet, 3> Xi, Xi0;
        jXi(ej.data(), zj.data(), K, KB, Aa, kind, Xi.data());
        jXi(zero.data(), zj.data(), K, KB, Aa, kind, Xi0.data());
        std::array<Jet, 3> xp; // _jpow(xj, degw)
        xp[0].fill(0.);
        xp[0][0] = 1.;
        // the products by jmul_s(), zero terms skipped (xj and wj are polynomials of degree
        // <= 2, wj of W.len[a] terms): the same values
        for (int m = 1; m <= degw; ++m) {
            jmul_s(xp[size_t(m - 1)].data(), m, xj.data(), 2, K, xp[size_t(m)].data());
        }
        Jet g{}, wj, diff, t;
        for (int a = 0; a <= Aa; ++a) {
            wj.fill(0.);
            for (int j = 0; j < W.len[a]; ++j) {
                for (int i = 0; i < K; ++i) wj[size_t(i)] += W.c[a][j] * xp[size_t(j)][size_t(i)];
            }
            for (int i = 0; i < K; ++i)
                diff[size_t(i)] = Xi[size_t(a)][size_t(i)] - Xi0[size_t(a)][size_t(i)];
            jmul_s(wj.data(), W.len[a], diff.data(), jlen(diff.data(), K), K, t.data());
            for (int i = 0; i < K; ++i) g[size_t(i)] += t[size_t(i)];
        }
        double sum = 0., p = 1.;
        for (int i = 0; i < K; i += 2) {
            sum += g[size_t(i)] * p / double(i + 1);
            p *= 0.25; // 0.5^i, exact
        }
        return dx * sum;
    }
    const int K = K_LONG_D + 1;
    const double mu = dz / de, nu = dx / de;
    Jet zj{}, zero{};
    zj[0] = zm;
    zj[1] = 1.;
    const int Lmax = (K - 1) + degw;
    const int mmax = Aa + Lmax + 1;
    std::array<std::array<Jet, MAXXI>, 2> XiE;
    for (int s = 0; s < 2; ++s) {
        Jet eta{};
        eta[0] = s == 0 ? e1 : e2;
        jXi(eta.data(), zj.data(), K + 1, K + 1, mmax, kind, XiE[size_t(s)].data());
    }
    std::array<Jet, 3> Xi0;
    jXi(zero.data(), zj.data(), K + 1, K + 1, Aa, kind, Xi0.data());
    double tot = 0.;
    double Q[JCAP], c[JCAP];
    // the powers the loops below take, each std::pow() call made once (the same values)
    const int nqmax = K - 1 + degw + 1;
    double cp[JCAP], pe1[JCAP], pe2[JCAP], pmu[JCAP];
    for (int m = 0; m < nqmax; ++m) {
        cp[m] = std::pow(-em, double(m));
        pe1[m] = std::pow(e1, double(m + 1));
        pe2[m] = std::pow(e2, double(m + 1));
    }
    for (int k = 0; k < K; ++k) pmu[k] = std::pow(mu, double(k));
    for (int a = 0; a <= Aa; ++a) {
        double wt[3];
        pshift(W.c[a], W.len[a], xm, wt); // W[a](xm + nu tau)
        for (int j = 0; j < W.len[a]; ++j) wt[j] *= std::pow(nu, double(j));
        for (int k = 0; k < K; ++k) {
            const int nq = k + W.len[a]; // tau^k W(...)
            for (int i = 0; i < k; ++i) Q[i] = 0.;
            for (int j = 0; j < W.len[a]; ++j) Q[k + j] = wt[j];
            pshift_cp(Q, nq, cp, k, c); // in powers of eta
            double sk = 0.;
            for (int l = 0; l < nq; ++l) {
                const double cl = c[l];
                if (cl == 0.) continue;
                const double p1 = pe1[l], p2 = pe2[l];
                const double F0 =
                    (p1 * XiE[0][size_t(a)][size_t(k)] - XiE[0][size_t(a + l + 1)][size_t(k)]) /
                    (l + 1);
                const double F1 =
                    (p2 * XiE[1][size_t(a)][size_t(k)] - XiE[1][size_t(a + l + 1)][size_t(k)]) /
                    (l + 1);
                sk += cl * (F1 - F0 - Xi0[size_t(a)][size_t(k)] * (p2 - p1) / (l + 1));
            }
            tot += pmu[k] * sk;
        }
    }
    return dx / de * tot;
}

/// _poly_polygon: int over the polygon V of a quadratic polynomial f, exact: fan triangles,
/// edge-midpoint rule (exact for degree <= 2).
template <typename F>
double poly_polygon(const Vec3* V, const int k, F&& f)
{
    double tot = 0.;
    for (int i = 1; i < k - 1; ++i) {
        const Vec3 &a = V[0], &b = V[i], &c = V[i + 1];
        const double area = 0.5 * (b - a).cross(c - a).norm();
        tot = tot +
              area * ((f(Vec3((a + b) / 2)) + f(Vec3((b + c) / 2)) + f(Vec3((c + a) / 2))) / 3.);
    }
    return tot;
}

/// _split_polygon: the polygon V cap {g.x <= h} (2D clip in its plane); false if fewer than 3
/// vertices remain.
bool split_polygon(
    const Vec3* V,
    const int k,
    const Vec3& g,
    const double h,
    std::vector<Vec3>& out)
{
    out.clear();
    thread_local std::vector<double> sv;
    sv.resize(size_t(k));
    double* sp = sv.data();
    for (int i = 0; i < k; ++i) sp[i] = V[i].dot(g) - h;
    for (int i = 0; i < k; ++i) {
        const int j = (i + 1) % k;
        if (sp[i] <= 0) out.push_back(V[i]);
        if ((sp[i] < 0 && 0 < sp[j]) || (sp[j] < 0 && 0 < sp[i])) {
            out.push_back(V[i] + (sp[i] / (sp[i] - sp[j])) * (V[j] - V[i]));
        }
    }
    return out.size() >= 3;
}

/// face_line_derivs: (P1, P2) = (int_F lam r dA, int_F lam^2 grad r dA), r = distance to the line
/// {A + l e}; the same three branches as face_line(), the series up to EPS_PAR_D. want_P2 false:
/// P1 alone (P2 left 0), the same operations for P1, so the same P1 bit for bit (the front form's
/// edge flux, which needs P1 only).
void face_line_derivs(
    const Vec3* V,
    const int k,
    const Vec3& n,
    const Vec3& A,
    const Vec3& e,
    const double l0,
    const Vec3& g0,
    double& P1,
    Vec3& P2,
    const bool want_P2 = true)
{
    P2.setZero();
    const double sn = e.dot(n);
    const Vec3 a = g0 - g0.dot(n) * n;
    const auto lamf = [&](const Vec3& Y) { return l0 + Y.dot(g0); };
    double vmax = 0.;
    for (int i = 0; i < k; ++i) vmax = std::max(vmax, maxabs(V[i]));
    const double scale = vmax + maxabs(A);
    Vec3 inplane_edges = Vec3::Zero();
    for (int i = 0; i < k && want_P2; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        Vec3 w0 = x1 - A;
        w0 = w0 - w0.dot(e) * e;
        const Vec3 tp = t - t.dot(e) * e;
        const double lam[2] = {l0 + g0.dot(x1), g0.dot(t)};
        double lam2[3];
        pmul(lam, 2, lam, 2, lam2);
        inplane_edges += t.cross(n) * seg_int(w0, tp, ell, lam2, 3, 1);
    }
    if (std::abs(sn) <= 16 * EPS && std::abs(n.dot(V[0] - A)) <= 16 * EPS * scale) {
        // the face plane contains the line: r = |eta|, grad r = sign(eta) e_eta in the plane
        Vec3 eh = n.cross(e);
        eh /= eh.norm();
        P1 = 0.;
        P2.setZero();
        std::vector<Vec3> Vs;
        for (const double sg : {1., -1.}) {
            if (split_polygon(V, k, -sg * eh, -sg * eh.dot(A), Vs)) {
                P1 += poly_polygon(Vs.data(), int(Vs.size()), [&](const Vec3& Y) {
                    return lamf(Y) * sg * ((Y - A).dot(eh));
                });
                if (!want_P2) continue;
                P2 = P2 + sg * eh * poly_polygon(Vs.data(), int(Vs.size()), [&](const Vec3& Y) {
                              const double l = lamf(Y);
                              return l * l;
                          });
            }
        }
        return;
    }
    if (std::abs(sn) < 0.5) {
        const Vec3 ep = e - sn * n;
        const double cth = ep.norm();
        const Vec3 exi = ep / cth;
        const Vec3 eeta = n.cross(exi);
        Vec3 vm = Vec3::Zero();
        for (int i = 0; i < k; ++i) vm += V[i];
        vm /= double(k);
        const Vec3 Ac = A + (vm - A).dot(e) * e;
        const double H = n.dot(V[0] - Ac);
        const auto xi_of = [&](int i) { return (V[i] - Ac).dot(exi); };
        const auto eta_of = [&](int i) { return (V[i] - Ac).dot(eeta); };
        const auto zeta_of = [&](double xi) { return xi * sn - H * cth; };
        double ximin = std::numeric_limits<double>::infinity(), ximax = -ximin;
        double zmin = ximin, zmax = -ximin;
        for (int i = 0; i < k; ++i) {
            const double xi = xi_of(i), z = zeta_of(xi);
            ximin = std::min(ximin, xi);
            ximax = std::max(ximax, xi);
            zmin = std::min(zmin, z);
            zmax = std::max(zmax, z);
        }
        if (zmin > 0. || zmax < 0.) {
            const double sg = zeta_of(xi_of(0)) > 0 ? 1. : -1.;
            const double zamin = sg > 0 ? zmin : -zmax;
            if (std::abs(sn) * (ximax - ximin) < EPS_PAR_D * zamin) {
                const double lA = l0 + g0.dot(Ac) + H * g0.dot(n);
                const double le = g0.dot(eeta), lx = g0.dot(exi);
                Weights W1, W2;
                W1.na = 2;
                W1.len[0] = 2;
                W1.c[0][0] = lA;
                W1.c[0][1] = lx;
                W1.len[1] = 1;
                W1.c[1][0] = le;
                W2.na = 3;
                W2.len[0] = 3;
                W2.c[0][0] = lA * lA;
                W2.c[0][1] = 2 * lA * lx;
                W2.c[0][2] = lx * lx;
                W2.len[1] = 2;
                W2.c[1][0] = 2 * le * lA;
                W2.c[1][1] = 2 * le * lx;
                W2.len[2] = 1;
                W2.c[2][0] = le * le;
                P1 = 0.;
                double Z = 0.;
                for (int i = 0; i < k; ++i) {
                    const int j = (i + 1) % k;
                    const double xii = xi_of(i), xij = xi_of(j);
                    if (xij != xii) {
                        const double ei = eta_of(i), ej = eta_of(j);
                        const double zi = sg * zeta_of(xii), zj = sg * zeta_of(xij);
                        // W2 first: its series is one longer, so W1's reads its jet bases
                        // (jet_base())
                        if (want_P2) Z -= edge_np_w(ei, ej, zi, zj, xii, xij, W2, 1);
                        P1 -= edge_np_w(ei, ej, zi, zj, xii, xij, W1, 0);
                    }
                }
                // n.w = -cos(theta) zeta on the plane; Z = int lam^2 za / r, zeta = sg za
                if (want_P2) P2 = inplane_edges - 2 * a * P1 + n * (-cth * sg * Z);
                return;
            }
        }
    }
    // radial branch: r is homogeneous of degree 1 about q in the face plane
    const Vec3 q = A + (n.dot(V[0] - A) / sn) * e;
    const double lq = l0 + g0.dot(q);
    P1 = 0.;
    double Pn = 0.;
    const double one[1] = {1.};
    for (int i = 0; i < k; ++i) {
        const Vec3& x1 = V[i];
        const Vec3 d = V[(i + 1) % k] - x1;
        const double ell = d.norm();
        if (!(ell > 0.)) continue;
        const Vec3 t = d / ell;
        const Vec3 m = t.cross(n);
        const double h = (x1 - q).dot(m);
        if (h == 0.) continue;
        Vec3 w0 = x1 - A;
        w0 = w0 - w0.dot(e) * e;
        const Vec3 tp = t - t.dot(e) * e;
        const double L1[2] = {a.dot(x1 - q), a.dot(t)}; // a.(y - q) along the edge
        const double nw[2] = {-sn * e.dot(x1 - q), -sn * e.dot(t)}; // n.w along the edge
        double L1nw[3], L1L1[3], L1L1nw[4];
        pmul(L1, 2, nw, 2, L1nw);
        pmul(L1, 2, L1, 2, L1L1);
        pmul(L1L1, 3, nw, 2, L1L1nw);
        P1 += h * (lq * seg_int(w0, tp, ell, one, 1, 1) / 3. + seg_int(w0, tp, ell, L1, 2, 1) / 4.);
        if (!want_P2) continue;
        Pn += h * (lq * lq * seg_int(w0, tp, ell, nw, 2, -1) / 2. +
                   2 * lq * seg_int(w0, tp, ell, L1nw, 3, -1) / 3. +
                   seg_int(w0, tp, ell, L1L1nw, 4, -1) / 4.);
    }
    if (want_P2) P2 = inplane_edges - 2 * a * P1 + n * Pn;
}

/// An elementary piece (exact_mesh.elementary): a convex piece on which the integrand is ONE
/// smooth function phi: kind 0 affine phi = W.y - c (W in `dir`), 1 the distance to the line
/// {p + l dir}, 2 the distance to the point p. `rule_a`: the member of smallest integral on a
/// rule-(a) piece (a bound).
struct Elem
{
    Poly Q;
    int kind = 0;
    Vec3 p, dir;
    double c = 0.;
    bool rule_a = false;
};

/// _piece_value
double piece_value(const Elem& e, const Ctx& cx)
{
    if (e.kind == 0) {
        const auto [vol, cen] = volume_centroid(e.Q);
        return vol * (e.dir.dot(cen) - e.c);
    }
    if (e.kind == 1) return vol_line(e.Q, e.p, e.dir, cx);
    return vol_point(e.Q, e.p, cx);
}

/// _moments: (P1, P2) on the trace polygon Vf of a piece: int lam phi, int lam^2 grad phi.
void moments(
    const Vec3* Vf,
    const int k,
    const Vec3& nf,
    const Elem& e,
    const double l0,
    const Vec3& g0,
    double& P1,
    Vec3& P2,
    const Ctx& cx)
{
    const auto lam = [&](const Vec3& Y) { return l0 + Y.dot(g0); };
    if (e.kind == 0) {
        P1 = poly_polygon(Vf, k, [&](const Vec3& Y) { return lam(Y) * (Y.dot(e.dir) - e.c); });
        P2 = e.dir * poly_polygon(Vf, k, [&](const Vec3& Y) {
                 const double l = lam(Y);
                 return l * l;
             });
        return;
    }
    if (e.kind == 1) {
        face_line_derivs(Vf, k, nf, e.p, e.dir, l0, g0, P1, P2);
        return;
    }
    face_point_derivs(Vf, k, nf, e.p, l0, g0, P1, P2, cx);
}

/// _cut_integral: int over the segment [y1, y2] of phi lam^2.
double cut_integral(const Vec3& y1, const Vec3& y2, const Elem& e, const double l0, const Vec3& g0)
{
    if (e.kind == 0) {
        // _seg_poly3: Simpson, exact for degree <= 3
        const auto f = [&](const Vec3& y) {
            const double l = l0 + g0.dot(y);
            return (e.dir.dot(y) - e.c) * (l * l);
        };
        const double ell = (y2 - y1).norm();
        return ell * (f(y1) + 4 * f(Vec3(0.5 * (y1 + y2))) + f(y2)) / 6.;
    }
    const Vec3 d = y2 - y1;
    const double ell = d.norm();
    const Vec3 t = d / ell;
    const double lam[2] = {l0 + g0.dot(y1), g0.dot(t)};
    double lam2[3];
    pmul(lam, 2, lam, 2, lam2);
    if (e.kind == 1) {
        const Vec3 w0 = y1 - e.p;
        return seg_int(w0 - w0.dot(e.dir) * e.dir, t - t.dot(e.dir) * e.dir, ell, lam2, 3, 1);
    }
    return seg_int(y1 - e.p, t, ell, lam2, 3, 1);
}


// ================================================================================================
// The front form (EXPERIMENTAL_band_volume_rule "front_flux"; ExactBandIntegral.hpp). Every
// function below is the Python prototype's function of the same name (front_form.py,
// d_vertex.py, e_hessian.py, scratchpad front_form/ of 2026-10-13), on the partition, closed
// forms and clips above.
// ================================================================================================

/// area_centroid: signed area (along n) and centroid of the planar polygon V (fan from V[0]).
std::pair<double, Vec3> area_centroid(const Vec3* V, const int k, const Vec3& n)
{
    const Vec3& o = V[0];
    double A = 0.;
    Vec3 m = Vec3::Zero();
    for (int i = 1; i + 1 < k; ++i) {
        const double a = 0.5 * n.dot((V[i] - o).cross(V[i + 1] - o));
        A += a;
        m += a * (o + V[i] + V[i + 1]) / 3.;
    }
    if (A != 0.) return {A, m / A};
    Vec3 c = Vec3::Zero();
    for (int i = 0; i < k; ++i) c += V[i];
    return {A, c / double(k)};
}

/// A labelled part's field (labeled()): kind 0 affine d = dir.x - c, 1 the distance to the line
/// {p + l dir}, 2 the distance to the point p. `far`: an affine part of a face whose far side
/// was dropped (the offset side), so its edges on the face's plane are cut edges (front_face()).
struct Field
{
    int kind = 0;
    Vec3 p = Vec3::Zero(), dir = Vec3::Zero();
    double c = 0.;
    bool far = false;
};

/// flux: int_V G.n dA, V a convex polygon CCW about the unit normal n (flux_face, flux_edge,
/// flux_vertex):
///  - face:   G.n = (d^2/2 - delta d)(W.n), quadratic: poly_polygon(), exact;
///  - edge:   G.n = (r/3 - delta/2) L, L(x) = n.(x - x_e) = g.(x - a), g = n - (n.e) e affine:
///            (1/3) int L r dA (face_line_derivs()' P1, lambda = L) - (delta/2) area L(centroid);
///  - vertex: G.n = (|x - v|/4 - delta/3) h, h = n.(x - v) constant on the plane:
///            h (face_point() / 4 - delta area / 3).
/// MEASURED (Python, test_A.py): each within 1e-12 of an adaptive quadrature on random polygons.
double
flux(const Field& fd, const Vec3* V, const int k, const Vec3& n, const double delta, const Ctx& cx)
{
    if (fd.kind == 0) {
        const double wn = fd.dir.dot(n);
        if (wn == 0.) return 0.;
        return wn * poly_polygon(V, k, [&](const Vec3& Y) {
                   const double d = Y.dot(fd.dir) - fd.c;
                   return 0.5 * d * d - delta * d;
               });
    }
    if (fd.kind == 1) {
        const Vec3& a = fd.p;
        const Vec3& e = fd.dir;
        const Vec3 g = n - n.dot(e) * e;
        const double l0 = -g.dot(a);
        double P1;
        Vec3 P2;
        face_line_derivs(V, k, n, a, e, l0, g, P1, P2, false);
        const auto [A, cen] = area_centroid(V, k, n);
        return P1 / 3. - 0.5 * delta * A * (l0 + g.dot(cen));
    }
    const double h = n.dot(V[0] - fd.p);
    if (h == 0.) return 0.;
    const double A = area_centroid(V, k, n).first;
    return h * (face_point(V, k, n, fd.p, cx) / 4. - delta * A / 3.);
}

/// A squared distance as a quadratic q(x) = x.A x + b.x + c (quad_of).
struct Quad
{
    Eigen::Matrix3d A;
    Vec3 b;
    double c = 0.;
};

Quad quad_of(const Loc& g)
{
    Quad q;
    if (g.kind == 0) {
        const double s = g.dir.dot(g.p);
        q.A = g.dir * g.dir.transpose();
        q.b = -2. * s * g.dir;
        q.c = s * s;
    } else if (g.kind == 1) {
        const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - g.dir * g.dir.transpose();
        q.A = P;
        q.b = -2. * (P * g.p);
        q.c = g.p.dot(P * g.p);
    } else {
        q.A = Eigen::Matrix3d::Identity();
        q.b = -2. * g.p;
        q.c = g.p.dot(g.p);
    }
    return q;
}

/// ring_candidates: the points of the ring V (CCW about n) where a quadratic's extremes over it
/// can lie, appended to pts: the vertices, each edge's stationary point inside it, and the
/// stationary point in the plane when it lies in the ring.
void ring_candidates(
    const Quad& q,
    const Vec3* V,
    const int k,
    const Vec3& n,
    std::vector<Vec3>& pts)
{
    for (int i = 0; i < k; ++i) pts.push_back(V[i]);
    for (int i = 0; i < k; ++i) {
        const Vec3& p = V[i];
        const Vec3 u = V[(i + 1) % k] - p;
        const Vec3 Au = q.A * u;
        const double al = u.dot(Au);
        const double be = 2. * p.dot(Au) + q.b.dot(u);
        if (al != 0.) {
            const double t = -be / (2. * al);
            if (0. < t && t < 1.) pts.push_back(p + t * u);
        }
    }
    const auto [u, w] = plane_basis(n);
    Eigen::Matrix<double, 3, 2> U;
    U.col(0) = u;
    U.col(1) = w;
    Vec3 p0 = Vec3::Zero();
    for (int i = 0; i < k; ++i) p0 += V[i];
    p0 /= double(k);
    const Eigen::Matrix2d Ar = U.transpose() * q.A * U;
    const Eigen::Vector2d br = U.transpose() * (2. * (q.A * p0) + q.b);
    const double sc = std::max(1., Ar.cwiseAbs().maxCoeff());
    if (!(std::abs(Ar.determinant()) > 1e-12 * sc * sc)) return;
    const Eigen::Vector2d z = (2. * Ar).partialPivLu().solve(-br);
    thread_local std::vector<Eigen::Vector2d> Z;
    Z.resize(size_t(k));
    for (int i = 0; i < k; ++i) Z[size_t(i)] = U.transpose() * (V[i] - p0);
    double a2 = 0.;
    for (int i = 0; i < k; ++i) {
        const Eigen::Vector2d &a = Z[size_t(i)], &b = Z[size_t((i + 1) % k)];
        a2 += a[0] * b[1] - b[0] * a[1];
    }
    if (a2 < 0.) std::reverse(Z.begin(), Z.end());
    for (int i = 0; i < k; ++i) { // _in_ring, tol 0
        const Eigen::Vector2d &p = Z[size_t(i)], &r = Z[size_t((i + 1) % k)];
        if ((r[0] - p[0]) * (z[1] - p[1]) - (r[1] - p[1]) * (z[0] - p[0]) < 0.) return;
    }
    pts.push_back(p0 + U * z);
}

/// poly_candidates: ring_candidates() of every face, and the stationary point in space when it
/// lies in Q.
void poly_candidates(const Quad& q, const Poly& Q, std::vector<Vec3>& pts)
{
    for (int f = 0; f < Q.faces(); ++f)
        ring_candidates(q, Q.ring(f), Q.size(f), Q.N[size_t(f)], pts);
    const double sc = std::max(1., q.A.cwiseAbs().maxCoeff());
    if (!(std::abs(q.A.determinant()) > 1e-12 * sc * sc * sc)) return;
    const Vec3 x = (2. * q.A).partialPivLu().solve(-q.b);
    for (int f = 0; f < Q.faces(); ++f) {
        if (Q.N[size_t(f)].dot(x - Q.ring(f)[0]) > 0.) return;
    }
    pts.push_back(x);
}

/// q_tol: the rounding bound of q = d_a^2 - d_b^2 at a vertex of a clipped set. The vertex lies
/// within the clip tolerance 16 eps (|h| + scale) <= 64 eps xscale of the exact one, and
/// |grad q| <= 2 (d_a + d_b) <= 4 dmax; plus q's own evaluation rounding, 64 eps dmax^2. MEASURED
/// (Python, slot, concave edge, faces 90/91): a cell vertex on the bisector plane gave
/// q = +5e-21 against d^2 = 1.1e-9, where a bound of 1e-12 d^2 = 1.1e-21 called it a crossing.
double q_tol(const double dmax, const double xscale)
{
    return 4. * dmax * 64. * EPS * xscale + 64. * EPS * dmax * dmax;
}

/// sign_status: the sign of q = d_a^2 - d_b^2 over a convex set whose extremes lie among pts
/// (located with the quadratic's coefficients, evaluated from the distances themselves: the
/// expanded x.A x + b.x + c rounds at eps (n.a)^2, MEASURED (Python) 3.7e-20 against
/// q = 1.4e-20 at a cube edge). A: d_a <= d_b everywhere (up to tol), B: the reverse, Tie: equal
/// up to tol, Cross: a sign change.
enum class Sign { A, B, Tie, Cross };
Sign sign_status(const Loc& a, const Loc& b, const std::vector<Vec3>& pts, const double tol)
{
    double lo = std::numeric_limits<double>::infinity(), hi = -lo;
    for (const Vec3& y : pts) {
        const double da = d_feat(a, y), db = d_feat(b, y);
        const double v = da * da - db * db;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
    }
    if (hi <= tol && lo >= -tol) return Sign::Tie;
    if (hi <= tol) return Sign::A;
    if (lo >= -tol) return Sign::B;
    return Sign::Cross;
}

/// bisector_planes: the unit planes (g, h), g.x = h, whose union is {d_a = d_b}, appended to out;
/// false where that set is curved (face-edge, face-vertex, edge-vertex, skew edges). Face-face:
/// n_a.(x - a_a) = +- n_b.(x - a_b); vertex-vertex: one plane; parallel edges: one plane;
/// edges meeting at p: (y.(e1 - e2))(y.(e1 + e2)) = 0, y = x - p.
bool bisector_planes(const Loc& a, const Loc& b, std::vector<std::pair<Vec3, double>>& out)
{
    const auto add = [&](const Vec3& g, const double h) {
        const double ng = g.norm();
        if (ng > 1e-14) out.push_back({g / ng, h / ng});
    };
    if (a.kind == 0 && b.kind == 0) {
        for (const double s : {1., -1.})
            add(a.dir - s * b.dir, a.dir.dot(a.p) - s * b.dir.dot(b.p));
        return true;
    }
    if (a.kind == 2 && b.kind == 2) {
        add(2. * (b.p - a.p), b.p.dot(b.p) - a.p.dot(a.p));
        return true;
    }
    if (a.kind == 1 && b.kind == 1) {
        const Vec3 &a1 = a.p, &e1 = a.dir, &a2 = b.p, &e2 = b.dir;
        const Vec3 cr = e1.cross(e2);
        if (cr.norm() <= 1e-12) { // parallel: |P(x - a1)|^2 = |P(x - a2)|^2
            const Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - e1 * e1.transpose();
            add(2. * (P * (a2 - a1)), a2.dot(P * a2) - a1.dot(P * a1));
            return true;
        }
        const Vec3 w = a2 - a1;
        if (std::abs(w.dot(cr)) > 1e-12 * cr.norm() * (w.norm() + 1e-300)) return false; // skew
        Eigen::Matrix<double, 3, 2> M2; // a1 + s e1 = a2 + t e2
        M2.col(0) = e1;
        M2.col(1) = -e2;
        const Eigen::Vector2d st = M2.colPivHouseholderQr().solve(w);
        const Vec3 p = a1 + st[0] * e1;
        for (const Vec3& g : {Vec3(e1 - e2), Vec3(e1 + e2)}) add(g, g.dot(p));
        return true;
    }
    return false;
}

/// The two kinds of convex set the front form labels: a planar ring (a front face's piece,
/// pclip()) and a polytope (a cell's piece, clip()). cut(): R cut by {g.x <= h} into out (out =
/// R when R is inside up to tol, `same` then set); false when nothing is inside.
struct RingOps
{
    using Set = Ring;
    Vec3 n;
    const Ctx* cx;
    bool cut(const Ring& R, const Vec3& g, const double h, Ring& out, bool& same) const
    {
        Ring tmp;
        const Cut c = pclip(R, g, h, tmp, *cx);
        same = c == Cut::Same;
        if (c == Cut::None) return false;
        out = same ? R : std::move(tmp);
        return true;
    }
    const std::vector<Vec3>& verts(const Ring& R) const { return R; }
    void candidates(const Quad& q, const Ring& R, std::vector<Vec3>& pts) const
    {
        ring_candidates(q, R.data(), int(R.size()), n, pts);
    }
};
struct PolyOps
{
    using Set = Poly;
    const Ctx* cx;
    bool cut(const Poly& Q, const Vec3& g, const double h, Poly& out, bool& same) const
    {
        Poly tmp;
        const Cut c = clip(Q, g, h, tmp, *cx);
        same = c == Cut::Same;
        if (c == Cut::None) return false;
        out = same ? Q : std::move(tmp);
        return true;
    }
    const std::vector<Vec3>& verts(const Poly& Q) const { return Q.V; }
    void candidates(const Quad& q, const Poly& Q, std::vector<Vec3>& pts) const
    {
        poly_candidates(q, Q, pts);
    }
};

/// resolve: the labelled cells of the convex set R with member set S, (cell, k), k the member
/// nearest on the cell. A member whose q = d^2 - d_other^2 is >= 0 over all of R (up to q_tol)
/// is dropped; the pairs still crossing must have planar bisectors, R is split by all of them
/// and each cell resolved again, once. fail(ka, kb, R's vertices, curved) throws: a crossing
/// pair with a curved bisector (curved), or members still crossing after the split.
template <typename Ops, typename Fail>
void resolve(
    const typename Ops::Set& R,
    const std::vector<int>& S,
    const std::vector<Loc>& loc,
    const Ops& ops,
    const double scale,
    const int depth,
    std::vector<std::pair<typename Ops::Set, int>>& out,
    Fail&& fail)
{
    if (S.size() == 1) {
        out.push_back({R, S[0]});
        return;
    }
    const std::vector<Vec3>& Vr = ops.verts(R);
    double dmax = 0.;
    for (const int k : S) {
        for (const Vec3& y : Vr) dmax = std::max(dmax, d_feat(loc[size_t(k)], y));
    }
    const double tol = q_tol(dmax, scale);
    std::vector<char> alive(S.size(), 1);
    std::vector<std::pair<size_t, size_t>> cross;
    std::vector<Vec3> pts;
    for (size_t i = 0; i < S.size(); ++i) {
        for (size_t j = i + 1; j < S.size(); ++j) {
            if (!alive[i] || !alive[j]) continue;
            const Loc &a = loc[size_t(S[i])], &b = loc[size_t(S[j])];
            Quad q = quad_of(a);
            const Quad qb = quad_of(b);
            q.A -= qb.A;
            q.b -= qb.b;
            q.c -= qb.c;
            pts.clear();
            ops.candidates(q, R, pts);
            const Sign st = sign_status(a, b, pts, tol);
            if (st == Sign::A || st == Sign::Tie) {
                alive[j] = 0;
            } else if (st == Sign::B) {
                alive[i] = 0;
            } else {
                cross.push_back({i, j});
            }
        }
    }
    std::vector<int> al;
    for (size_t i = 0; i < S.size(); ++i) {
        if (alive[i]) al.push_back(S[i]);
    }
    if (al.size() == 1) {
        out.push_back({R, al[0]});
        return;
    }
    std::vector<std::pair<Vec3, double>> planes;
    for (const auto& [i, j] : cross) {
        if (!alive[i] || !alive[j]) continue;
        if (!bisector_planes(loc[size_t(S[i])], loc[size_t(S[j])], planes)) {
            fail(S[i], S[j], Vr, true);
        }
    }
    if (depth > 0 || planes.empty()) fail(al[0], al[1], Vr, false);
    std::vector<typename Ops::Set> cells(1, R), next;
    typename Ops::Set C;
    for (const auto& [g, h] : planes) {
        next.clear();
        for (const auto& Cl : cells) {
            for (const double s : {1., -1.}) {
                bool same;
                if (ops.cut(Cl, s * g, s * h, C, same)) next.push_back(C);
            }
        }
        std::swap(cells, next);
    }
    for (const auto& Cl : cells) resolve(Cl, al, loc, ops, scale, depth + 1, out, fail);
}

/// labeled: the parts of R on which the label g's distance is one smooth function, on the
/// offset side (ExactBandIntegral.hpp). A face is split by its plane into its two affine sides,
/// of which only the offset side is kept when the face has a side plane (sp), both otherwise; a
/// ring lying on the plane (inside both clips up to tol) is one part, not two (MEASURED, Python,
/// cube_out vertex 4294, three boundary faces of B on the input: counted twice it doubled
/// int (d - delta) phi_v). An edge or vertex part is clipped by its side plane.
template <typename Ops>
void labeled(
    const typename Ops::Set& R,
    const Loc& g,
    const SidePlane& sp,
    const int sigma,
    const Ops& ops,
    std::vector<std::pair<typename Ops::Set, Field>>& out)
{
    const bool side = sigma != 0 && sp.has;
    typename Ops::Set C;
    bool same = false;
    if (g.kind == 0) {
        const Vec3& n = g.dir;
        const double hn = n.dot(g.p);
        for (const double s : {1., -1.}) { // keep s n.(x - a0) >= 0
            if (side && !(double(sigma) * s * n.dot(sp.N) > 0.)) continue; // the far side
            if (!ops.cut(R, -s * n, -s * hn, C, same)) continue;
            Field fd;
            fd.kind = 0;
            fd.dir = s * n;
            fd.c = s * hn;
            fd.far = side;
            out.push_back({C, fd});
            if (same) break;
        }
        return;
    }
    Field fd;
    fd.kind = g.kind;
    fd.p = g.p;
    fd.dir = g.dir;
    if (!side) {
        out.push_back({R, fd});
        return;
    }
    if (ops.cut(R, -double(sigma) * sp.N, -double(sigma) * sp.N.dot(sp.ql), C, same)) {
        out.push_back({C, fd});
    }
}

/// box_polytope (d_vertex.py): the axis box [lo, hi] as a polytope, faces outward.
void box_polytope(const Vec3& lo, const Vec3& hi, Poly& out)
{
    Vec3 c[8];
    for (int i = 0; i < 8; ++i) {
        c[i] = Vec3(i & 4 ? hi[0] : lo[0], i & 2 ? hi[1] : lo[1], i & 1 ? hi[2] : lo[2]);
    }
    const int T[6][4] =
        {{0, 1, 3, 2}, {4, 6, 7, 5}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 2, 6, 4}, {1, 5, 7, 3}};
    const Vec3 mid = 0.5 * (lo + hi);
    out.clear();
    int renum[8] = {-1, -1, -1, -1, -1, -1, -1, -1};
    for (const auto& q : T) {
        int idx[4] = {q[0], q[1], q[2], q[3]};
        Vec3 n = (c[idx[1]] - c[idx[0]]).cross(c[idx[2]] - c[idx[0]]);
        if (n.dot(c[idx[0]] - mid) < 0.) {
            std::reverse(idx, idx + 4);
            n = -n;
        }
        Vec3 V[4];
        int ids[4];
        for (int j = 0; j < 4; ++j) {
            V[j] = c[idx[j]];
            if (renum[idx[j]] < 0) renum[idx[j]] = out.nv++;
            ids[j] = renum[idx[j]];
        }
        out.add_face(V, ids, 4, n / n.norm());
    }
}

const char* kind_name(const int kind)
{
    return kind == 0 ? "face" : kind == 1 ? "edge" : "vertex";
}

} // namespace

// ------------------------------------------------------------------------------------------------
// ExactBandIntegral
// ------------------------------------------------------------------------------------------------

ExactBandIntegral::ExactBandIntegral(
    const Eigen::MatrixXd& Vin,
    const Eigen::MatrixXi& Fin,
    const std::vector<int8_t>& face_side,
    const int sigma,
    const size_t value_slots)
{
    static std::atomic<uint64_t> n_constructed{0};
    m_id = ++n_constructed;
    if (value_slots > 0) {
        size_t n = kValueStripes;
        while (n < value_slots) n *= 2;
        m_values.resize(n);
        m_value_locks = std::make_unique<std::mutex[]>(kValueStripes);
    }
    // Mesh.__init__: np.unique(V[F], axis=0) -- the referenced vertices, merged by exact
    // coordinates and sorted lexicographically.
    const int nf = int(Fin.rows());
    std::vector<Vec3> corner(size_t(3 * nf));
    for (int f = 0; f < nf; ++f) {
        for (int c = 0; c < 3; ++c) corner[size_t(3 * f + c)] = Vin.row(Fin(f, c)).head<3>();
    }
    std::vector<int> ord(corner.size());
    std::iota(ord.begin(), ord.end(), 0);
    const auto lex = [&](int a, int b) {
        const Vec3 &p = corner[size_t(a)], &q = corner[size_t(b)];
        if (p[0] != q[0]) return p[0] < q[0];
        if (p[1] != q[1]) return p[1] < q[1];
        return p[2] < q[2];
    };
    std::sort(ord.begin(), ord.end(), lex);
    std::vector<Vec3> V;
    std::vector<int> inv(corner.size());
    for (size_t i = 0; i < ord.size(); ++i) {
        if (i == 0 || lex(ord[i - 1], ord[i])) V.push_back(corner[size_t(ord[i])]);
        inv[size_t(ord[i])] = int(V.size()) - 1;
    }
    std::vector<std::array<int, 3>> F(static_cast<size_t>(nf));
    for (int f = 0; f < nf; ++f) {
        for (int c = 0; c < 3; ++c) F[size_t(f)][size_t(c)] = inv[size_t(3 * f + c)];
    }
    const auto unit = [](const Vec3& v) -> Vec3 { return v / v.norm(); };

    // edges in first-insertion order (the Python dict), with their incident (face, third vertex)
    std::map<std::pair<int, int>, int> eid;
    std::vector<std::pair<int, int>> E;
    std::vector<std::vector<std::pair<int, int>>> E_faces;
    m_face_feats.resize(size_t(nf));
    for (int f = 0; f < nf; ++f) {
        for (int k = 0; k < 3; ++k) {
            const int a = F[size_t(f)][size_t(k)], b = F[size_t(f)][size_t((k + 1) % 3)],
                      c = F[size_t(f)][size_t((k + 2) % 3)];
            const std::pair<int, int> key{std::min(a, b), std::max(a, b)};
            auto it = eid.find(key);
            if (it == eid.end()) {
                it = eid.emplace(key, int(E.size())).first;
                E.push_back(key);
                E_faces.emplace_back();
            }
            E_faces[size_t(it->second)].push_back({f, c});
            m_face_feats[size_t(f)][size_t(k)] = it->second;
            m_face_feats[size_t(f)][size_t(3 + k)] = a;
        }
    }
    std::vector<std::vector<int>> vedges(V.size());
    for (const auto& [a, b] : E) {
        vedges[size_t(a)].push_back(b);
        vedges[size_t(b)].push_back(a);
    }
    m_nF = nf;
    m_nE = int(E.size());
    for (auto& ff : m_face_feats) {
        for (int k = 0; k < 3; ++k) ff[size_t(k)] += m_nF;
        for (int k = 3; k < 6; ++k) ff[size_t(k)] += m_nF + m_nE;
    }

    // features: (kind, data, half-spaces); region = {x : g.(x - q) <= 0 for all (g, q)}
    for (int f = 0; f < nf; ++f) {
        const Vec3 a[3] = {
            V[size_t(F[size_t(f)][0])],
            V[size_t(F[size_t(f)][1])],
            V[size_t(F[size_t(f)][2])]};
        Feature ft;
        ft.kind = 0;
        ft.dir = unit((a[1] - a[0]).cross(a[2] - a[0]));
        ft.p = a[0];
        ft.h_begin = int(m_hs.size());
        for (int k = 0; k < 3; ++k) {
            const Vec3 &p = a[k], &q = a[(k + 1) % 3], &r = a[(k + 2) % 3];
            Vec3 m = unit((q - p).cross(ft.dir));
            if (m.dot(r - p) > 0) m = -m;
            m_hs.push_back({m, p});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = a[0].cwiseMin(a[1]).cwiseMin(a[2]);
        ft.hi = a[0].cwiseMax(a[1]).cwiseMax(a[2]);
        m_feat.push_back(ft);
        m_A.push_back(a[0]);
        m_B.push_back(a[1]);
        m_C.push_back(a[2]);
    }
    for (size_t i = 0; i < E.size(); ++i) {
        const Vec3 &a = V[size_t(E[i].first)], &b = V[size_t(E[i].second)];
        Feature ft;
        ft.kind = 1;
        ft.p = a;
        ft.dir = unit(b - a);
        const Vec3& e = ft.dir;
        ft.h_begin = int(m_hs.size());
        m_hs.push_back({-e, a});
        m_hs.push_back({e, b});
        for (const auto& [fi, ic] : E_faces[i]) {
            const Vec3 w = a - V[size_t(ic)];
            const Vec3 m = unit(w - w.dot(e) * e); // in the face plane, away from c
            m_hs.push_back({-m, a});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = a.cwiseMin(b);
        ft.hi = a.cwiseMax(b);
        m_feat.push_back(ft);
    }
    for (size_t iv = 0; iv < V.size(); ++iv) {
        const Vec3& v = V[iv];
        Feature ft;
        ft.kind = 2;
        ft.p = v;
        ft.dir.setZero();
        ft.h_begin = int(m_hs.size());
        for (const int w : vedges[iv]) {
            const Vec3 g = unit(V[size_t(w)] - v);
            m_hs.push_back({g, v});
        }
        ft.h_end = int(m_hs.size());
        ft.lo = ft.hi = v;
        m_feat.push_back(ft);
    }

    // The offset side (exact_mesh.Mesh._side_planes): per feature the plane of the pseudonormal
    // sign test (class doc), the face normals oriented out of the solid by face_side.
    if (sigma != 0) {
        if (int(face_side.size()) != nf) {
            log_and_throw_error(
                "ExactBandIntegral: {} face sides for {} triangles",
                face_side.size(),
                nf);
        }
        m_sigma = sigma > 0 ? 1 : -1;
        const size_t ng = m_feat.size();
        m_side_has.assign(ng, 0);
        m_side_n.assign(ng, Vec3::Zero());
        m_side_q.assign(ng, Vec3::Zero());
        std::vector<Vec3> nfo(static_cast<size_t>(nf));
        for (int f = 0; f < nf; ++f) {
            nfo[size_t(f)] = m_feat[size_t(f)].dir * double(face_side[size_t(f)]);
            if (face_side[size_t(f)] == 0) continue;
            m_side_has[size_t(f)] = 1;
            m_side_n[size_t(f)] = nfo[size_t(f)];
            m_side_q[size_t(f)] = V[size_t(F[size_t(f)][0])];
        }
        for (size_t i = 0; i < E.size(); ++i) {
            const auto& inc = E_faces[i];
            if (inc.size() != 2 || face_side[size_t(inc[0].first)] == 0 ||
                face_side[size_t(inc[1].first)] == 0) {
                continue;
            }
            const Vec3 m = nfo[size_t(inc[0].first)] + nfo[size_t(inc[1].first)];
            const double nm = m.norm();
            if (!(nm > 0.)) continue;
            const size_t g = size_t(m_nF) + i;
            m_side_has[g] = 1;
            m_side_n[g] = m / nm;
            m_side_q[g] = V[size_t(E[i].first)];
        }
        std::vector<Vec3> pv(V.size(), Vec3::Zero());
        std::vector<char> ok(V.size(), 1);
        for (int f = 0; f < nf; ++f) {
            for (int k = 0; k < 3; ++k) {
                const int iv = F[size_t(f)][size_t(k)];
                if (face_side[size_t(f)] == 0) {
                    ok[size_t(iv)] = 0;
                    continue;
                }
                const Vec3& a = V[size_t(iv)];
                const Vec3 u = V[size_t(F[size_t(f)][size_t((k + 1) % 3)])] - a;
                const Vec3 w = V[size_t(F[size_t(f)][size_t((k + 2) % 3)])] - a;
                pv[size_t(iv)] += std::atan2(u.cross(w).norm(), u.dot(w)) * nfo[size_t(f)];
            }
        }
        for (size_t i = 0; i < E.size(); ++i) {
            if (E_faces[i].size() != 2) ok[size_t(E[i].first)] = ok[size_t(E[i].second)] = 0;
        }
        for (size_t iv = 0; iv < V.size(); ++iv) {
            const double nm = pv[iv].norm();
            if (!ok[iv] || !(nm > 0.)) continue;
            const size_t g = size_t(m_nF + m_nE) + iv;
            m_side_has[g] = 1;
            m_side_n[g] = pv[iv] / nm;
            m_side_q[g] = V[iv];
        }
    }

    // the triangle tree (InputTriangles' construction)
    if (nf == 0) return;
    std::vector<Vec3> centroid(static_cast<size_t>(nf));
    Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity()), hi = -lo;
    for (int f = 0; f < nf; ++f) {
        centroid[size_t(f)] = (m_A[size_t(f)] + m_B[size_t(f)] + m_C[size_t(f)]) / 3.;
        lo = lo.cwiseMin(m_feat[size_t(f)].lo);
        hi = hi.cwiseMax(m_feat[size_t(f)].hi);
    }
    m_order.resize(size_t(nf));
    std::iota(m_order.begin(), m_order.end(), 0);
    m_nodes.reserve(size_t(2 * nf));
    build_node(0, nf, centroid, 1e-9 * std::max((hi - lo).norm(), 1e-300));
}

int ExactBandIntegral::build_node(
    const int begin,
    const int end,
    const std::vector<Eigen::Vector3d>& centroid,
    const double pad)
{
    const int id = int(m_nodes.size());
    m_nodes.emplace_back();
    Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity());
    Vec3 hi = -lo, clo = lo, chi = -lo;
    for (int j = begin; j < end; ++j) {
        const int f = m_order[size_t(j)];
        lo = lo.cwiseMin(m_feat[size_t(f)].lo);
        hi = hi.cwiseMax(m_feat[size_t(f)].hi);
        clo = clo.cwiseMin(centroid[size_t(f)]);
        chi = chi.cwiseMax(centroid[size_t(f)]);
    }
    m_nodes[size_t(id)].lo = lo - Vec3::Constant(pad);
    m_nodes[size_t(id)].hi = hi + Vec3::Constant(pad);
    m_nodes[size_t(id)].begin = begin;
    m_nodes[size_t(id)].end = end;
    if (end - begin <= 4) return id;
    int axis = 0;
    (chi - clo).maxCoeff(&axis);
    const int mid = (begin + end) / 2;
    std::nth_element(
        m_order.begin() + begin,
        m_order.begin() + mid,
        m_order.begin() + end,
        [&](const int a, const int b) {
            return centroid[size_t(a)][axis] < centroid[size_t(b)][axis] ||
                   (centroid[size_t(a)][axis] == centroid[size_t(b)][axis] && a < b);
        });
    const int left = build_node(begin, mid, centroid, pad);
    const int right = build_node(mid, end, centroid, pad);
    m_nodes[size_t(id)].left = left;
    m_nodes[size_t(id)].right = right;
    return id;
}

double ExactBandIntegral::upper_bound(const Eigen::Vector3d* t, const int n) const
{
    // Branch and bound: max over the corners of the distance to a node's (padded) box is a lower
    // bound of max_i d_P(t_i) for every triangle P in it, so the result is the exact minimum.
    double m = std::numeric_limits<double>::infinity();
    if (m_nodes.empty()) return m;
    const auto lower = [&](const Node& nd) {
        double s = 0.;
        for (int i = 0; i < n; ++i) {
            s = std::max(s, (nd.lo - t[i]).cwiseMax(t[i] - nd.hi).cwiseMax(0.).norm());
        }
        return s;
    };
    std::vector<std::pair<double, int>> stack = {{lower(m_nodes[0]), 0}};
    while (!stack.empty()) {
        const auto [lb, id] = stack.back();
        stack.pop_back();
        if (lb > m) continue;
        const Node& nd = m_nodes[size_t(id)];
        if (nd.left < 0) {
            for (int j = nd.begin; j < nd.end; ++j) {
                const int f = m_order[size_t(j)];
                double s = 0.;
                for (int i = 0; i < n; ++i) {
                    s = std::max(
                        s,
                        point_tri_dist(t[i], m_A[size_t(f)], m_B[size_t(f)], m_C[size_t(f)]));
                    if (s >= m)
                        break; // this triangle cannot lower the min: the result is unchanged
                }
                m = std::min(m, s);
            }
            continue;
        }
        const double ll = lower(m_nodes[size_t(nd.left)]), lr = lower(m_nodes[size_t(nd.right)]);
        if (ll <= lr) {
            stack.push_back({lr, nd.right});
            stack.push_back({ll, nd.left});
        } else {
            stack.push_back({ll, nd.left});
            stack.push_back({lr, nd.right});
        }
    }
    return m;
}

void ExactBandIntegral::candidates(
    const Eigen::Vector3d& tlo,
    const Eigen::Vector3d& thi,
    const double r,
    std::vector<int>& out) const
{
    // Faces from the tree; then their edges and vertices. A face's box contains its edges' and
    // vertices' boxes, so its box distance is never larger: every edge or vertex within r has an
    // incident face within r (exactly so in floating point: the per-axis gaps are monotone).
    out.clear();
    if (m_nodes.empty()) return;
    std::vector<int> stack = {0};
    while (!stack.empty()) {
        const Node& nd = m_nodes[size_t(stack.back())];
        stack.pop_back();
        if (box_dist(nd.lo, nd.hi, tlo, thi) > r) continue;
        if (nd.left >= 0) {
            stack.push_back(nd.left);
            stack.push_back(nd.right);
            continue;
        }
        for (int j = nd.begin; j < nd.end; ++j) {
            const int f = m_order[size_t(j)];
            if (box_dist(m_feat[size_t(f)].lo, m_feat[size_t(f)].hi, tlo, thi) > r) continue;
            out.push_back(f);
            for (const int g : m_face_feats[size_t(f)]) {
                if (box_dist(m_feat[size_t(g)].lo, m_feat[size_t(g)].hi, tlo, thi) <= r) {
                    out.push_back(g);
                }
            }
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
}

struct ExactBandIntegral::Local
{
    Vec3 c0; ///< the local origin: the centroid of the cell or face
    std::vector<Loc> loc; ///< the candidate features, local
    std::vector<int> cand; ///< their feature indices
    std::vector<std::pair<Vec3, double>> hs; ///< their half-spaces, local
    double U = 0.; ///< min over P of max over the corners of d_P
    double tol_cut = 0., tol_any = 0.; ///< the partition's tolerance budget (Ctx)
    double scale = 0.; ///< the rounding scale (Ctx::scale)
};

struct ExactBandIntegral::Work : Local
{
    Poly Q0; ///< t in local coordinates
    std::vector<Piece> pieces; ///< the partition of t with member sets
};

struct ExactBandIntegral::TriWork : Local
{
    Ring V0; ///< f in local coordinates
    std::vector<RingPiece> pieces; ///< the partition of f with member sets
};

void ExactBandIntegral::localize(const Eigen::Vector3d* t, const int n, Local& w, Stats* stats)
    const
{
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    const Vec3& c0 = w.c0;
    std::vector<Loc>& loc = w.loc;
    std::vector<std::pair<Vec3, double>>& hs = w.hs;
    loc.clear();
    hs.clear();
    w.scale = 0.;
    {
        Timer tm(cx, &Stats::t_candidates);
        Vec3 tlo = t[0], thi = t[0];
        for (int i = 0; i < n; ++i) {
            tlo = tlo.cwiseMin(t[i]);
            thi = thi.cwiseMax(t[i]);
        }
        w.U = upper_bound(t, n);
        std::vector<int>& cand = w.cand;
        candidates(tlo, thi, w.U * (1 + 1e-12), cand);
        loc.reserve(cand.size());
        for (const int gi : cand) {
            const Feature& ft = m_feat[size_t(gi)];
            Loc l;
            l.kind = ft.kind;
            l.p = ft.p - c0;
            l.dir = ft.dir;
            l.h_begin = int(hs.size());
            for (int i = ft.h_begin; i < ft.h_end; ++i) {
                const auto& [g, q] = m_hs[size_t(i)];
                hs.push_back({g, g.dot(q - c0)}); // in the cell's frame: see m_hs
                w.scale = std::max(w.scale, maxabs(q - c0));
            }
            l.h_end = int(hs.size());
            loc.push_back(l);
        }
        if (stats) stats->candidates += int64_t(cand.size());
    }
    // The rounding scale: the largest magnitude among the terms the planes and vertices are built
    // from, max |t_i - c0| (the corners) and max |q - c0| (the points the candidates' half-space
    // offsets h = g.(q - c0) are taken from), max norm (exact_mesh._cell_scale). A tolerance on
    // the clipped polytope's own coordinates alone (|h| + max |V|) missed what the offsets and
    // the inherited vertices carry. MEASURED: on an inward cylinder cell the candidates' points
    // lay 0.355 from c0 while a clip saw 3.9e-3; two copies of one vertex, made by different
    // clips 3-6 tol apart, fell on both sides of a plane parallel to a face 1.9 tol away, the
    // faces disagreed about the cut, and the pieces lost 3.8e-4 of the cell.
    for (int i = 0; i < n; ++i) w.scale = std::max(w.scale, maxabs(t[i] - c0));
}

bool ExactBandIntegral::partition(const std::array<Eigen::Vector3d, 4>& T, Work& w, Stats* stats)
    const
{
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    w.tol_cut = w.tol_any = 0.;
    cx.tol_cut = &w.tol_cut;
    cx.tol_any = &w.tol_any;
    // _partition, step 0: candidates in local coordinates (origin c0 = centroid of t). A flat
    // cell is 0 before any search (the Python searches first; the value is the same).
    w.c0 = (((T[0] + T[1]) + T[2]) + T[3]) / 4.;
    const Vec3& c0 = w.c0;
    std::array<Vec3, 4> Tl;
    for (int i = 0; i < 4; ++i) Tl[size_t(i)] = T[size_t(i)] - c0;
    if (!tet_polytope(Tl, w.Q0)) return false;
    localize(T.data(), 4, w, stats);
    const std::vector<Loc>& loc = w.loc;
    const std::vector<std::pair<Vec3, double>>& hs = w.hs;
    cx.scale = w.scale;

    // step 1: the convex pieces of t with constant member sets
    std::vector<Piece>& pieces = w.pieces;
    std::vector<Piece> next;
    pieces.assign(1, Piece());
    pieces[0].Q = w.Q0;
    std::vector<Poly> outs;
    for (int k = 0; k < int(loc.size()); ++k) {
        const std::pair<Vec3, double>* H = hs.data() + loc[size_t(k)].h_begin;
        const int nh = loc[size_t(k)].h_end - loc[size_t(k)].h_begin;
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
                if (in.Q.faces() > 0) {
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

ExactBandIntegral::Partition::Partition() = default;
ExactBandIntegral::Partition::~Partition() = default;
ExactBandIntegral::Partition::Partition(Partition&&) noexcept = default;
ExactBandIntegral::Partition& ExactBandIntegral::Partition::operator=(Partition&&) noexcept =
    default;

const ExactBandIntegral::Work* ExactBandIntegral::cell_partition(
    const std::array<Eigen::Vector3d, 4>& T,
    Stats* stats,
    Work* own,
    Partition* keep) const
{
    if (stats) return partition(T, *own, stats) ? own : nullptr;
    if (!keep) {
        // MEASURED (Geneva base, inward, collapse pass): 29.8 s with a per-thread table of the
        // last 64 partitions, 27.1 s with this one scratch partition, whose vectors keep their
        // storage from call to call (and 365 s with a fresh partition per call, before the value
        // table).
        thread_local Work scratch;
        return partition(T, scratch, nullptr) ? &scratch : nullptr;
    }
    double key[12];
    for (int i = 0; i < 4; ++i) std::memcpy(key + 3 * i, T[size_t(i)].data(), 3 * sizeof(double));
    if (keep->m_w && keep->m_owner == m_id && std::memcmp(keep->m_key, key, sizeof(key)) == 0) {
        return keep->m_ok ? keep->m_w.get() : nullptr;
    }
    if (!keep->m_w) keep->m_w = std::make_unique<Work>();
    keep->m_owner = 0; // none until partition() returns (it may throw)
    keep->m_ok = partition(T, *keep->m_w, nullptr);
    std::memcpy(keep->m_key, key, sizeof(key));
    keep->m_owner = m_id;
    return keep->m_ok ? keep->m_w.get() : nullptr;
}

void ExactBandIntegral::tri_partition(const std::array<Eigen::Vector3d, 3>& f, TriWork& w) const
{
    Ctx cx;
    w.tol_cut = w.tol_any = 0.;
    cx.tol_cut = &w.tol_cut;
    cx.tol_any = &w.tol_any;
    // _tri_partition: step 0 as partition()'s, origin c0 = f's centroid (P3.mean(0)).
    w.c0 = ((f[0] + f[1]) + f[2]) / 3.;
    localize(f.data(), 3, w, nullptr);
    const std::vector<Loc>& loc = w.loc;
    const std::vector<std::pair<Vec3, double>>& hs = w.hs;
    cx.scale = w.scale;
    w.V0 = {Vec3(f[0] - w.c0), Vec3(f[1] - w.c0), Vec3(f[2] - w.c0)};

    // step 1: the convex pieces (rings) of f with constant member sets
    std::vector<RingPiece>& pieces = w.pieces;
    std::vector<RingPiece> next;
    pieces.assign(1, RingPiece());
    pieces[0].R = w.V0;
    std::vector<Ring> outs;
    for (int k = 0; k < int(loc.size()); ++k) {
        const std::pair<Vec3, double>* H = hs.data() + loc[size_t(k)].h_begin;
        const int nh = loc[size_t(k)].h_end - loc[size_t(k)].h_begin;
        next.clear();
        for (RingPiece& pc : pieces) {
            const Side st = pclassify(pc.R, H, nh, cx);
            if (st == Side::In) {
                pc.S.push_back(k);
                next.push_back(std::move(pc));
            } else if (st == Side::Out) {
                next.push_back(std::move(pc));
            } else {
                RingPiece in;
                outs.clear();
                psplit(pc.R, H, nh, in.R, outs, cx);
                if (!in.R.empty()) {
                    in.S = pc.S;
                    in.S.push_back(k);
                    next.push_back(std::move(in));
                }
                for (Ring& ro : outs) {
                    RingPiece o;
                    o.R = std::move(ro);
                    o.S = pc.S;
                    next.push_back(std::move(o));
                }
            }
        }
        std::swap(pieces, next);
    }
}

double ExactBandIntegral::max_distance(const std::array<Eigen::Vector3d, 3>& f, MaxInfo* info) const
{
    TriWork w;
    tri_partition(f, w);
    Ctx cx; // the sub-pieces' clips: no tolerance budget (as the Python)
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    double best = -std::numeric_limits<double>::infinity();
    Vec3 at = Vec3::Zero();
    bool ra_best = false;
    int rule_a_pieces = 0;
    std::vector<int> S;
    std::vector<const Loc*> members;
    for (const RingPiece& pc : w.pieces) {
        if (pc.S.empty()) continue;
        S = pc.S;
        if (S.size() > 1) S = prune_points(pc.R, S, loc);
        bool all_faces = true, all_verts = true;
        members.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            all_faces = all_faces && loc[size_t(k)].kind == 0;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
        }
        // the max over a ring's vertices of a convex function (first maximiser, as np.argmax)
        double val = -std::numeric_limits<double>::infinity();
        Vec3 x = Vec3::Zero();
        const auto take = [&](const Ring& R, auto&& dist) {
            for (const Vec3& p : R) {
                const double dv = dist(p);
                if (dv > val) {
                    val = dv;
                    x = p;
                }
            }
        };
        bool ra = false;
        if (S.size() == 1) {
            take(pc.R, [&](const Vec3& p) { return d_feat(*members[0], p); });
        } else if (all_faces) {
            pfaces_min_pieces(
                pc.R,
                members,
                cx,
                [&](const Ring& Rs, const Vec3& W, const double c) {
                    take(Rs, [&](const Vec3& p) { return p.dot(W) - c; });
                });
        } else if (all_verts) {
            pvertices_min_pieces(pc.R, members, cx, [&](const Ring& Rs, const Vec3& v) {
                take(Rs, [&](const Vec3& p) { return (p - v).norm(); });
            });
        } else { // rule (a): min over g of max over the piece of d_g >= max over the piece of d
            double vmin = std::numeric_limits<double>::infinity();
            Vec3 xmin = Vec3::Zero();
            for (const Loc* g : members) {
                val = -std::numeric_limits<double>::infinity();
                take(pc.R, [&](const Vec3& p) { return d_feat(*g, p); });
                if (val < vmin) {
                    vmin = val;
                    xmin = x;
                }
            }
            val = vmin;
            x = xmin;
            ra = true;
            ++rule_a_pieces;
        }
        if (val > best) {
            best = val;
            at = x;
            ra_best = ra;
        }
    }
    check_face_pieces(f, w, "max_distance");
    if (info) {
        info->at = at + w.c0;
        info->rule_a = ra_best;
        info->U = w.U;
        info->candidates = int(loc.size());
        info->pieces = int(w.pieces.size());
        info->rule_a_pieces = rule_a_pieces;
    }
    return best;
}

double ExactBandIntegral::value(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    Stats* stats) const
{
    if (m_values.empty() || stats) return evaluate(T, delta, stats, nullptr);
    return value_tabled(T, delta, nullptr);
}

double ExactBandIntegral::value(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    Partition& keep) const
{
    if (m_values.empty()) return evaluate(T, delta, nullptr, &keep);
    return value_tabled(T, delta, &keep);
}

void ExactBandIntegral::reserve_values(const size_t slots) const
{
    if (m_values.empty() || m_values.size() >= slots) return;
    size_t n = m_values.size();
    while (n < slots) n *= 2;
    m_values.assign(n, ValueSlot{});
}

double ExactBandIntegral::value_tabled(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    Partition* keep) const
{
    // the value table (see value())
    double in[13];
    for (int i = 0; i < 4; ++i) std::memcpy(in + 3 * i, T[size_t(i)].data(), 3 * sizeof(double));
    in[12] = delta;
    uint64_t h = 0x9e3779b97f4a7c15ull;
    for (const double x : in) {
        uint64_t u;
        std::memcpy(&u, &x, sizeof(u));
        h = (h ^ u) * 0xbf58476d1ce4e5b9ull;
        h ^= h >> 31;
    }
    h *= 0x94d049bb133111ebull;
    h ^= h >> 32;
    const size_t k = size_t(h) & (m_values.size() - 1);
    ValueSlot& sl = m_values[k];
    std::mutex& mx = m_value_locks[k & (kValueStripes - 1)];
    {
        std::lock_guard<std::mutex> lock(mx);
        if (sl.full && std::memcmp(sl.in, in, sizeof(in)) == 0) return sl.out;
    }
    const double v = evaluate(T, delta, nullptr, keep);
    std::lock_guard<std::mutex> lock(mx);
    std::memcpy(sl.in, in, sizeof(in));
    sl.out = v;
    sl.full = true;
    return v;
}

double ExactBandIntegral::evaluate(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    Stats* stats,
    Partition* keep) const
{
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    Timer t_all(cx, &Stats::t_total);
    std::optional<Work> own;
    if (stats) own.emplace();
    const Work* wp = cell_partition(T, stats, own ? &*own : nullptr, keep);
    if (!wp) return 0.;
    const Work& w = *wp;
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    const std::vector<Piece>& pieces = w.pieces;
    const Poly& Q0 = w.Q0;

    // steps 2-4: integrate each piece
    double total = 0., vol_sum = 0., empty_vol = 0., exact_vol = 0., rule_a_vol = 0.;
    int64_t rule_a_pieces = 0;
    std::vector<int> S;
    std::vector<const Loc*> members;
    // The offset side (class doc; exact_mesh.band_integral): total_c sums each piece's kept part,
    // int (d - delta); a cell with nothing clipped returns the unclipped total - delta v0.
    const bool side = m_sigma != 0;
    const auto side_plane = [&](const int k) {
        SidePlane sp;
        const size_t gi = size_t(w.cand[size_t(k)]);
        sp.has = m_side_has[gi];
        sp.N = m_side_n[gi];
        sp.ql = m_side_q[gi] - w.c0;
        return sp;
    };
    double total_c = 0.;
    bool clipped = false;
    int64_t n_split = 0, n_D = 0, n_D_over = 0, n_D_pos = 0;
    std::vector<SidePlane> planes;
    std::vector<SidePart> parts;
    Poly Rc;
    for (const Piece& pc : pieces) {
        const double vol = volume_centroid(pc.Q).first;
        vol_sum += vol;
        if (pc.S.empty()) {
            empty_vol += vol;
            continue;
        }
        S = pc.S;
        if (S.size() > 1) S = prune_members(pc.Q, S, loc);
        bool all_faces = true, all_verts = true;
        members.clear();
        planes.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            all_faces = all_faces && loc[size_t(k)].kind == 0;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
            if (side) planes.push_back(side_plane(k));
        }
        double val, vc = 0.;
        if (S.size() == 1) {
            val = int_g(*members[0], pc.Q, cx);
            exact_vol += vol;
            if (side) {
                const Cut ck = side_clip(pc.Q, planes[0], m_sigma, Rc, cx);
                if (ck == Cut::Same) {
                    vc = val - delta * vol;
                } else {
                    clipped = true;
                    if (ck == Cut::New)
                        vc = int_g(*members[0], Rc, cx) - delta * volume_centroid(Rc).first;
                }
            }
        } else if (all_faces) {
            if (!side) {
                val = faces_min(pc.Q, members, cx);
            } else {
                val = 0.;
                faces_min_pieces(
                    pc.Q,
                    members,
                    cx,
                    [&](const Poly& R,
                        const Vec3& W,
                        const double c,
                        const int i,
                        const double sg) {
                        const auto [v_, cen] = volume_centroid(R);
                        val += v_ * (W.dot(cen) - c);
                        // R lies on sg n_i.(x - a_i) >= 0: on face i's offset side iff that is
                        // sigma N_i
                        if (!planes[size_t(i)].has ||
                            double(m_sigma) * sg *
                                    members[size_t(i)]->dir.dot(planes[size_t(i)].N) >
                                0.) {
                            vc += v_ * (W.dot(cen) - c) - delta * v_;
                        } else {
                            clipped = true;
                        }
                    });
            }
            exact_vol += vol;
        } else if (all_verts) {
            if (!side) {
                val = vertices_min(pc.Q, members, cx);
            } else {
                val = 0.;
                vertices_min_pieces(
                    pc.Q,
                    members,
                    cx,
                    [&](const Poly& R, const Vec3& v, const int i) {
                        val += vol_point(R, v, cx);
                        const Cut ck = side_clip(R, planes[size_t(i)], m_sigma, Rc, cx);
                        if (ck == Cut::Same) {
                            vc += vol_point(R, v, cx) - delta * volume_centroid(R).first;
                        } else {
                            clipped = true;
                            if (ck == Cut::New)
                                vc += vol_point(Rc, v, cx) - delta * volume_centroid(Rc).first;
                        }
                    });
            }
            exact_vol += vol;
        } else { // rule (a): min over g of int d_g >= int min_g d_g, an upper bound
            val = std::numeric_limits<double>::infinity();
            for (const Loc* g : members) val = std::min(val, int_g(*g, pc.Q, cx));
            rule_a_vol += vol;
            ++rule_a_pieces;
            if (side) {
                side_split(pc.Q, planes, m_sigma, parts, cx);
                bool all_on = parts.size() == 1;
                for (const char o : parts[0].on) all_on = all_on && o;
                if (all_on) {
                    vc = val - delta * vol;
                } else {
                    clipped = true;
                    ++n_split;
                    for (const SidePart& sp : parts) {
                        bool all = true, any = false;
                        for (const char o : sp.on) {
                            all = all && o;
                            any = any || o;
                        }
                        if (all) {
                            double m = std::numeric_limits<double>::infinity();
                            for (const Loc* g : members) m = std::min(m, int_g(*g, sp.R, cx));
                            vc += m - delta * volume_centroid(sp.R).first;
                        } else if (any) {
                            double mx_all;
                            const double b = d_bound(sp.R, members, delta, mx_all);
                            vc += b;
                            ++n_D;
                            n_D_over += mx_all > delta;
                            n_D_pos += b > 0.;
                        }
                    }
                }
            }
        }
        total += val;
        total_c += vc;
    }
    if (stats && side) {
        stats->side_clipped += clipped;
        stats->side_rule_a_split += n_split;
        stats->side_D += n_D;
        stats->side_D_over += n_D_over;
        stats->side_D_pos += n_D_pos;
    }
    const double v0 = volume_centroid(Q0).first;
    if (stats) {
        ++stats->cells;
        stats->pieces += int64_t(pieces.size());
        stats->rule_a_pieces += rule_a_pieces;
        stats->vol += v0;
        stats->exact_vol += exact_vol;
        stats->rule_a_vol += rule_a_vol;
        stats->empty_vol += empty_vol;
    }
    check_cell_pieces(T, delta, w, vol_sum, empty_vol);
    return clipped ? total_c : total - delta * v0;
}

double ExactBandIntegral::value_grad_hess(
    const std::array<Eigen::Vector3d, 4>& T,
    const int kv,
    const double delta,
    Eigen::Vector3d& G,
    Eigen::Matrix3d& H,
    Stats* stats) const
{
    return value_grad_hess_impl(T, kv, delta, G, H, stats, nullptr);
}

double ExactBandIntegral::value_grad_hess(
    const std::array<Eigen::Vector3d, 4>& T,
    const int kv,
    const double delta,
    Eigen::Vector3d& G,
    Eigen::Matrix3d& H,
    Partition& keep) const
{
    return value_grad_hess_impl(T, kv, delta, G, H, nullptr, &keep);
}

double ExactBandIntegral::value_grad_hess_impl(
    const std::array<Eigen::Vector3d, 4>& T,
    const int kv,
    const double delta,
    Eigen::Vector3d& G,
    Eigen::Matrix3d& H,
    Stats* stats,
    Partition* keep) const
{
    G.setZero();
    H.setZero();
    Ctx cx;
    cx.st = stats;
    cx.timing = stats && stats->timing;
    Timer t_all(cx, &Stats::t_total);
    std::optional<Work> own;
    if (stats) own.emplace();
    const Work* wp = cell_partition(T, stats, own ? &*own : nullptr, keep);
    if (!wp) return 0.;
    const Work& w = *wp;
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;

    // elementary(): pieces on which the integrand is ONE smooth function. A face member is split
    // by its plane (affine per side); a rule-(a) piece carries the member of smallest integral
    // (lowest candidate on a tie), so its derivative is that of the BOUND. With the offset side
    // (class doc; exact_mesh.elementary): only the offset-side parts, the rule-(a) parts where the
    // members' planes disagree going to Dl.
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
    struct DPart
    {
        Poly R;
        std::vector<const Loc*> members;
    };
    std::vector<DPart> Dl;
    std::vector<Elem> pieces;
    Poly tmp, Rc;
    const auto add = [&](const Poly& Q, const Loc& g, const bool ra, const SidePlane& plane) {
        if (g.kind == 0) {
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
                e.rule_a = ra;
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
        e.kind = g.kind;
        e.p = g.p;
        e.dir = g.dir;
        e.rule_a = ra;
        pieces.push_back(std::move(e));
    };
    std::vector<const Loc*> members;
    std::vector<SidePlane> planes;
    std::vector<SidePart> parts;
    for (const Piece& pc : w.pieces) {
        if (pc.S.empty()) continue;
        const std::vector<int> S = pc.S.size() > 1 ? prune_members(pc.Q, pc.S, loc) : pc.S;
        bool all_faces = true, all_verts = true;
        members.clear();
        planes.clear();
        for (const int k : S) {
            members.push_back(&loc[size_t(k)]);
            planes.push_back(side_plane(k));
            all_faces = all_faces && loc[size_t(k)].kind == 0;
            all_verts = all_verts && loc[size_t(k)].kind == 2;
        }
        if (S.size() == 1) {
            add(pc.Q, loc[size_t(S[0])], false, planes[0]);
        } else if (all_faces) {
            faces_min_pieces(
                pc.Q,
                members,
                cx,
                [&](const Poly& R, const Vec3& W, const double c, const int i, const double sg) {
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
            vertices_min_pieces(pc.Q, members, cx, [&](const Poly& R, const Vec3& v, const int i) {
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
                if (all) {
                    double best = std::numeric_limits<double>::infinity();
                    int bk = -1;
                    for (const int k : S) { // ascending k: the first minimum is the lowest on a tie
                        const double v = int_g(loc[size_t(k)], sp.R, cx);
                        if (v < best) best = v, bk = k;
                    }
                    add(sp.R, loc[size_t(bk)], true, SidePlane());
                } else if (any) {
                    DPart d;
                    d.R = sp.R;
                    d.members = members;
                    Dl.push_back(std::move(d));
                }
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
                f.dir = Vec3::Zero();
                f.c = delta;
                f.rule_a = e.rule_a;
                sh.push_back(std::move(e));
                sh.push_back(std::move(f));
            }
        }
        pieces = std::move(sh);
    }

    // lambda0 = l0 + g0.y: 1 at x, 0 on the opposite face (face kv of tet_polytope is opposite
    // vertex kv)
    std::array<Vec3, 4> Tl;
    for (int i = 0; i < 4; ++i) Tl[size_t(i)] = T[size_t(i)] - w.c0;
    const Vec3 x = Tl[size_t(kv)];
    const Poly& Q0 = w.Q0;
    const Vec3* Vo = Q0.ring(kv);
    const Vec3& no = Q0.N[size_t(kv)];
    const double den = no.dot(x - Vo[0]);
    const Vec3 g0 = no / den;
    const double l0 = -no.dot(Vo[0]) / den;
    double scale = 0.;
    for (const Vec3& p : Tl) scale = std::max(scale, maxabs(p));
    const double tol = 64 * EPS * scale;
    std::array<std::pair<Vec3, double>, 4> tet_planes;
    for (int i = 0; i < 4; ++i)
        tet_planes[size_t(i)] = {Q0.N[size_t(i)], Q0.N[size_t(i)].dot(Q0.ring(i)[0])};
    struct TFace
    {
        int i;
        Vec3 nu;
        Eigen::Matrix3d S; // column k = e_k x (a - b)
        double nN; // |N_F|
        Vec3 a, b; // its other two corners, N_F = (x - b) x (a - b) outward
    };
    std::vector<TFace> faces;
    Vec3 gradV = Vec3::Zero();
    for (int i = 0; i < 4; ++i) {
        if (i == kv) continue;
        const Vec3* V = Q0.ring(i);
        Vec3 ab[3];
        int nab = 0;
        for (int j = 0; j < 3; ++j) {
            if (maxabs(V[j] - x) > 0) ab[nab++] = V[j];
        }
        if (nab != 2) continue; // not reached: the face holds x and two other corners
        Vec3 a = ab[0], b = ab[1];
        Vec3 N = (x - b).cross(a - b);
        const Vec3& n = Q0.N[size_t(i)];
        if (N.dot(n) < 0) {
            std::swap(a, b);
            N = -N;
        }
        const Vec3 wab = a - b;
        TFace tf;
        tf.i = i;
        tf.nu = n;
        for (int k = 0; k < 3; ++k) tf.S.col(k) = Vec3::Unit(k).cross(wab);
        tf.nN = N.norm();
        tf.a = a;
        tf.b = b;
        faces.push_back(tf);
        gradV += n * N.norm() / 6.; // area / 3
    }

    // the boundary terms of one piece (exact_mesh.value_grad_hess, accumulate), added to G, H
    const auto accumulate = [&](const Elem& e, Vec3& Ga, Eigen::Matrix3d& Ha) {
        const Poly& Q = e.Q;
        // Rounding sliver (thickness = volume / largest face area at rounding level, e.g. the gap
        // between the input plane and a cell face lying on it): skipped. Its neighbours' cut
        // terms pair up across it, and the neighbour's face on the coincident plane is recognised
        // below as lying on the cell face (plane test, not the face's identity), so the trace
        // comes from the piece inside t.
        double amax = 0.;
        for (int f = 0; f < Q.faces(); ++f) {
            const Vec3* Vg = Q.ring(f);
            double s = 0.;
            for (int j = 1; j < Q.size(f) - 1; ++j)
                s += (Vg[j] - Vg[0]).cross(Vg[j + 1] - Vg[0]).norm();
            amax = std::max(amax, 0.5 * s);
        }
        if (volume_centroid(Q).first <= 1e3 * tol * amax) return;
        std::array<bool, 3> on_tface = {false, false, false}; // Q has a face on faces[i]
        for (int f = 0; f < Q.faces(); ++f) {
            const Vec3* Vf = Q.ring(f);
            const int kf = Q.size(f);
            const Vec3& nf = Q.N[size_t(f)];
            for (size_t fi = 0; fi < faces.size(); ++fi) {
                const TFace& tf = faces[fi];
                if (nf.dot(tf.nu) < 1 - 1e-12) continue;
                double dev = 0.;
                for (int j = 0; j < kf; ++j) {
                    dev =
                        std::max(dev, std::abs(Vf[j].dot(tf.nu) - tet_planes[size_t(tf.i)].second));
                }
                if (dev > 1e3 * tol) continue; // same scale as the sliver test
                on_tface[fi] = true;
                double P1;
                Vec3 P2;
                moments(Vf, kf, nf, e, l0, g0, P1, P2, cx);
                Ga += tf.nu * P1;
                Ha += tf.nu * P2.transpose() + tf.S * (P1 / tf.nN);
                for (int ee = 0; ee < kf; ++ee) {
                    const Vec3& y1 = Vf[ee];
                    const Vec3& y2 = Vf[(ee + 1) % kf];
                    if (maxabs(y2 - y1) <= tol) continue;
                    bool t_edge = false; // an edge of t: moves with it
                    for (int j = 0; j < 4 && !t_edge; ++j) {
                        const auto& [m, h] = tet_planes[size_t(j)];
                        t_edge = j != tf.i && std::abs(m.dot(y1) - h) <= tol &&
                                 std::abs(m.dot(y2) - h) <= tol;
                    }
                    if (t_edge) continue;
                    // the neighbouring face of Q along this edge: the face whose ring runs it
                    // backwards (a clip result is a closed surface, its faces sharing each edge by
                    // vertex id). Only if none does, the face whose plane holds both endpoints (its
                    // ring may carry extra collinear vertices on the edge); if several do (a
                    // zero-thickness sliver), the one traversing it backwards. The plane test alone
                    // missed the cap of a clipped piece whose crossings lie off its plane by more
                    // than tol (a cut nearly parallel to an edge): MEASURED, t100026, the piece's
                    // cut edge on t lost its term and the Hessian came 3e-4 to 86% off its central
                    // differences; unclipped, the two pieces at such an edge lost theirs together
                    // (exact_mesh.value_grad_hess).
                    int first = -1, ncand = 0;
                    Vec3 gc = Vec3::Zero();
                    {
                        const int ia = Q.ring_id(f)[ee], ib = Q.ring_id(f)[(ee + 1) % kf];
                        for (int g = 0; g < Q.faces() && first < 0; ++g) {
                            if (g == f) continue;
                            const int* rg = Q.ring_id(g);
                            const int kg = Q.size(g);
                            for (int j = 0; j < kg; ++j) {
                                if (rg[j] == ib && rg[(j + 1) % kg] == ia) {
                                    first = g;
                                    ncand = 1;
                                    break;
                                }
                            }
                        }
                    }
                    if (first < 0) { // not found by ids: the planes
                        for (int g = 0; g < Q.faces(); ++g) {
                            if (g == f) continue;
                            const Vec3& ng = Q.N[size_t(g)];
                            const double hg = ng.dot(Q.ring(g)[0]);
                            if (std::abs(ng.dot(y1) - hg) <= tol &&
                                std::abs(ng.dot(y2) - hg) <= tol) {
                                if (first < 0) first = g;
                                ++ncand;
                            }
                        }
                    }
                    if (first < 0) continue;
                    gc = Q.N[size_t(first)];
                    if (ncand > 1) {
                        for (int g = 0; g < Q.faces(); ++g) {
                            if (g == f) continue;
                            const Vec3& ng = Q.N[size_t(g)];
                            const Vec3* Vg = Q.ring(g);
                            const int kg = Q.size(g);
                            const double hg = ng.dot(Vg[0]);
                            if (!(std::abs(ng.dot(y1) - hg) <= tol &&
                                  std::abs(ng.dot(y2) - hg) <= tol))
                                continue;
                            bool back = false;
                            for (int j = 0; j < kg && !back; ++j) {
                                back = maxabs(Vg[j] - y2) <= tol &&
                                       maxabs(Vg[(j + 1) % kg] - y1) <= tol;
                            }
                            if (back) {
                                gc = ng;
                                break;
                            }
                        }
                    }
                    const Vec3 gp = gc - gc.dot(tf.nu) * tf.nu;
                    if (gp.norm() <= 1e-12)
                        continue; // a plane parallel to F does not slide over it
                    const double P3 = cut_integral(y1, y2, e, l0, g0);
                    Ha -= tf.nu * gc.transpose() * (P3 / gp.norm());
                }
                break; // a polygon lies on at most one face of t
            }
        }
        // Q touching a face F of t through x along an edge only (Q has no face on F): F holds a
        // line where two of Q's fixed planes meet. Moving x into t tilts F into Q, whose trace on
        // F is then a strip between those two planes, of width O(|dx|) and zero area at x, but
        // both of its edges slide over F: each face of Q through the edge adds its cut term,
        //     - nu_F (x) g/|g_F| int_edge phi lambda0^2 ds,   g that face's outward normal,
        // the term of the strip's edge on its plane. The pieces on either side of the line on F
        // pair their cut edges with these, not with each other: they meet the line on different
        // planes (g differs), and only the strip's terms make the sum vanish where phi is
        // continuous. MEASURED (stored cube band, turn 1, a front vertex on the symmetry plane
        // x = y, F in that plane through the cube's corner v, so F holds the line x = y = -1/2
        // below v): the face piece (bottom face's prism) and the vertex piece (v's cone) met on
        // that line, on the planes x = -1/2 and y = -1/2, the edge piece between them touching F
        // there only. Without this term the cell's Hessian came 98% off its central differences
        // at every step 1e-3..1e-7 delta (forward and backward differences agreeing to 1.3e-6:
        // no kink), and 26 of the 1100 front vertices' Hessians 5-57% off; with it, 1.5e-9 at
        // most (tests, exact-band-integral-hessian-contact).
        for (size_t fi = 0; fi < faces.size(); ++fi) {
            if (on_tface[fi]) continue;
            const TFace& tf = faces[fi];
            const double hF = tet_planes[size_t(tf.i)].second;
            for (int g = 0; g < Q.faces(); ++g) {
                const Vec3* Vg = Q.ring(g);
                const int kg = Q.size(g);
                const Vec3& ng = Q.N[size_t(g)];
                const Vec3 gp = ng - ng.dot(tf.nu) * tf.nu;
                if (gp.norm() <= 1e-12) continue; // a plane parallel to F does not slide over it
                for (int ee = 0; ee < kg; ++ee) {
                    const Vec3& y1 = Vg[ee];
                    const Vec3& y2 = Vg[(ee + 1) % kg];
                    if (maxabs(y2 - y1) <= tol) continue;
                    // on F: the face test's scale
                    if (std::abs(tf.nu.dot(y1) - hF) > 1e3 * tol ||
                        std::abs(tf.nu.dot(y2) - hF) > 1e3 * tol)
                        continue;
                    bool t_edge = false; // an edge of t: moves with it
                    for (int j = 0; j < 4 && !t_edge; ++j) {
                        const auto& [m, h] = tet_planes[size_t(j)];
                        t_edge = j != tf.i && std::abs(m.dot(y1) - h) <= tol &&
                                 std::abs(m.dot(y2) - h) <= tol;
                    }
                    if (t_edge) continue;
                    const double P3 = cut_integral(y1, y2, e, l0, g0);
                    Ha -= tf.nu * ng.transpose() * (P3 / gp.norm());
                }
            }
        }
    };
    // _vertex_motion: dv/dx (column k = dv/dx_k) and d2v[i](k, l) = d^2 v_i / dx_k dx_l for a vertex
    // v of the polytope R: v solves A(x) v = s(x) for three independent planes of R through v, a
    // face F of t through x the moving one ((x - b) x w).(y - b) = 0 (w = a - b; affine in x),
    // every other plane fixed. Differentiating A v - s = 0 (A, s affine): A dv_k = -r_k with
    // r_k,j = (w_j x (v - b_j))_k on moving rows; A d2v_kl = -[(w_j x dv_l)_k + (w_j x dv_k)_l]_j.
    // Fewer than three independent planes: zero (v fixed).
    const auto vertex_motion = [&](const Vec3& v,
                                   const Poly& R,
                                   const Vec3& xv,
                                   const double tl,
                                   Eigen::Matrix3d& dv,
                                   std::array<Eigen::Matrix3d, 3>& d2v) {
        struct Row
        {
            int moving;
            Vec3 n, w, b;
        };
        std::vector<Row> rows;
        for (int f = 0; f < R.faces(); ++f) {
            const Vec3& nf = R.N[size_t(f)];
            if (std::abs(nf.dot(v) - nf.dot(R.ring(f)[0])) > tl) continue;
            const TFace* mv = nullptr;
            for (const TFace& tf : faces) {
                if (nf.dot(tf.nu) >= 1 - 1e-12 &&
                    std::abs(tf.nu.dot(v) - tet_planes[size_t(tf.i)].second) <= tl) {
                    mv = &tf;
                    break;
                }
            }
            if (mv) {
                rows.push_back({0, (xv - mv->b).cross(mv->a - mv->b), mv->a - mv->b, mv->b});
            } else {
                rows.push_back({1, nf, Vec3::Zero(), R.ring(f)[0]});
            }
        }
        std::stable_sort(rows.begin(), rows.end(), [](const Row& p, const Row& q) {
            return p.moving < q.moving;
        });
        std::vector<const Row*> sel;
        for (const Row& r : rows) {
            const Vec3 nr = r.n / r.n.norm();
            if (sel.size() == 1 && (sel[0]->n / sel[0]->n.norm()).cross(nr).norm() <= 1e-9)
                continue;
            if (sel.size() == 2) {
                Eigen::Matrix3d Nm;
                Nm.row(0) = (sel[0]->n / sel[0]->n.norm()).transpose();
                Nm.row(1) = (sel[1]->n / sel[1]->n.norm()).transpose();
                Nm.row(2) = nr.transpose();
                if (std::abs(Nm.determinant()) <= 1e-9) continue;
            }
            sel.push_back(&r);
            if (sel.size() == 3) break;
        }
        dv.setZero();
        for (auto& m : d2v) m.setZero();
        if (sel.size() < 3) return;
        Eigen::Matrix3d A;
        for (int j = 0; j < 3; ++j) A.row(j) = sel[size_t(j)]->n.transpose();
        const Eigen::Matrix3d Ai = A.inverse();
        for (int k = 0; k < 3; ++k) {
            Vec3 rk = Vec3::Zero();
            for (int j = 0; j < 3; ++j) {
                if (sel[size_t(j)]->moving == 0)
                    rk[j] = sel[size_t(j)]->w.cross(v - sel[size_t(j)]->b)[k];
            }
            dv.col(k) = -Ai * rk;
        }
        for (int k = 0; k < 3; ++k) {
            for (int l = 0; l < 3; ++l) {
                Vec3 q = Vec3::Zero();
                for (int j = 0; j < 3; ++j) {
                    if (sel[size_t(j)]->moving != 0) continue;
                    const Vec3& wj = sel[size_t(j)]->w;
                    q[j] = wj.cross(Vec3(dv.col(l)))[k] + wj.cross(Vec3(dv.col(k)))[l];
                }
                const Vec3 dkl = -Ai * q;
                for (int i = 0; i < 3; ++i) d2v[size_t(i)](k, l) = dkl[i];
            }
        }
    };
    double Jd = 0.;
    for (const Elem& e : pieces) {
        Jd += piece_value(e, cx);
        accumulate(e, G, H);
    }
    // The disagreeing rule-(a) parts D: vol(D) c(x), c = max(0, d_g*(v*) - delta) (class doc)
    for (const DPart& d : Dl) {
        const Poly& R = d.R;
        int gi = -1, vi = -1;
        double cmin = std::numeric_limits<double>::infinity();
        for (size_t m = 0; m < d.members.size(); ++m) {
            double mx = -std::numeric_limits<double>::infinity();
            int im = -1;
            for (size_t j = 0; j < R.P.size(); ++j) {
                const double dv = d_feat(*d.members[m], R.P[j]);
                if (dv > mx) mx = dv, im = int(j);
            }
            if (mx < cmin) cmin = mx, gi = int(m), vi = im;
        }
        const double c = cmin - delta;
        if (!(c > 0.)) continue;
        const double vR = volume_centroid(R).first;
        Elem one;
        one.Q = R;
        one.kind = 0;
        one.dir = Vec3::Zero();
        one.c = -1.; // phi = 1: vol(R)
        Vec3 Gv = Vec3::Zero();
        Eigen::Matrix3d Hv = Eigen::Matrix3d::Zero();
        accumulate(one, Gv, Hv);
        Vec3 gd;
        Eigen::Matrix3d hd;
        d_derivs(*d.members[size_t(gi)], R.P[size_t(vi)], gd, hd);
        Eigen::Matrix3d dv;
        std::array<Eigen::Matrix3d, 3> d2v; // d2v[i](k, l) = d^2 v_i / dx_k dx_l
        vertex_motion(R.P[size_t(vi)], R, x, tol, dv, d2v);
        const Vec3 gc = dv.transpose() * gd;
        Eigen::Matrix3d hc = dv.transpose() * hd * dv;
        for (int i = 0; i < 3; ++i) hc += gd[i] * d2v[size_t(i)];
        Jd += vR * c;
        G += c * Gv + vR * gc;
        H += c * Hv + Gv * gc.transpose() + gc * Gv.transpose() + vR * hc;
    }
    if (clipped) return Jd;
    G = G - delta * gradV;
    return Jd - delta * volume_centroid(Q0).first;
}

void ExactBandIntegral::check_cell_pieces(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    const Work& w,
    const double vol_sum,
    const double empty_vol) const
{
    // The two consistency checks, with bounds derived from the partition's own tolerances.
    //  - A clip that cuts R by the plane (g, h) builds both parts with vertices snapped within tol
    //    of the plane and rings cleaned at tol (max norm), so each part differs from the exact one
    //    only in the slab |g.x - h| <= sqrt(3) tol, whose section of R is at most A(R)/2 <= A(t)/2
    //    (R lies in t, and the surface area of nested convex bodies is monotone): the volume of
    //    the two parts moves by at most sqrt(3) tol A(t) per cutting clip.
    //  - In exact arithmetic the candidates' regions cover t, so a piece with no member exists only
    //    because a decision taken with a tolerance (a clip, or a region test) put a slab of
    //    half-width tol on the wrong side: at most tol A(t) of volume per decision.
    //  - Each volume is a fan sum of m triple products of vectors no longer than D, t's longest
    //    edge: its rounding is at most (13 m + m^2) eps D^3 / 6.
    // A lost or doubled piece or face exceeds these by orders of magnitude (MEASURED: a face kept
    // and capped twice made a sphere band cell 2.6e-6 over its volume, 84000 times its bound
    // 4.3e-16); the former fixed 1e-9 v0 did not scale with the cell's shape.
    const Poly& Q0 = w.Q0;
    const double v0 = volume_centroid(Q0).first;
    double At = 0., D = 0.;
    for (int f = 0; f < Q0.faces(); ++f) {
        const Vec3* V = Q0.ring(f);
        At += 0.5 * (V[1] - V[0]).cross(V[2] - V[0]).norm();
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < i; ++j) D = std::max(D, (T[size_t(i)] - T[size_t(j)]).norm());
    }
    const auto rho = [&](const Poly& Q) {
        double m = 0.;
        for (int f = 0; f < Q.faces(); ++f) m += Q.size(f) - 2;
        return (13. * m + m * m) * EPS * D * D * D / 6.;
    };
    double rho_all = rho(Q0), rho_empty = 0.;
    for (const Piece& pc : w.pieces) {
        rho_all += rho(pc.Q);
        if (pc.S.empty()) rho_empty += rho(pc.Q);
    }
    const double tile_bound = std::sqrt(3.) * At * w.tol_cut + rho_all;
    const double empty_bound = At * (w.tol_any + std::sqrt(3.) * w.tol_cut) + rho_empty;
    if (!(empty_vol <= empty_bound)) {
        log_and_throw_error(
            "ExactBandIntegral: pieces outside every local region, volume {} of {} (bound {}) | {}",
            empty_vol,
            v0,
            empty_bound,
            cell_string(T, delta));
    }
    if (!(std::abs(vol_sum - v0) <= tile_bound)) {
        log_and_throw_error(
            "ExactBandIntegral: pieces do not tile t: {} vs {} (bound {}) | {}",
            vol_sum,
            v0,
            tile_bound,
            cell_string(T, delta));
    }
}

void ExactBandIntegral::check_face_pieces(
    const std::array<Eigen::Vector3d, 3>& f,
    const TriWork& w,
    const char* who) const
{
    // The checks of value(), on areas. A cutting clip moves the parts' boundary only within the
    // slab |g.x - h| <= sqrt(3) tol, whose section of the ring has area at most 2 sqrt(3) tol D
    // (D f's longest edge, its longest chord); a decision taken with a tolerance misplaces at most
    // a slab of half-width tol, area 2 tol D; a fan sum of m terms of size <= D^2 rounds by at
    // most (13 m + m^2) eps D^2 / 2.
    Vec3 nf = (w.V0[1] - w.V0[0]).cross(w.V0[2] - w.V0[0]);
    const double nn = nf.norm();
    if (nn > 0.) nf /= nn;
    double area_sum = 0., empty_area = 0.;
    for (const RingPiece& pc : w.pieces) {
        const double a = ring_area(pc.R, nf);
        area_sum += a;
        if (pc.S.empty()) empty_area += a;
    }
    const double A0 = ring_area(w.V0, nf);
    double D = 0.;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < i; ++j) D = std::max(D, (f[size_t(i)] - f[size_t(j)]).norm());
    }
    const auto rho = [&](const Ring& R) {
        const double m = double(R.size()) - 2.;
        return (13. * m + m * m) * EPS * D * D / 2.;
    };
    double rho_all = rho(w.V0), rho_empty = 0.;
    for (const RingPiece& pc : w.pieces) {
        rho_all += rho(pc.R);
        if (pc.S.empty()) rho_empty += rho(pc.R);
    }
    const double tile_bound = 2. * std::sqrt(3.) * D * w.tol_cut + rho_all;
    const double empty_bound = 2. * D * (w.tol_any + std::sqrt(3.) * w.tol_cut) + rho_empty;
    if (!(empty_area <= empty_bound)) {
        log_and_throw_error(
            "ExactBandIntegral::{}: pieces outside every local region, area {} of {} "
            "(bound {}) | {}",
            who,
            empty_area,
            A0,
            empty_bound,
            face_string(f));
    }
    if (!(std::abs(area_sum - A0) <= tile_bound)) {
        log_and_throw_error(
            "ExactBandIntegral::{}: pieces do not tile f: {} vs {} (bound {}) | {}",
            who,
            area_sum,
            A0,
            tile_bound,
            face_string(f));
    }
}

struct ExactBandIntegral::FaceWork
{
    TriWork w; ///< f's partition
    Vec3 n = Vec3::Zero(); ///< f's unit normal; zero for a flat face
    std::vector<std::pair<Ring, Field>> parts; ///< the labelled parts on the offset side
};

ExactBandIntegral::FaceParts::FaceParts() = default;
ExactBandIntegral::FaceParts::~FaceParts() = default;
ExactBandIntegral::FaceParts::FaceParts(FaceParts&&) noexcept = default;
ExactBandIntegral::FaceParts& ExactBandIntegral::FaceParts::operator=(FaceParts&&) noexcept =
    default;

void ExactBandIntegral::label_face(const std::array<Eigen::Vector3d, 3>& f, FaceWork& fw) const
{
    TriWork& w = fw.w;
    fw.parts.clear();
    tri_partition(f, w);
    check_face_pieces(f, w, "front_face");
    Ctx cx; // the sub-pieces' clips: no tolerance budget (as max_distance())
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    const Ring& V0 = w.V0;
    Vec3 n = (V0[1] - V0[0]).cross(V0[2] - V0[0]);
    const double nn = n.norm();
    fw.n.setZero();
    if (!(nn > 0.)) return; // a flat face
    n /= nn;
    fw.n = n;
    const RingOps ops{n, &cx};
    const auto side_plane = [&](const int k) {
        SidePlane sp;
        if (m_sigma == 0) return sp;
        const size_t gi = size_t(w.cand[size_t(k)]);
        sp.has = m_side_has[gi];
        sp.N = m_side_n[gi];
        sp.ql = m_side_q[gi] - w.c0;
        return sp;
    };
    const auto fail = [&](const int ka,
                          const int kb,
                          const std::vector<Vec3>& Vr,
                          const bool curved) {
        Vec3 c = Vec3::Zero();
        for (const Vec3& y : Vr) c += y;
        c = c / double(Vr.size()) + w.c0;
        log_and_throw_error(
            "ExactBandIntegral::front_face: {} between feature {} ({}) and feature {} ({}) on a "
            "piece at ({}, {}, {}) | {}",
            curved ? "the bisector is curved and crosses" : "members still cross after the split",
            w.cand[size_t(ka)],
            kind_name(loc[size_t(ka)].kind),
            w.cand[size_t(kb)],
            kind_name(loc[size_t(kb)].kind),
            c[0],
            c[1],
            c[2],
            face_string(f));
    };
    std::vector<int> S;
    std::vector<std::pair<Ring, int>> cells;
    for (const RingPiece& pc : w.pieces) {
        if (pc.S.empty()) continue;
        S = pc.S.size() > 1 ? prune_points(pc.R, pc.S, loc) : pc.S;
        cells.clear();
        resolve(pc.R, S, loc, ops, w.scale, 0, cells, fail);
        for (const auto& [C, k] : cells) {
            labeled(C, loc[size_t(k)], side_plane(k), m_sigma, ops, fw.parts);
        }
    }
}

const ExactBandIntegral::FaceWork& ExactBandIntegral::face_parts(
    const std::array<Eigen::Vector3d, 3>& f,
    FaceParts* keep) const
{
    double key[9];
    for (int i = 0; i < 3; ++i) std::memcpy(key + 3 * i, f[size_t(i)].data(), 3 * sizeof(double));
    if (keep && keep->m_w && keep->m_owner == m_id &&
        std::memcmp(keep->m_key, key, sizeof(key)) == 0) {
        return *keep->m_w;
    }
    // without `keep` a per-thread scratch, its storage reused from call to call (as
    // cell_partition())
    thread_local FaceWork scratch;
    FaceWork* fw = &scratch;
    if (keep) {
        if (!keep->m_w) keep->m_w = std::make_unique<FaceWork>();
        keep->m_owner = 0; // none until the parts are made (label_face() may throw)
        fw = keep->m_w.get();
    }
    label_face(f, *fw);
    if (keep) {
        std::memcpy(keep->m_key, key, sizeof(key));
        keep->m_owner = m_id;
    }
    return *fw;
}

double ExactBandIntegral::front_face(
    const std::array<Eigen::Vector3d, 3>& f,
    const double delta,
    FaceParts* keep) const
{
    const FaceWork& fw = face_parts(f, keep);
    if (fw.n.isZero(0.)) return 0.; // a flat face: no area
    Ctx cx;
    cx.scale = fw.w.scale;
    double val = 0.;
    for (const auto& [R, fd] : fw.parts) val += flux(fd, R.data(), int(R.size()), fw.n, delta, cx);
    return val;
}

void ExactBandIntegral::front_face_derivs(
    const std::array<Eigen::Vector3d, 3>& f,
    const double delta,
    const int kv,
    Eigen::Vector3d& grad,
    Eigen::Matrix3d& hess,
    FaceParts* keep) const
{
    grad.setZero();
    hess.setZero();
    const FaceWork& fw = face_parts(f, keep);
    const TriWork& w = fw.w;
    const Vec3& n = fw.n;
    if (n.isZero(0.)) return; // a flat face: no area, every term 0
    Ctx cx;
    cx.scale = w.scale;
    const Ring& V0 = w.V0;
    // phi = l0 + g0.y, the hat function of corner kv: 1 there, 0 on the opposite edge; f ordered
    // (x, b, c) from kv, N = (b - x) x (c - x), column l of dN/dx = e_l x (b - c)
    const Vec3 &x = V0[size_t(kv)], &b = V0[size_t((kv + 1) % 3)], &c = V0[size_t((kv + 2) % 3)];
    const Vec3 m = n.cross(c - b);
    const Vec3 g0 = m / (x - b).dot(m);
    const double l0 = -g0.dot(b);
    const double nN = (b - x).cross(c - x).norm();
    Eigen::Matrix3d dN;
    for (int l = 0; l < 3; ++l) dN.col(l) = Vec3::Unit(l).cross(b - c);
    const auto lam = [&](const Vec3& Y) { return l0 + Y.dot(g0); };
    // a part on the input (d = 0 on every vertex): grad d from the band side (front_face() doc)
    const double tolS = 64 * EPS * (ring_max_abs(V0) + maxabs(w.c0));
    double gi = 0.;
    Vec3 P2 = Vec3::Zero();
    Eigen::Matrix3d Hcut = Eigen::Matrix3d::Zero();
    for (const auto& [R, fd] : fw.parts) {
        const Vec3* V = R.data();
        const int nk = int(R.size());
        if (fd.kind == 0) {
            gi += poly_polygon(V, nk, [&](const Vec3& Y) {
                return (Y.dot(fd.dir) - fd.c - delta) * lam(Y);
            });
            // Without a side both affine sides are parts, and a part lying on the plane takes
            // the side labeled() met first; grad d is then the band side's, -n (n out of the
            // band; Python e_hessian.py). With a side the part IS the offset side, the side the
            // band is on: its own W.
            double on = 0.;
            for (int i = 0; i < nk; ++i) on = std::max(on, std::abs(V[i].dot(fd.dir) - fd.c));
            const Vec3 W = !fd.far && on <= tolS ? Vec3(-n) : fd.dir;
            P2 += W * poly_polygon(V, nk, [&](const Vec3& Y) {
                      const double l = lam(Y);
                      return l * l;
                  });
            if (!fd.far) continue;
            // The cut edges: where f crosses the face's plane into the far side, chi (d - delta)
            // jumps from -delta to 0, and the line slides over f as x moves (value_grad_hess()'s
            // cut edges, which the offset side keeps there too). An edge of f moves with it: no
            // term. A part lying on the plane has no cut edge (g_c along n: g_c,f = 0).
            // MEASURED (tests, a ring astride a cube face): without these terms the Hessian came
            // 2.1 (relative) off its central differences and 1.3 off the "exact" rule's.
            const double tolc = 64 * EPS * (std::abs(fd.c) + std::max(ring_max_abs(R), w.scale));
            const Vec3 gc = -fd.dir; // out of the kept side
            const Vec3 gp = gc - gc.dot(n) * n;
            if (gp.norm() <= 1e-12) continue;
            for (int i = 0; i < nk; ++i) {
                const Vec3 &y1 = V[i], &y2 = V[(i + 1) % nk];
                if (maxabs(y2 - y1) <= tolc) continue;
                if (std::abs(y1.dot(fd.dir) - fd.c) > tolc ||
                    std::abs(y2.dot(fd.dir) - fd.c) > tolc) {
                    continue;
                }
                bool f_edge = false;
                for (int e = 0; e < 3 && !f_edge; ++e) {
                    const Vec3& P = V0[size_t(e)];
                    const Vec3 t = (V0[size_t((e + 1) % 3)] - P).normalized();
                    const auto off = [&](const Vec3& y) {
                        const Vec3 r = y - P;
                        return maxabs(r - r.dot(t) * t);
                    };
                    f_edge = off(y1) <= tolc && off(y2) <= tolc;
                }
                if (f_edge) continue;
                Elem e; // phi = d - delta on the kept side
                e.kind = 0;
                e.dir = fd.dir;
                e.c = fd.c + delta;
                const double P3 = cut_integral(y1, y2, e, l0, g0);
                Hcut -= n * gc.transpose() * (P3 / gp.norm());
            }
            continue;
        }
        const auto [A, cen] = area_centroid(V, nk, n);
        double p1;
        Vec3 p2;
        if (fd.kind == 1) {
            face_line_derivs(V, nk, n, fd.p, fd.dir, l0, g0, p1, p2);
        } else {
            face_point_derivs(V, nk, n, fd.p, l0, g0, p1, p2, cx);
        }
        gi += p1 - delta * A * (l0 + g0.dot(cen));
        P2 += p2;
    }
    grad = gi * n;
    hess = n * P2.transpose() + (gi / nN) * dN + Hcut;
}

double ExactBandIntegral::bisector_jump(
    const std::array<Eigen::Vector3d, 4>& T,
    const double delta,
    Partition* keep) const
{
    const Work* wp = cell_partition(T, nullptr, nullptr, keep);
    if (!wp) return 0.;
    const Work& w = *wp;
    Ctx cx;
    cx.scale = w.scale;
    const std::vector<Loc>& loc = w.loc;
    const Poly& Q0 = w.Q0;
    const PolyOps ops{&cx};
    double vol_sum = 0., empty_vol = 0.;
    for (const Piece& pc : w.pieces) {
        const double v = volume_centroid(pc.Q).first;
        vol_sum += v;
        if (pc.S.empty()) empty_vol += v;
    }
    check_cell_pieces(T, delta, w, vol_sum, empty_vol);
    const auto side_plane = [&](const int k) {
        SidePlane sp;
        if (m_sigma == 0) return sp;
        const size_t gi = size_t(w.cand[size_t(k)]);
        sp.has = m_side_has[gi];
        sp.N = m_side_n[gi];
        sp.ql = m_side_q[gi] - w.c0;
        return sp;
    };
    const auto fail = [&](const int ka,
                          const int kb,
                          const std::vector<Vec3>& Vr,
                          const bool curved) {
        Vec3 c = Vec3::Zero();
        for (const Vec3& y : Vr) c += y;
        c = c / double(Vr.size()) + w.c0;
        log_and_throw_error(
            "ExactBandIntegral::bisector_jump: {} between feature {} ({}) and feature {} ({}) on "
            "a piece at ({}, {}, {}) | {}",
            curved ? "the bisector is curved and crosses" : "members still cross after the split",
            w.cand[size_t(ka)],
            kind_name(loc[size_t(ka)].kind),
            w.cand[size_t(kb)],
            kind_name(loc[size_t(kb)].kind),
            c[0],
            c[1],
            c[2],
            cell_string(T, delta));
    };
    // the labelled parts on the offset side
    struct Part
    {
        Poly R;
        Field fd;
        int k;
    };
    std::vector<Part> kept;
    std::vector<int> S;
    std::vector<std::pair<Poly, int>> cells;
    std::vector<std::pair<Poly, Field>> parts;
    bool one_label = true;
    for (const Piece& pc : w.pieces) {
        if (pc.S.empty()) continue;
        S = pc.S.size() > 1 ? prune_members(pc.Q, pc.S, loc) : pc.S;
        cells.clear();
        resolve(pc.Q, S, loc, ops, w.scale, 0, cells, fail);
        for (const auto& [C, k] : cells) {
            parts.clear();
            labeled(C, loc[size_t(k)], side_plane(k), m_sigma, ops, parts);
            for (auto& [R, fd] : parts) {
                one_label = one_label && (kept.empty() || kept.front().k == k);
                kept.push_back({std::move(R), fd, k});
            }
        }
    }
    // One feature labelling every kept part: G is one field on them, and D_t = 0 exactly.
    if (one_label) return 0.;
    // Otherwise D_t = sum over the parts C of oint_dC G.n dA - oint_dt G.n dA (sum over C of
    // int_C div G = J_t): the flux of each part's field through its faces not on dt. A face on
    // one of the label's own region planes carries none (G_k is tangent to them: a face's prism
    // planes hold its normal, an edge's and a vertex's planes pass through it or are
    // perpendicular to it), and is skipped. The faces between two parts of one label cancel;
    // the jumps are where the label changes. The prototype summed only the faces lying on a
    // planar bisector of the label and another candidate (within 1e3 16 eps of the scale); that
    // missed the jumps across a region plane NEAR a bisector: MEASURED (Geneva base, inward,
    // stored side cells 0 and 7): the label changed across the prism plane of a nearly coplanar
    // neighbour face, 2.4e-8 and 1.1e-7 from their bisector, and D_t came 8.7e-11 and 2.6e-10
    // off J_t - oint_dt G.n dA (1.1e-8 and 1.4e-8 of delta vol); with every face, 1.9e-16.
    std::array<std::pair<Vec3, double>, 4> tet_planes;
    for (int i = 0; i < 4; ++i) {
        tet_planes[size_t(i)] = {Q0.N[size_t(i)], Q0.N[size_t(i)].dot(Q0.ring(i)[0])};
    }
    double tot = 0.;
    for (const Part& p : kept) {
        const Loc& g = loc[size_t(p.k)];
        const Poly& R = p.R;
        for (int f = 0; f < R.faces(); ++f) {
            const Vec3* V = R.ring(f);
            const int nk = R.size(f);
            const Vec3& nf = R.N[size_t(f)];
            double ma = 0.;
            for (int i = 0; i < nk; ++i) ma = std::max(ma, maxabs(V[i]));
            const double tol = 1e3 * 16 * EPS * std::max(ma, w.scale);
            const auto on = [&](const Vec3& gn, const double h) { // a supporting plane of the part
                if (nf.dot(gn) < 1 - 1e-12) return false;
                for (int i = 0; i < nk; ++i) {
                    if (std::abs(V[i].dot(gn) - h) > tol) return false;
                }
                return true;
            };
            bool skip = false;
            for (const auto& [gn, h] : tet_planes) skip = skip || on(gn, h);
            for (int i = g.h_begin; i < g.h_end && !skip; ++i) {
                skip = on(w.hs[size_t(i)].first, w.hs[size_t(i)].second);
            }
            if (!skip) tot += flux(p.fd, V, nk, nf, delta, cx);
        }
    }
    return tot;
}

void ExactBandIntegral::prepare_front_flux(const double delta)
{
    const double R = 2. * delta;
    m_ff_reach = R;
    m_ff_boxes.clear();
    m_ff_nodes.clear();
    m_ff_order.clear();
    Ctx cx;
    std::vector<int> near;
    Poly Q, tmp;
    const Vec3 r2 = Vec3::Constant(2. * R), r1 = Vec3::Constant(R);
    for (int g = 0; g < int(m_feat.size()); ++g) {
        const Feature& fg = m_feat[size_t(g)];
        candidates(fg.lo - r2, fg.hi + r2, 0., near); // every h whose box grown by R meets g's
        for (const int h : near) {
            if (h <= g) continue;
            const Feature& fh = m_feat[size_t(h)];
            const Vec3 lo = (fg.lo - r1).cwiseMax(fh.lo - r1),
                       hi = (fg.hi + r1).cwiseMin(fh.hi + r1);
            if ((lo.array() > hi.array()).any()) continue;
            const Vec3 c0 = 0.5 * (lo + hi);
            box_polytope(lo - c0, hi - c0, Q);
            cx.scale = maxabs(hi - c0);
            bool alive = true;
            for (const Feature* ft : {&fg, &fh}) {
                for (int i = ft->h_begin; i < ft->h_end && alive; ++i) {
                    const auto& [gn, q] = m_hs[size_t(i)];
                    cx.scale = std::max(cx.scale, maxabs(q - c0));
                    alive = clip_inplace(Q, gn, gn.dot(q - c0), tmp, cx);
                }
            }
            for (const int k : {g, h}) { // the offset side of both
                if (!alive || m_sigma == 0 || !m_side_has[size_t(k)]) continue;
                const Vec3 N = double(m_sigma) * m_side_n[size_t(k)];
                alive = clip_inplace(Q, -N, -N.dot(m_side_q[size_t(k)] - c0), tmp, cx);
            }
            if (!alive || !(volume_centroid(Q).first > 0.)) continue;
            FFBox b;
            b.lo = Vec3::Constant(std::numeric_limits<double>::infinity());
            b.hi = -b.lo;
            for (const Vec3& p : Q.V) {
                b.lo = b.lo.cwiseMin(p + c0);
                b.hi = b.hi.cwiseMax(p + c0);
            }
            for (int f = 0; f < Q.faces(); ++f) {
                b.planes.push_back({Q.N[size_t(f)], Q.N[size_t(f)].dot(Q.ring(f)[0] + c0)});
            }
            m_ff_boxes.push_back(std::move(b));
        }
    }
    if (m_ff_boxes.empty()) return;
    m_ff_order.resize(m_ff_boxes.size());
    std::iota(m_ff_order.begin(), m_ff_order.end(), 0);
    m_ff_nodes.reserve(2 * m_ff_boxes.size());
    build_ff_node(0, int(m_ff_boxes.size()));
}

int ExactBandIntegral::build_ff_node(const int begin, const int end)
{
    const int id = int(m_ff_nodes.size());
    m_ff_nodes.emplace_back();
    Vec3 lo = Vec3::Constant(std::numeric_limits<double>::infinity());
    Vec3 hi = -lo, clo = lo, chi = -lo;
    for (int j = begin; j < end; ++j) {
        const FFBox& b = m_ff_boxes[size_t(m_ff_order[size_t(j)])];
        lo = lo.cwiseMin(b.lo);
        hi = hi.cwiseMax(b.hi);
        clo = clo.cwiseMin(0.5 * (b.lo + b.hi));
        chi = chi.cwiseMax(0.5 * (b.lo + b.hi));
    }
    m_ff_nodes[size_t(id)].lo = lo;
    m_ff_nodes[size_t(id)].hi = hi;
    m_ff_nodes[size_t(id)].begin = begin;
    m_ff_nodes[size_t(id)].end = end;
    if (end - begin <= 4) return id;
    int axis = 0;
    (chi - clo).maxCoeff(&axis);
    const int mid = (begin + end) / 2;
    const auto cen = [&](const int a) {
        const FFBox& b = m_ff_boxes[size_t(a)];
        return b.lo[axis] + b.hi[axis];
    };
    std::nth_element(
        m_ff_order.begin() + begin,
        m_ff_order.begin() + mid,
        m_ff_order.begin() + end,
        [&](const int a, const int b) { return cen(a) < cen(b) || (cen(a) == cen(b) && a < b); });
    const int left = build_ff_node(begin, mid);
    const int right = build_ff_node(mid, end);
    m_ff_nodes[size_t(id)].left = left;
    m_ff_nodes[size_t(id)].right = right;
    return id;
}

bool ExactBandIntegral::front_flux_flag(const Eigen::Vector3d* t, const int n) const
{
    if (m_ff_nodes.empty() || n <= 0) return false;
    Vec3 tlo = t[0], thi = t[0];
    double ma = 0.;
    for (int i = 0; i < n; ++i) {
        const Vec3& p = t[i];
        tlo = tlo.cwiseMin(p);
        thi = thi.cwiseMax(p);
        ma = std::max(ma, maxabs(p));
    }
    const auto apart = [&](const Vec3& lo, const Vec3& hi) {
        return (lo.array() > thi.array()).any() || (hi.array() < tlo.array()).any();
    };
    thread_local std::vector<int> stack;
    stack.assign(1, 0);
    while (!stack.empty()) {
        const Node& nd = m_ff_nodes[size_t(stack.back())];
        stack.pop_back();
        if (apart(nd.lo, nd.hi)) continue;
        if (nd.left >= 0) {
            stack.push_back(nd.left);
            stack.push_back(nd.right);
            continue;
        }
        for (int j = nd.begin; j < nd.end; ++j) {
            const FFBox& b = m_ff_boxes[size_t(m_ff_order[size_t(j)])];
            if (apart(b.lo, b.hi)) continue;
            // separated if one face plane has all the points outside it beyond rounding (a cell
            // touching the polytope only on its boundary has no part of its interior in it)
            bool separated = false;
            for (const auto& [g, h] : b.planes) {
                double smin = std::numeric_limits<double>::infinity();
                for (int i = 0; i < n; ++i) smin = std::min(smin, t[i].dot(g) - h);
                if (smin > 64 * EPS * (std::abs(h) + ma)) {
                    separated = true;
                    break;
                }
            }
            if (!separated) return true;
        }
    }
    return false;
}

} // namespace wmtk::components::topological_offset
