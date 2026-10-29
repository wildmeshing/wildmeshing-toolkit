// Volume maps of single operations. A small tet mesh goes through them by hand, recorded as
// TetMesh records its own (TetMesh.cpp): at each one the tracked faces around it, before and
// after, to a TrackedSurfaceRecorder, and the tets at its vertices to a VolumeRecorder. The
// map of the two histories must then take every point into its own region, orientation kept
// (det J > 0), and back.

#include <wmtk/components/tracking/VolumeMap.hpp>
#include <wmtk/components/tracking/volume_geometry.hpp>
#include <wmtk/history/TrackedRecorder.hpp>
#include <wmtk/history/VolumeHistory.hpp>

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tracking;

namespace {

using MTet = std::array<size_t, 4>;
using MFace = std::array<size_t, 3>;

MFace sorted3(MFace f)
{
    std::sort(f.begin(), f.end());
    return f;
}

MTet sorted4(MTet t)
{
    std::sort(t.begin(), t.end());
    return t;
}

bool has(const MTet& t, const size_t v)
{
    return std::find(t.begin(), t.end(), v) != t.end();
}

std::array<MFace, 4> faces_of(const MTet& t)
{
    return {
        {sorted3({{t[1], t[2], t[3]}}),
         sorted3({{t[0], t[2], t[3]}}),
         sorted3({{t[0], t[1], t[3]}}),
         sorted3({{t[0], t[1], t[2]}})}};
}

/// A state of a scene: positions, tets and the side of each.
struct State
{
    std::vector<Eigen::Vector3d> P;
    std::vector<MTet> T;
    std::vector<int64_t> side;
};

/**
 * A tet mesh whose tets each have a side, 0 or 1, and some of whose faces are tracked -- at
 * first the faces between the sides. A region is a side (simwild), or, with one_region, all the
 * tets are one region and the tracked faces stand apart (tetwild). Tets are kept positively
 * oriented (volume::orient), and an operation that would make one flat or inverted is refused.
 */
class Scene
{
public:
    State s;
    std::set<MFace> tracked;
    bool one_region = false;
    /// How far from the plane the grid's surface starts on a surface vertex may go.
    double envelope = 1;
    /// The state after each operation the volume history recorded.
    std::vector<State> after_edit;

    /// n^3 unit cubes, each in six tets (Kuhn), side 1 above z = z0 and 0 below, the faces
    /// between tracked. Vertices off the box's faces move by up to 1/16 in each coordinate,
    /// and those on z = z0 only within the plane: the sides stay on their sides.
    static Scene grid(const int n, const int z0, const uint32_t seed = 1)
    {
        Scene g;
        g.m_n = n;
        g.m_z0 = z0;
        std::mt19937 rng(seed);
        const auto jitter = [&rng]() { return double(int(rng() % 9) - 4) / 64; };
        for (int x = 0; x <= n; ++x) {
            for (int y = 0; y <= n; ++y) {
                for (int z = 0; z <= n; ++z) {
                    Eigen::Vector3d p(x, y, z);
                    const int c[3] = {x, y, z};
                    for (int k = 0; k < 3; ++k) {
                        const bool on_box = c[k] == 0 || c[k] == n;
                        const bool on_plane = k == 2 && z == z0;
                        if (!on_box && !on_plane) p[k] += jitter();
                    }
                    g.s.P.push_back(p);
                }
            }
        }
        const int perms[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
        for (int x = 0; x < n; ++x) {
            for (int y = 0; y < n; ++y) {
                for (int z = 0; z < n; ++z) {
                    for (const auto& p : perms) {
                        int c[3] = {x, y, z};
                        MTet t;
                        t[0] = g.vertex(c[0], c[1], c[2]);
                        for (int k = 0; k < 3; ++k) {
                            ++c[p[k]];
                            t[k + 1] = g.vertex(c[0], c[1], c[2]);
                        }
                        if (g.orient(t) < 0) std::swap(t[2], t[3]);
                        g.s.T.push_back(t);
                        g.s.side.push_back(z >= z0 ? 1 : 0);
                    }
                }
            }
        }
        for (const MTet& t : g.s.T) REQUIRE(g.orient(t) > 0);
        g.m_box_lo = Eigen::Vector3d::Zero();
        g.m_box_hi = Eigen::Vector3d::Constant(n);
        g.track_interface();
        return g;
    }

    /// A mesh given as it is, tracked faces and all; its box is that of its vertices.
    static Scene of(State st, const std::vector<MFace>& tracked_faces, const bool one_region)
    {
        Scene g;
        g.s = std::move(st);
        for (const MFace& f : tracked_faces) g.tracked.insert(sorted3(f));
        g.one_region = one_region;
        g.m_box_lo = g.m_box_hi = g.s.P[0];
        for (const auto& p : g.s.P) {
            g.m_box_lo = g.m_box_lo.cwiseMin(p);
            g.m_box_hi = g.m_box_hi.cwiseMax(p);
        }
        for (const MTet& t : g.s.T) REQUIRE(g.orient(t) > 0);
        return g;
    }

    size_t vertex(const int x, const int y, const int z) const
    {
        return size_t((x * (m_n + 1) + y) * (m_n + 1) + z);
    }

    /// Tracks the faces between tets of different sides, only those.
    void track_interface()
    {
        tracked.clear();
        std::map<MFace, std::vector<size_t>> at;
        for (size_t t = 0; t < s.T.size(); ++t) {
            for (const MFace& f : faces_of(s.T[t])) at[f].push_back(t);
        }
        for (const auto& [f, ts] : at) {
            if (ts.size() == 2 && s.side[ts[0]] != s.side[ts[1]]) tracked.insert(f);
        }
    }

    /// Starts recording, from here. The scene must not move from then on.
    void start()
    {
        m_start = s;
        m_volume = std::make_unique<history::VolumeRecorder>();
        std::vector<MFace> faces(tracked.begin(), tracked.end());
        m_surface = std::make_shared<history::TrackedSurfaceRecorder>(
            faces,
            [this](const size_t v) { return s.P[v]; },
            [this](const MFace& f) { return oriented(f); });
    }

    const State& first() const { return m_start; }
    history::SurfaceHistory surface_history() const { return m_surface->history(); }
    history::VolumeHistory volume_history() const
    {
        history::VolumeHistory h = m_volume->history();
        h.surface = history::surface_key(m_surface->history());
        return h;
    }
    /// The tags of a tet of a side, which make its region (TetMesh::tet_tags).
    std::vector<int64_t> tags_of(const int64_t side) const
    {
        return one_region ? std::vector<int64_t>{} : std::vector<int64_t>{side};
    }

    /// Collapses i into j, if that keeps the mesh a manifold, its tets the right way round and
    /// none much worse than they were (as the applications ask), the box where it is (i inside
    /// it), and the tracked faces it moves off others; a vertex of the surface stays within the
    /// envelope, and the sheet's boundary where it is (unless `sheet_boundary`).
    bool collapse(const size_t i, const size_t j, const bool sheet_boundary = false)
    {
        if (on_box(i)) return false;
        if (on_surface(i)) {
            if (!in_envelope(s.P[j])) return false;
            if (on_box(j) && !sheet_boundary) return false;
        }
        const std::vector<size_t> Si = tets_at({i});
        if (std::none_of(Si.begin(), Si.end(), [&](size_t t) { return has(s.T[t], j); })) {
            return false;
        }
        std::set<MTet> others;
        double worst_before = 1;
        for (const size_t t : tets_at({j})) {
            if (!has(s.T[t], i)) others.insert(sorted4(s.T[t]));
        }
        for (const size_t t : tets_at({i, j})) worst_before = std::min(worst_before, shape(s.T[t]));
        for (const size_t t : Si) {
            if (has(s.T[t], j)) continue;
            MTet a = s.T[t];
            std::replace(a.begin(), a.end(), i, j);
            if (orient(a) <= 0 || others.count(sorted4(a))) return false;
            if (shape(a) < std::min(worst_before, kShape)) return false;
        }
        if (!link_condition(i, j)) return false;
        std::set<MFace> after;
        for (MFace f : tracked) {
            const bool hi = std::find(f.begin(), f.end(), i) != f.end();
            const bool hj = std::find(f.begin(), f.end(), j) != f.end();
            if (hi && hj) continue;
            if (hi) std::replace(f.begin(), f.end(), i, j);
            if (!after.insert(sorted3(f)).second) return false;
        }
        const history::VolumeCapture before = capture({i, j});
        history::TrackedSurfaceRecorder::Change c;
        c.kind = history::EditKind::collapse_edge;
        c.v = {i, j};
        c.before = tracked_around({i, j}, false);
        c.written = {j};
        State next;
        next.P = s.P;
        for (size_t t = 0; t < s.T.size(); ++t) {
            if (has(s.T[t], i) && has(s.T[t], j)) continue;
            MTet a = s.T[t];
            std::replace(a.begin(), a.end(), i, j);
            next.T.push_back(a);
            next.side.push_back(s.side[t]);
        }
        s = std::move(next);
        tracked = std::move(after);
        c.after = tracked_around({j}, false);
        const auto edits = m_surface->commit(std::move(c));
        const size_t recorded = m_volume->size();
        m_volume->commit(history::VolumeOp::collapse, edits, {i, j}, before, capture({j}));
        if (m_volume->size() > recorded) after_edit.push_back(s);
        return true;
    }

    /// Replaces the tets `removed` (indices) by `added`, each turned the right way round and
    /// given the side of the tet its centroid was in; the faces between the added ones are
    /// tracked where their sides differ -- or, if given, the faces of the added ones tracked
    /// are `tracked_after`, those of the removed ones no longer. Refused if a tet is flat, or
    /// a side's volume changes (the volume, with `tracked_after`).
    bool swap(
        const std::vector<size_t>& removed,
        std::vector<MTet> added,
        const history::VolumeOp op,
        const std::vector<MFace>* tracked_after = nullptr)
    {
        std::vector<size_t> ring;
        for (const size_t t : removed) ring.insert(ring.end(), s.T[t].begin(), s.T[t].end());
        std::sort(ring.begin(), ring.end());
        ring.erase(std::unique(ring.begin(), ring.end()), ring.end());
        std::vector<int64_t> sides;
        double volume[2] = {0, 0};
        for (const size_t t : removed) volume[s.side[t]] += signed_volume(s.T[t]);
        for (MTet& a : added) {
            if (orient(a) < 0) std::swap(a[2], a[3]);
            if (orient(a) <= 0) return false;
            Eigen::Vector3d c = Eigen::Vector3d::Zero();
            for (const size_t v : a) c += s.P[v] / 4;
            int64_t side = -1;
            for (const size_t t : removed) {
                const MTet& b = s.T[t];
                if (volume::inside(c, s.P[b[0]], s.P[b[1]], s.P[b[2]], s.P[b[3]])) side = s.side[t];
            }
            if (side < 0) return false;
            sides.push_back(side);
            volume[side] -= signed_volume(a);
        }
        if (tracked_after ? std::abs(volume[0] + volume[1]) > 1e-12
                          : std::abs(volume[0]) > 1e-12 || std::abs(volume[1]) > 1e-12) {
            return false;
        }
        const history::VolumeCapture before = capture(ring);
        history::TrackedSurfaceRecorder::Change c;
        c.kind = history::EditKind::swap_edge;
        c.before = tracked_around(ring, true);
        std::map<MFace, std::vector<size_t>> faces_before, faces_after;
        for (const size_t t : removed) {
            for (const MFace& f : faces_of(s.T[t])) faces_before[f].push_back(t);
        }
        for (size_t k = 0; k < added.size(); ++k) {
            for (const MFace& f : faces_of(added[k])) faces_after[f].push_back(k);
        }
        if (tracked_after) {
            for (const auto& [f, ts] : faces_before) tracked.erase(f);
            for (const MFace& f : *tracked_after) tracked.insert(sorted3(f));
        } else {
            for (const auto& [f, ts] : faces_before) {
                if (ts.size() == 2) tracked.erase(f);
            }
            for (const auto& [f, ks] : faces_after) {
                if (ks.size() == 2 && sides[ks[0]] != sides[ks[1]]) tracked.insert(f);
            }
        }
        State next;
        next.P = s.P;
        const std::set<size_t> gone(removed.begin(), removed.end());
        for (size_t t = 0; t < s.T.size(); ++t) {
            if (gone.count(t)) continue;
            next.T.push_back(s.T[t]);
            next.side.push_back(s.side[t]);
        }
        for (size_t k = 0; k < added.size(); ++k) {
            next.T.push_back(added[k]);
            next.side.push_back(sides[k]);
        }
        s = std::move(next);
        c.after = tracked_around(ring, true);
        const auto edits = m_surface->commit(std::move(c));
        const size_t recorded = m_volume->size();
        m_volume->commit(op, edits, {}, before, capture(ring));
        if (m_volume->size() > recorded) after_edit.push_back(s);
        return true;
    }

    /// The 4-4 swap of edge (a, b) into the edge between two of its ring's vertices, c and d.
    bool swap_44(const size_t a, const size_t b, const size_t c, const size_t d)
    {
        std::vector<size_t> around;
        std::set<size_t> ring;
        for (const size_t t : tets_at({a})) {
            if (!has(s.T[t], b)) continue;
            around.push_back(t);
            for (const size_t v : s.T[t]) {
                if (v != a && v != b) ring.insert(v);
            }
        }
        if (around.size() != 4 || ring.size() != 4 || !ring.count(c) || !ring.count(d)) {
            return false;
        }
        ring.erase(c);
        ring.erase(d);
        std::vector<MTet> added;
        for (const size_t e : ring) {
            added.push_back({{c, d, a, e}});
            added.push_back({{c, d, b, e}});
        }
        return swap(around, added, history::VolumeOp::swap_44);
    }

    /// Moves v, inside the box, to x, if its tets stay the right way round and none much
    /// worse than they were.
    bool move(const size_t v, const Eigen::Vector3d& x)
    {
        if (on_box(v)) return false;
        if (on_surface(v) && !in_envelope(x)) return false;
        const history::VolumeCapture before = capture({v});
        double worst_before = 1;
        for (const size_t t : tets_at({v})) worst_before = std::min(worst_before, shape(s.T[t]));
        const Eigen::Vector3d old = s.P[v];
        s.P[v] = x;
        for (const size_t t : tets_at({v})) {
            if (orient(s.T[t]) <= 0 || shape(s.T[t]) < std::min(worst_before, kShape)) {
                s.P[v] = old;
                return false;
            }
        }
        history::TrackedSurfaceRecorder::Change c;
        c.kind = history::EditKind::move_vertices;
        c.written = {v};
        const auto edits = m_surface->commit(std::move(c));
        const size_t recorded = m_volume->size();
        m_volume->commit(history::VolumeOp::smooth, edits, {v}, before, capture({v}));
        if (m_volume->size() > recorded) after_edit.push_back(s);
        return true;
    }

    int orient(const MTet& t) const
    {
        return volume::orient(s.P[t[0]], s.P[t[1]], s.P[t[2]], s.P[t[3]]);
    }

    /// A tet's volume over that of the regular tet with its mean squared edge length: 1 at
    /// best, 0 when flat.
    double shape(const MTet& t) const
    {
        double l2 = 0;
        for (int a = 0; a < 4; ++a) {
            for (int b = a + 1; b < 4; ++b) l2 += (s.P[t[a]] - s.P[t[b]]).squaredNorm() / 6;
        }
        return signed_volume(t) / (std::pow(l2, 1.5) / (6 * std::sqrt(2.)));
    }

    /// Whether v is a corner of a tracked face.
    bool on_surface(const size_t v) const
    {
        return std::any_of(tracked.begin(), tracked.end(), [v](const MFace& f) {
            return f[0] == v || f[1] == v || f[2] == v;
        });
    }

    /// Whether x is within a cube's side of the plane the grid's surface starts on, as an
    /// application's envelope keeps its surface near the input; anywhere for a scene with no
    /// grid.
    bool in_envelope(const Eigen::Vector3d& x) const
    {
        return m_n == 0 || std::abs(x[2] - m_z0) <= envelope;
    }

    /// Whether v is on the box's faces, which the operations keep where they are.
    bool on_box(const size_t v) const
    {
        for (int k = 0; k < 3; ++k) {
            if (m_box_lo[k] == s.P[v][k] || m_box_hi[k] == s.P[v][k]) return true;
        }
        return false;
    }

private:
    /// The shape no tet an operation makes may be below, unless the worst it replaces is.
    static constexpr double kShape = 0.02;

    double signed_volume(const MTet& t) const
    {
        const auto& P = s.P;
        return (P[t[1]] - P[t[0]]).dot((P[t[2]] - P[t[0]]).cross(P[t[3]] - P[t[0]])) / 6;
    }

    std::vector<size_t> tets_at(const std::vector<size_t>& vs) const
    {
        std::vector<size_t> out;
        for (size_t t = 0; t < s.T.size(); ++t) {
            if (std::any_of(vs.begin(), vs.end(), [&](size_t v) { return has(s.T[t], v); })) {
                out.push_back(t);
            }
        }
        return out;
    }

    /// TetMesh::volume_capture.
    history::VolumeCapture capture(const std::vector<size_t>& vs) const
    {
        history::VolumeCapture c;
        std::set<size_t> vertices;
        for (const size_t t : tets_at(vs)) {
            c.tets.push_back(s.T[t]);
            c.tags.push_back(tags_of(s.side[t]));
            vertices.insert(s.T[t].begin(), s.T[t].end());
        }
        c.vertices.assign(vertices.begin(), vertices.end());
        for (const size_t v : c.vertices) c.positions.push_back(s.P[v]);
        c.surface = m_surface->vertex_ids(c.vertices);
        return c;
    }

    /// TetMesh::tracked_faces_around.
    std::vector<MFace> tracked_around(const std::vector<size_t>& vs, const bool all) const
    {
        std::set<MFace> out;
        for (const size_t t : tets_at(vs)) {
            for (const MFace& f : faces_of(s.T[t])) {
                int among = 0;
                for (const size_t v : f) among += std::count(vs.begin(), vs.end(), v) > 0;
                if (all ? among < 3 : among == 0) continue;
                if (tracked.count(f)) out.insert(f);
            }
        }
        return {out.begin(), out.end()};
    }

    /// A tracked face's corners, its normal out of its tet of the lower side (the first such).
    MFace oriented(const MFace& f) const
    {
        size_t best = s.T.size();
        for (size_t t = 0; t < s.T.size(); ++t) {
            if (!has(s.T[t], f[0]) || !has(s.T[t], f[1]) || !has(s.T[t], f[2])) continue;
            if (best == s.T.size() || s.side[t] < s.side[best]) best = t;
        }
        size_t d = 0;
        for (const size_t v : s.T[best]) {
            if (v != f[0] && v != f[1] && v != f[2]) d = v;
        }
        MFace g = f;
        if (volume::orient(s.P[g[0]], s.P[g[1]], s.P[g[2]], s.P[d]) > 0) std::swap(g[1], g[2]);
        return g;
    }

    /// Lk(i) and Lk(j) meet in Lk(ij), the boundary closed off by a vertex at infinity.
    bool link_condition(const size_t i, const size_t j) const
    {
        constexpr size_t inf = size_t(-1);
        std::map<MFace, int> count;
        for (const MTet& t : s.T) {
            for (const MFace& f : faces_of(t)) ++count[f];
        }
        using Simplex = std::vector<size_t>;
        const auto close = [](std::vector<size_t> c, std::set<Simplex>& out) {
            std::sort(c.begin(), c.end());
            const size_t n = c.size();
            for (size_t mask = 1; mask < (size_t(1) << n); ++mask) {
                Simplex x;
                for (size_t k = 0; k < n; ++k) {
                    if (mask & (size_t(1) << k)) x.push_back(c[k]);
                }
                out.insert(x);
            }
        };
        const auto link = [&](const std::vector<size_t>& of) {
            std::set<Simplex> out;
            const auto holds = [&](const auto& cell) {
                return std::all_of(of.begin(), of.end(), [&](size_t v) {
                    return std::find(cell.begin(), cell.end(), v) != cell.end();
                });
            };
            const auto rest = [&](const auto& cell) {
                std::vector<size_t> r;
                for (const size_t v : cell) {
                    if (std::find(of.begin(), of.end(), v) == of.end()) r.push_back(v);
                }
                return r;
            };
            for (const MTet& t : s.T) {
                if (holds(t)) close(rest(t), out);
            }
            for (const auto& [f, n] : count) {
                if (n != 1 || !holds(f)) continue;
                std::vector<size_t> r = rest(f);
                r.push_back(inf);
                close(r, out);
            }
            return out;
        };
        const std::set<Simplex> li = link({i}), lj = link({j}), lij = link({i, j});
        std::set<Simplex> both;
        std::set_intersection(
            li.begin(),
            li.end(),
            lj.begin(),
            lj.end(),
            std::inserter(both, both.end()));
        return both == lij;
    }

    State m_start;
    int m_n = 0;
    double m_z0 = 0;
    Eigen::Vector3d m_box_lo = Eigen::Vector3d::Zero(), m_box_hi = Eigen::Vector3d::Zero();
    std::shared_ptr<history::TrackedSurfaceRecorder> m_surface;
    std::unique_ptr<history::VolumeRecorder> m_volume;
};

/// The tet of `st` holding x, and its side; -1 if none does.
std::pair<int64_t, int64_t> locate(const State& st, const Eigen::Vector3d& x)
{
    volume::Tets T;
    for (const MTet& t : st.T)
        T.push_back({{uint32_t(t[0]), uint32_t(t[1]), uint32_t(t[2]), uint32_t(t[3])}});
    const auto loc = volume::locate_in(T, st.P, x, [](size_t) { return true; });
    if (!loc) return {-1, -1};
    return {int64_t(loc->tet), st.side[loc->tet]};
}

/// What mapping points of a scene's first state through its histories found.
struct Checked
{
    size_t points = 0, moved = 0, region_kept = 0, det_positive = 0, located = 0;
    double round_trip = 0;
};

/// Maps points in the first state's tets at the vertices `near` (at random coordinates, away
/// from their faces) to the last state and back, each with its tet's region -- back, with any,
/// if asked -- and three more at a step from each, for the Jacobian.
Checked check_map(
    const Scene& sc,
    const VolumeMap& map,
    const std::vector<size_t>& near,
    const size_t n,
    const bool back_in_any_region = false,
    const State* last = nullptr)
{
    const State& first = sc.first();
    std::vector<size_t> tets;
    for (size_t t = 0; t < first.T.size(); ++t) {
        if (std::any_of(near.begin(), near.end(), [&](size_t v) { return has(first.T[t], v); })) {
            tets.push_back(t);
        }
    }
    REQUIRE(!tets.empty());
    std::mt19937 rng(5);
    const double h = 1e-6;
    std::vector<VolumePoint> pts;
    std::vector<int64_t> sides;
    for (size_t i = 0; i < n; ++i) {
        const size_t t = tets[rng() % tets.size()];
        Eigen::Vector4d b;
        for (int k = 0; k < 4; ++k) b[k] = 1 + double(rng() % 1000) / 100;
        b /= b.sum();
        Eigen::Vector3d x = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) x += b[k] * first.P[first.T[t][k]];
        const auto r = map.region_of(sc.tags_of(first.side[t]));
        REQUIRE(r);
        pts.push_back({x, *r});
        for (int k = 0; k < 3; ++k) pts.push_back({x + h * Eigen::Vector3d::Unit(k), *r});
        sides.push_back(first.side[t]);
    }
    const auto there = map.to_output(pts);
    std::vector<VolumePoint> returning = there;
    if (back_in_any_region) {
        for (auto& p : returning) p.region = VolumePoint::kAnyRegion;
    }
    const auto back = map.to_input(returning);
    Checked c;
    c.points = n;
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d y = there[4 * i].position;
        Eigen::Matrix3d J;
        for (int k = 0; k < 3; ++k) J.col(k) = (there[4 * i + k + 1].position - y) / h;
        c.det_positive += J.determinant() > 0;
        c.moved += y != pts[4 * i].position;
        const auto [t, side] = locate(last ? *last : sc.s, y);
        c.located += t >= 0;
        c.region_kept += sc.one_region || side == sides[i];
        c.round_trip = std::max(c.round_trip, (back[4 * i].position - pts[4 * i].position).norm());
    }
    return c;
}

/// The kinds of every pair of every operation of the map.
std::multiset<std::string> kinds(const VolumeMap& map)
{
    std::multiset<std::string> out;
    for (size_t k = 0; k < map.history().edits.size(); ++k) {
        for (const std::string& s : map.op_kinds(k)) out.insert(s);
    }
    return out;
}

} // namespace

namespace {

/// Random operations on the scene, as an application would try them: collapses of an edge of a
/// random tet (moving the sheet's boundary too, if `sheet_boundary`), moves of a vertex by up to
/// 1/8, 4-4 swaps of an edge of a tracked face. What the scene refuses is skipped. Returns how
/// many of each were done.
std::array<size_t, 3>
random_run(Scene& sc, const uint32_t seed, const int attempts, const bool sheet_boundary = false)
{
    std::mt19937 rng(seed);
    std::array<size_t, 3> done = {{0, 0, 0}};
    for (int attempt = 0; attempt < attempts; ++attempt) {
        const MTet t = sc.s.T[rng() % sc.s.T.size()];
        const int a = int(rng() % 4), b = (a + 1 + int(rng() % 3)) % 4;
        const int what = int(rng() % 3);
        if (what == 0) {
            done[0] += sc.collapse(t[a], t[b], sheet_boundary);
        } else if (what == 1) {
            Eigen::Vector3d x = sc.s.P[t[a]];
            for (int k = 0; k < 3; ++k) x[k] += double(int(rng() % 9) - 4) / 32;
            done[1] += sc.move(t[a], x);
        } else if (!sc.tracked.empty()) {
            auto it = sc.tracked.begin();
            std::advance(it, rng() % sc.tracked.size());
            const MFace f = *it;
            const size_t ea = f[rng() % 3], eb = f[rng() % 3];
            if (ea == eb) continue;
            std::set<size_t> ring;
            for (const MTet& u : sc.s.T) {
                if (!has(u, ea) || !has(u, eb)) continue;
                for (const size_t v : u) {
                    if (v != ea && v != eb) ring.insert(v);
                }
            }
            const std::vector<size_t> r(ring.begin(), ring.end());
            bool ok = false;
            for (size_t x = 0; x < r.size() && !ok; ++x) {
                for (size_t y = x + 1; y < r.size() && !ok; ++y)
                    ok = sc.swap_44(ea, eb, r[x], r[y]);
            }
            done[2] += ok;
        }
    }
    return done;
}

/// Every point of the first state's tets lands in the last state's, in its region, with det J
/// > 0, and comes back.
void check_all_points(const Scene& sc, const VolumeMap& map, const size_t n)
{
    std::vector<size_t> all(sc.first().P.size());
    for (size_t v = 0; v < all.size(); ++v) all[v] = v;
    const Checked c = check_map(sc, map, all, n);
    CHECK(c.located == n);
    CHECK(c.region_kept == n);
    CHECK(c.det_positive == n);
    CHECK(c.moved > 0);
    CHECK(c.round_trip < 1e-9);
}

} // namespace

TEST_CASE("volume-ops", "[volume_map]")
{
    // Runs of random operations on a grid of 4^3 cubes split by the plane z = 2, the tracked
    // surface between two regions, each run chosen for the kinds of local map it needs
    // (VolumeMap::op_kinds), all of whose operations map and keep the surface: direct collapses
    // ("small", a side alone, "single", both ends' stars), vertex moves ("move"), 4-4 swaps on
    // the surface ("swap side"), a surface vertex collapsed into one off the surface, mapped the
    // other way round ("interior j->i", "move i"), a side whose tets all hold the survivor
    // ("vanishing side"), and a side with a chord, through the decomposition ("split",
    // "refined-a", "unsplit"), which with one region all round, as in tetwild, takes the whole
    // collapse through it.
    struct Run
    {
        uint32_t seed;
        bool one_region;
        std::vector<std::string> kinds;
    };
    const std::vector<Run> runs = {
        {3, false, {"move", "single", "small", "swap side", "interior j->i", "move i"}},
        {24, false, {"vanishing side"}},
        {105, false, {"split", "refined-a", "unsplit"}},
        {105, true, {"split", "refined-a", "unsplit"}},
    };
    for (const Run& run : runs) {
        const bool one_region = run.one_region;
        CAPTURE(one_region, run.seed);
        Scene sc = Scene::grid(4, 2, run.seed);
        sc.one_region = one_region;
        sc.start();
        const auto done = random_run(sc, run.seed, 3000);
        CHECK(done[0] > 0);
        CHECK(done[1] > 0);
        CHECK(done[2] > 0);
        const VolumeMap map(sc.surface_history(), sc.volume_history());
        map.build_all();
        CHECK(map.stats().failed == 0);
        CHECK(map.stats().surface_not_kept == 0);
        const std::multiset<std::string> got = kinds(map);
        for (const std::string& k : run.kinds) {
            CAPTURE(k);
            CHECK(got.count(k) > 0);
        }
        check_all_points(sc, map, 500);
        CHECK(map.stats().region_changed == 0);
    }
}

TEST_CASE("volume-ops-volume-only", "[volume_map]")
{
    // Surface vertices collapsed far, onto the box where the sheet's boundary is, which no pair
    // keeping the surface maps: each such operation is mapped as a bijection of the volume
    // alone ("volume only"). A point may change region there -- and is in its new region for
    // the operations after -- but does not leave the volume nor lose its orientation, and comes
    // back.
    Scene sc = Scene::grid(4, 2, 1);
    sc.envelope = std::numeric_limits<double>::infinity();
    sc.start();
    random_run(sc, 1, 3000, true);
    const VolumeMap map(sc.surface_history(), sc.volume_history());
    map.build_all();
    CHECK(map.stats().failed == 0);
    CHECK(map.stats().surface_not_kept > 0);
    CHECK(kinds(map).count("volume only") > 0);
    std::vector<size_t> all(sc.first().P.size());
    for (size_t i = 0; i < all.size(); ++i) all[i] = i;
    const Checked c = check_map(sc, map, all, 500);
    CHECK(c.located == c.points);
    CHECK(c.det_positive == c.points);
    CHECK(c.moved > 0);
    CHECK(c.round_trip < 1e-9);
    CHECK(map.stats().region_changed > 0);
}

TEST_CASE("volume-ops-swap-decomposition", "[volume_map]")
{
    // tetwild's 3-2 flip of a sliver on the surface, one region all round: the tracked faces
    // (a,b,c) and (a,b,d) of the sliver (a,b,c,d) become (a,c,d) and (b,c,d), and the two sides
    // of the surface the swap's tets make before are one after, so the swap is mapped as an
    // exact split of (a,b) and a collapse ("swap split", then the collapse's steps).
    State st;
    st.P = {{0, 0, 0}, {1, 1, 0}, {1, 0, -0.1}, {0, 1, -0.1}, {0.5, 0.5, 1}, {0.5, 0.5, -1}};
    enum { a, b, c, d, e, f };
    st.T = {{{a, b, c, d}}, {{a, b, d, e}}, {{a, b, e, c}}, {{a, c, d, f}}, {{b, c, d, f}}};
    st.side = {0, 1, 1, 0, 0};
    for (MTet& t : st.T) {
        if (volume::orient(st.P[t[0]], st.P[t[1]], st.P[t[2]], st.P[t[3]]) < 0) {
            std::swap(t[2], t[3]);
        }
    }
    Scene sc = Scene::of(st, {{{a, b, c}}, {{a, b, d}}}, true);
    sc.start();
    const std::vector<MFace> after = {{{a, c, d}}, {{b, c, d}}};
    REQUIRE(
        sc.swap({0, 1, 2}, {{{c, d, e, a}}, {{c, d, e, b}}}, history::VolumeOp::swap_32, &after));
    const auto h = sc.surface_history();
    REQUIRE(h.edits.size() == 1);
    CHECK(h.edits[0].kind == history::EditKind::swap_edge);
    const VolumeMap map(h, sc.volume_history());
    map.build_all();
    CHECK(map.stats().failed == 0);
    CHECK(map.stats().decompositions == 1);
    const auto got = kinds(map);
    CHECK(got.count("swap split") == 1);
    const std::vector<size_t> near = {a, b, c, d};
    const Checked ck = check_map(sc, map, near, 500);
    CHECK(ck.located == ck.points);
    CHECK(ck.det_positive == ck.points);
    CHECK(ck.moved > 0);
    CHECK(ck.round_trip < 1e-9);
}
