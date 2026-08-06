#include "SimplifySegments.hpp"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <queue>
#include <set>
#include <utility>
#include <vector>

namespace wmtk::utils {

namespace {

using Key = std::pair<int, int>;

Key key_of(int a, int b)
{
    return a < b ? Key(a, b) : Key(b, a);
}

} // namespace

size_t
simplify_segments(MatrixXd& V, MatrixXi& E, const SampleEnvelope& envelope, bool use_link_condition)
{
    const int nv = static_cast<int>(V.rows());
    const int ne = static_cast<int>(E.rows());
    if (ne == 0 || nv == 0) {
        return 0;
    }

    std::vector<Vector2d> pos(nv);
    for (int i = 0; i < nv; ++i) {
        pos[i] = Vector2d(V(i, 0), V(i, 1));
    }

    std::vector<std::array<int, 2>> seg(ne);
    std::vector<std::vector<int>> v2e(nv);
    std::set<Key> present;
    for (int e = 0; e < ne; ++e) {
        seg[e] = {E(e, 0), E(e, 1)};
        v2e[seg[e][0]].push_back(e);
        v2e[seg[e][1]].push_back(e);
        present.insert(key_of(seg[e][0], seg[e][1]));
    }

    // Open endpoints (valence < 2) are frozen either way: the envelope is one-sided, so
    // nothing would notice a curve eroding inwards from its own tip. Junctions -- and,
    // because they end up with valence >= 4, any vertex shared between two input curves --
    // are frozen only under the link condition. Matches 3D, where freeze_boundary() runs
    // unconditionally and the link condition is the separate, optional guard.
    const auto is_frozen = [&](int v) {
        return use_link_condition ? v2e[v].size() != 2 : v2e[v].size() < 2;
    };
    std::vector<char> frozen(nv, 0);
    for (int v = 0; v < nv; ++v) {
        frozen[v] = is_frozen(v);
    }

    std::vector<char> edge_alive(ne, 1);
    std::vector<char> vert_alive(nv, 1);

    const auto len2 = [&](int e) { return (pos[seg[e][0]] - pos[seg[e][1]]).squaredNorm(); };

    // Shortest first; the segment id breaks ties so equal lengths do not leave the order to
    // chance.
    using Item = std::pair<double, int>;
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    for (int e = 0; e < ne; ++e) {
        queue.emplace(len2(e), e);
    }

    size_t n_removed = 0;
    std::vector<int> touched; // alive segments whose geometry changes, minus the collapsed one
    std::vector<int> doomed; // subset of `touched` that the collapse removes outright

    while (!queue.empty()) {
        const auto [l2, e] = queue.top();
        queue.pop();
        if (!edge_alive[e]) {
            continue;
        }
        if (l2 != len2(e)) {
            continue; // stale entry: this segment moved since it was pushed
        }

        const int a = seg[e][0];
        const int b = seg[e][1];
        if (frozen[a] && frozen[b]) {
            continue;
        }

        // Collapse onto the frozen endpoint when there is one, so junctions and open ends
        // keep their exact position; onto the midpoint otherwise. Same rule as the 3D
        // ShortestEdgeCollapse.
        int keep, drop;
        Vector2d m;
        if (frozen[a]) {
            keep = a;
            drop = b;
            m = pos[a];
        } else if (frozen[b]) {
            keep = b;
            drop = a;
            m = pos[b];
        } else {
            keep = std::min(a, b);
            drop = std::max(a, b);
            m = 0.5 * (pos[a] + pos[b]);
        }
        // Under the link condition `drop` is never frozen, so it has valence exactly 2: it
        // loses e and hands its other segment to `keep`, which is why the collapse leaves
        // `keep`'s valence -- and hence every frozen flag -- unchanged, and why they never
        // need recomputing. Without it `drop` may be a junction of any valence, and all of
        // its segments move to `keep`.
        if (use_link_condition && v2e[drop].size() != 2) {
            continue; // defensive: the invariant above should make this unreachable
        }

        // Every segment incident to either endpoint changes geometry: those on `drop` are
        // relabelled onto `keep`, and those on `keep` move because `keep` moves to m.
        touched.clear();
        for (const int f : v2e[drop]) {
            if (f != e && edge_alive[f]) {
                touched.push_back(f);
            }
        }
        for (const int f : v2e[keep]) {
            if (f != e && edge_alive[f]) {
                touched.push_back(f);
            }
        }

        const auto position_after = [&](int v) { return (v == drop || v == keep) ? m : pos[v]; };

        // Guards, all before any mutation.
        bool ok = true;
        std::set<Key> new_keys; // keys of the relabelled segments, to catch collisions
        doomed.clear(); // relabelled segments that come out degenerate or duplicated
        for (const int f : touched) {
            const int u = seg[f][0] == drop ? keep : seg[f][0];
            const int w = seg[f][1] == drop ? keep : seg[f][1];
            const bool relabelled = (seg[f][0] == drop || seg[f][1] == drop);
            bool dies = (u == w); // collapsed to a point (a parallel segment)
            if (!dies && relabelled) {
                const Key k = key_of(u, w);
                // The relabelled segment must not land on top of one that already exists,
                // nor on another relabelled one. `present` still holds f's old key, which
                // cannot equal k because k contains `keep` and f did not.
                dies = (present.count(k) != 0 || !new_keys.insert(k).second);
            }
            if (dies) {
                // Both cases only arise for a segment incident to `drop`. Under the link
                // condition they abort the collapse, which is what keeps the network's
                // topology intact. Without it the segment is dropped instead: its geometry
                // is either a point or a copy of a segment that survives, so nothing leaves
                // the envelope either way.
                if (use_link_condition) {
                    ok = false;
                    break;
                }
                doomed.push_back(f);
                continue;
            }
            const std::array<Vector2d, 2> s = {
                {position_after(seg[f][0]), position_after(seg[f][1])}};
            if (envelope.is_outside(s)) {
                ok = false;
                break;
            }
        }
        if (!ok) {
            continue;
        }

        // Apply.
        present.erase(key_of(a, b));
        for (const int f : v2e[drop]) {
            if (f == e || !edge_alive[f]) {
                continue;
            }
            present.erase(key_of(seg[f][0], seg[f][1]));
            // `doomed` is empty on all but a handful of collapses, and never reached at all
            // under the link condition, so the linear scan is cheaper than a set.
            if (std::find(doomed.begin(), doomed.end(), f) != doomed.end()) {
                edge_alive[f] = 0;
                continue; // seg[f] keeps its old endpoints; only `f != e` reads them below
            }
            if (seg[f][0] == drop) {
                seg[f][0] = keep;
            }
            if (seg[f][1] == drop) {
                seg[f][1] = keep;
            }
            present.insert(key_of(seg[f][0], seg[f][1]));
            v2e[keep].push_back(f);
        }
        pos[keep] = m;
        edge_alive[e] = 0;
        vert_alive[drop] = 0;
        v2e[drop].clear();
        const auto purge = [&](int v) {
            v2e[v].erase(
                std::remove_if(v2e[v].begin(), v2e[v].end(), [&](int f) { return !edge_alive[f]; }),
                v2e[v].end());
        };
        purge(keep);
        // Dropping a duplicate lowers the valence of its far endpoint, and can lower
        // `keep`'s below 2 (collapsing one side of a triangle leaves `keep` a dangling
        // tip), so the frozen flags have to be refreshed. Unreachable under the link
        // condition, where nothing dies and `keep`'s valence cannot fall.
        for (const int f : doomed) {
            for (const int v : {seg[f][0], seg[f][1]}) {
                if (v != drop && v != keep) {
                    purge(v);
                    frozen[v] = is_frozen(v);
                }
            }
        }
        if (!doomed.empty()) {
            frozen[keep] = is_frozen(keep);
            // A vertex left with no segment at all carries no input geometry -- its position
            // is a moved midpoint, not an input point -- so retire it rather than hand the
            // arrangement a free point that was never there. Input free points have valence
            // 0 from the start and are never an endpoint of a collapse, so they survive.
            if (v2e[keep].empty()) {
                vert_alive[keep] = 0;
                ++n_removed;
            }
        }
        ++n_removed;

        for (const int f : v2e[keep]) {
            queue.emplace(len2(f), f);
        }
    }

    if (n_removed == 0) {
        return 0;
    }

    // Compact. Vertices with no incident segment are kept: in 2D they are free points that
    // the arrangement still triangulates.
    std::vector<int> remap(nv, -1);
    int nv_new = 0;
    for (int v = 0; v < nv; ++v) {
        if (vert_alive[v]) {
            remap[v] = nv_new++;
        }
    }
    MatrixXd V_new(nv_new, 2);
    for (int v = 0; v < nv; ++v) {
        if (vert_alive[v]) {
            V_new.row(remap[v]) = pos[v];
        }
    }
    int ne_new = 0;
    for (int e = 0; e < ne; ++e) {
        ne_new += edge_alive[e] ? 1 : 0;
    }
    MatrixXi E_new(ne_new, 2);
    int idx = 0;
    for (int e = 0; e < ne; ++e) {
        if (!edge_alive[e]) {
            continue;
        }
        assert(vert_alive[seg[e][0]] && vert_alive[seg[e][1]]);
        assert(seg[e][0] != seg[e][1]);
        E_new(idx, 0) = remap[seg[e][0]];
        E_new(idx, 1) = remap[seg[e][1]];
        ++idx;
    }

    V = V_new;
    E = E_new;
    return n_removed;
}

} // namespace wmtk::utils
