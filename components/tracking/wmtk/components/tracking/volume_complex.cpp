#include "volume_complex.hpp"

#include <algorithm>
#include <map>
#include <numeric>

namespace wmtk::components::tracking::volume {

Tri sorted(Tri t)
{
    std::sort(t.begin(), t.end());
    return t;
}

Tet sorted(Tet t)
{
    std::sort(t.begin(), t.end());
    return t;
}

bool contains(const Tet& t, const Vid v)
{
    return std::find(t.begin(), t.end(), v) != t.end();
}

bool contains(const Tri& t, const Vid v)
{
    return std::find(t.begin(), t.end(), v) != t.end();
}

std::array<Tri, 4> faces_of(const Tet& t)
{
    return {
        {sorted(Tri{{t[1], t[2], t[3]}}),
         sorted(Tri{{t[0], t[2], t[3]}}),
         sorted(Tri{{t[0], t[1], t[3]}}),
         sorted(Tri{{t[0], t[1], t[2]}})}};
}

std::set<Tet> keys(const Tets& T)
{
    std::set<Tet> k;
    for (const Tet& t : T) k.insert(sorted(t));
    return k;
}

std::optional<Faces> boundary(const Tets& T)
{
    std::map<Tri, int> count;
    for (const Tet& t : T) {
        for (const Tri& f : faces_of(t)) ++count[f];
    }
    Faces B;
    for (const auto& [f, n] : count) {
        if (n > 2) return std::nullopt;
        if (n == 1) B.insert(f);
    }
    return B;
}

namespace {

/// A union-find over 0..n-1.
struct Roots
{
    std::vector<size_t> r;
    explicit Roots(size_t n)
        : r(n)
    {
        std::iota(r.begin(), r.end(), 0);
    }
    size_t find(size_t x)
    {
        while (r[x] != x) x = r[x] = r[r[x]];
        return x;
    }
    void join(size_t a, size_t b) { r[find(a)] = find(b); }
};

using Edge = std::array<Vid, 2>;

Edge edge(Vid a, Vid b)
{
    return a < b ? Edge{{a, b}} : Edge{{b, a}};
}

} // namespace

bool is_ball(const Tets& T)
{
    const auto B = boundary(T);
    if (!B || B->empty()) return false;
    std::map<Edge, int> ec;
    std::map<Vid, std::vector<std::pair<Vid, Vid>>> link;
    for (const Tri& f : *B) {
        for (int k = 0; k < 3; ++k) {
            const Vid a = f[k], b = f[(k + 1) % 3], c = f[(k + 2) % 3];
            ++ec[edge(a, b)];
            link[a].emplace_back(b, c);
        }
    }
    for (const auto& [e, n] : ec) {
        if (n != 2) return false;
    }
    // every vertex's link in the surface is one cycle
    for (const auto& [v, es] : link) {
        std::map<Vid, std::vector<Vid>> adj;
        for (const auto& [x, y] : es) {
            adj[x].push_back(y);
            adj[y].push_back(x);
        }
        for (const auto& [x, n] : adj) {
            if (n.size() != 2) return false;
        }
        const Vid start = adj.begin()->first;
        Vid prev = start, cur = start;
        size_t steps = 0;
        bool first = true;
        while (true) {
            const auto& n = adj[cur];
            const Vid nxt = (first || n[0] != prev) ? n[0] : n[1];
            first = false;
            prev = cur;
            cur = nxt;
            ++steps;
            if (cur == start || steps > adj.size()) break;
        }
        if (steps != adj.size()) return false;
    }
    std::vector<Vid> vs;
    for (const auto& [v, _] : link) vs.push_back(v);
    std::map<Vid, size_t> ix;
    for (size_t k = 0; k < vs.size(); ++k) ix[vs[k]] = k;
    Roots roots(vs.size());
    for (const auto& [e, n] : ec) roots.join(ix[e[0]], ix[e[1]]);
    for (size_t k = 0; k < vs.size(); ++k) {
        if (roots.find(k) != roots.find(0)) return false;
    }
    return int64_t(vs.size()) - int64_t(ec.size()) + int64_t(B->size()) == 2;
}

std::optional<std::vector<Vid>> disk_cycle(const Faces& F)
{
    if (F.empty()) return std::vector<Vid>{};
    std::map<Edge, int> ec;
    std::set<Vid> verts;
    for (const Tri& f : F) {
        for (int k = 0; k < 3; ++k) {
            ++ec[edge(f[k], f[(k + 1) % 3])];
            verts.insert(f[k]);
        }
    }
    std::map<Vid, std::vector<Vid>> adj;
    for (const auto& [e, n] : ec) {
        if (n > 2) return std::nullopt;
        if (n == 1) {
            adj[e[0]].push_back(e[1]);
            adj[e[1]].push_back(e[0]);
        }
    }
    if (adj.empty()) return std::nullopt;
    for (const auto& [v, n] : adj) {
        if (n.size() != 2) return std::nullopt;
    }
    const Vid start = adj.begin()->first;
    std::vector<Vid> cyc = {start};
    Vid prev = start, cur = start;
    bool first = true;
    while (true) {
        const auto& n = adj[cur];
        const Vid nxt = (first || n[0] != prev) ? n[0] : n[1];
        first = false;
        prev = cur;
        cur = nxt;
        if (cur == start) break;
        cyc.push_back(cur);
        if (cyc.size() > adj.size()) return std::nullopt;
    }
    if (cyc.size() != adj.size()) return std::nullopt;
    if (int64_t(verts.size()) - int64_t(ec.size()) + int64_t(F.size()) != 1) return std::nullopt;
    return cyc;
}

std::optional<Tets> collapse(const Tets& T, const Vid a, const Vid b)
{
    Tets out;
    std::set<Tet> seen;
    for (Tet t : T) {
        if (contains(t, a)) {
            if (contains(t, b)) continue;
            std::replace(t.begin(), t.end(), a, b);
        }
        if (!seen.insert(sorted(t)).second) return std::nullopt;
        out.push_back(t);
    }
    return out;
}

std::vector<Tets> sides(const Tets& T_all, const Vid c, const Faces& surf)
{
    Tets T;
    for (const Tet& t : T_all) {
        if (contains(t, c)) T.push_back(t);
    }
    std::map<Tri, std::vector<size_t>> ft;
    for (size_t k = 0; k < T.size(); ++k) {
        for (const Tri& f : faces_of(T[k])) {
            if (contains(f, c)) ft[f].push_back(k);
        }
    }
    Roots roots(T.size());
    for (const auto& [f, ks] : ft) {
        if (ks.size() == 2 && !surf.count(f)) roots.join(ks[0], ks[1]);
    }
    std::map<size_t, Tets> groups;
    for (size_t k = 0; k < T.size(); ++k) groups[roots.find(k)].push_back(T[k]);
    std::vector<Tets> out;
    for (auto& [g, ts] : groups) out.push_back(std::move(ts));
    return out;
}

Faces surf_after(const Faces& surf, const Vid i, const Vid j)
{
    Faces out;
    for (Tri f : surf) {
        if (contains(f, i)) {
            if (contains(f, j)) continue;
            std::replace(f.begin(), f.end(), i, j);
            f = sorted(f);
        }
        out.insert(f);
    }
    return out;
}

std::optional<PairSpec> make_pair(
    const std::string& kind,
    const Tets& Tb,
    const Tets& Ta,
    const Faces& sb,
    const Faces& sa,
    std::string* why)
{
    const auto fail = [&](const char* w) -> std::optional<PairSpec> {
        if (why) *why = kind + ": " + w;
        return std::nullopt;
    };
    if (Ta.empty()) return fail("after patch empty");
    if (!is_ball(Tb)) return fail("before patch not a ball");
    if (!is_ball(Ta)) return fail("after patch not a ball");
    const Faces Bb = *boundary(Tb), Ba = *boundary(Ta);
    PairSpec p{kind, Tb, Ta, {}, {}, {}};
    std::set_intersection(
        Bb.begin(),
        Bb.end(),
        Ba.begin(),
        Ba.end(),
        std::inserter(p.fixed, p.fixed.end()));
    std::set_difference(
        Bb.begin(),
        Bb.end(),
        p.fixed.begin(),
        p.fixed.end(),
        std::inserter(p.cap_before, p.cap_before.end()));
    std::set_difference(
        Ba.begin(),
        Ba.end(),
        p.fixed.begin(),
        p.fixed.end(),
        std::inserter(p.cap_after, p.cap_after.end()));
    for (const Tri& f : p.cap_before) {
        if (!sb.count(f)) return fail("a changed boundary face is not a tracked face");
    }
    for (const Tri& f : p.cap_after) {
        if (!sa.count(f)) return fail("a changed boundary face is not a tracked face");
    }
    if (!p.cap_before.empty() || !p.cap_after.empty()) {
        const auto yb = disk_cycle(p.cap_before), ya = disk_cycle(p.cap_after);
        if (!yb || !ya) return fail("cap not a disk");
        const auto edges_of = [](const std::vector<Vid>& y) {
            std::set<Edge> es;
            for (size_t k = 0; k < y.size(); ++k) es.insert(edge(y[k], y[(k + 1) % y.size()]));
            return es;
        };
        if (edges_of(*yb) != edges_of(*ya)) return fail("caps have different outlines");
    }
    return p;
}

std::vector<Vid> chords(const Tets& M, const Vid i, const Vid j)
{
    std::map<Tri, int> fc;
    for (const Tet& t : M) {
        for (const Tri& f : faces_of(t)) ++fc[f];
    }
    std::set<Vid> ring;
    for (const Tet& t : M) {
        for (const Tri& f : faces_of(t)) {
            if (contains(f, i) && fc[f] == 1) {
                for (const Vid v : f) {
                    if (v != i) ring.insert(v);
                }
            }
        }
    }
    std::vector<Vid> out;
    for (const Vid x : ring) {
        if (x == j) continue;
        const auto it = fc.find(sorted(Tri{{i, j, x}}));
        if (it != fc.end() && it->second == 2) out.push_back(x);
    }
    return out;
}

char side_class(const Tets& M, const Vid i, const Vid j)
{
    if (std::all_of(M.begin(), M.end(), [j](const Tet& t) { return contains(t, j); })) return 'b';
    return chords(M, i, j).empty() ? 'a' : 'c';
}

namespace {

/// The collapse of i into j on K, side by side, every side of class a or b; appends its pairs.
std::optional<Tets> collapse_steps(
    const Tets& K,
    const Vid i,
    const Vid j,
    const Faces& surf,
    const std::string& tag,
    std::vector<PairSpec>& pairs,
    std::string* why)
{
    const Faces sa = surf_after(surf, i, j);
    for (const Tets& M : sides(K, i, surf)) {
        const char c = side_class(M, i, j);
        if (c == 'a') {
            Tets after;
            for (Tet t : M) {
                if (contains(t, j)) continue;
                std::replace(t.begin(), t.end(), i, j);
                after.push_back(t);
            }
            auto p = make_pair(tag + "a", M, after, surf, sa, why);
            if (!p) return std::nullopt;
            pairs.push_back(std::move(*p));
        } else if (c == 'b') {
            const std::set<Tet> km = keys(M);
            const Tets* home = nullptr;
            const auto js = sides(K, j, surf);
            size_t touching = 0;
            for (const Tets& N : js) {
                const std::set<Tet> kn = keys(N);
                bool any = false, all = true;
                for (const Tet& t : km) {
                    if (kn.count(t)) {
                        any = true;
                    } else {
                        all = false;
                    }
                }
                if (any) {
                    ++touching;
                    if (all) home = &N;
                }
            }
            if (touching != 1 || !home) {
                if (why) *why = tag + "b: the vanishing tets are not in one side of j's star";
                return std::nullopt;
            }
            Tets after;
            for (const Tet& t : *home) {
                if (!contains(t, i)) after.push_back(t);
            }
            auto p = make_pair(tag + "b", *home, after, surf, sa, why);
            if (!p) return std::nullopt;
            pairs.push_back(std::move(*p));
        } else {
            if (why) *why = tag + ": a chord left after the splits";
            return std::nullopt;
        }
    }
    auto K2 = collapse(K, i, j);
    if (!K2 && why) *why = tag + ": the collapse makes a tet twice";
    return K2;
}

} // namespace

std::optional<Decomposition> decompose_collapse(
    const Tets& K,
    const Vid i,
    const Vid j,
    const Faces& surf,
    const Vid first_virtual,
    std::string* why)
{
    const auto fail = [&](const char* w) -> std::optional<Decomposition> {
        if (why) *why = w;
        return std::nullopt;
    };
    const auto direct = collapse(K, i, j);
    if (!direct) return fail("the collapse makes a tet twice");
    Decomposition d;
    std::vector<Vid> X;
    for (const Tets& M : sides(K, i, surf)) {
        if (side_class(M, i, j) == 'c') {
            for (const Vid x : chords(M, i, j)) X.push_back(x);
        }
    }
    for (const Vid x : X) {
        for (const Tri& f : surf) {
            if (contains(f, j) && contains(f, x)) return fail("a chord on the tracked surface");
        }
    }
    Tets K1 = K;
    for (size_t n = 0; n < X.size(); ++n) {
        const Vid w = first_virtual + Vid(n), x = X[n];
        Tets Tjx, rest, split;
        for (const Tet& t : K1) {
            (contains(t, j) && contains(t, x) ? Tjx : rest).push_back(t);
        }
        for (Tet t : Tjx) {
            std::replace(t.begin(), t.end(), j, w);
            split.push_back(t);
        }
        for (Tet t : Tjx) {
            std::replace(t.begin(), t.end(), x, w);
            split.push_back(t);
        }
        auto p = make_pair("split", Tjx, split, surf, surf, why);
        if (!p) return std::nullopt;
        d.pairs.push_back(std::move(*p));
        rest.insert(rest.end(), split.begin(), split.end());
        K1 = std::move(rest);
        d.virtuals.emplace_back(w, x);
    }
    d.split = K1;
    auto K2 = collapse_steps(K1, i, j, surf, X.empty() ? "direct-" : "refined-", d.pairs, why);
    if (!K2) return std::nullopt;
    d.collapsed = *K2;
    const Faces sa = surf_after(surf, i, j);
    for (const auto& [w, x] : d.virtuals) {
        for (const Tri& f : sa) {
            if (contains(f, w)) return fail("a virtual vertex on the tracked surface");
        }
        Tets Tw, after;
        for (const Tet& t : *K2) {
            if (contains(t, w)) Tw.push_back(t);
        }
        for (Tet t : Tw) {
            if (contains(t, x)) continue;
            std::replace(t.begin(), t.end(), w, x);
            after.push_back(t);
        }
        auto p = make_pair("unsplit", Tw, after, sa, sa, why);
        if (!p) return std::nullopt;
        d.pairs.push_back(std::move(*p));
        K2 = collapse(*K2, w, x);
        if (!K2) return fail("an unsplit makes a tet twice");
    }
    if (keys(*K2) != keys(*direct)) return fail("the decomposition does not make the collapse");
    return d;
}

} // namespace wmtk::components::tracking::volume
