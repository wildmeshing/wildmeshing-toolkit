#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/MultiVoronoiEnumeration.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <random>
#include <set>
#include <vector>

using namespace wmtk::components::simwild::multi_voronoi;
using Eigen::Vector2d;

// ---------------------------------------------------------------------------
// The 2D Voronoi fill by enumeration on single triangles, with three point regions, so that
// every medial axis is a straight line (the hackmd's assumptions hold) and the diagram is a
// triple point P with three rays. Each corner is placed in a region's cell, on a ray (two
// labels) for a tie, and the triangle is filled through a triangle-soup mesh adapter.
//
// The triple point is placed at the face's centroid, not at P, so the faces around it are not
// exact. The checks are the ones that hold anyway: every final face shares a label with its
// corners' sets and gets it; no fallback; and every final face with no triple point corner gets
// the region nearest its centroid.
// ---------------------------------------------------------------------------
namespace {

struct Sites
{
    std::array<Vector2d, 3> s;
    Vector2d P;
    std::map<LabelSet, Vector2d> ray; // the Voronoi edge of a pair, from P

    explicit Sites(const std::array<Vector2d, 3>& in)
        : s(in)
    {
        const Vector2d a = s[0], b = s[1], c = s[2];
        const double d =
            2 * (a.x() * (b.y() - c.y()) + b.x() * (c.y() - a.y()) + c.x() * (a.y() - b.y()));
        P = Vector2d(
            (a.squaredNorm() * (b.y() - c.y()) + b.squaredNorm() * (c.y() - a.y()) +
             c.squaredNorm() * (a.y() - b.y())) /
                d,
            (a.squaredNorm() * (c.x() - b.x()) + b.squaredNorm() * (a.x() - c.x()) +
             c.squaredNorm() * (b.x() - a.x())) /
                d);
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                const int k = 3 - i - j;
                const Vector2d e = s[j] - s[i];
                Vector2d u = Vector2d(-e.y(), e.x()).normalized();
                if ((P + u - s[k]).norm() < (P + u - s[i]).norm()) {
                    u = -u;
                }
                ray[label_bit(i) | label_bit(j)] = u;
            }
        }
    }
    double dist(const Vector2d& x, const int i) const { return (x - s[i]).norm(); }
};

/// A triangle soup with the adapter interface of fill_component_2d.
struct Soup
{
    explicit Soup(const Sites& s)
        : sites(s)
    {}

    const Sites& sites;
    std::vector<Vector2d> P;
    std::vector<LabelSet> L0; // the corners' sets
    std::map<size_t, std::array<size_t, 3>> faces;
    std::map<size_t, int> label;
    size_t next_face = 0;

    std::array<size_t, 3> face_vertices(const size_t f) const { return faces.at(f); }
    Vector2d position(const size_t v) const { return P[v]; }
    double distance(const Vector2d& x, const int l) const { return sites.dist(x, l); }
    LabelSet initial_labels(const size_t v) const { return L0.at(v); }
    bool alive(const size_t f) const { return faces.count(f) > 0; }
    size_t add_face(const std::array<size_t, 3>& t)
    {
        faces[next_face] = t;
        return next_face++;
    }
    size_t split_edge(const size_t a, const size_t b, const Vector2d& x)
    {
        const size_t n = P.size();
        P.push_back(x);
        std::vector<size_t> on_edge;
        for (const auto& [f, t] : faces) {
            if (std::count(t.begin(), t.end(), a) && std::count(t.begin(), t.end(), b)) {
                on_edge.push_back(f);
            }
        }
        for (const size_t f : on_edge) {
            const auto t = faces.at(f);
            faces.erase(f);
            label.erase(f);
            std::array<size_t, 3> ta = t, tb = t;
            for (int k = 0; k < 3; ++k) {
                if (t[k] == b) ta[k] = n; // keeps a
                if (t[k] == a) tb[k] = n; // keeps b
            }
            add_face(ta);
            add_face(tb);
        }
        return n;
    }
    size_t split_face(const size_t f, const Vector2d& x)
    {
        const size_t n = P.size();
        P.push_back(x);
        const auto t = faces.at(f);
        faces.erase(f);
        label.erase(f);
        add_face({{t[0], t[1], n}});
        add_face({{t[1], t[2], n}});
        add_face({{t[2], t[0], n}});
        return n;
    }
    std::vector<size_t> faces_around(const size_t v) const
    {
        std::vector<size_t> out;
        for (const auto& [f, t] : faces) {
            if (std::count(t.begin(), t.end(), v)) out.push_back(f);
        }
        return out;
    }
    void set_label(const size_t f, const int l) { label[f] = l; }
};

double area(const Vector2d& a, const Vector2d& b, const Vector2d& c)
{
    const Vector2d u = b - a, v = c - a;
    return 0.5 * (u.x() * v.y() - u.y() * v.x());
}

LabelSet parse(const std::string& s)
{
    LabelSet out = 0;
    for (const char ch : s) out |= label_bit(ch - 'a');
    return out;
}

LabelSet nearest_set(const Sites& st, const Vector2d& x)
{
    return nearest_labels(
        [&](const Vector2d& p, const int i) { return st.dist(p, i); },
        x,
        LabelSet(7),
        1e-10);
}

} // namespace

TEST_CASE("multi_voronoi_2d_cases", "[simwild][tight_seal][multi_voronoi]")
{
    // the corner patterns with no three-label corner (the hackmd's 14 cases)
    const std::vector<std::array<std::string, 3>> patterns = {
        {{"a", "a", "a"}},
        {{"a", "a", "b"}},
        {{"a", "a", "ab"}},
        {{"a", "ab", "b"}},
        {{"a", "ab", "ab"}},
        {{"a", "b", "c"}},
        {{"a", "a", "bc"}},
        {{"a", "ab", "c"}},
        {{"a", "ab", "ac"}},
        {{"a", "ab", "bc"}},
        {{"a", "bc", "bc"}},
        {{"ab", "ab", "ac"}},
        {{"ab", "ac", "bc"}}};
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> U(-1, 1), T(0.08, 1.6);

    std::map<FaceKind, size_t> kinds;
    size_t realised = 0;
    for (const auto& pat : patterns) {
        const std::array<LabelSet, 3> S = {{parse(pat[0]), parse(pat[1]), parse(pat[2])}};
        for (int trial = 0; trial < 1500; ++trial) {
            const Sites st(
                {{Vector2d(U(rng), U(rng)), Vector2d(U(rng), U(rng)), Vector2d(U(rng), U(rng))}});
            if (std::abs(area(st.s[0], st.s[1], st.s[2])) < 0.15) continue;
            std::array<Vector2d, 3> X;
            bool ok = true;
            for (int k = 0; k < 3 && ok; ++k) {
                if (label_count(S[k]) == 2) {
                    X[k] = st.P + T(rng) * st.ray.at(S[k]);
                } else {
                    ok = false;
                    for (int tries = 0; tries < 400; ++tries) {
                        const Vector2d p = st.P + 1.6 * Vector2d(U(rng), U(rng));
                        if (nearest_set(st, p) == S[k]) {
                            X[k] = p;
                            ok = true;
                            break;
                        }
                    }
                }
            }
            if (!ok) continue;
            const double A = area(X[0], X[1], X[2]);
            const double size =
                std::max({(X[1] - X[0]).norm(), (X[2] - X[1]).norm(), (X[0] - X[2]).norm()});
            if (std::abs(A) < 1e-3 * size * size) continue;
            if (A < 0) std::swap(X[1], X[2]);
            std::array<LabelSet, 3> SS = S;
            if (A < 0) std::swap(SS[1], SS[2]);
            bool labels_match = true;
            for (int k = 0; k < 3; ++k) labels_match &= nearest_set(st, X[k]) == SS[k];
            if (!labels_match) continue;
            ++realised;

            Soup soup(st);
            soup.P.assign(X.begin(), X.end());
            soup.L0.assign(SS.begin(), SS.end());
            const size_t f0 = soup.add_face({{0, 1, 2}});
            const auto plan = plan_face_2d(
                [&](const Vector2d& x, const int l) { return st.dist(x, l); },
                LabelSet(7),
                1e-10,
                X,
                SS);
            ++kinds[plan.kind];
            CHECK(plan.kind != FaceKind::Other);

            const FillStats2D stats = fill_component_2d(soup, {f0}, LabelSet(7), 1e-10);
            CHECK(stats.fallback_labels == 0);
            CHECK(stats.refused_splits == 0);
            CHECK(!stats.gave_up);

            double total = 0;
            for (const auto& [f, t] : soup.faces) {
                const double a = area(soup.P[t[0]], soup.P[t[1]], soup.P[t[2]]);
                CHECK(a > -1e-12); // no inverted face
                total += a;
                REQUIRE(soup.label.count(f) == 1);
                const int l = soup.label.at(f);
                // its corners share it (the label is not a fallback)
                bool shared = true;
                bool at_P = false;
                for (const size_t v : t) {
                    // the sets of the vertices made by the fill are not kept by the soup; a
                    // vertex of the original triangle has its set in L0
                    if (v < 3) shared &= has_label(SS[v], l);
                    if (v >= 3 && label_count(nearest_set(st, soup.P[v])) == 1) at_P = true;
                }
                CHECK(shared);
                // away from the (approximate) triple point the fill is exact
                if (!at_P && plan.kind != FaceKind::Triple) {
                    const Vector2d c = (soup.P[t[0]] + soup.P[t[1]] + soup.P[t[2]]) / 3;
                    CHECK(has_label(nearest_set(st, c), l));
                }
            }
            CHECK(std::abs(total - std::abs(A)) < 1e-9 * size * size); // area kept
        }
    }
    CHECK(realised > 5000);
    // every kind of face shows up, the strip with two doubly crossed edges included
    CHECK(kinds[FaceKind::One] > 0);
    CHECK(kinds[FaceKind::Chord] > 0);
    CHECK(kinds[FaceKind::Strip] > 0);
    CHECK(kinds[FaceKind::Triple] > 0);
}

// ---------------------------------------------------------------------------
// 3D: single tets with three point regions, so that every sheet is a plane and the triple line a
// straight line. Corners are placed in a region's cell, on a sheet (two labels) or on the triple
// line (three), for random corner patterns, and the tet is filled through a tet-soup adapter.
// Checked: no fallback; every final tet has positive volume and its label is shared by its
// corners; every final tet that does not touch a face-split vertex (on the triple line, placed
// at the face's centroid) gets the region nearest its centroid; the volume is kept.
// ---------------------------------------------------------------------------
namespace {

using Eigen::Vector3d;

struct Sites3
{
    std::array<Vector3d, 3> s;
    Vector3d O, n;
    std::map<LabelSet, Vector3d> ray; // the sheet of a pair, away from the triple line

    explicit Sites3(const std::array<Vector3d, 3>& in)
        : s(in)
    {
        const Vector3d ab = s[1] - s[0], ac = s[2] - s[0];
        const Vector3d nn = ab.cross(ac);
        n = nn.normalized();
        O = s[0] + (nn.cross(ab) * ac.squaredNorm() + ac.cross(nn) * ab.squaredNorm()) /
                       (2 * nn.squaredNorm());
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                const int k = 3 - i - j;
                Vector3d u = n.cross(s[j] - s[i]).normalized();
                if ((O + u - s[k]).norm() < (O + u - s[i]).norm()) u = -u;
                ray[label_bit(i) | label_bit(j)] = u;
            }
        }
    }
    double dist(const Vector3d& x, const int i) const { return (x - s[i]).norm(); }
    LabelSet nearest(const Vector3d& x, const double eps = 1e-10) const
    {
        return nearest_labels(
            [&](const Vector3d& p, int i) { return dist(p, i); },
            x,
            LabelSet(7),
            eps);
    }
};

struct TetSoup
{
    explicit TetSoup(const Sites3& s)
        : sites(s)
    {}

    const Sites3& sites;
    std::vector<Vector3d> P;
    std::vector<LabelSet> L0;
    std::map<size_t, std::array<size_t, 4>> tets;
    std::map<size_t, int> label;
    std::set<size_t> on_triple_line; // vertices made by face splits
    size_t next = 0;

    std::array<size_t, 4> tet_vertices(const size_t t) const { return tets.at(t); }
    Vector3d position(const size_t v) const { return P[v]; }
    double distance(const Vector3d& x, const int l) const { return sites.dist(x, l); }
    LabelSet initial_labels(const size_t v) const { return L0.at(v); }
    bool alive(const size_t t) const { return tets.count(t) > 0; }
    void add(const std::array<size_t, 4>& t) { tets[next++] = t; }
    bool has(const std::array<size_t, 4>& t, const size_t v) const
    {
        return std::find(t.begin(), t.end(), v) != t.end();
    }
    size_t split_edge(const size_t a, const size_t b, const Vector3d& x)
    {
        const size_t n = P.size();
        P.push_back(x);
        std::vector<size_t> on;
        for (const auto& [t, vs] : tets) {
            if (has(vs, a) && has(vs, b)) on.push_back(t);
        }
        for (const size_t t : on) {
            const auto vs = tets.at(t);
            tets.erase(t);
            label.erase(t);
            std::array<size_t, 4> ta = vs, tb = vs;
            for (int k = 0; k < 4; ++k) {
                if (vs[k] == b) ta[k] = n;
                if (vs[k] == a) tb[k] = n;
            }
            add(ta);
            add(tb);
        }
        return n;
    }
    size_t split_face(const size_t a, const size_t b, const size_t c, const Vector3d& x)
    {
        const size_t n = P.size();
        P.push_back(x);
        on_triple_line.insert(n);
        std::vector<size_t> on;
        for (const auto& [t, vs] : tets) {
            if (has(vs, a) && has(vs, b) && has(vs, c)) on.push_back(t);
        }
        for (const size_t t : on) {
            const auto vs = tets.at(t);
            tets.erase(t);
            label.erase(t);
            for (const size_t y : {a, b, c}) {
                std::array<size_t, 4> ch = vs;
                for (int k = 0; k < 4; ++k) {
                    if (vs[k] == y) ch[k] = n;
                }
                add(ch);
            }
        }
        return n;
    }
    std::vector<size_t> tets_around(const size_t v) const
    {
        std::vector<size_t> out;
        for (const auto& [t, vs] : tets) {
            if (has(vs, v)) out.push_back(t);
        }
        return out;
    }
    void set_label(const size_t t, const int l) { label[t] = l; }
};

double volume(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d)
{
    return (b - a).cross(c - a).dot(d - a) / 6;
}

} // namespace

TEST_CASE("multi_voronoi_3d_cases", "[simwild][tight_seal][multi_voronoi]")
{
    std::mt19937 rng(11);
    std::uniform_real_distribution<double> U(-1, 1), R(0.08, 1.6), H(-1.6, 1.6);
    std::uniform_int_distribution<int> lab(0, 2), kind(0, 9);

    std::map<TetKind, size_t> kinds;
    size_t realised = 0;
    size_t revisited = 0;
    for (int trial = 0; trial < 30000 && realised < 6000; ++trial) {
        const Sites3 st(
            {{Vector3d(U(rng), U(rng), U(rng)),
              Vector3d(U(rng), U(rng), U(rng)),
              Vector3d(U(rng), U(rng), U(rng))}});
        if ((st.s[1] - st.s[0]).cross(st.s[2] - st.s[0]).norm() < 0.3) continue;
        std::array<LabelSet, 4> S;
        std::array<Vector3d, 4> X;
        bool ok = true;
        int triples = 0;
        for (int k = 0; k < 4 && ok; ++k) {
            const int r = kind(rng);
            if (r < 6) {
                S[k] = label_bit(lab(rng));
                ok = false;
                for (int tries = 0; tries < 400; ++tries) {
                    const Vector3d p = st.O + 1.6 * Vector3d(U(rng), U(rng), U(rng));
                    if (st.nearest(p, 0.02) == S[k] && st.nearest(p) == S[k]) {
                        X[k] = p;
                        ok = true;
                        break;
                    }
                }
            } else if (r < 9) {
                const int i = lab(rng);
                S[k] = label_bit(i) | label_bit((i + 1) % 3);
                X[k] = st.O + H(rng) * st.n + R(rng) * st.ray.at(S[k]);
            } else {
                S[k] = 7;
                X[k] = st.O + H(rng) * st.n;
                ++triples;
            }
        }
        if (!ok || triples > 2) continue;
        double V0 = volume(X[0], X[1], X[2], X[3]);
        double size = 0;
        for (int k = 0; k < 4; ++k) {
            for (int l = k + 1; l < 4; ++l) size = std::max(size, (X[k] - X[l]).norm());
        }
        if (std::abs(V0) < 1e-3 * size * size * size) continue;
        if (V0 < 0) {
            std::swap(X[1], X[2]);
            std::swap(S[1], S[2]);
            V0 = -V0;
        }
        bool match = true;
        for (int k = 0; k < 4; ++k) match &= st.nearest(X[k]) == S[k];
        if (!match) continue;
        ++realised;

        std::array<std::vector<Crossing>, 6> crossings;
        for (int k = 0; k < 6; ++k) {
            const int a = tet_edges[k][0], b = tet_edges[k][1];
            edge_crossings(
                [&](const Vector3d& p, int i) { return st.dist(p, i); },
                LabelSet(7),
                1e-10,
                X[a],
                X[b],
                0.,
                S[a],
                1.,
                S[b],
                crossings[k]);
        }
        const TetPlan3D plan =
            classify_tet([&](const Vector3d& p, int i) { return st.dist(p, i); }, X, S, crossings);
        ++kinds[plan.kind];
        CHECK(plan.kind != TetKind::Other);

        TetSoup soup(st);
        soup.P.assign(X.begin(), X.end());
        soup.L0.assign(S.begin(), S.end());
        soup.add({{0, 1, 2, 3}});
        const FillStats3D stats = fill_component_3d(soup, {0}, LabelSet(7), 1e-10);
        CHECK(stats.fallback_labels == 0);
        CHECK(stats.refused_splits == 0);
        CHECK(!stats.gave_up);
        revisited += stats.visits > 0;

        double total = 0;
        for (const auto& [t, vs] : soup.tets) {
            const double v = volume(soup.P[vs[0]], soup.P[vs[1]], soup.P[vs[2]], soup.P[vs[3]]);
            CHECK(v > 1e-15 * size * size * size); // no flat or inverted tet
            total += v;
            REQUIRE(soup.label.count(t) == 1);
            const int l = soup.label.at(t);
            bool touches_line = false;
            for (const size_t x : vs) {
                if (x < 4) CHECK(has_label(S[x], l));
                touches_line = touches_line || soup.on_triple_line.count(x) > 0;
            }
            if (!touches_line) {
                const Vector3d c =
                    (soup.P[vs[0]] + soup.P[vs[1]] + soup.P[vs[2]] + soup.P[vs[3]]) / 4;
                CHECK(has_label(st.nearest(c, 1e-7), l));
            }
        }
        CHECK(std::abs(total - V0) < 1e-9 * size * size * size); // volume kept
    }
    CHECK(realised > 3000);
    CHECK(kinds[TetKind::Sheet] > 0);
    CHECK(kinds[TetKind::Slab] > 0);
    CHECK(kinds[TetKind::Y] > 0);
    CHECK(revisited > 0); // some tets need a repair visit
}
