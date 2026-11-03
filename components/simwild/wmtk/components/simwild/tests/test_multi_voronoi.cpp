#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/MultiVoronoiEnumeration.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <random>
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
