#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <bitset>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * @file
 * Voronoi fill by enumeration, for tight_seal_geom.
 *
 * A component C of the join region borders two or three regions, its labels (tight_seal_geom
 * leaves components bordering more alone; the face plans below assume at most three regions in a
 * face, and a face that meets more is an "other" case). The fill relabels C's
 * cells with the generalized Voronoi diagram (GVD) of those regions, approximated cell by cell
 * from the label sets of the cells' vertices (the hackmd "geometric tight seal", section
 * "voronoi filling via enumeration"). Two assumptions: an edge whose end sets share a label lies
 * in that label's region (on the medial axis if they share two), and inside a cell each part of
 * the GVD is straight.
 *
 * L(v), the label set of vertex v: the labels of the regions v touches; for a vertex touching
 * none, the labels tied for the nearest region. For a set S, d_S(x) = min over s in S of d_s(x).
 *  - Closed edge, L(u) & L(v) != 0: never split.
 *  - Open edge: split where d_L(u) - d_L(v) vanishes; the new vertex gets the labels tied there,
 *    one from each end. If they share nothing with either end, a third region cuts across the
 *    edge (the edge is "fake"), and it has two crossings, found the same way on each side.
 *  - Boundary word (2D): going around a face through its corners and crossings, each piece of
 *    boundary between two neighbours lies in the region their sets share. Merged, the word says
 *    what the face holds: x, one region; xy, one chord; xyz, a triple point, at which the face is
 *    split (at its centroid for now) before its edges; xyzy, a strip of y, whose doubly crossed
 *    edge is split first, so that the other crossings join its crossings.
 *  - Each new cell gets the label its corners share; one that shares none is visited again
 *    (only a strip whose two edges are both crossed twice leaves one).
 *
 * 3D (tets): the same tests on the tet's edges and faces. A face whose word is xyz, with no
 * corner holding all three labels, is pierced by the triple line (where the three regions meet)
 * and is split at it (at its centroid for now). A tet holds one region, a sheet (two regions), a
 * slab (three regions, two sheets that do not meet) or a Y (three sheets meeting along the
 * triple line, which runs through the tet: two pierced faces, or one and a corner holding all
 * three labels). The pierced faces are split first, then the edges, those crossed twice first;
 * no tet split is needed. See fill_component_3d.
 *
 * The edge crossings, the label sets and the face word do not depend on the dimension; the
 * fills work on any mesh through the adapters documented at fill_component_2d and
 * fill_component_3d.
 *
 * Future option, not implemented: place the triple point exactly. It is put at the face's
 * centroid (2D: the face; 3D: the pierced face), so the boundaries that meet there are bent.
 * Solving d_a = d_b = d_c on the face by Newton (two equations in the face's plane; the gradient
 * of d_i is the unit vector from region i's nearest point), started from the centroid of the
 * face's switch points and falling back to the centroid if it leaves the face, would place it
 * exactly. On control1 frame 0 (3D) the centroid leaves 3.5% of the filled volume with a region
 * other than the nearest, most of it around the triple lines.
 */
namespace wmtk::components::simwild::multi_voronoi {

/// A set of labels 0..63, as bits.
using LabelSet = std::uint64_t;
constexpr int max_labels = 64;
constexpr std::size_t invalid_id = std::size_t(-1);

inline LabelSet label_bit(const int i)
{
    return LabelSet(1) << i;
}
inline bool has_label(const LabelSet s, const int i)
{
    return (s >> i) & 1;
}
inline int label_count(const LabelSet s)
{
    return int(std::bitset<max_labels>(s).count());
}
/// The smallest label of a non-empty set.
inline int lowest_label(const LabelSet s)
{
    for (int i = 0; i < max_labels; ++i) {
        if (has_label(s, i)) {
            return i;
        }
    }
    return -1;
}

// ---------------------------------------------------------------------------------------------
// Any dimension. `dist(x, i)` is the distance from point x to region i.
// ---------------------------------------------------------------------------------------------

/// d_S(x): the distance from x to the nearest region of S (+inf for S empty).
template <typename Vec, typename Dist>
double distance_to_set(const Dist& dist, const Vec& x, const LabelSet S)
{
    double d = std::numeric_limits<double>::infinity();
    for (int i = 0; i < max_labels && (S >> i) != 0; ++i) {
        if (has_label(S, i)) {
            d = std::min(d, dist(x, i));
        }
    }
    return d;
}

/// The labels of `candidates` tied for nearest at x: within eps of the smallest distance.
template <typename Vec, typename Dist>
LabelSet nearest_labels(const Dist& dist, const Vec& x, const LabelSet candidates, const double eps)
{
    std::array<double, max_labels> d;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < max_labels && (candidates >> i) != 0; ++i) {
        if (has_label(candidates, i)) {
            d[i] = dist(x, i);
            best = std::min(best, d[i]);
        }
    }
    LabelSet out = 0;
    for (int i = 0; i < max_labels && (candidates >> i) != 0; ++i) {
        if (has_label(candidates, i) && d[i] <= best + eps) {
            out |= label_bit(i);
        }
    }
    return out;
}

/// One label of S nearest at x, the smallest on a tie (-1 for S empty).
template <typename Vec, typename Dist>
int nearest_label(const Dist& dist, const Vec& x, const LabelSet S)
{
    int best_i = -1;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < max_labels && (S >> i) != 0; ++i) {
        if (has_label(S, i)) {
            const double d = dist(x, i);
            if (d < best) {
                best = d;
                best_i = i;
            }
        }
    }
    return best_i;
}

/// A point of an edge where the nearest region changes: at parameter t along the edge, with the
/// labels tied there.
struct Crossing
{
    double t;
    LabelSet labels;
};

constexpr int bisection_steps = 50;
/// How many regions in a row may cut across one edge.
constexpr int max_fake_depth = 4;

/**
 * @brief The crossings of edge a -> b between parameters t0 and t1, where the label sets are S0
 * and S1, appended to `out` in increasing t. Nothing if S0 and S1 share a label (a closed edge).
 *
 * The crossing is where the regions of S0 stop being nearest, the zero of d_S0 - d_S1 (by
 * bisection). If the labels tied there (`all` are the candidates, eps the tie tolerance) share
 * nothing with S0 or S1, a third region cuts across the edge: the edge is fake, and each side is
 * searched again with that region's labels.
 */
template <typename Vec, typename Dist>
void edge_crossings(
    const Dist& dist,
    const LabelSet all,
    const double eps,
    const Vec& a,
    const Vec& b,
    const double t0,
    const LabelSet S0,
    const double t1,
    const LabelSet S1,
    std::vector<Crossing>& out,
    const int depth = 0)
{
    if (S0 & S1) {
        return;
    }
    const auto at = [&](const double t) -> Vec { return a + t * (b - a); };
    const auto f = [&](const double t) {
        const Vec x = at(t);
        return distance_to_set(dist, x, S0) - distance_to_set(dist, x, S1);
    };
    double lo = t0;
    double hi = t1;
    for (int i = 0; i < bisection_steps; ++i) {
        const double mid = 0.5 * (lo + hi);
        (f(mid) < 0 ? lo : hi) = mid;
    }
    const double tm = 0.5 * (lo + hi);
    const Vec m = at(tm);
    LabelSet L = nearest_labels(dist, m, all, eps);
    const bool meets0 = (L & S0) != 0;
    const bool meets1 = (L & S1) != 0;
    if (!meets0 && !meets1 && depth < max_fake_depth) {
        edge_crossings(dist, all, eps, a, b, t0, S0, tm, L, out, depth + 1);
        edge_crossings(dist, all, eps, a, b, tm, L, t1, S1, out, depth + 1);
        return;
    }
    // A real crossing: one label from each side. The tie can miss a side by rounding (d_S0 and
    // d_S1 agree only to the bisection's precision); then that side's nearest label is added.
    if (!meets0) {
        L |= label_bit(nearest_label(dist, m, S0));
    }
    if (!meets1) {
        L |= label_bit(nearest_label(dist, m, S1));
    }
    out.push_back({tm, L});
}

// ---------------------------------------------------------------------------------------------
// 2D
// ---------------------------------------------------------------------------------------------

/// What a face holds, from its boundary word.
enum class FaceKind {
    One, ///< one region: no split
    Chord, ///< xy: one boundary across the face
    Strip, ///< xyzy: a strip of y, two boundaries
    Triple, ///< xyz: a triple point inside, the face is split at it
    TripleAtCorner, ///< xyz with a corner holding all three: that corner is the triple point
    Other ///< anything else (the assumptions fail): the crossings are split, nothing more
};

struct FacePlan2D
{
    FaceKind kind = FaceKind::Other;
    int label = -1; ///< FaceKind::One: the face's label
    LabelSet regions = 0; ///< the labels of the face's boundary word
    std::array<std::vector<Crossing>, 3> crossings; ///< of edge k, from corner k to corner k + 1
    std::array<int, 3> edge_order = {{0, 1, 2}}; ///< the order to split the edges in
    LabelSet triple = 0; ///< FaceKind::Triple: the labels of the vertex the face is split at
};

/**
 * @brief Tests 1 and 4 for a face with corners X, corner label sets S and the crossings of its
 * edges (edge k from corner k to corner k + 1, parameters from corner k): what the face holds
 * and the order to split its edges in. Vec is the point type: a face of a triangle mesh, or of
 * a tet.
 */
template <typename Dist, typename Vec>
FacePlan2D classify_face(
    const Dist& dist,
    const std::array<Vec, 3>& X,
    const std::array<LabelSet, 3>& S,
    std::array<std::vector<Crossing>, 3> crossings)
{
    FacePlan2D plan;
    const Vec centroid = (X[0] + X[1] + X[2]) / 3;

    // 1. closed: the corners share a label
    const LabelSet common = S[0] & S[1] & S[2];
    if (common) {
        plan.kind = FaceKind::One;
        plan.label =
            label_count(common) == 1 ? lowest_label(common) : nearest_label(dist, centroid, common);
        plan.regions = label_bit(plan.label);
        return plan;
    }
    plan.crossings = std::move(crossings);

    // 4. the boundary word: the region of each piece between neighbouring points, merged
    std::vector<std::pair<Vec, LabelSet>> pts;
    for (int k = 0; k < 3; ++k) {
        pts.emplace_back(X[k], S[k]);
        for (const Crossing& c : plan.crossings[k]) {
            pts.emplace_back(X[k] + c.t * (X[(k + 1) % 3] - X[k]), c.labels);
        }
    }
    std::vector<int> word;
    for (size_t q = 0; q < pts.size(); ++q) {
        const auto& [xa, la] = pts[q];
        const auto& [xb, lb] = pts[(q + 1) % pts.size()];
        const LabelSet R = la & lb;
        int region = -1;
        if (label_count(R) == 1) {
            region = lowest_label(R);
        } else if (R != 0) {
            // the piece runs along a medial axis: the region just inside the face
            const Vec mid = 0.5 * (xa + xb);
            region = nearest_label(dist, Vec(mid + 1e-3 * (centroid - mid)), R);
        }
        if (word.empty() || word.back() != region) {
            word.push_back(region);
        }
    }
    while (word.size() > 1 && word.front() == word.back()) {
        word.pop_back();
    }
    for (const int r : word) {
        if (r >= 0) {
            plan.regions |= label_bit(r);
        }
    }

    const bool broken = std::find(word.begin(), word.end(), -1) != word.end();
    if (!broken && word.size() == 1) {
        plan.kind = FaceKind::One; // every piece in one region, so every corner holds it
        plan.label = word[0];
        for (auto& c : plan.crossings) {
            c.clear();
        }
        return plan;
    }
    if (!broken && word.size() == 2) {
        plan.kind = FaceKind::Chord;
    } else if (!broken && word.size() == 3 && word[0] != word[2]) {
        plan.triple = label_bit(word[0]) | label_bit(word[1]) | label_bit(word[2]);
        plan.kind = FaceKind::Triple;
        for (const LabelSet s : S) {
            if ((s & plan.triple) == plan.triple) {
                plan.kind = FaceKind::TripleAtCorner;
            }
        }
    } else if (
        !broken && word.size() == 4 &&
        ((word[0] == word[2] && word[1] != word[3]) ||
         (word[1] == word[3] && word[0] != word[2]))) {
        plan.kind = FaceKind::Strip;
    }
    if (plan.kind != FaceKind::Triple && plan.kind != FaceKind::TripleAtCorner) {
        // the edge with the most crossings first
        std::stable_sort(plan.edge_order.begin(), plan.edge_order.end(), [&](int i, int j) {
            return plan.crossings[i].size() > plan.crossings[j].size();
        });
    }
    return plan;
}

/// The plan for a face with corners X and corner label sets S, by the four tests.
template <typename Dist>
FacePlan2D plan_face_2d(
    const Dist& dist,
    const LabelSet all,
    const double eps,
    const std::array<Eigen::Vector2d, 3>& X,
    const std::array<LabelSet, 3>& S)
{
    std::array<std::vector<Crossing>, 3> crossings;
    if ((S[0] & S[1] & S[2]) == 0) {
        for (int k = 0; k < 3; ++k) {
            const int l = (k + 1) % 3;
            edge_crossings(dist, all, eps, X[k], X[l], 0., S[k], 1., S[l], crossings[k]);
        }
    }
    return classify_face(dist, X, S, std::move(crossings));
}

struct FillStats2D
{
    std::size_t visits = 0; ///< faces that still needed a split after the planned ones
    std::size_t chord = 0;
    std::size_t strip = 0;
    std::size_t triple = 0;
    std::size_t triple_at_corner = 0;
    std::size_t other = 0;
    std::size_t fake_edges = 0;
    std::size_t face_splits = 0;
    std::size_t edge_splits = 0;
    std::size_t refused_splits = 0;
    std::size_t fallback_labels = 0; ///< faces labelled by their centroid's nearest region
    bool gave_up = false; ///< the visit cap was reached

    FillStats2D& operator+=(const FillStats2D& o)
    {
        visits += o.visits;
        chord += o.chord;
        strip += o.strip;
        triple += o.triple;
        triple_at_corner += o.triple_at_corner;
        other += o.other;
        fake_edges += o.fake_edges;
        face_splits += o.face_splits;
        edge_splits += o.edge_splits;
        refused_splits += o.refused_splits;
        fallback_labels += o.fallback_labels;
        gave_up = gave_up || o.gave_up;
        return *this;
    }
};

/**
 * @brief The Voronoi fill of one component of a triangle mesh.
 *
 *  1. Every face of the component is planned on the mesh as it is (each edge's crossings are
 *     found once, from its own end sets, so the two faces on an edge agree).
 *  2. The splits run in an order that satisfies every face's plan: first the faces with a triple
 *     point inside, at their centroid; then the edges crossed twice; then the other crossings.
 *     A face's own order (face before edges; the doubly crossed edge first) is kept that way,
 *     and no face is cut by its neighbours before its plan is known. (Visiting faces one at a
 *     time instead lets a neighbour's splits cut a face whose edge is crossed twice before its
 *     visit, and the splits can then creep towards that edge without end.)
 *  3. The faces are then visited: a face whose corners share a label gets it; any other (two
 *     edges of one face crossed twice leave one) is planned again and split, and the faces
 *     around its new vertices are visited in turn. A face whose plan splits nothing, and every
 *     face after 20 visits per face of the component, is labelled by its centroid's nearest
 *     region.
 *
 * `mesh` is an adapter with:
 *  - std::array<size_t, 3> face_vertices(size_t f) const
 *  - Eigen::Vector2d position(size_t v) const
 *  - double distance(const Eigen::Vector2d& x, int label) const: to that label's region
 *  - LabelSet initial_labels(size_t v) const: asked for every vertex of `faces` before the first
 *    split
 *  - bool alive(size_t f) const: f is a face of the component
 *  - size_t split_edge(size_t v0, size_t v1, const Eigen::Vector2d& x): the new vertex, or
 *    invalid_id if refused
 *  - size_t split_face(size_t f, const Eigen::Vector2d& x): likewise
 *  - std::vector<size_t> faces_around(size_t v) const: the component's faces at v
 *  - void set_label(size_t f, int label)
 *
 * @param faces the component's faces
 * @param all the labels (the component's regions)
 * @param eps the tie tolerance on distances
 */
template <typename Mesh>
FillStats2D fill_component_2d(
    Mesh& mesh,
    const std::vector<std::size_t>& faces,
    const LabelSet all,
    const double eps)
{
    FillStats2D stats;
    const auto dist = [&](const Eigen::Vector2d& x, const int l) { return mesh.distance(x, l); };

    // The vertices' label sets: those of the component's vertices before any split (labelling
    // faces must not change them), then those the splits give.
    std::unordered_map<std::size_t, LabelSet> L;
    for (const std::size_t f : faces) {
        for (const std::size_t v : mesh.face_vertices(f)) {
            if (L.count(v) == 0) {
                L[v] = mesh.initial_labels(v);
            }
        }
    }
    const auto labels = [&](const std::size_t v) {
        const auto it = L.find(v);
        return it != L.end() ? it->second : (L[v] = mesh.initial_labels(v));
    };
    const auto count_kind = [&](const FacePlan2D& plan) {
        stats.chord += plan.kind == FaceKind::Chord;
        stats.strip += plan.kind == FaceKind::Strip;
        stats.triple += plan.kind == FaceKind::Triple;
        stats.triple_at_corner += plan.kind == FaceKind::TripleAtCorner;
        stats.other += plan.kind == FaceKind::Other;
    };

    // 1. the plans, on the mesh as it is
    struct EdgeSplits
    {
        std::size_t u, w; // the crossings run from u to w
        std::vector<Eigen::Vector2d> points;
        std::vector<LabelSet> labels;
    };
    std::map<std::pair<std::size_t, std::size_t>, EdgeSplits> edges;
    std::vector<std::pair<std::size_t, LabelSet>> triple_faces; // face, its triple point's labels
    for (const std::size_t f : faces) {
        const std::array<std::size_t, 3> vs = mesh.face_vertices(f);
        const std::array<LabelSet, 3> S = {{labels(vs[0]), labels(vs[1]), labels(vs[2])}};
        if (S[0] & S[1] & S[2]) {
            continue;
        }
        const std::array<Eigen::Vector2d, 3> X = {
            {mesh.position(vs[0]), mesh.position(vs[1]), mesh.position(vs[2])}};
        std::array<std::vector<Crossing>, 3> crossings;
        for (int k = 0; k < 3; ++k) {
            const int l = (k + 1) % 3;
            const auto key = std::minmax(vs[k], vs[l]);
            auto it = edges.find(key);
            if (it == edges.end()) {
                // from the lower vertex, so both faces of the edge find the same crossings
                const int a = vs[k] < vs[l] ? k : l;
                const int b = a == k ? l : k;
                std::vector<Crossing> c;
                edge_crossings(dist, all, eps, X[a], X[b], 0., S[a], 1., S[b], c);
                EdgeSplits es{vs[a], vs[b], {}, {}};
                for (const Crossing& cr : c) {
                    es.points.push_back(X[a] + cr.t * (X[b] - X[a]));
                    es.labels.push_back(cr.labels);
                }
                stats.fake_edges += c.size() > 1;
                it = edges.emplace(key, std::move(es)).first;
            }
            // as parameters from corner k
            const EdgeSplits& es = it->second;
            const double len2 = (X[l] - X[k]).squaredNorm();
            for (std::size_t q = 0; q < es.points.size(); ++q) {
                crossings[k].push_back(
                    {(es.points[q] - X[k]).dot(X[l] - X[k]) / len2, es.labels[q]});
            }
            std::sort(
                crossings[k].begin(),
                crossings[k].end(),
                [](const Crossing& p, const Crossing& q) { return p.t < q.t; });
        }
        const FacePlan2D plan = classify_face(dist, X, S, std::move(crossings));
        count_kind(plan);
        if (plan.kind == FaceKind::Triple) {
            triple_faces.emplace_back(f, plan.triple);
        }
    }

    // 2. the splits: triple points, then the edges crossed twice, then the others
    for (const auto& [f, triple] : triple_faces) {
        const std::array<std::size_t, 3> vs = mesh.face_vertices(f);
        const Eigen::Vector2d centroid =
            (mesh.position(vs[0]) + mesh.position(vs[1]) + mesh.position(vs[2])) / 3;
        const std::size_t p = mesh.split_face(f, centroid);
        if (p == invalid_id) {
            ++stats.refused_splits;
            continue;
        }
        L[p] = triple;
        ++stats.face_splits;
    }
    for (const bool doubly : {true, false}) {
        for (const auto& [key, es] : edges) {
            if (es.points.empty() || (es.points.size() > 1) != doubly) {
                continue;
            }
            std::size_t a = es.u; // what is left of the edge is a -> es.w
            for (std::size_t q = 0; q < es.points.size(); ++q) {
                const std::size_t v = mesh.split_edge(a, es.w, es.points[q]);
                if (v == invalid_id) {
                    ++stats.refused_splits;
                    break;
                }
                L[v] = es.labels[q];
                ++stats.edge_splits;
                a = v;
            }
        }
    }

    // 3. the labels, with a visit for each face that still needs a split
    std::vector<std::size_t> stack;
    for (const std::size_t f : faces) {
        if (mesh.alive(f)) {
            stack.push_back(f);
        }
    }
    // every face a split made has a vertex with two labels or more (a crossing or a triple point)
    for (const auto& [v, s] : L) {
        if (label_count(s) > 1) {
            for (const std::size_t g : mesh.faces_around(v)) {
                stack.push_back(g);
            }
        }
    }
    const std::size_t cap = 20 * faces.size() + 100;
    while (!stack.empty()) {
        const std::size_t f = stack.back();
        stack.pop_back();
        if (!mesh.alive(f)) {
            continue;
        }
        const std::array<std::size_t, 3> vs = mesh.face_vertices(f);
        const std::array<LabelSet, 3> S = {{labels(vs[0]), labels(vs[1]), labels(vs[2])}};
        const std::array<Eigen::Vector2d, 3> X = {
            {mesh.position(vs[0]), mesh.position(vs[1]), mesh.position(vs[2])}};
        const Eigen::Vector2d centroid = (X[0] + X[1] + X[2]) / 3;

        const LabelSet common = S[0] & S[1] & S[2];
        if (common) {
            mesh.set_label(
                f,
                label_count(common) == 1 ? lowest_label(common)
                                         : nearest_label(dist, centroid, common));
            continue;
        }
        if (++stats.visits > cap) {
            stats.gave_up = true;
            mesh.set_label(f, nearest_label(dist, centroid, all));
            ++stats.fallback_labels;
            continue;
        }

        const FacePlan2D plan = plan_face_2d(dist, all, eps, X, S);
        count_kind(plan);
        if (plan.kind == FaceKind::One) {
            mesh.set_label(f, plan.label);
            continue;
        }
        for (const auto& c : plan.crossings) {
            stats.fake_edges += c.size() > 1;
        }

        std::vector<std::size_t> new_vertices;
        if (plan.kind == FaceKind::Triple) {
            const std::size_t p = mesh.split_face(f, centroid);
            if (p == invalid_id) {
                ++stats.refused_splits;
                mesh.set_label(f, nearest_label(dist, centroid, all));
                ++stats.fallback_labels;
                continue;
            }
            L[p] = plan.triple;
            new_vertices.push_back(p);
            ++stats.face_splits;
        }
        for (const int k : plan.edge_order) {
            const int l = (k + 1) % 3;
            std::size_t a = vs[k]; // what is left of the edge is a -> vs[l]
            for (const Crossing& c : plan.crossings[k]) {
                const std::size_t v = mesh.split_edge(a, vs[l], X[k] + c.t * (X[l] - X[k]));
                if (v == invalid_id) {
                    ++stats.refused_splits;
                    break;
                }
                L[v] = c.labels;
                new_vertices.push_back(v);
                ++stats.edge_splits;
                a = v;
            }
        }
        if (new_vertices.empty()) {
            mesh.set_label(f, nearest_label(dist, centroid, all));
            ++stats.fallback_labels;
            continue;
        }
        for (const std::size_t v : new_vertices) {
            for (const std::size_t g : mesh.faces_around(v)) {
                stack.push_back(g);
            }
        }
    }
    return stats;
}

// ---------------------------------------------------------------------------------------------
// 3D
// ---------------------------------------------------------------------------------------------

/// The corners of a tet's edge k, and of its face k (the face opposite corner k).
constexpr std::array<std::array<int, 2>, 6> tet_edges = {
    {{{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}}}};
constexpr std::array<std::array<int, 3>, 4> tet_faces = {
    {{{1, 2, 3}}, {{0, 2, 3}}, {{0, 1, 3}}, {{0, 1, 2}}}};

/// What a tet holds.
enum class TetKind {
    One, ///< one region: no split
    Sheet, ///< two regions, one sheet between them
    Slab, ///< three regions, two sheets that do not meet: the triple line does not run through
    Y, ///< three sheets meeting along the triple line, which runs through the tet
    Other ///< a face of no known kind
};

struct TetPlan3D
{
    TetKind kind = TetKind::Other;
    int label = -1; ///< TetKind::One: the tet's label
    std::array<std::vector<Crossing>, 6> crossings; ///< of edge k, from its first corner
    std::array<FacePlan2D, 4> faces; ///< of face k; FaceKind::Triple: pierced
    std::array<int, 6> edge_order = {{0, 1, 2, 3, 4, 5}}; ///< the order to split the edges in
};

/// The crossings along a -> b, for corners a and b of a tet, from the crossings of its edges.
inline std::vector<Crossing>
oriented_crossings(const std::array<std::vector<Crossing>, 6>& crossings, const int a, const int b)
{
    for (int k = 0; k < 6; ++k) {
        if (tet_edges[k][0] == a && tet_edges[k][1] == b) {
            return crossings[k];
        }
        if (tet_edges[k][0] == b && tet_edges[k][1] == a) {
            std::vector<Crossing> out;
            for (auto it = crossings[k].rbegin(); it != crossings[k].rend(); ++it) {
                out.push_back({1 - it->t, it->labels});
            }
            return out;
        }
    }
    return {};
}

/**
 * @brief The plan for a tet with corners X, corner label sets S and the crossings of its edges:
 * the word of each face (test 4 of 2D on the face; a face with word xyz and no corner holding
 * all three is pierced by the triple line) and what the tet holds. The triple line runs through
 * the tet when two faces are pierced, or one face and a corner holds all three labels.
 */
template <typename Dist, typename Vec>
TetPlan3D classify_tet(
    const Dist& dist,
    const std::array<Vec, 4>& X,
    const std::array<LabelSet, 4>& S,
    std::array<std::vector<Crossing>, 6> crossings)
{
    TetPlan3D plan;
    const LabelSet common = S[0] & S[1] & S[2] & S[3];
    if (common) {
        const Vec centroid = (X[0] + X[1] + X[2] + X[3]) / 4;
        plan.kind = TetKind::One;
        plan.label =
            label_count(common) == 1 ? lowest_label(common) : nearest_label(dist, centroid, common);
        return plan;
    }
    plan.crossings = std::move(crossings);
    LabelSet regions = 0;
    int pierced = 0;
    int triple_corners = 0;
    bool other = false;
    for (int f = 0; f < 4; ++f) {
        const auto& c = tet_faces[f];
        std::array<std::vector<Crossing>, 3> fc;
        for (int k = 0; k < 3; ++k) {
            fc[k] = oriented_crossings(plan.crossings, c[k], c[(k + 1) % 3]);
        }
        plan.faces[f] = classify_face(
            dist,
            std::array<Vec, 3>{{X[c[0]], X[c[1]], X[c[2]]}},
            std::array<LabelSet, 3>{{S[c[0]], S[c[1]], S[c[2]]}},
            std::move(fc));
        regions |= plan.faces[f].regions;
        pierced += plan.faces[f].kind == FaceKind::Triple;
        other = other || plan.faces[f].kind == FaceKind::Other;
    }
    for (const LabelSet s : S) {
        triple_corners += label_count(s) >= 3;
    }
    if (other) {
        plan.kind = TetKind::Other;
    } else if (label_count(regions) <= 2) {
        plan.kind = TetKind::Sheet;
    } else if (pierced == 2 || (pierced == 1 && triple_corners >= 1)) {
        plan.kind = TetKind::Y;
    } else {
        plan.kind = TetKind::Slab;
    }
    // the edges with the most crossings first
    std::stable_sort(plan.edge_order.begin(), plan.edge_order.end(), [&](int i, int j) {
        return plan.crossings[i].size() > plan.crossings[j].size();
    });
    return plan;
}

struct FillStats3D
{
    std::size_t visits = 0; ///< tets that still needed a split after the planned ones
    std::size_t sheet = 0;
    std::size_t slab = 0;
    std::size_t y = 0;
    std::size_t other = 0;
    std::size_t fake_edges = 0;
    std::size_t face_splits = 0;
    std::size_t edge_splits = 0;
    std::size_t refused_splits = 0;
    std::size_t fallback_labels = 0; ///< tets labelled by their centroid's nearest region
    bool gave_up = false; ///< the visit cap was reached

    FillStats3D& operator+=(const FillStats3D& o)
    {
        visits += o.visits;
        sheet += o.sheet;
        slab += o.slab;
        y += o.y;
        other += o.other;
        fake_edges += o.fake_edges;
        face_splits += o.face_splits;
        edge_splits += o.edge_splits;
        refused_splits += o.refused_splits;
        fallback_labels += o.fallback_labels;
        gave_up = gave_up || o.gave_up;
        return *this;
    }
};

/**
 * @brief The Voronoi fill of one component of a tet mesh, as fill_component_2d.
 *
 *  1. Every tet is planned on the mesh as it is: each edge's crossings and each face's word are
 *     found once, so the tets on an edge or a face agree.
 *  2. The splits: first every pierced face, at its centroid (its vertex labelled with the three
 *     labels); then the edges crossed twice; then the other crossings. In a tet with two
 *     pierced faces, the second face split happens in a child of the first whose opposite
 *     corner is the first piercing point, so the triple line's segment through the tet becomes
 *     an edge (with a corner holding all three labels, that corner is the one opposite the
 *     pierced face); the edge splits build the sheets around it. No tet split is needed.
 *  3. The tets are visited: a tet whose corners share a label gets it; any other (only some
 *     tets with two or more edges crossed twice leave one) is planned again and split, and the
 *     tets around its new vertices are visited in turn. A tet whose plan splits nothing, and
 *     every tet after 20 visits per tet of the component, is labelled by its centroid's nearest
 *     region.
 *
 * `mesh` is an adapter with:
 *  - std::array<size_t, 4> tet_vertices(size_t t) const
 *  - Eigen::Vector3d position(size_t v) const
 *  - double distance(const Eigen::Vector3d& x, int label) const
 *  - LabelSet initial_labels(size_t v) const: asked for every vertex of `tets` before the first
 *    split
 *  - bool alive(size_t t) const: t is a tet of the component
 *  - size_t split_edge(size_t v0, size_t v1, const Eigen::Vector3d& x): the new vertex, or
 *    invalid_id if refused
 *  - size_t split_face(size_t v0, size_t v1, size_t v2, const Eigen::Vector3d& x): likewise
 *  - std::vector<size_t> tets_around(size_t v) const: the component's tets at v
 *  - void set_label(size_t t, int label)
 */
template <typename Mesh>
FillStats3D fill_component_3d(
    Mesh& mesh,
    const std::vector<std::size_t>& tets,
    const LabelSet all,
    const double eps)
{
    using Vec = Eigen::Vector3d;
    FillStats3D stats;
    const auto dist = [&](const Vec& x, const int l) { return mesh.distance(x, l); };

    std::unordered_map<std::size_t, LabelSet> L;
    for (const std::size_t t : tets) {
        for (const std::size_t v : mesh.tet_vertices(t)) {
            if (L.count(v) == 0) {
                L[v] = mesh.initial_labels(v);
            }
        }
    }
    const auto labels = [&](const std::size_t v) {
        const auto it = L.find(v);
        return it != L.end() ? it->second : (L[v] = mesh.initial_labels(v));
    };
    const auto count_kind = [&](const TetPlan3D& plan) {
        stats.sheet += plan.kind == TetKind::Sheet;
        stats.slab += plan.kind == TetKind::Slab;
        stats.y += plan.kind == TetKind::Y;
        stats.other += plan.kind == TetKind::Other;
    };
    const auto corners = [&](const std::array<std::size_t, 4>& vs) {
        std::array<Vec, 4> X;
        std::array<LabelSet, 4> S;
        for (int k = 0; k < 4; ++k) {
            X[k] = mesh.position(vs[k]);
            S[k] = labels(vs[k]);
        }
        return std::make_pair(X, S);
    };

    // 1. the plans, on the mesh as it is
    struct EdgeSplits
    {
        std::size_t u, w; // the crossings run from u to w
        std::vector<Vec> points;
        std::vector<LabelSet> labels;
    };
    std::map<std::pair<std::size_t, std::size_t>, EdgeSplits> edges;
    std::map<std::array<std::size_t, 3>, LabelSet> pierced; // a face, its vertex's labels
    for (const std::size_t t : tets) {
        const std::array<std::size_t, 4> vs = mesh.tet_vertices(t);
        const auto [X, S] = corners(vs);
        if (S[0] & S[1] & S[2] & S[3]) {
            continue;
        }
        std::array<std::vector<Crossing>, 6> crossings;
        for (int k = 0; k < 6; ++k) {
            const int a = tet_edges[k][0];
            const int b = tet_edges[k][1];
            const auto key = std::minmax(vs[a], vs[b]);
            auto it = edges.find(key);
            if (it == edges.end()) {
                // from the lower vertex, so all the tets on the edge find the same crossings
                const int p = vs[a] < vs[b] ? a : b;
                const int q = p == a ? b : a;
                std::vector<Crossing> c;
                edge_crossings(dist, all, eps, X[p], X[q], 0., S[p], 1., S[q], c);
                EdgeSplits es{vs[p], vs[q], {}, {}};
                for (const Crossing& cr : c) {
                    es.points.push_back(X[p] + cr.t * (X[q] - X[p]));
                    es.labels.push_back(cr.labels);
                }
                stats.fake_edges += c.size() > 1;
                it = edges.emplace(key, std::move(es)).first;
            }
            const EdgeSplits& es = it->second;
            const double len2 = (X[b] - X[a]).squaredNorm();
            for (std::size_t r = 0; r < es.points.size(); ++r) {
                crossings[k].push_back(
                    {(es.points[r] - X[a]).dot(X[b] - X[a]) / len2, es.labels[r]});
            }
            std::sort(
                crossings[k].begin(),
                crossings[k].end(),
                [](const Crossing& x, const Crossing& y) { return x.t < y.t; });
        }
        const TetPlan3D plan = classify_tet(dist, X, S, std::move(crossings));
        count_kind(plan);
        for (int f = 0; f < 4; ++f) {
            if (plan.faces[f].kind == FaceKind::Triple) {
                std::array<std::size_t, 3> key = {
                    {vs[tet_faces[f][0]], vs[tet_faces[f][1]], vs[tet_faces[f][2]]}};
                std::sort(key.begin(), key.end());
                pierced.emplace(key, plan.faces[f].triple);
            }
        }
    }

    // 2. the splits: pierced faces, then the edges crossed twice, then the others
    for (const auto& [f, triple] : pierced) {
        const Vec centroid = (mesh.position(f[0]) + mesh.position(f[1]) + mesh.position(f[2])) / 3;
        const std::size_t p = mesh.split_face(f[0], f[1], f[2], centroid);
        if (p == invalid_id) {
            ++stats.refused_splits;
            continue;
        }
        L[p] = triple;
        ++stats.face_splits;
    }
    for (const bool doubly : {true, false}) {
        for (const auto& [key, es] : edges) {
            if (es.points.empty() || (es.points.size() > 1) != doubly) {
                continue;
            }
            std::size_t a = es.u; // what is left of the edge is a -> es.w
            for (std::size_t r = 0; r < es.points.size(); ++r) {
                const std::size_t v = mesh.split_edge(a, es.w, es.points[r]);
                if (v == invalid_id) {
                    ++stats.refused_splits;
                    break;
                }
                L[v] = es.labels[r];
                ++stats.edge_splits;
                a = v;
            }
        }
    }

    // 3. the labels, with a visit for each tet that still needs a split
    std::vector<std::size_t> stack;
    for (const std::size_t t : tets) {
        if (mesh.alive(t)) {
            stack.push_back(t);
        }
    }
    // every tet a split made has a vertex with two labels or more
    for (const auto& [v, s] : L) {
        if (label_count(s) > 1) {
            for (const std::size_t g : mesh.tets_around(v)) {
                stack.push_back(g);
            }
        }
    }
    const std::size_t cap = 20 * tets.size() + 100;
    while (!stack.empty()) {
        const std::size_t t = stack.back();
        stack.pop_back();
        if (!mesh.alive(t)) {
            continue;
        }
        const std::array<std::size_t, 4> vs = mesh.tet_vertices(t);
        const auto [X, S] = corners(vs);
        const Vec centroid = (X[0] + X[1] + X[2] + X[3]) / 4;
        const LabelSet common = S[0] & S[1] & S[2] & S[3];
        if (common) {
            mesh.set_label(
                t,
                label_count(common) == 1 ? lowest_label(common)
                                         : nearest_label(dist, centroid, common));
            continue;
        }
        if (++stats.visits > cap) {
            stats.gave_up = true;
            mesh.set_label(t, nearest_label(dist, centroid, all));
            ++stats.fallback_labels;
            continue;
        }

        std::array<std::vector<Crossing>, 6> crossings;
        for (int k = 0; k < 6; ++k) {
            const int a = tet_edges[k][0];
            const int b = tet_edges[k][1];
            edge_crossings(dist, all, eps, X[a], X[b], 0., S[a], 1., S[b], crossings[k]);
            stats.fake_edges += crossings[k].size() > 1;
        }
        const TetPlan3D plan = classify_tet(dist, X, S, crossings);
        count_kind(plan);

        std::vector<std::size_t> new_vertices;
        for (int f = 0; f < 4; ++f) {
            if (plan.faces[f].kind != FaceKind::Triple) {
                continue;
            }
            const auto& c = tet_faces[f];
            const std::size_t p = mesh.split_face(
                vs[c[0]],
                vs[c[1]],
                vs[c[2]],
                Vec((X[c[0]] + X[c[1]] + X[c[2]]) / 3));
            if (p == invalid_id) {
                ++stats.refused_splits;
                continue;
            }
            L[p] = plan.faces[f].triple;
            new_vertices.push_back(p);
            ++stats.face_splits;
        }
        for (const int k : plan.edge_order) {
            const int a = tet_edges[k][0];
            const int b = tet_edges[k][1];
            std::size_t u = vs[a]; // what is left of the edge is u -> vs[b]
            for (const Crossing& c : plan.crossings[k]) {
                const std::size_t v = mesh.split_edge(u, vs[b], Vec(X[a] + c.t * (X[b] - X[a])));
                if (v == invalid_id) {
                    ++stats.refused_splits;
                    break;
                }
                L[v] = c.labels;
                new_vertices.push_back(v);
                ++stats.edge_splits;
                u = v;
            }
        }
        if (new_vertices.empty()) {
            mesh.set_label(t, nearest_label(dist, centroid, all));
            ++stats.fallback_labels;
            continue;
        }
        for (const std::size_t v : new_vertices) {
            for (const std::size_t g : mesh.tets_around(v)) {
                stack.push_back(g);
            }
        }
    }
    return stats;
}

} // namespace wmtk::components::simwild::multi_voronoi
