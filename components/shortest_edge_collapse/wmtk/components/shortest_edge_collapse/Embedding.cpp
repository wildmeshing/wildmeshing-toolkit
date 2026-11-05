#include "Embedding.hpp"

#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/predicates.hpp>

#include <algorithm>
#include <atomic>
#include <numeric>

namespace wmtk::components::shortest_edge_collapse {

namespace {

namespace pred = wmtk::utils::predicates;
using Eigen::Vector2d;
using Eigen::Vector3d;

int orient3(const Vector3d& a, const Vector3d& b, const Vector3d& c, const Vector3d& d)
{
    return int(pred::orient3d(a, b, c, d));
}

int orient2(const Vector2d& a, const Vector2d& b, const Vector2d& c)
{
    return int(pred::orient2d(a, b, c));
}

/// A coordinate plane that the non-degenerate triangle (a, b, c) projects onto without
/// degenerating, from predicates::is_degenerate()'s three, so one exists. Restricted to the
/// triangle's plane the projection is an affine bijection: it keeps the orientation signs of
/// points in that plane up to one common flip, and the tests below only compare signs taken in
/// one projection.
struct Projection
{
    int i = 0, j = 1;
    Vector2d operator()(const Vector3d& p) const { return Vector2d(p[i], p[j]); }
};

Projection projection_of(const Vector3d& a, const Vector3d& b, const Vector3d& c)
{
    for (int dim = 0; dim < 3; ++dim) {
        const Projection P{dim, (dim + 1) % 3};
        if (orient2(P(a), P(b), P(c)) != 0) return P;
    }
    log_and_throw_error("triangles_meet_outside_shared: a degenerate triangle");
}

/// Collinear x, p, q: is x on the closed segment pq? Coordinate comparisons, exact.
bool on_segment(const Vector2d& p, const Vector2d& q, const Vector2d& x)
{
    return std::min(p[0], q[0]) <= x[0] && x[0] <= std::max(p[0], q[0]) &&
           std::min(p[1], q[1]) <= x[1] && x[1] <= std::max(p[1], q[1]);
}

/// Do the closed segments pq and rs meet, p != q and r != s? A proper crossing, or an end of one
/// on the other (which covers collinear overlaps: two intervals of a line meet iff an end of one
/// is in the other).
bool segments_meet(const Vector2d& p, const Vector2d& q, const Vector2d& r, const Vector2d& s)
{
    const int o1 = orient2(p, q, r), o2 = orient2(p, q, s);
    const int o3 = orient2(r, s, p), o4 = orient2(r, s, q);
    if (o1 * o2 < 0 && o3 * o4 < 0) return true;
    return (o1 == 0 && on_segment(p, q, r)) || (o2 == 0 && on_segment(p, q, s)) ||
           (o3 == 0 && on_segment(r, s, p)) || (o4 == 0 && on_segment(r, s, q));
}

/// Is x in the closed non-degenerate triangle abc: on no edge's outer side?
bool in_triangle(const Vector2d& a, const Vector2d& b, const Vector2d& c, const Vector2d& x)
{
    const int s0 = orient2(a, b, x), s1 = orient2(b, c, x), s2 = orient2(c, a, x);
    return !((s0 < 0 || s1 < 0 || s2 < 0) && (s0 > 0 || s1 > 0 || s2 > 0));
}

/// Does the closed segment pq, p != q, meet the closed non-degenerate triangle abc?
bool segment_meets_triangle(
    const Vector3d& p,
    const Vector3d& q,
    const Vector3d& a,
    const Vector3d& b,
    const Vector3d& c)
{
    const int sp = orient3(a, b, c, p), sq = orient3(a, b, c, q);
    if (sp == sq && sp != 0) return false; // strictly on one side of the plane
    if (sp == 0 && sq == 0) {
        // In the plane: an end in the triangle, or the segment across one of its edges.
        const Projection P = projection_of(a, b, c);
        const Vector2d p2 = P(p), q2 = P(q), a2 = P(a), b2 = P(b), c2 = P(c);
        return in_triangle(a2, b2, c2, p2) || in_triangle(a2, b2, c2, q2) ||
               segments_meet(p2, q2, a2, b2) || segments_meet(p2, q2, b2, c2) ||
               segments_meet(p2, q2, c2, a2);
    }
    // The segment meets the plane in one point, so the line pq is not in it, and passes through
    // the closed triangle iff it is on the outer side of none of the triangle's edges. (All
    // three signs 0 would put the line in the plane.)
    const int e0 = orient3(p, q, a, b), e1 = orient3(p, q, b, c), e2 = orient3(p, q, c, a);
    return !((e0 < 0 || e1 < 0 || e2 < 0) && (e0 > 0 || e1 > 0 || e2 > 0));
}

/// Does the edge from v to a, a != v, run from v into the closed non-degenerate triangle
/// (v, b, c), meeting it in more than v? Iff a is in the triangle's plane and the direction a - v
/// in the triangle's corner at v: on c's side of the line vb and on b's side of the line vc,
/// either line included -- the corner, less than a half-plane, is exactly that wedge, and its
/// apex is excluded with a != v.
bool enters(const Vector3d& v, const Vector3d& a, const Vector3d& b, const Vector3d& c)
{
    if (orient3(v, b, c, a) != 0) return false; // it leaves the plane at v
    const Projection P = projection_of(v, b, c);
    const Vector2d v2 = P(v), a2 = P(a), b2 = P(b), c2 = P(c);
    const int s = orient2(v2, b2, c2);
    return s * orient2(v2, b2, a2) >= 0 && s * orient2(v2, a2, c2) >= 0;
}

Vector3d centre(const BoxTree::Box& b)
{
    return 0.5 * (b.min() + b.max());
}

} // namespace

bool triangles_meet_outside_shared(
    const std::array<size_t, 3>& ia,
    const std::array<Vector3d, 3>& A,
    const std::array<size_t, 3>& ib,
    const std::array<Vector3d, 3>& B)
{
    // in_b[k]: the corner of B that is A's corner k, or -1.
    std::array<int, 3> in_b = {{-1, -1, -1}};
    int n_shared = 0;
    for (int k = 0; k < 3; ++k) {
        for (int l = 0; l < 3; ++l) {
            if (ia[k] == ib[l]) {
                in_b[k] = l;
                ++n_shared;
            }
        }
    }

    if (n_shared == 0) {
        for (int k = 0; k < 3; ++k) {
            if (segment_meets_triangle(A[k], A[(k + 1) % 3], B[0], B[1], B[2]) ||
                segment_meets_triangle(B[k], B[(k + 1) % 3], A[0], A[1], A[2])) {
                return true;
            }
        }
        return false;
    }
    if (n_shared == 1) {
        const int ka = int(
            std::find_if(in_b.begin(), in_b.end(), [](int l) { return l >= 0; }) - in_b.begin());
        const int kb = in_b[ka];
        const Vector3d& v = A[ka];
        const Vector3d& a1 = A[(ka + 1) % 3];
        const Vector3d& a2 = A[(ka + 2) % 3];
        const Vector3d& b1 = B[(kb + 1) % 3];
        const Vector3d& b2 = B[(kb + 2) % 3];
        return segment_meets_triangle(a1, a2, B[0], B[1], B[2]) ||
               segment_meets_triangle(b1, b2, A[0], A[1], A[2]) || enters(v, a1, b1, b2) ||
               enters(v, a2, b1, b2) || enters(v, b1, a1, a2) || enters(v, b2, a1, a2);
    }
    if (n_shared == 2) {
        const int ka = int(std::find(in_b.begin(), in_b.end(), -1) - in_b.begin());
        int kb = 0;
        while (std::find(in_b.begin(), in_b.end(), kb) != in_b.end()) ++kb;
        const Vector3d& u = A[(ka + 1) % 3];
        const Vector3d& w = A[(ka + 2) % 3];
        if (orient3(u, w, A[ka], B[kb]) != 0) return false; // they meet in the edge's line only
        const Projection P = projection_of(u, w, A[ka]);
        return orient2(P(u), P(w), P(A[ka])) == orient2(P(u), P(w), P(B[kb]));
    }
    return true; // one triangle twice
}

Crossings count_crossings(
    const std::vector<Vector3d>& V,
    const std::vector<std::array<size_t, 3>>& F,
    const int num_threads)
{
    const auto corners = [&](const size_t f) {
        return std::array<Vector3d, 3>{{V[F[f][0]], V[F[f][1]], V[F[f][2]]}};
    };
    // A degenerate triangle keeps the empty box, which meets nothing: it is in no pair, and the
    // predicate, which needs both triangles non-degenerate, never sees it.
    std::vector<char> flat(F.size(), 0);
    std::vector<BoxTree::Box> boxes(F.size());
    std::atomic<size_t> n_flat{0}, n_pairs{0};
    threading::parallel_for(
        threading::range(0, F.size()),
        [&](const threading::range& r) {
            size_t local = 0;
            for (size_t f = r.begin(); f < r.end(); ++f) {
                const auto p = corners(f);
                if (pred::is_degenerate(p[0], p[1], p[2])) {
                    flat[f] = 1;
                    ++local;
                    continue;
                }
                for (const Vector3d& x : p) boxes[f].extend(x);
            }
            n_flat += local;
        },
        num_threads);
    const BoxTree tree(boxes);
    threading::parallel_for(
        threading::range(0, F.size()),
        [&](const threading::range& r) {
            size_t local = 0;
            for (size_t i = r.begin(); i < r.end(); ++i) {
                if (flat[i]) continue;
                const auto pi = corners(i);
                tree.any(boxes[i], [&](const size_t j) {
                    // each pair once
                    if (j > i && triangles_meet_outside_shared(F[i], pi, F[j], corners(j))) {
                        ++local;
                    }
                    return false; // all of them
                });
            }
            n_pairs += local;
        },
        num_threads);
    return {n_pairs.load(), n_flat.load()};
}

BoxTree::BoxTree(const std::vector<Box>& boxes)
{
    if (boxes.empty()) return;
    m_nodes.reserve(2 * boxes.size() - 1);
    std::vector<size_t> items(boxes.size());
    std::iota(items.begin(), items.end(), size_t(0));
    build(boxes, items.data(), items.data() + items.size());
}

int BoxTree::build(const std::vector<Box>& boxes, size_t* begin, size_t* end)
{
    const int id = int(m_nodes.size());
    m_nodes.emplace_back();
    if (end - begin == 1) {
        m_nodes[id].box = boxes[*begin];
        m_nodes[id].item = *begin;
        return id;
    }
    // Halves by the median centre along the axis the centres spread most.
    Box centres;
    for (const size_t* p = begin; p != end; ++p) centres.extend(centre(boxes[*p]));
    int axis = 0;
    centres.diagonal().maxCoeff(&axis);
    size_t* mid = begin + (end - begin) / 2;
    std::nth_element(begin, mid, end, [&](size_t a, size_t b) {
        return centre(boxes[a])[axis] < centre(boxes[b])[axis];
    });
    const int left = build(boxes, begin, mid);
    const int right = build(boxes, mid, end);
    m_nodes[id].left = left;
    m_nodes[id].right = right;
    m_nodes[id].box = m_nodes[left].box.merged(m_nodes[right].box);
    return id;
}

} // namespace wmtk::components::shortest_edge_collapse
