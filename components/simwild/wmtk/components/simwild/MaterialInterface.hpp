#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/utils/Rational.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

/**
 * Minimization diagrams of piecewise linear functions on a tet mesh: the partition of the mesh in
 * which each point is labelled by the function that is smallest there.
 *
 * This is the material interface (MI) of Du, Zhou, Carr and Ju, "Robust Computation of Implicit
 * Surface Networks for Piecewise Linear Functions" (SIGGRAPH 2022), with min in place of max: the
 * paper labels each point by the largest function. The min diagram of f_1, ..., f_n is the max
 * diagram of -f_1, ..., -f_n, and Sense::Max gives the paper's. The sense enters through two
 * functions only, keep_sign and beats, and the code that calls them is marked SENSE: which side of
 * a plane f_m = f_j the cell of m keeps (keep_sign), which functions are dropped from a tet for
 * being beaten at all its corners (active_functions), and which planes cannot cut a cell
 * (partition_tet).
 *
 * As in the paper:
 *  - in each tet, each function is the linear interpolation of its values at the tet's corners;
 *  - a vertex is never given coordinates while the diagram is built: it is encoded by the planes it
 *    lies on (Section 5.1), and every decision is a sign computed exactly from the input values,
 * the barycentric predicate (Section 5.2, Eq. 6). The paper evaluates it with PCK; here a floating
 *    point filter with an error bound, then rational arithmetic (wmtk::Rational) when the filter
 *    cannot decide;
 *  - a convex cell is cut by a plane bottom-up from its vertex signs, with the paper's rules for
 *    vertices exactly on the plane (Section 4.1);
 *  - vertices made in different tets are unified by what they are (Section 7), so the cells of all
 *    tets form one conforming polyhedral complex.
 * One difference: the paper builds a tet's diagram incrementally, one function at a time, as one
 * cell complex in which the new function's pieces are merged into one cell (Section 4.2). Here each
 * function's cell, the convex set where it is smallest, is built on its own: the tet cut in turn by
 * the planes f_m = f_j of the other functions. The cells are the same; there is no merging step; a
 * tet with k functions takes O(k^2) cuts instead of O(k) insertions, which costs nothing for the
 * few functions a tet usually has.
 *
 * Layers, each usable on its own:
 *  1. exact::det_sign: the exact sign of a small determinant of doubles.
 *  2. Joint, joint_sign, barycentric_sign, joint_barycentric: vertices encoded by planes, and the
 *     predicates and coordinates of them.
 *  3. Cell, initial_cell, cut_cell: a convex cell of a tet, and its exact cut by one plane.
 *  4. active_functions, partition_tet: the cells of one tet.
 *  5. canonical_key: what a vertex is, whatever planes made it.
 *  6. MeshPartition, partition_mesh: the cells of every tet, vertices unified across tets.
 *  7. tetrahedralize, vertex_position: the cells as labelled tets, conforming, and the points.
 */
namespace wmtk::components::simwild::material_interface {

// ================================================================================================
// 1. Exact signs of small determinants
// ================================================================================================

namespace exact {

/// The permutations of {0, ..., n-1} (1 <= n <= 5) with their signs: the terms of a Leibniz
/// expansion.
struct Permutations
{
    std::vector<std::array<int, 5>> order;
    std::vector<int> sign;
};

inline const Permutations& permutations(const int n)
{
    static const std::array<Permutations, 6> table = [] {
        std::array<Permutations, 6> t;
        for (int k = 1; k <= 5; ++k) {
            std::array<int, 5> p{{0, 1, 2, 3, 4}};
            do {
                int inversions = 0;
                for (int i = 0; i < k; ++i) {
                    for (int j = i + 1; j < k; ++j) {
                        inversions += p[i] > p[j] ? 1 : 0;
                    }
                }
                t[k].order.push_back(p);
                t[k].sign.push_back(inversions % 2 == 0 ? 1 : -1);
            } while (std::next_permutation(p.begin(), p.begin() + k));
        }
        return t;
    }();
    return table[n];
}

/// The determinant of the n x n matrix a (row-major, 1 <= n <= 5) in floating point, by the Leibniz
/// expansion. If abs_terms is given, it receives the sum of the absolute values of the terms.
inline double det_leibniz(const double* a, const int n, double* abs_terms = nullptr)
{
    const Permutations& p = permutations(n);
    double det = 0.;
    double abs_sum = 0.;
    for (size_t t = 0; t < p.order.size(); ++t) {
        double product = 1.;
        for (int i = 0; i < n; ++i) {
            product *= a[i * n + p.order[t][i]];
        }
        det += p.sign[t] * product;
        abs_sum += std::abs(product);
    }
    if (abs_terms != nullptr) {
        *abs_terms = abs_sum;
    }
    return det;
}

/// The determinant of the n x n matrix a (row-major, 1 <= n <= 5), exactly: Gaussian elimination in
/// rationals.
inline Rational det_rational(const Rational* a, const int n)
{
    std::array<Rational, 25> m;
    for (int i = 0; i < n * n; ++i) {
        m[i] = a[i];
    }
    Rational det(1.);
    for (int c = 0; c < n; ++c) {
        int pivot = c;
        while (pivot < n && m[pivot * n + c].get_sign() == 0) {
            ++pivot;
        }
        if (pivot == n) {
            return Rational(0.);
        }
        if (pivot != c) {
            for (int k = 0; k < n; ++k) {
                std::swap(m[pivot * n + k], m[c * n + k]);
            }
            det = -det;
        }
        det = det * m[c * n + c];
        for (int r = c + 1; r < n; ++r) {
            if (m[r * n + c].get_sign() == 0) {
                continue;
            }
            const Rational factor = m[r * n + c] / m[c * n + c];
            for (int k = c + 1; k < n; ++k) {
                m[r * n + k] = m[r * n + k] - factor * m[c * n + k];
            }
        }
    }
    return det;
}

/**
 * @brief The sign of the determinant of the n x n matrix a of finite doubles (row-major,
 * 1 <= n <= 5), exact.
 *
 * In floating point first: each of the N = n! products of the Leibniz expansion has a relative
 * error of at most (n-1)u, and their sum adds at most (N-1)u times the sum of their absolute values
 * S (u = 2^-53), so a result above 2(n+N)uS has the right sign. Otherwise in rationals.
 */
inline int det_sign(const double* a, const int n)
{
    double abs_terms = 0.;
    const double det = det_leibniz(a, n, &abs_terms);
    const double u = std::numeric_limits<double>::epsilon() / 2;
    const double bound = 2. * double(n + permutations(n).order.size()) * u * abs_terms;
    if (std::isfinite(abs_terms) && std::abs(det) > bound && std::abs(det) > 1e-290) {
        return det > 0 ? 1 : -1;
    }
    std::array<Rational, 25> r;
    for (int i = 0; i < n * n; ++i) {
        r[i] = a[i];
    }
    return det_rational(r.data(), n).get_sign();
}

} // namespace exact

// ================================================================================================
// 2. Vertices encoded by planes, and the predicates on them
// ================================================================================================

/**
 * Which function labels a point. Min: the smallest, the diagram wanted here (a minimization
 * diagram). Max: the largest, the paper's material interface. SENSE marks the code that depends on
 * it.
 */
enum class Sense { Min, Max };

/// SENSE. Whether the value a beats the value b, strictly.
inline bool beats(const double a, const double b, const Sense sense)
{
    return sense == Sense::Min ? a < b : a > b;
}

/// SENSE. For the cell of function m cut by the plane f_m = f_j, given s = sign(f_j - f_m) at a
/// vertex: >= 0 if the vertex stays in the cell (f_j does not beat f_m there), < 0 if it is cut off.
inline int keep_sign(const int s, const Sense sense)
{
    return sense == Sense::Min ? s : -s;
}

/**
 * A plane of a tet's cells, in the tet's local terms (paper, Section 5.1). 0..3: the tet's boundary
 * faces, label k for the face opposite corner k, where the barycentric coordinate b_k is 0.
 * first_function + i: function i of the tet (TetValues), standing, in the cell of function m, for
 * the plane f_m = f_i.
 */
using Label = int;
inline constexpr Label first_function = 4;

inline bool is_boundary(const Label l)
{
    return l < first_function;
}

/**
 * A vertex, encoded by the planes it is made of (paper, Section 5.1): four labels, ascending. Its
 * boundary labels put it on a face of the tet, the one spanned by the other corners, as many as its
 * function labels; its functions are equal there. One function at a corner, two on an edge, three
 * on a face, four inside, so the point is unique. A vertex of the cell of function m always has m
 * among its labels, and all its functions are smallest there.
 */
using Joint = std::array<Label, 4>;

/// The functions of one tet: their ids, ascending, and their values at its corners (values[i][c]:
/// function i at corner c). Labels first_function + i refer to them.
struct TetValues
{
    std::vector<size_t> functions;
    std::vector<std::array<double, 4>> values;
};

/// A joint taken apart: the corners not on its boundary planes, ascending, and its functions
/// (indices into TetValues), n of each.
struct JointParts
{
    int n = 0;
    std::array<int, 4> corners{};
    std::array<int, 4> functions{};
};

inline JointParts joint_parts(const Joint& joint)
{
    JointParts p;
    std::array<bool, 4> on_plane{{false, false, false, false}};
    int n_functions = 0;
    for (const Label l : joint) {
        if (is_boundary(l)) {
            on_plane[l] = true;
        } else {
            p.functions[n_functions++] = l - first_function;
        }
    }
    for (int c = 0; c < 4; ++c) {
        if (!on_plane[c]) {
            p.corners[p.n++] = c;
        }
    }
    if (p.n != n_functions || p.n == 0) {
        throw std::logic_error("material_interface: a joint that is no point");
    }
    return p;
}

/// The (n+1) x (n+1) matrix of the barycentric predicate (paper, Eq. 4-6) at a joint: a row
/// [1, f restricted to the joint's corners] for each function f of the joint, then `last`.
template <typename T>
void joint_matrix(const TetValues& tv, const JointParts& p, const T* last, T* a)
{
    const int m = p.n + 1;
    for (int r = 0; r < p.n; ++r) {
        a[r * m] = T(1.);
        for (int c = 0; c < p.n; ++c) {
            a[r * m + 1 + c] = T(tv.values[p.functions[r]][p.corners[c]]);
        }
    }
    for (int c = 0; c < m; ++c) {
        a[p.n * m + c] = last[c];
    }
}

/// The sign of det A, A the predicate's matrix with last row [0, 1, ..., 1]. Its value, times the
/// joint's barycentric coordinates t, is the vector of determinants with last rows [0, e_c]; it is
/// not 0 for a joint that is a point.
inline int joint_det_sign(const TetValues& tv, const JointParts& p)
{
    std::array<double, 5> last{};
    for (int c = 0; c <= p.n; ++c) {
        last[c] = c == 0 ? 0. : 1.;
    }
    std::array<double, 25> a{};
    joint_matrix(tv, p, last.data(), a.data());
    const int s = exact::det_sign(a.data(), p.n + 1);
    if (s == 0) {
        throw std::logic_error("material_interface: a joint whose planes do not meet in a point");
    }
    return s;
}

/**
 * @brief The sign of g - f at a joint, f any of its functions (they are all equal there), g given
 * by its values at the tet's corners: the barycentric predicate sign(det A') x sign(det A) (paper,
 * Eq. 6).
 *
 * With t the joint's barycentric coordinates and lambda its functions' common value there, the row
 * [-lambda, t] spans the kernel of the joint's rows [1, f], so a determinant with last row L is
 * det(A) (L . [-lambda, t]): g(v) - lambda for L = [1, g], 1 for L = [0, 1, ..., 1]. Every entry is
 * an input value: no difference of two inputs is rounded.
 */
inline int joint_sign(const TetValues& tv, const Joint& joint, const std::array<double, 4>& g)
{
    const JointParts p = joint_parts(joint);
    std::array<double, 5> last{};
    last[0] = 1.;
    for (int c = 0; c < p.n; ++c) {
        last[1 + c] = g[p.corners[c]];
    }
    std::array<double, 25> a{};
    joint_matrix(tv, p, last.data(), a.data());
    return exact::det_sign(a.data(), p.n + 1) * joint_det_sign(tv, p);
}

/// The sign of the joint's barycentric coordinate for `corner`: 0 if a boundary plane of the joint
/// is opposite that corner, else that of det(A_c) / det(A), A_c with last row [0, e_c].
inline int barycentric_sign(const TetValues& tv, const Joint& joint, const int corner)
{
    const JointParts p = joint_parts(joint);
    const auto it = std::find(p.corners.begin(), p.corners.begin() + p.n, corner);
    if (it == p.corners.begin() + p.n) {
        return 0;
    }
    std::array<double, 5> last{};
    last[1 + (it - p.corners.begin())] = 1.;
    std::array<double, 25> a{};
    joint_matrix(tv, p, last.data(), a.data());
    return exact::det_sign(a.data(), p.n + 1) * joint_det_sign(tv, p);
}

/// The joint's barycentric coordinates in the tet, in floating point (Cramer's rule, then scaled
/// to sum to 1).
inline std::array<double, 4> joint_barycentric(const TetValues& tv, const Joint& joint)
{
    const JointParts p = joint_parts(joint);
    std::array<double, 25> a{};
    std::array<double, 5> last{};
    for (int c = 1; c <= p.n; ++c) {
        last[c] = 1.;
    }
    joint_matrix(tv, p, last.data(), a.data());
    const double det = exact::det_leibniz(a.data(), p.n + 1);
    std::array<double, 4> t{{0., 0., 0., 0.}};
    double sum = 0.;
    for (int i = 0; i < p.n; ++i) {
        last.fill(0.);
        last[1 + i] = 1.;
        joint_matrix(tv, p, last.data(), a.data());
        t[p.corners[i]] = exact::det_leibniz(a.data(), p.n + 1) / det;
        sum += t[p.corners[i]];
    }
    for (double& x : t) {
        x /= sum;
    }
    return t;
}

/// The joint's barycentric coordinates in the tet, exactly.
inline std::array<Rational, 4> joint_barycentric_exact(const TetValues& tv, const Joint& joint)
{
    const JointParts p = joint_parts(joint);
    std::array<Rational, 25> a;
    std::array<Rational, 5> last;
    for (int c = 1; c <= p.n; ++c) {
        last[c] = 1.;
    }
    joint_matrix(tv, p, last.data(), a.data());
    const Rational det = exact::det_rational(a.data(), p.n + 1);
    std::array<Rational, 4> t;
    for (int i = 0; i < p.n; ++i) {
        for (int c = 0; c <= p.n; ++c) {
            last[c] = c == 1 + i ? 1. : 0.;
        }
        joint_matrix(tv, p, last.data(), a.data());
        t[p.corners[i]] = exact::det_rational(a.data(), p.n + 1) / det;
    }
    return t;
}

// ================================================================================================
// 3. A convex cell of a tet, and its exact cut by one plane
// ================================================================================================

/// A face of a cell: the plane it lies on, and its vertices (indices into Cell::vertices) in order,
/// counterclockwise seen from outside the cell if the tet is positively oriented (clockwise if not).
struct CellFace
{
    Label plane = -1;
    std::vector<uint32_t> vertices;
};

/// A convex cell of a tet: the set where its function (a label) is smallest, or, while it is cut, a
/// part of the tet holding that set. Every vertex is on at least three faces.
struct Cell
{
    Label function = -1;
    std::vector<Joint> vertices;
    std::vector<CellFace> faces;
};

[[noreturn]] inline void cut_failure(const std::string& what)
{
    throw std::logic_error("material_interface: cut_cell: " + what);
}

/// The whole tet as a cell of `function`: its corners, corner c the joint of the three boundary
/// planes through it and the function, and its four faces.
inline Cell initial_cell(const Label function)
{
    Cell cell;
    cell.function = function;
    for (Label c = 0; c < 4; ++c) {
        Joint j{};
        int k = 0;
        for (Label b = 0; b < 4; ++b) {
            if (b != c) {
                j[k++] = b;
            }
        }
        j[3] = function;
        cell.vertices.push_back(j);
    }
    // face k opposite corner k; outward for a positively oriented tet
    cell.faces = {{0, {1, 2, 3}}, {1, {0, 3, 2}}, {2, {0, 1, 3}}, {3, {0, 2, 1}}};
    return cell;
}

/**
 * @brief Cuts a cell by a plane, keeping the side of its vertices of sign >= 0 (paper,
 * Section 4.1): each edge whose ends have opposite signs is split, each face with both signs loses
 * its negative part to a new edge on the plane, and these edges, with those of the cell already on
 * the plane, close a new face there.
 *
 * A vertex of sign 0 is on the plane: it is kept and nothing is split there. Nothing changes if no
 * vertex is negative. If none is positive, the kept part is at most a face, and the cell is empty:
 * returns false.
 *
 * @param plane labels the new face, and goes into the joints of the new vertices: a new vertex is
 * made of the two planes of the faces through its edge, the cell's function and `plane`.
 * @param sign one per vertex: -1, 0 or 1. They must be exact; with signs that no plane could have,
 * this throws.
 */
inline bool cut_cell(Cell& cell, const Label plane, const std::vector<int>& sign)
{
    bool any_negative = false;
    bool any_positive = false;
    for (const int s : sign) {
        any_negative = any_negative || s < 0;
        any_positive = any_positive || s > 0;
    }
    if (!any_negative) {
        return true;
    }
    if (!any_positive) {
        cell.vertices.clear();
        cell.faces.clear();
        return false;
    }
    const auto fail = [](const std::string& what) { cut_failure(what); };

    // the face holding each directed edge
    std::map<std::pair<uint32_t, uint32_t>, size_t> face_of;
    for (size_t f = 0; f < cell.faces.size(); ++f) {
        const auto& vs = cell.faces[f].vertices;
        for (size_t i = 0; i < vs.size(); ++i) {
            face_of[{vs[i], vs[(i + 1) % vs.size()]}] = f;
        }
    }
    enum Kind { Kept, Removed, Mixed };
    std::vector<Kind> kind(cell.faces.size());
    for (size_t f = 0; f < cell.faces.size(); ++f) {
        bool negative = false;
        bool positive = false;
        for (const uint32_t v : cell.faces[f].vertices) {
            negative = negative || sign[v] < 0;
            positive = positive || sign[v] > 0;
        }
        kind[f] = !negative ? Kept : (!positive ? Removed : Mixed);
    }

    std::vector<Joint> vertices = cell.vertices;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> cut_of; // undirected edge -> new vertex
    const auto cut = [&](const uint32_t u, const uint32_t w) -> uint32_t {
        const std::pair<uint32_t, uint32_t> key = std::minmax(u, w);
        const auto it = cut_of.find(key);
        if (it != cut_of.end()) {
            return it->second;
        }
        Joint j{
            {cell.function,
             cell.faces[face_of.at({u, w})].plane,
             cell.faces[face_of.at({w, u})].plane,
             plane}};
        std::sort(j.begin(), j.end());
        vertices.push_back(j);
        return cut_of[key] = uint32_t(vertices.size() - 1);
    };

    // the new face's edges, each directed opposite to the face it comes from: next vertex
    std::map<uint32_t, uint32_t> cap_next;
    const auto add_cap_edge = [&](const uint32_t from, const uint32_t to) {
        if (!cap_next.emplace(from, to).second) {
            fail("two edges of the new face leave one vertex");
        }
    };
    std::vector<CellFace> faces;
    for (size_t f = 0; f < cell.faces.size(); ++f) {
        const CellFace& face = cell.faces[f];
        const auto& vs = face.vertices;
        const size_t n = vs.size();
        if (kind[f] == Removed) {
            continue;
        }
        if (kind[f] == Kept) {
            // an edge on the plane between this face and a removed one is an edge of the new face
            for (size_t i = 0; i < n; ++i) {
                const uint32_t u = vs[i];
                const uint32_t w = vs[(i + 1) % n];
                if (sign[u] == 0 && sign[w] == 0 && kind[face_of.at({w, u})] == Removed) {
                    add_cap_edge(w, u);
                }
            }
            faces.push_back(face);
            continue;
        }
        // Mixed: walk from a positive vertex, so that the negative vertices are one run
        size_t start = 0;
        while (sign[vs[start]] <= 0) {
            ++start;
        }
        CellFace kept{face.plane, {}};
        int64_t before = -1; // the last point kept before the negative run
        int64_t after = -1; // the first point kept after it
        int runs = 0;
        for (size_t i = 0; i < n; ++i) {
            const uint32_t u = vs[(start + i) % n];
            const uint32_t w = vs[(start + i + 1) % n];
            if (sign[u] >= 0) {
                kept.vertices.push_back(u);
            }
            if (sign[u] >= 0 && sign[w] < 0) {
                ++runs;
                if (sign[u] > 0) {
                    const uint32_t x = cut(u, w);
                    kept.vertices.push_back(x);
                    before = x;
                } else {
                    before = u;
                }
            } else if (sign[u] < 0 && sign[w] >= 0) {
                if (sign[w] > 0) {
                    const uint32_t x = cut(u, w);
                    kept.vertices.push_back(x);
                    after = x;
                } else {
                    after = w;
                }
            }
        }
        if (runs != 1 || before < 0 || after < 0) {
            fail("a face crossed by the plane more than once");
        }
        add_cap_edge(uint32_t(after), uint32_t(before));
        faces.push_back(std::move(kept));
    }

    // the new face, on the plane
    CellFace cap{plane, {}};
    if (cap_next.empty()) {
        fail("no face on the plane");
    }
    const uint32_t first = cap_next.begin()->first;
    for (uint32_t v = first;;) {
        cap.vertices.push_back(v);
        const auto it = cap_next.find(v);
        if (it == cap_next.end() || cap.vertices.size() > cap_next.size()) {
            fail("the new face's edges do not close");
        }
        v = it->second;
        if (v == first) {
            break;
        }
    }
    if (cap.vertices.size() != cap_next.size() || cap.vertices.size() < 3) {
        fail("the new face's edges are not one loop");
    }
    faces.push_back(std::move(cap));

    // drop the negative vertices, renumber
    const size_t n_old = cell.vertices.size();
    std::vector<uint32_t> new_id(vertices.size(), std::numeric_limits<uint32_t>::max());
    Cell out;
    out.function = cell.function;
    for (size_t v = 0; v < vertices.size(); ++v) {
        if (v >= n_old || sign[v] >= 0) {
            new_id[v] = uint32_t(out.vertices.size());
            out.vertices.push_back(vertices[v]);
        }
    }
    std::vector<bool> used(out.vertices.size(), false);
    for (CellFace& face : faces) {
        for (uint32_t& v : face.vertices) {
            v = new_id[v];
            if (v == std::numeric_limits<uint32_t>::max()) {
                fail("a kept face with a removed vertex");
            }
            used[v] = true;
        }
    }
    if (std::find(used.begin(), used.end(), false) != used.end()) {
        fail("a kept vertex on no face");
    }
    out.faces = std::move(faces);
    cell = std::move(out);
    return true;
}

// ================================================================================================
// 4. The cells of one tet
// ================================================================================================

/**
 * @brief The functions that can label a point of a tet: the indices of those of `values` (one per
 * function, its values at the four corners, functions in ascending id order) that are kept.
 *
 * SENSE. A function is dropped if another beats it at all four corners, as it then does everywhere
 * in the tet (paper, Section 3), or if it equals one of lower index at all four corners, as ties go
 * to the lower index. A function kept may still label no part of the tet.
 */
inline std::vector<size_t> active_functions(
    const std::vector<std::array<double, 4>>& values,
    const Sense sense)
{
    std::vector<size_t> active;
    for (size_t i = 0; i < values.size(); ++i) {
        bool dropped = false;
        for (size_t j = 0; j < values.size() && !dropped; ++j) {
            if (j == i) {
                continue;
            }
            bool beaten = true;
            bool equal = true;
            for (int c = 0; c < 4; ++c) {
                beaten = beaten && beats(values[j][c], values[i][c], sense);
                equal = equal && values[j][c] == values[i][c];
            }
            dropped = beaten || (equal && j < i);
        }
        if (!dropped) {
            active.push_back(i);
        }
    }
    return active;
}

/// The diagram in one tet: its functions, and one convex cell for each function that is smallest on
/// a part of the tet with a volume.
struct TetPartition
{
    TetValues values;
    std::vector<Cell> cells;
};

/**
 * @brief The cells of one tet: for each function m of `values`, the tet cut in turn by the planes
 * f_m = f_j of the others, keeping the side where f_m is not beaten (SENSE).
 *
 * The cells cover the tet and do not overlap: where two functions tie on a part with a volume, they
 * are equal at all four corners and the lower index takes it. `values` should hold no function that
 * active_functions drops; those would only cost time.
 */
inline TetPartition partition_tet(TetValues values, const Sense sense = Sense::Min)
{
    TetPartition out;
    const size_t k = values.functions.size();
    for (size_t m = 0; m < k; ++m) {
        Cell cell = initial_cell(first_function + Label(m));
        bool alive = true;
        for (size_t j = 0; j < k && alive; ++j) {
            if (j == m) {
                continue;
            }
            // SENSE. f_j beats f_m nowhere if it does at no corner (both are linear): no cut. Equal
            // at every corner, they are equal everywhere, and the lower index takes the cell.
            bool can_cut = false;
            bool equal = true;
            for (int c = 0; c < 4; ++c) {
                can_cut = can_cut || beats(values.values[j][c], values.values[m][c], sense);
                equal = equal && values.values[j][c] == values.values[m][c];
            }
            if (equal) {
                alive = j > m;
                continue;
            }
            if (!can_cut) {
                continue;
            }
            std::vector<int> sign(cell.vertices.size());
            bool all_zero = true;
            for (size_t v = 0; v < cell.vertices.size(); ++v) {
                sign[v] = keep_sign(joint_sign(values, cell.vertices[v], values.values[j]), sense);
                all_zero = all_zero && sign[v] == 0;
            }
            if (all_zero) {
                // f_j = f_m everywhere: the lower index takes the cell
                alive = j > m;
                continue;
            }
            alive = cut_cell(cell, first_function + Label(j), sign);
        }
        if (alive) {
            out.cells.push_back(std::move(cell));
        }
    }
    out.values = std::move(values);
    return out;
}

// ================================================================================================
// 5. What a vertex is
// ================================================================================================

/**
 * What a vertex of a cell is, whatever planes made it: the corners of the smallest face of the tet
 * holding it (those with a positive barycentric coordinate there) and every function smallest there
 * (equal to the cell's), ascending, in the tet's local terms (corners 0..3, functions as indices
 * into TetValues). Two vertices are the same point if and only if they have the same key.
 */
struct LocalKey
{
    std::vector<int> corners;
    std::vector<int> functions;

    bool operator<(const LocalKey& o) const
    {
        return std::tie(corners, functions) < std::tie(o.corners, o.functions);
    }
};

inline LocalKey canonical_key(const TetValues& tv, const Joint& joint)
{
    const JointParts p = joint_parts(joint);
    LocalKey key;
    for (int i = 0; i < p.n; ++i) {
        if (barycentric_sign(tv, joint, p.corners[i]) > 0) {
            key.corners.push_back(p.corners[i]);
        }
    }
    for (size_t f = 0; f < tv.functions.size(); ++f) {
        const Label l = first_function + Label(f);
        if (std::find(joint.begin(), joint.end(), l) != joint.end() ||
            joint_sign(tv, joint, tv.values[f]) == 0) {
            key.functions.push_back(int(f));
        }
    }
    return key;
}

// ================================================================================================
// 6. The diagram of a tet mesh
// ================================================================================================

/**
 * The diagram of a tet mesh: the cells of all its tets, sharing their vertices, faces matching
 * across tets.
 */
struct MeshPartition
{
    struct Vertex
    {
        /// the mesh vertices spanning the smallest simplex of the mesh holding the point, ascending
        /// (one: a mesh vertex; two: on an edge; three: on a face; four: inside a tet)
        std::vector<size_t> simplex;
        /// the functions smallest there, all equal, ascending
        std::vector<size_t> functions;
        /// a tet holding the point, its barycentric coordinates there, and the encoding it was
        /// made with in that tet: the tet's corners not on its boundary planes (local, ascending)
        /// and its functions (ids), as many of each (exact_barycentric recomputes them exactly)
        size_t tet = 0;
        std::array<double, 4> barycentric{};
        std::vector<int> joint_corners;
        std::vector<size_t> joint_functions;
    };
    struct Face
    {
        /// in order, counterclockwise seen from outside the cell in a positively oriented tet
        std::vector<size_t> vertices;
        /// on the tet's boundary: its corner opposite the face (0..3), else -1
        int tet_face = -1;
        /// inside the tet: the function of the cell beyond, else -1
        int64_t other_function = -1;
    };
    struct Cell
    {
        size_t tet = 0;
        size_t function = 0;
        std::vector<Face> faces;
    };
    std::vector<Vertex> vertices;
    std::vector<Cell> cells;
};

/**
 * @brief The minimization diagram (SENSE) of functions over a tet mesh, each function the linear
 * interpolation of its values at the mesh vertices.
 *
 * @param tets the tets, by their corners' mesh vertex ids.
 * @param n_functions the number of functions; they are 0 .. n_functions-1.
 * @param value value(v, f): function f at mesh vertex v, finite.
 * @param candidates if given, candidates(t): the functions to consider in tet t, ascending. They
 * must agree on shared faces, at least on the functions smallest somewhere on them; leaving out a
 * function that a tet's other functions beat at all its corners changes nothing.
 */
inline MeshPartition partition_mesh(
    const std::vector<std::array<size_t, 4>>& tets,
    const size_t n_functions,
    const std::function<double(size_t, size_t)>& value,
    const Sense sense = Sense::Min,
    const std::function<std::vector<size_t>(size_t)>& candidates = {})
{
    MeshPartition out;
    std::map<std::pair<std::vector<size_t>, std::vector<size_t>>, size_t> vertex_of;
    std::vector<size_t> all(n_functions);
    for (size_t f = 0; f < n_functions; ++f) {
        all[f] = f;
    }
    for (size_t t = 0; t < tets.size(); ++t) {
        const std::vector<size_t> considered = candidates ? candidates(t) : all;
        std::vector<std::array<double, 4>> values(considered.size());
        for (size_t i = 0; i < considered.size(); ++i) {
            if (i > 0 && considered[i] <= considered[i - 1]) {
                throw std::invalid_argument(
                    "material_interface: a tet's candidate functions are not ascending");
            }
            for (int c = 0; c < 4; ++c) {
                values[i][c] = value(tets[t][c], considered[i]);
                if (!std::isfinite(values[i][c])) {
                    throw std::invalid_argument(
                        "material_interface: a function value is not finite");
                }
            }
        }
        TetValues tv;
        const std::vector<size_t> active = active_functions(values, sense);
        for (const size_t i : active) {
            tv.functions.push_back(considered[i]);
            tv.values.push_back(values[i]);
        }
        if (tv.functions.empty()) {
            continue;
        }
        // The functions dropped for equalling a kept one at all four corners: equal everywhere in
        // the tet, they are smallest wherever it is, and a vertex's key must hold them too, or a
        // neighbour where they differ would key the same point otherwise.
        std::vector<std::vector<size_t>> twins(active.size());
        for (size_t i = 0; i < values.size(); ++i) {
            if (std::find(active.begin(), active.end(), i) != active.end()) {
                continue;
            }
            for (size_t a = 0; a < active.size(); ++a) {
                if (values[i] == values[active[a]]) {
                    twins[a].push_back(considered[i]);
                    break;
                }
            }
        }
        const TetPartition tp = partition_tet(std::move(tv), sense);

        std::map<Joint, size_t> global_of; // the joints met in this tet
        const auto global_vertex = [&](const Joint& joint) -> size_t {
            const auto it = global_of.find(joint);
            if (it != global_of.end()) {
                return it->second;
            }
            const LocalKey local = canonical_key(tp.values, joint);
            std::pair<std::vector<size_t>, std::vector<size_t>> key;
            for (const int c : local.corners) {
                key.first.push_back(tets[t][c]);
            }
            std::sort(key.first.begin(), key.first.end());
            for (const int f : local.functions) {
                key.second.push_back(tp.values.functions[f]);
                key.second.insert(key.second.end(), twins[f].begin(), twins[f].end());
            }
            std::sort(key.second.begin(), key.second.end());
            auto [found, inserted] = vertex_of.emplace(key, out.vertices.size());
            if (inserted) {
                MeshPartition::Vertex v;
                v.simplex = key.first;
                v.functions = key.second;
                v.tet = t;
                v.barycentric = joint_barycentric(tp.values, joint);
                const JointParts p = joint_parts(joint);
                for (int i = 0; i < p.n; ++i) {
                    v.joint_corners.push_back(p.corners[i]);
                    v.joint_functions.push_back(tp.values.functions[p.functions[i]]);
                }
                out.vertices.push_back(std::move(v));
            }
            return global_of[joint] = found->second;
        };

        for (const Cell& cell : tp.cells) {
            MeshPartition::Cell c;
            c.tet = t;
            c.function = tp.values.functions[cell.function - first_function];
            for (const CellFace& face : cell.faces) {
                MeshPartition::Face f;
                for (const uint32_t v : face.vertices) {
                    f.vertices.push_back(global_vertex(cell.vertices[v]));
                }
                if (is_boundary(face.plane)) {
                    f.tet_face = face.plane;
                } else {
                    f.other_function = int64_t(tp.values.functions[face.plane - first_function]);
                }
                c.faces.push_back(std::move(f));
            }
            out.cells.push_back(std::move(c));
        }
    }
    return out;
}

/// A vertex's barycentric coordinates in its tet (MeshPartition::Vertex::tet), exactly, from the
/// same values that partition_mesh was given.
inline std::array<Rational, 4> exact_barycentric(
    const MeshPartition::Vertex& vertex,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::function<double(size_t, size_t)>& value)
{
    TetValues tv;
    Joint joint{};
    std::array<bool, 4> spanned{{false, false, false, false}};
    for (const int c : vertex.joint_corners) {
        spanned[c] = true;
    }
    int k = 0;
    for (Label c = 0; c < 4; ++c) {
        if (!spanned[c]) {
            joint[k++] = c;
        }
    }
    for (size_t i = 0; i < vertex.joint_functions.size(); ++i) {
        tv.functions.push_back(vertex.joint_functions[i]);
        std::array<double, 4> v{};
        for (int c = 0; c < 4; ++c) {
            v[c] = value(tets[vertex.tet][c], vertex.joint_functions[i]);
        }
        tv.values.push_back(v);
        joint[k++] = first_function + Label(i);
    }
    return joint_barycentric_exact(tv, joint);
}

// ================================================================================================
// 7. The cells as tets, and the points
// ================================================================================================

/// A tet of the cells' tetrahedralization: its vertices (MeshPartition::vertices), the function of
/// its cell and the mesh tet it lies in.
struct LabelledTet
{
    std::array<size_t, 4> vertices{};
    size_t function = 0;
    size_t tet = 0;
};

/**
 * @brief The cells cut into tets, conforming across cells and tets, with no new vertex: each face
 * is fanned from its smallest vertex id (so a face shared by two cells is cut the same way from
 * both), and each cell is coned from its smallest vertex id over its faces that do not hold it. The
 * tets have the orientation of the mesh tets they lie in.
 */
inline std::vector<LabelledTet> tetrahedralize(const MeshPartition& partition)
{
    std::vector<LabelledTet> out;
    for (const MeshPartition::Cell& cell : partition.cells) {
        size_t apex = std::numeric_limits<size_t>::max();
        for (const auto& face : cell.faces) {
            for (const size_t v : face.vertices) {
                apex = std::min(apex, v);
            }
        }
        for (const auto& face : cell.faces) {
            const auto& vs = face.vertices;
            if (std::find(vs.begin(), vs.end(), apex) != vs.end()) {
                continue;
            }
            const size_t n = vs.size();
            const size_t s = std::min_element(vs.begin(), vs.end()) - vs.begin();
            for (size_t i = 1; i + 1 < n; ++i) {
                out.push_back(
                    {{apex, vs[s], vs[(s + i) % n], vs[(s + i + 1) % n]}, cell.function, cell.tet});
            }
        }
    }
    return out;
}

/// A vertex's position, in floating point, from its tet's corners: position(v), a mesh vertex's.
inline Vector3d vertex_position(
    const MeshPartition::Vertex& vertex,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::function<Vector3d(size_t)>& position)
{
    Vector3d p = Vector3d::Zero();
    for (int c = 0; c < 4; ++c) {
        if (vertex.barycentric[c] != 0.) {
            p += vertex.barycentric[c] * position(tets[vertex.tet][c]);
        }
    }
    return p;
}

} // namespace wmtk::components::simwild::material_interface
