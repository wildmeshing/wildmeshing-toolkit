#pragma once

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

/**
 * The fill of the geometric tight seal (tight seal v2 note, section 4.2,
 * https://hackmd.io/@Spenbert02/HkX3vAvaGg). It reaches the mesh only through the hooks of
 * PairFill::Mesh, so the same code fills triangles (N = 3) and tets (N = 4).
 *
 * Each component to fill (C_i in the note) has K functions, one per color it borders: the
 * distances to those colors, taken at the vertices the mesh had before the fill and linear in
 * each of the component's cells then (its original cells). The fill splits edges, nothing else,
 * until the corners of every cell share a function smallest there; by linearity that function
 * is smallest on the whole cell, and labels it.
 *
 * Vertices. Each vertex v holds the values d_k(v). The note keeps a list E(v) of pairs (k, l)
 * counted as exactly equal at v, whatever their stored values; here a pair is in E(v) exactly when
 * its stored values are equal, so every comparison is between plain values and is transitive. A
 * vertex the mesh had keeps its values (E(v): the pairs with equal input values). A vertex x the
 * fill makes on the edge (a, b), at fraction s from a, takes d_k(x) = (1 - s) d_k(a) + s d_k(b) for
 * every k, and then ties, in classes whose members all take the class's least value:
 *  - the pair (i, j) it was made for (in its component only);
 *  - every pair tied at both a and b (with equal inputs the interpolation gives equal values, so
 *    such a pair is only tied explicitly once another tie moved one of its values);
 *  - every pair whose values at x lie strictly on one side while a and b both lie on the other
 *    side or on the level (closed side): x must stay on its edge's side of every level set, as the
 *    note's remark asks of each split;
 *  - and, by the classes, every pair tied through others: d_k = d_l and d_l = d_m give d_k = d_m.
 * Each tie can move values, so the last two are checked again until nothing changes. The note
 * does not close E(x) this way. Without it, a vertex x at a triple point, made for (i, j) on an
 * edge whose ends tie (i, m), gets (i, j) and (i, m) but not (j, m); rounding then puts d_j - d_m a
 * few ulps off 0, the edges from x cross that level set, and their splits pile up at x without
 * ending the cells there. In exact arithmetic neither the closure nor the side check ever ties a
 * pair the note does not; they are counted (Stats::n_closed). A vertex's label set: the functions
 * with the least value there.
 *
 * Cells. A cell is finalized if its corners' label sets share a function, which labels it (the
 * smallest if several). An original cell is finalized when all its children are.
 *
 * The fill, for each original cell t in turn, unless it is finalized. A function is dominated in t
 * if another is strictly smaller at all of t's corners; it is then smallest nowhere in t, and is
 * left out. For each pair (i, j) of the others in turn, each vertex of t's children has a sign,
 * that of d_i - d_j (0 for a tie). A queue starts with t's unfinalized children. A cell popped is
 * skipped if it is gone (split since it was queued), finalized, or has no edge from a + vertex to
 * a - one; otherwise its first such edge (a, b), corners sorted by id, is split where d_i - d_j
 * vanishes, s = δ(a) / (δ(a) - δ(b)) with δ = d_i - d_j. Each cell the split makes, in t or not:
 * takes its parent's label if its parent was finalized; else its own, if it is finalized; else is
 * queued if it lies in t; else waits for its own original cell.
 *
 * Each split removes one edge across the pair's level set and makes none (every new edge ends at
 * the new vertex, a tie for the pair), so each pair ends. Every vertex of t lies on the closed side
 * its edge's ends share of each level set, so a pair done stays done, and a function dominated at
 * t's corners stays at least its dominator everywhere in t. Once t's last pair is done, no child of
 * t has an edge across a level set of t's other functions: "d_k <= d_l at every corner" is then a
 * total order on them in each child (comparisons at a vertex are transitive), and its least
 * function is smallest at every corner, dominated functions included. So each child is
 * finalized. A cell left unfinalized (only by a refused split) is labelled by the function
 * smallest at its centroid, and counted (Stats::n_fallback).
 */
namespace wmtk::components::simwild::tight_seal {

template <int N> // corners of a cell: 3 (triangles) or 4 (tets)
class PairFill
{
public:
    using Cell = std::array<size_t, N>;

    /// What the fill needs from the mesh.
    struct Mesh
    {
        /// The corners of a live cell.
        std::function<Cell(size_t)> corners;
        /// d_k(v) in component c: its function k at a vertex v the mesh had before the fill.
        std::function<double(size_t c, size_t k, size_t v)> value;
        /// Splits the edge (a, b) where the linear function with values delta_a at a and delta_b
        /// at b, of opposite signs, vanishes: at fraction delta_a / (delta_a - delta_b) from a.
        /// Returns the new vertex, or size_t(-1) if the split is refused. Sets `children` to a
        /// (parent, child) pair for each cell the split made, from every cell on the edge, in a
        /// component or not.
        std::function<size_t(
            size_t a,
            size_t b,
            double delta_a,
            double delta_b,
            std::vector<std::pair<size_t, size_t>>& children)>
            split;
    };

    struct Stats
    {
        size_t n_original = 0; ///< original cells
        size_t n_original_finalized = 0; ///< of them, finalized before any split
        size_t n_pairs = 0; ///< pairs of functions run, over the original cells
        size_t n_dominated = 0; ///< functions left out as dominated, over the original cells
        size_t n_split = 0; ///< edges split
        size_t n_refused = 0; ///< splits the mesh refused
        size_t n_closed = 0; ///< pairs new vertices tie by the closure or the side check
        size_t n_cells = 0; ///< cells labelled
        size_t n_fallback = 0; ///< of them, not finalized: labelled at their centroid
    };

    /// components[c]: the original cells of component c (each in one component only);
    /// n_functions[c]: its number of functions, one per color it borders.
    PairFill(
        const Mesh& mesh,
        const std::vector<std::vector<size_t>>& components,
        const std::vector<size_t>& n_functions)
        : m_mesh(mesh)
        , m_n_functions(n_functions)
        , m_vertices(components.size())
    {
        if (n_functions.size() != components.size()) {
            log_and_throw_error(
                "PairFill: {} components but {} function counts",
                components.size(),
                n_functions.size());
        }
        for (size_t c = 0; c < components.size(); ++c) {
            for (const size_t cell : components[c]) {
                grow(cell);
                if (m_orig[cell] >= 0) {
                    log_and_throw_error("PairFill: cell {} is in two components", cell);
                }
                m_orig[cell] = int(m_originals.size());
                m_originals.push_back({c, m_mesh.corners(cell), {cell}});
            }
        }
        m_stats.n_original = m_originals.size();
    }

    /// Fills: afterwards every live cell of the components has a label.
    void run()
    {
        for (size_t o = 0; o < m_originals.size(); ++o) {
            const size_t cell = *m_originals[o].children.begin();
            m_label[cell] = finalized_label(cell);
            m_stats.n_original_finalized += m_label[cell] >= 0 ? 1 : 0;
        }

        std::vector<std::pair<size_t, size_t>> children;
        for (size_t o = 0; o < m_originals.size(); ++o) {
            Original& t = m_originals[o];
            if (std::all_of(t.children.begin(), t.children.end(), [&](const size_t cell) {
                    return m_label[cell] >= 0;
                })) {
                continue;
            }
            const std::vector<size_t> functions = undominated(t);
            m_stats.n_dominated += m_n_functions[t.comp] - functions.size();
            for (size_t p = 0; p < functions.size(); ++p) {
                for (size_t q = p + 1; q < functions.size(); ++q) {
                    const size_t i = functions[p];
                    const size_t j = functions[q];
                    ++m_stats.n_pairs;
                    std::deque<size_t> queue;
                    for (const size_t cell : t.children) {
                        if (m_label[cell] < 0) {
                            queue.push_back(cell);
                        }
                    }
                    while (!queue.empty()) {
                        const size_t cell = queue.front();
                        queue.pop_front();
                        if (t.children.count(cell) == 0 || m_label[cell] >= 0) {
                            continue;
                        }
                        const auto [a, b] = crossing_edge(t.comp, cell, i, j);
                        if (a == size_t(-1)) {
                            continue;
                        }
                        const Vertex& A = vertex(t.comp, a);
                        const Vertex& B = vertex(t.comp, b);
                        const double delta_a = A.d[i] - A.d[j];
                        const double delta_b = B.d[i] - B.d[j];
                        const double s = delta_a / (delta_a - delta_b);
                        const size_t x = m_mesh.split(a, b, delta_a, delta_b, children);
                        if (x == size_t(-1)) {
                            ++m_stats.n_refused;
                            continue;
                        }
                        ++m_stats.n_split;
                        m_new_vertices.push_back(x);
                        m_made.emplace(x, Made{a, b, s, t.comp, i, j});
                        adopt(children, o, queue);
                    }
                }
            }
        }

        for (Original& t : m_originals) {
            for (const size_t cell : t.children) {
                ++m_stats.n_cells;
                if (m_label[cell] < 0) {
                    m_label[cell] = smallest_at_centroid(t.comp, cell);
                    ++m_stats.n_fallback;
                }
            }
        }
    }

    /// After run(): the component of a live cell, -1 if in none.
    int component(const size_t cell) const
    {
        return cell < m_orig.size() && m_orig[cell] >= 0 ? int(m_originals[m_orig[cell]].comp) : -1;
    }
    /// After run(): the label of a live cell of a component, a function of that component.
    int label(const size_t cell) const { return cell < m_label.size() ? m_label[cell] : -1; }
    /// The vertices the fill made, in order.
    const std::vector<size_t>& new_vertices() const { return m_new_vertices; }
    const Stats& stats() const { return m_stats; }

private:
    struct Vertex
    {
        std::vector<double> d; // the values, one per function; tied ones are equal
        std::vector<size_t> labels; // the label set, ascending
    };
    // where the fill made a vertex: on the edge (a, b) at fraction s from a, for the pair (i, j)
    // of component comp
    struct Made
    {
        size_t a, b;
        double s;
        size_t comp, i, j;
    };
    struct Original
    {
        size_t comp;
        Cell corners;
        std::set<size_t> children; // its live cells, itself before any split
    };

    /// The sign of d_k - d_l at v, 0 for a tie.
    static int compare(const Vertex& v, const size_t k, const size_t l)
    {
        return (v.d[k] > v.d[l]) - (v.d[k] < v.d[l]);
    }

    /// The functions with the least value at v.
    static std::vector<size_t> label_set(const Vertex& v)
    {
        const double least = *std::min_element(v.d.begin(), v.d.end());
        std::vector<size_t> labels;
        for (size_t k = 0; k < v.d.size(); ++k) {
            if (v.d[k] == least) {
                labels.push_back(k);
            }
        }
        return labels;
    }

    /// The vertex made at fraction s of the edge from A to B; `pair`, if given, is the pair it
    /// is made for.
    Vertex interpolate(
        const Vertex& A,
        const Vertex& B,
        const double s,
        const std::pair<size_t, size_t>* pair)
    {
        const size_t K = A.d.size();
        Vertex x;
        x.d.resize(K);
        const double r = 1. - s;
        for (size_t k = 0; k < K; ++k) {
            x.d[k] = r * A.d[k] + s * B.d[k];
        }
        // the tie classes, each rooted at its smallest function
        std::vector<size_t> root(K);
        for (size_t k = 0; k < K; ++k) {
            root[k] = k;
        }
        const auto find = [&](size_t k) {
            while (root[k] != k) {
                k = root[k] = root[root[k]];
            }
            return k;
        };
        const auto tie = [&](size_t k, size_t l) {
            k = find(k);
            l = find(l);
            if (k > l) {
                std::swap(k, l);
            }
            root[l] = k;
        };
        if (pair) {
            tie(pair->first, pair->second);
        }
        for (bool tied = true; tied;) {
            // each class takes its least value
            std::vector<double> least(K, std::numeric_limits<double>::infinity());
            for (size_t k = 0; k < K; ++k) {
                least[find(k)] = std::min(least[find(k)], x.d[k]);
            }
            for (size_t k = 0; k < K; ++k) {
                x.d[k] = least[find(k)];
            }
            // then each pair stays on the closed side its edge's ends share
            tied = false;
            for (size_t k = 0; k < K; ++k) {
                for (size_t l = k + 1; l < K; ++l) {
                    if (find(k) == find(l)) {
                        continue;
                    }
                    const int sx = compare(x, k, l);
                    const int sa = compare(A, k, l);
                    const int sb = compare(B, k, l);
                    if ((sa >= 0 && sb >= 0 && sx < 0) || (sa <= 0 && sb <= 0 && sx > 0)) {
                        tie(k, l);
                        tied = true;
                        ++m_stats.n_closed;
                    }
                }
            }
        }
        return x;
    }

    /// v's data in component c, computed the first time it is asked for: a vertex the mesh had
    /// takes the mesh's values, one the fill made is interpolated from its edge's ends.
    const Vertex& vertex(const size_t c, const size_t v)
    {
        std::unordered_map<size_t, Vertex>& data = m_vertices[c];
        if (const auto it = data.find(v); it != data.end()) {
            return it->second;
        }
        Vertex x;
        if (const auto it = m_made.find(v); it == m_made.end()) {
            x.d.resize(m_n_functions[c]);
            for (size_t k = 0; k < x.d.size(); ++k) {
                x.d[k] = m_mesh.value(c, k, v);
            }
        } else {
            const Made made = it->second;
            const std::pair<size_t, size_t> pair(made.i, made.j);
            // references into an unordered_map stay valid as it grows
            const Vertex& A = vertex(c, made.a);
            const Vertex& B = vertex(c, made.b);
            x = interpolate(A, B, made.s, made.comp == c ? &pair : nullptr);
        }
        x.labels = label_set(x);
        return data.emplace(v, std::move(x)).first->second;
    }

    /// The smallest function in all the corners' label sets, -1 if there is none.
    int finalized_label(const size_t cell)
    {
        const size_t c = m_originals[m_orig[cell]].comp;
        const Cell vs = m_mesh.corners(cell);
        std::vector<size_t> common = vertex(c, vs[0]).labels;
        for (int q = 1; q < N && !common.empty(); ++q) {
            const std::vector<size_t>& labels = vertex(c, vs[q]).labels;
            std::vector<size_t> both;
            std::set_intersection(
                common.begin(),
                common.end(),
                labels.begin(),
                labels.end(),
                std::back_inserter(both));
            common = std::move(both);
        }
        return common.empty() ? -1 : int(common.front());
    }

    /// The functions of t's component that no other is strictly smaller than at all of t's
    /// corners, ascending.
    std::vector<size_t> undominated(const Original& t)
    {
        const size_t K = m_n_functions[t.comp];
        std::array<const Vertex*, N> corner;
        for (int q = 0; q < N; ++q) {
            corner[q] = &vertex(t.comp, t.corners[q]);
        }
        // the functions smallest at a corner first: they dominate most of the others
        std::vector<size_t> order;
        for (int q = 0; q < N; ++q) {
            order.insert(order.end(), corner[q]->labels.begin(), corner[q]->labels.end());
        }
        for (size_t l = 0; l < K; ++l) {
            order.push_back(l);
        }
        std::vector<size_t> functions;
        for (size_t k = 0; k < K; ++k) {
            const bool dominated = std::any_of(order.begin(), order.end(), [&](const size_t l) {
                if (l == k) {
                    return false;
                }
                for (int q = 0; q < N; ++q) {
                    if (compare(*corner[q], l, k) >= 0) {
                        return false;
                    }
                }
                return true;
            });
            if (!dominated) {
                functions.push_back(k);
            }
        }
        return functions;
    }

    /// The first edge of the cell, corners sorted by id, from a vertex with d_i > d_j to one with
    /// d_i < d_j; (-1, -1) if none.
    std::pair<size_t, size_t>
    crossing_edge(const size_t c, const size_t cell, const size_t i, const size_t j)
    {
        Cell vs = m_mesh.corners(cell);
        std::sort(vs.begin(), vs.end());
        std::array<int, N> signs;
        for (int q = 0; q < N; ++q) {
            signs[q] = compare(vertex(c, vs[q]), i, j);
        }
        for (int p = 0; p < N; ++p) {
            for (int q = p + 1; q < N; ++q) {
                if (signs[p] * signs[q] < 0) {
                    return {vs[p], vs[q]};
                }
            }
        }
        return {size_t(-1), size_t(-1)};
    }

    /// The cells a split made take their parents' places: each takes its parent's original cell
    /// and, if its parent was finalized, its label; else its own, if it is finalized; else it is
    /// queued if it lies in o.
    void adopt(
        const std::vector<std::pair<size_t, size_t>>& children,
        const size_t o,
        std::deque<size_t>& queue)
    {
        // the parents' states first: a child may take its parent's id
        std::map<size_t, std::pair<int, int>> parents;
        for (const auto& [parent, child] : children) {
            grow(parent);
            parents.emplace(parent, std::make_pair(m_orig[parent], m_label[parent]));
        }
        for (const auto& [parent, state] : parents) {
            if (state.first >= 0) {
                m_originals[state.first].children.erase(parent);
            }
        }
        for (const auto& [parent, child] : children) {
            grow(child);
            const auto [orig, label] = parents.at(parent);
            m_orig[child] = orig;
            m_label[child] = -1;
            if (orig < 0) {
                continue; // in no component
            }
            m_originals[orig].children.insert(child);
            if (label >= 0) {
                m_label[child] = label;
                continue;
            }
            m_label[child] = finalized_label(child);
            if (m_label[child] < 0 && size_t(orig) == o) {
                queue.push_back(child);
            }
        }
    }

    /// The function smallest at the cell's centroid (the mean of its corners' values), the
    /// smallest index on a tie.
    int smallest_at_centroid(const size_t c, const size_t cell)
    {
        std::vector<double> sum(m_n_functions[c], 0.);
        for (const size_t v : m_mesh.corners(cell)) {
            const Vertex& x = vertex(c, v);
            for (size_t k = 0; k < sum.size(); ++k) {
                sum[k] += x.d[k];
            }
        }
        return int(std::min_element(sum.begin(), sum.end()) - sum.begin());
    }

    void grow(const size_t cell)
    {
        if (cell >= m_orig.size()) {
            m_orig.resize(cell + 1, -1);
            m_label.resize(cell + 1, -1);
        }
    }

    const Mesh& m_mesh;
    std::vector<size_t> m_n_functions;
    std::vector<Original> m_originals;
    std::vector<std::unordered_map<size_t, Vertex>> m_vertices; // per component
    std::unordered_map<size_t, Made> m_made;
    std::vector<int> m_orig; // per cell id: its original cell, -1 if in no component
    std::vector<int> m_label; // per cell id: its label, -1 if none (yet)
    std::vector<size_t> m_new_vertices;
    Stats m_stats;
};

} // namespace wmtk::components::simwild::tight_seal
