#pragma once

#include <cassert>
#include "RawSimplex.hpp"

namespace wmtk::simplex {

class RawSimplexCollection
{
public:
    RawSimplexCollection() = default;

    // RawSimplexCollection(std::vector<RawSimplex>&& simplices = {})
    //     : m_simplices(std::move(simplices))
    //{}

    /**
     * @brief Return const reference to the RawSimplex vector.
     */
    // const std::vector<RawSimplex>& simplex_vector() const { return m_simplices; }

    /**
     * @brief Return vector of all vertices.
     */
    const std::vector<Vertex>& vertices() const;
    const std::vector<Edge>& edges() const;
    const std::vector<Face>& faces() const;
    const std::vector<Tet>& tets() const;

    /**
     * @brief Add simplex to the collection.
     *
     * There is no sorting or any check if the vertex already exists
     */
    void add(const Vertex& s) { m_v.emplace_back(s); }
    void add(const Edge& s) { m_e.emplace_back(s); }
    void add(const Face& s) { m_f.emplace_back(s); }
    void add(const Tet& s) { m_t.emplace_back(s); }

    void add(const RawSimplexCollection& simplex_collection);

    /**
     * @brief Sort simplex vector and remove duplicates.
     */
    void sort_and_clean();

    /**
     * @brief Check if simplex is contained in collection.
     *
     * Collection musst be sorted! Peform `sort_and_clean` before calling this function.
     */
    bool contains(const Vertex& simplex) const
    {
        assert(std::is_sorted(m_v.begin(), m_v.end()));
        return std::binary_search(m_v.begin(), m_v.end(), simplex);
    }
    bool contains(const Edge& simplex) const
    {
        assert(std::is_sorted(m_e.begin(), m_e.end()));
        return std::binary_search(m_e.begin(), m_e.end(), simplex);
    }
    bool contains(const Face& simplex) const
    {
        assert(std::is_sorted(m_f.begin(), m_f.end()));
        return std::binary_search(m_f.begin(), m_f.end(), simplex);
    }
    bool contains(const Tet& simplex) const
    {
        assert(std::is_sorted(m_t.begin(), m_t.end()));
        return std::binary_search(m_t.begin(), m_t.end(), simplex);
    }

    /**
     * @brief Get union of two simplex collections.
     *
     * The collections must be sorted!
     */
    static RawSimplexCollection get_union(
        const RawSimplexCollection& collection_a,
        const RawSimplexCollection& collection_b);

    /**
     * @brief Get intersection of two simplex collections.
     *
     * The collections must be sorted!
     */
    static RawSimplexCollection get_intersection(
        const RawSimplexCollection& collection_a,
        const RawSimplexCollection& collection_b);

    /**
     * @brief Check if the two simplex collections are equal
     *
     * The collections must be cleaned and sorted.
     */
    static bool are_simplex_collections_equal(
        const RawSimplexCollection& collection_a,
        const RawSimplexCollection& collection_b);

    /**
     * @brief Get all faces of the simplex
     */
    template <int N>
    static RawSimplexCollection faces_from_simplex(const RawSimplex<N>& simplex);

    size_t size() const;

private:
    std::vector<Vertex> m_v;
    std::vector<Edge> m_e;
    std::vector<Face> m_f;
    std::vector<Tet> m_t;
};

template <int N>
inline RawSimplexCollection RawSimplexCollection::faces_from_simplex(const RawSimplex<N>& simplex)
{
    const auto& v = simplex.vertices();

    RawSimplexCollection sc;

    if constexpr (N == 1) {
        // do nothing
    } else if constexpr (N == 2) {
        sc.m_v.emplace_back(Vertex(v[0]));
        sc.m_v.emplace_back(Vertex(v[1]));
    } else if constexpr (N == 3) {
        sc.m_v.emplace_back(Vertex(v[0]));
        sc.m_v.emplace_back(Vertex(v[1]));
        sc.m_v.emplace_back(Vertex(v[2]));
        sc.m_e.emplace_back(Edge(v[0], v[1]));
        sc.m_e.emplace_back(Edge(v[0], v[2]));
        sc.m_e.emplace_back(Edge(v[1], v[2]));
    } else {
        static_assert(N == 4);

        sc.m_v.emplace_back(Vertex(v[0]));
        sc.m_v.emplace_back(Vertex(v[1]));
        sc.m_v.emplace_back(Vertex(v[2]));
        sc.m_v.emplace_back(Vertex(v[3]));
        sc.m_e.emplace_back(Edge(v[0], v[1]));
        sc.m_e.emplace_back(Edge(v[0], v[2]));
        sc.m_e.emplace_back(Edge(v[0], v[3]));
        sc.m_e.emplace_back(Edge(v[1], v[2]));
        sc.m_e.emplace_back(Edge(v[1], v[3]));
        sc.m_e.emplace_back(Edge(v[2], v[3]));
        sc.m_f.emplace_back(Face(v[0], v[1], v[2]));
        sc.m_f.emplace_back(Face(v[0], v[1], v[3]));
        sc.m_f.emplace_back(Face(v[0], v[2], v[3]));
        sc.m_f.emplace_back(Face(v[1], v[2], v[3]));
    }

    return sc;
}

} // namespace wmtk::simplex