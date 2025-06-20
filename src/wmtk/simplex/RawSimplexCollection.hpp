#pragma once

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
     * @brief Return vector of all simplices of the requested dimension.
     */
    template <int DIM>
    const std::vector<RawSimplex<DIM + 1>>& simplex_vector() const
    {
        if constexpr (DIM == 0) {
            return m_v;
        } else if constexpr (DIM == 1) {
            return m_e;
        } else if constexpr (DIM == 2) {
            return m_f;
        } else {
            static_assert(DIM == 3);
            return m_t;
        }
    }

    const std::vector<RawSimplex<1>> vertices() const;
    const std::vector<RawSimplex<2>> edges() const;
    const std::vector<RawSimplex<3>> faces() const;
    const std::vector<RawSimplex<4>> tets() const;

    /**
     * @brief Add simplex to the collection.
     *
     * There is no sorting or any check if the vertex already exists
     */
    template <int N>
    void add(const RawSimplex<N>& simplex)
    {
        if constexpr (N == 1) {
            m_v.emplace_back(simplex);
        } else if constexpr (N == 2) {
            m_e.emplace_back(simplex);
        } else if constexpr (N == 3) {
            m_f.emplace_back(simplex);
        } else {
            static_assert(N == 4);
            m_t.emplace_back(simplex);
        }
    }

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
    template <int N>
    bool contains(const RawSimplex<N>& simplex) const
    {
        static_assert(N > 0);
        static_assert(N < 5);

        const auto& vec = simplex_vector<N - 1>();
        assert(std::is_sorted(vec.begin(), vec.end()));
        return std::binary_search(vec.begin(), vec.end(), simplex);
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

    // auto begin() { return m_simplices.begin(); }
    // auto end() { return m_simplices.end(); }
    // auto begin() const { return m_simplices.begin(); }
    // auto end() const { return m_simplices.end(); }
    // auto cbegin() const { return m_simplices.cbegin(); }
    // auto cend() const { return m_simplices.cend(); }

    size_t size() const;

private:
    std::vector<RawSimplex<1>> m_v;
    std::vector<RawSimplex<2>> m_e;
    std::vector<RawSimplex<3>> m_f;
    std::vector<RawSimplex<4>> m_t;
};

template <int N>
inline RawSimplexCollection RawSimplexCollection::faces_from_simplex(const RawSimplex<N>& simplex)
{
    const auto& v = simplex.vertices();

    RawSimplexCollection sc;

    if constexpr (N == 1) {
        // do nothing
    } else if constexpr (N == 2) {
        sc.m_v.emplace_back(RawSimplex<1>(v[0]));
        sc.m_v.emplace_back(RawSimplex<1>(v[1]));
    } else if constexpr (N == 3) {
        sc.m_v.emplace_back(RawSimplex<1>(v[0]));
        sc.m_v.emplace_back(RawSimplex<1>(v[1]));
        sc.m_v.emplace_back(RawSimplex<1>(v[2]));
        sc.m_e.emplace_back(RawSimplex<2>(v[0], v[1]));
        sc.m_e.emplace_back(RawSimplex<2>(v[0], v[2]));
        sc.m_e.emplace_back(RawSimplex<2>(v[1], v[2]));
    } else {
        static_assert(N == 4);

        sc.m_v.emplace_back(RawSimplex<1>(v[0]));
        sc.m_v.emplace_back(RawSimplex<1>(v[1]));
        sc.m_v.emplace_back(RawSimplex<1>(v[2]));
        sc.m_v.emplace_back(RawSimplex<1>(v[3]));
        sc.m_e.emplace_back(RawSimplex<2>(v[0], v[1]));
        sc.m_e.emplace_back(RawSimplex<2>(v[0], v[2]));
        sc.m_e.emplace_back(RawSimplex<2>(v[0], v[3]));
        sc.m_e.emplace_back(RawSimplex<2>(v[1], v[2]));
        sc.m_e.emplace_back(RawSimplex<2>(v[1], v[3]));
        sc.m_e.emplace_back(RawSimplex<2>(v[2], v[3]));
        sc.m_f.emplace_back(RawSimplex<3>(v[0], v[1], v[2]));
        sc.m_f.emplace_back(RawSimplex<3>(v[0], v[1], v[3]));
        sc.m_f.emplace_back(RawSimplex<3>(v[0], v[2], v[3]));
        sc.m_f.emplace_back(RawSimplex<3>(v[1], v[2], v[3]));
    }

    return sc;
}

} // namespace wmtk::simplex