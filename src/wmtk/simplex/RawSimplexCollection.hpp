#pragma once

#include <wmtk/Mesh.hpp>

#include "RawSimplex.hpp"

namespace wmtk::simplex {
class SimplexCollection;

class RawSimplexCollection
{
public:
    RawSimplexCollection(std::vector<RawSimplex>&& simplices = {})
        : m_simplices(std::move(simplices))
    {}

    RawSimplexCollection(SimplexCollection&& sc);

    /**
     * @brief Return const reference to the RawSimplex vector.
     */
    const std::vector<RawSimplex>& simplex_vector() const { return m_simplices; }

    /**
     * @brief Return vector of all simplices of the requested dimension.
     */
    std::vector<RawSimplex> simplex_vector(const int64_t dimension) const;

    /**
     * @brief Add simplex to the collection.
     *
     * There is no sorting or any check if the vertex already exists
     */
    void add(const RawSimplex& simplex);

    void add(const Mesh& mesh, const Simplex& simplex);

    void add(const RawSimplexCollection& simplex_collection);

    void add(const SimplexCollection& simplex_collection);

    void add(const Mesh& mesh, const PrimitiveType& ptype, const std::vector<Tuple>& tuple_vec);

    /**
     * @brief Sort simplex vector and remove duplicates.
     */
    void sort_and_clean();

    /**
     * @brief Check if simplex is contained in collection.
     *
     * Collection musst be sorted! Peform `sort_and_clean` before calling this function.
     */
    bool contains(const RawSimplex& simplex) const;

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

    auto begin() { return m_simplices.begin(); }
    auto end() { return m_simplices.end(); }
    auto begin() const { return m_simplices.begin(); }
    auto end() const { return m_simplices.end(); }
    auto cbegin() const { return m_simplices.cbegin(); }
    auto cend() const { return m_simplices.cend(); }

private:
    std::vector<RawSimplex> m_simplices;
};

} // namespace wmtk::simplex