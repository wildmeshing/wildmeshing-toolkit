#pragma once

#include <vector>
#include <wmtk/Mesh.hpp>
#include "IdSimplex.hpp"

namespace wmtk::simplex {
class IdSimplexCollection
{
public:
    IdSimplexCollection(const Mesh& mesh, std::vector<IdSimplex>&& simplices = {})
        : m_mesh{mesh}
        , m_simplices(std::move(simplices))
    {
        m_simplices.reserve(100);
    }

    /**
     * @brief Return const reference to the simplex vector.
     */
    const std::vector<IdSimplex>& simplex_vector() const { return m_simplices; }
    /**
     * @brief Return vector of all simplices of the requested type.
     */
    std::vector<IdSimplex> simplex_vector(const PrimitiveType& ptype) const;

    const Mesh& mesh() const;

    /**
     * @brief Return vector of all simplices of the requested type, as tuples
     */
    std::vector<Tuple> simplex_vector_tuples(PrimitiveType ptype) const;
    std::vector<Tuple> simplex_vector_tuples() const;

    /**
     * @brief Add simplex to the collection.
     *
     * There is no sorting or any check if the vertex already exists
     */
    void add(const IdSimplex& simplex);

    void add(const IdSimplexCollection& simplex_collection);

    void add(const PrimitiveType ptype, const std::vector<Tuple>& tuple_vec);

    void add(const PrimitiveType ptype, const Tuple& tuple);
    /**
     * @brief Sort simplex vector and remove duplicates.
     */
    void sort_and_clean();
    void sort();

    /**
     * @brief Check if simplex is contained in collection.
     *
     * Collection musst be sorted! Peform `sort_and_clean` before calling this function.
     */
    bool contains(const IdSimplex& simplex) const;

    /**
     * @brief Get union of two simplex collections.
     *
     * The collections must be sorted!
     */
    static IdSimplexCollection get_union(
        const IdSimplexCollection& collection_a,
        const IdSimplexCollection& collection_b);

    /**
     * @brief Get intersection of two simplex collections.
     *
     * The collections must be sorted!
     */
    static IdSimplexCollection get_intersection(
        const IdSimplexCollection& collection_a,
        const IdSimplexCollection& collection_b);

    /**
     * @brief Check if the two simplex collections are equal
     *
     * The collections must be cleaned and sorted.
     */
    static bool are_simplex_collections_equal(
        const IdSimplexCollection& collection_a,
        const IdSimplexCollection& collection_b);


    auto begin() { return m_simplices.begin(); }
    auto end() { return m_simplices.end(); }
    auto begin() const { return m_simplices.begin(); }
    auto end() const { return m_simplices.end(); }
    auto cbegin() const { return m_simplices.cbegin(); }
    auto cend() const { return m_simplices.cend(); }

    bool operator==(const IdSimplexCollection& other) const;

    inline size_t size() const { return m_simplices.size(); }

    void reserve(const size_t new_cap);


protected:
    const Mesh& m_mesh;
    std::vector<IdSimplex> m_simplices;
};
} // namespace wmtk::simplex
