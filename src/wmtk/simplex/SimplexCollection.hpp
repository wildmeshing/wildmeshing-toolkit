#pragma once

#include <vector>
#include <wmtk/Mesh.hpp>
#include "Simplex.hpp"
#include "internal/SimplexEqualFunctor.hpp"
#include "internal/SimplexLessFunctor.hpp"

namespace wmtk::simplex {
class SimplexCollection
{
public:
    SimplexCollection(const Mesh& mesh, std::vector<Simplex>&& simplices = {})
        : m_mesh{mesh}
        , m_simplex_is_less(mesh)
        , m_simplex_is_equal(mesh)
        , m_simplices(std::move(simplices))
    {}

    /**
     * @brief Return const reference to the simplex vector.
     */
    const std::vector<Simplex>& simplex_vector() const { return m_simplices; }
    /**
     * @brief Return vector of all simplices of the requested type.
     */
    std::vector<Simplex> simplex_vector(const PrimitiveType& ptype) const;

    /**
     * @brief Add simplex to the collection.
     *
     * There is no sorting or any check if the vertex already exists
     */
    void add(const Simplex& simplex);

    void add(const SimplexCollection& simplex_collection);

    /**
     * @brief Sort simplex vector and remove duplicates.
     */
    void sort_and_clean();

    /**
     * @brief Check if simplex is contained in collection.
     *
     * Collection musst be sorted! Peform `sort_and_clean` before calling this function.
     */
    bool contains(const Simplex& simplex) const;

    /**
     * @brief Get union of two simplex collections.
     *
     * The collections must be sorted!
     */
    static SimplexCollection get_union(
        const SimplexCollection& collection_a,
        const SimplexCollection& collection_b);

    /**
     * @brief Get intersection of two simplex collections.
     *
     * The collections must be sorted!
     */
    static SimplexCollection get_intersection(
        const SimplexCollection& collection_a,
        const SimplexCollection& collection_b);

protected:
    internal::SimplexLessFunctor m_simplex_is_less;
    internal::SimplexEqualFunctor m_simplex_is_equal;

protected:
    const Mesh& m_mesh;
    std::vector<Simplex> m_simplices;
};
} // namespace wmtk::simplex
