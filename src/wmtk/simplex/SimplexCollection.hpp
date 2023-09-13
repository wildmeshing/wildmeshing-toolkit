#pragma once

#include <vector>
#include "../Mesh.hpp"
#include "Simplex.hpp"
#include "SimplexEqualFunctor.hpp"
#include "SimplexLessFunctor.hpp"

namespace wmtk::simplex {
class SimplexCollection
{
public:
    SimplexCollection(const Mesh& mesh)
        : m_mesh{mesh}
        , m_simplex_is_less(mesh)
        , m_simplex_is_equal(mesh)
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

private:
    SimplexLessFunctor m_simplex_is_less;
    SimplexEqualFunctor m_simplex_is_equal;

protected:
    const Mesh& m_mesh;
    std::vector<Simplex> m_simplices;
};
} // namespace wmtk::simplex