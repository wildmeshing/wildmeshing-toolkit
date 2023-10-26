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
        , m_simplices(std::move(simplices))
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

    auto begin() { return m_simplices.begin(); }
    auto end() { return m_simplices.end(); }
    auto begin() const { return m_simplices.begin(); }
    auto end() const { return m_simplices.end(); }
    auto cbegin() const { return m_simplices.cbegin(); }
    auto cend() const { return m_simplices.cend(); }


protected:
    const Mesh& m_mesh;
    std::vector<Simplex> m_simplices;

protected:
    internal::SimplexLessFunctor m_simplex_is_less;
    internal::SimplexEqualFunctor m_simplex_is_equal;
};
} // namespace wmtk::simplex
