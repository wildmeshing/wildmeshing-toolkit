#pragma once

#include <vector>
#include "Simplex.hpp"

namespace wmtk::simplex {
class SimplexCollection
{
public:
    /**
     * @brief Return const reference to the simplex vector.
     */
    const std::vector<Simplex>& simplex_vector() const { return m_simplices; }
    /**
     * @brief Return vector of all simplices of the requested type.
     */
    std::vector<Simplex> simplex_vector(const PrimitiveType& ptype) const;

    void add(const Simplex& simplex);

    /**
     * @brief Sort simplex vector and remove duplicates.
     */
    void sort_and_clean();

    static SimplexCollection get_union(
        const SimplexCollection& collection_a,
        const SimplexCollection& collection_b);
    // useful: std::sort, std::set_union/difference/intersection

    static SimplexCollection get_intersection(
        const SimplexCollection& collection_a,
        const SimplexCollection& collection_b);

protected:
    std::vector<Simplex> m_simplices;
};
} // namespace wmtk::simplex