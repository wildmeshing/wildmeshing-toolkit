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

    /**
     * @brief Return const reference to the RawSimplex vector.
     */
    const std::vector<RawSimplex>& simplex_vector() const { return m_simplices; }

    /**
     * @brief Return vector of all simplices of the requested dimension.
     */
    std::vector<RawSimplex> simplex_vector(const long dimension) const;

    /**
     * @brief Add simplex to the collection.
     *
     * There is no sorting or any check if the vertex already exists
     */
    void add(const RawSimplex& simplex);

    void add(const RawSimplexCollection& simplex_collection);

    void add(const SimplexCollection& simplex_collection);

    void add(const Mesh& mesh, const PrimitiveType& ptype, const std::vector<Tuple>& tuple_vec);

private:
    std::vector<RawSimplex> m_simplices;
};

} // namespace wmtk::simplex