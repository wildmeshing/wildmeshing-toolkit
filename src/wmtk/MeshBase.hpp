#pragma once

#include <cstdint>
#include <vector>
#include "PrimitiveType.hpp"
#include "Tuple.hpp"

namespace wmtk {


namespace simplex {
class Simplex;
class IdSimplex;
} // namespace simplex


class MeshBase
{
public:
    virtual ~MeshBase() = default;

    virtual std::vector<Tuple> get_all(PrimitiveType type) const = 0;

    virtual int64_t top_cell_dimension() const = 0;

    /**
     * @brief switch the orientation of the Tuple of the given dimension
     * @note this is not done in place. Return a new Tuple of the switched state
     *
     * @param m
     * @param type  d-0 -> switch vertex
                    d-1 -> switch edge
                    d-2 -> switch face
                    d-3 -> switch tetrahedron
    */
    virtual Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const = 0;

    /**
     * @brief check if a simplex (encoded as a tuple/primitive pair) lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    virtual bool is_boundary(PrimitiveType, const Tuple& tuple) const = 0;

    virtual int64_t id(const simplex::Simplex& s) const = 0;
    virtual int64_t id(const Tuple& tuple, PrimitiveType pt) const = 0;
};

} // namespace wmtk