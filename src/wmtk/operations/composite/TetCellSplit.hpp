#pragma once

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>

namespace wmtk::operations::composite {
/**
 * TetCellSplit
 *
 * This class is used to split a tetrahedra into 4 parts, we did this
 * by only using two atomic operations, EdgeSplit and EdgeCollapse
 * in order to make our lib more reliable.
 *
 * return tuple: the return tuple is the original tuple, arrowing to the new vertex,
 * the face belongs to the original (original_vertex,new vertex,switch_vertex(original_vertex))
 * the tetrahedra is the most front one.
 */
class TetCellSplit : public EdgeSplit, public EdgeCollapse
{
public:
    TetCellSplit(Mesh& m);

    PrimitiveType primitive_type() const override { return PrimitiveType::Tetrahedron; }

protected:
    std::vector<Simplex> execute(const Simplex& simplex) override;
    std::vector<Simplex> unmodified_primitives(const Simplex& simplex) const override;
};

} // namespace wmtk::operations::composite
