
#pragma once
#include <wmtk/operations/SingleTupleTriMeshOperation.h>


namespace wmtk {
/**
 * Swap an edge
 *
 * @param t Input Tuple for the edge to be swaped.
 * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new edge
 * introduced
 * @note swap edge a,b to edge c,d
 * @return if swap succeed
 */
class TriMeshEdgeSwapOperation : public SingleTupleTriMeshOperation
{
public:
    bool execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m) override;
    std::string name() const override;

    std::vector<Tuple> modified_triangles(const TriMesh& m) const override;
};
}
