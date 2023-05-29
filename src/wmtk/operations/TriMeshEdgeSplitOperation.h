
#pragma once
#include <wmtk/operations/SingleTupleTriMeshOperation.h>

namespace wmtk {
/**
 * Split an edge
 *
 * @param t Input Tuple for the edge to split.
 * @param[out] new_edges a vector of Tuples refering to the triangles incident to the new vertex
 * introduced
 * @return if split succeed
 */
class TriMeshEdgeSplitOperation : public SingleTupleTriMeshOperation
{
public:
    bool execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m) override;
    std::string name() const override;

    // returns a tuple to the new vertex created by this operation, where the
    // input is the tuple passed into after's ret_data.tuple.
    Tuple new_vertex(const TriMesh& m, const Tuple& t) const { return t.switch_vertex(m); }
    Tuple new_vertex(const TriMesh& m) const;
    std::array<Tuple, 2> original_endpoints(TriMesh& m, const Tuple& t) const;

    std::vector<Tuple> modified_triangles(const TriMesh& m) const override;
};
} // namespace wmtk
