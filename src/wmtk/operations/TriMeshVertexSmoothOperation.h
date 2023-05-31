
#pragma once
#include <wmtk/operations/SingleTupleTriMeshOperation.h>
#include <wmtk/TriMeshOperation.h>


namespace wmtk {
/**
 * Smooth a vertex
 *
 * @param t Input Tuple for the vertex
 * @note no geometry changed here
 * @return if smooth succeed
 */
class TriMeshVertexSmoothOperation : public SingleTupleTriMeshOperation
{
public:
    bool execute(TriMesh& m, const Tuple& t) override;
    bool before(TriMesh& m, const Tuple& t) override;
    bool after(TriMesh& m) override;
    std::string name() const override;
    // bool invariants(TriMesh& m, ExecuteReturnData& ret_data) override;

    std::vector<Tuple> modified_triangles(const TriMesh& m) const override;
};
}
