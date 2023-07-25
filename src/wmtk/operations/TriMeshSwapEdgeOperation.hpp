
#pragma once
#include "TupleOperation.hpp"

namespace wmtk {
// TODO: fill out pertinent virtuals
class TriMeshEdgeOperation : public TupleOperation
{
    TriMeshSwapEdgeOperation(TriMesh& m, Tuple& t);


    bool execute() override;
    std::vector<Tuple> modified_triangles() const override;

private:
    std::vector<Tuple> m_modified_triangles;
};
} // namespace wmtk
