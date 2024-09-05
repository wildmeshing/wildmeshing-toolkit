
#pragma once

#include <wmtk/operations/edge_mesh/EdgeOperationData.hpp>
#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/operations/tri_mesh/EdgeOperationData.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::operations::utils {

class MultiMeshEdgeSplitFunctor
{
public:
    void operator()(const Mesh&, const simplex::Simplex&) const {}
    wmtk::operations::EdgeOperationData run(Mesh&, const simplex::Simplex&) const;
    edge_mesh::EdgeOperationData operator()(EdgeMesh& m, const simplex::Simplex& s) const;
    tri_mesh::EdgeOperationData operator()(TriMesh& m, const simplex::Simplex& s) const;
    tet_mesh::EdgeOperationData operator()(TetMesh& m, const simplex::Simplex& s) const;
};

} // namespace wmtk::operations::utils
