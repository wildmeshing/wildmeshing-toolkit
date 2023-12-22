#pragma once

#include <wmtk/operations/data/EdgeMeshEdgeOperationData.hpp>
#include <wmtk/operations/data/TetMeshEdgeOperationData.hpp>
#include <wmtk/operations/data/TriMeshEdgeOperationData.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::operations::utils {

class MultiMeshEdgeCollapseFunctor
{
public:
    void operator()(const Mesh&, const simplex::Simplex&) const;
    data::EdgeMeshEdgeOperationData operator()(EdgeMesh& m, const simplex::Simplex& s) const;
    data::TriMeshEdgeOperationData operator()(TriMesh& m, const simplex::Simplex& s) const;
    data::TetMeshEdgeOperationData operator()(TetMesh& m, const simplex::Simplex& s) const;
};

} // namespace wmtk::operations::utils
