
#pragma once

#include <wmtk/operations/tet_mesh/EdgeOperationData.hpp>
#include <wmtk/operations/tri_mesh/EdgeOperationData.hpp>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::operations::utils {

struct MultiMeshEdgeSplitFunctor
{
    void operator()(const Mesh&, const Simplex&) const;
    tri_mesh::EdgeOperationData operator()(TriMesh& m, const simplex::Simplex& s) const;
    tet_mesh::EdgeOperationData operator()(TetMesh& m, const simplex::Simplex& s) const;
};

} // namespace wmtk::operations::utils
