#pragma once
#include <memory>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeCollapseFunctor.hpp>
#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>

namespace wmtk {
class Mesh;
class Tuple;

class InvariantCollection;
namespace operations::utils {


// Initializes any invariants for collapseting (which is None by default, but enabling a pattern
// with other operations)
std::shared_ptr<InvariantCollection> multimesh_edge_collapse_invariants(const Mesh& m);

using CollapseReturnData = wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCache<
    MultiMeshEdgeCollapseFunctor,
    wmtk::utils::metaprogramming::MeshVariantTraits,
    simplex::Simplex>;
CollapseReturnData multi_mesh_edge_collapse(Mesh& mesh, const Tuple& t);


} // namespace operations::utils
} // namespace wmtk
