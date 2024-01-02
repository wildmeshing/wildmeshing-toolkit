#pragma once

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeCollapseFunctor.hpp>
#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>


namespace wmtk::multimesh::operations {

using CollapseReturnData = wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCache<
    wmtk::operations::utils::MultiMeshEdgeCollapseFunctor,
    wmtk::utils::metaprogramming::MeshVariantTraits,
    simplex::Simplex>;
}
