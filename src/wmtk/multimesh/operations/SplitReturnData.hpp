#pragma once

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeSplitFunctor.hpp>
#include <wmtk/simplex/utils/MeshSimplexComparator.hpp>
#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>


namespace wmtk::multimesh::operations {

using SplitReturnData =
    wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCacheCustomComparator<
        wmtk::operations::utils::MultiMeshEdgeSplitFunctor,
        wmtk::utils::metaprogramming::MeshVariantTraits,
        wmtk::simplex::utils::MeshSimplexComparator,
        simplex::Simplex>;
} // namespace wmtk::multimesh::operations
