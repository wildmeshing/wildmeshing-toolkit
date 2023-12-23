#pragma once

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/operations/utils/MultiMeshEdgeSplitFunctor.hpp>
#include <wmtk/utils/metaprogramming/MeshVariantTraits.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>

#include <wmtk/Simplex.hpp>

namespace wmtk::multimesh::operations {

using SplitReturnData = wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCache<
    wmtk::operations::utils::MultiMeshEdgeSplitFunctor,
    wmtk::utils::metaprogramming::MeshVariantTraits,
    simplex::Simplex>;
} // namespace wmtk::multimesh::operations
