#pragma once
#include <wmtk/Accessor.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/CollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/Operation.hpp>
#include <wmtk/operations/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/BasicCollapseNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/BasicSplitNewAttributeStrategy.hpp>
#include <wmtk/operations/tri_mesh/PredicateAwareSplitNewAttributeStrategy.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
using namespace wmtk;

namespace wmtk::components {

using namespace operations;
using namespace operations::tri_mesh;
using namespace operations::composite;
using namespace adaptive_tessellation;
using namespace invariants;

void AT_smooth_interior(
    operations::internal::ATData& atdata,
    std::vector<std::shared_ptr<wmtk::operations::Operation>> ops);
} // namespace wmtk::components