#pragma once
#include <wmtk/Accessor.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMeshOperationExecutor.hpp>
#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/Operation.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <nlohmann/json.hpp>
using namespace wmtk;
using namespace wmtk::operations::composite;

namespace wmtk::components {

using namespace operations;
using namespace operations::tri_mesh;
using namespace adaptive_tessellation;
using namespace invariants;

void at(const nlohmann::json& j);
} // namespace wmtk::components