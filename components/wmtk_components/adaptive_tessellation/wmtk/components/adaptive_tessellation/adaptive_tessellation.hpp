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
#include <wmtk/components/base/Paths.hpp>
using namespace wmtk;
using namespace wmtk::operations::composite;

namespace wmtk::components {


void adaptive_tessellation(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache);
} // namespace wmtk::components