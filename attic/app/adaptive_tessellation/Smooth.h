#pragma once
#include <igl/Timer.h>
#include <wmtk/operations/TriMeshVertexSmoothOperation.h>
#include <wmtk/operations/TriMeshOperationShim.hpp>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/Energy2dOptimizationUtils.h>
#include <wmtk/utils/ScalarUtils.h>
#include <array>
#include <limits>
#include <optional>
#include <type_traits>
#include <typeinfo>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"
using namespace adaptive_tessellation;
using namespace wmtk;

class AdaptiveTessellationVertexSmoothOperation : public wmtk::TriMeshOperationShim<
                                                      AdaptiveTessellation,
                                                      AdaptiveTessellationVertexSmoothOperation,
                                                      wmtk::TriMeshVertexSmoothOperation>
{
public:
    bool execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m);
    std::vector<Tuple> modified_triangles(const TriMesh&) const override;
};

// a seam vertex can have more than one mirror vertex
class AdaptiveTessellationSmoothSeamVertexOperation
    : public wmtk::TriMeshOperationShim<
          AdaptiveTessellation,
          AdaptiveTessellationSmoothSeamVertexOperation,
          wmtk::TriMeshVertexSmoothOperation>
{
public:
    bool execute(AdaptiveTessellation& m, const Tuple& t);
    bool before(AdaptiveTessellation& m, const Tuple& t);
    bool after(AdaptiveTessellation& m);
    std::vector<Tuple> modified_triangles(const TriMesh&) const override;
};
