#include "adaptive_tessellation.hpp"
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>

namespace wmtk::components {} // namespace wmtk::components