#include "adaptive_tessellation.hpp"
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>

namespace wmtk::components {
void AT_smooth_interior(
    adaptive_tessellation::operations::internal::ATData& atdata,
    std::vector<std::shared_ptr<wmtk::operations::Operation>> ops)
{
    auto& uv_mesh = atdata.uv_mesh();
    auto& uv_handle = atdata.uv_handle();
    // Energy to optimize
    std::shared_ptr<wmtk::function::PerSimplexFunction> accuracy = std::make_shared<
        adaptive_tessellation::function::PerTriangleTextureIntegralAccuracyFunction>(
        uv_mesh,
        uv_handle,
        atdata.images());
    auto energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(uv_mesh, uv_handle, *accuracy);
    ops.emplace_back(std::make_shared<wmtk::operations::OptimizationSmoothing>(energy));
    ops.back()->add_invariant(std::make_shared<TriangleInversionInvariant>(uv_mesh, uv_handle));
    ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(uv_mesh));
    ops.back()->add_transfer_strategy(atdata.m_edge_length_update);
}
} // namespace wmtk::components