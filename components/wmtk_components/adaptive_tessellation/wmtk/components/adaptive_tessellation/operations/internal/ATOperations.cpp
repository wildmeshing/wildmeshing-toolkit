#include "ATOperations.hpp"
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>


#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>

#include <wmtk/components/adaptive_tessellation/invariants/MaxTriangleNumberInvariant.hpp>
#include <wmtk/components/adaptive_tessellation/invariants/RGBSplitInvariant.hpp>
#include <wmtk/components/adaptive_tessellation/invariants/RGBSwapInvariant.hpp>
#include <wmtk/invariants/BoundarySimplexInvariant.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/StateChanges.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>
#include <wmtk/operations/composite/TriFaceSplit.hpp>

#include <wmtk/components/adaptive_tessellation/operations/RGBSplit.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGBSplitWithPositionOptimization.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGBSwap.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGRefine.hpp>

#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <wmtk/Scheduler.hpp>

#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>

#include "predicates.h"

#include <fstream>
namespace wmtk::components::operations::internal {
using namespace wmtk::operations;
// using namespace operations::tri_mesh;
using namespace wmtk::operations::composite;
using namespace wmtk::function;
using namespace wmtk::invariants;

ATOperations::ATOperations(ATData& atdata, double target_distance, int64_t target_triangle_number)
    : m_atdata(atdata)
    , m_target_distance(target_distance)
    , m_target_triangle_number(target_triangle_number)
    , m_uv_accessor(m_atdata.uv_mesh().create_accessor(m_atdata.m_uv_handle.as<double>()))
    , m_uvmesh_xyz_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_uvmesh_xyz_handle.as<double>()))
    , m_distance_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_distance_error_handle.as<double>()))
    , m_curved_edge_length_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_curved_edge_length_handle.as<double>()))
    , m_face_rgb_state_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_face_rgb_state_handle.as<int64_t>()))
    , m_edge_rgb_state_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_edge_rgb_state_handle.as<int64_t>()))
    , m_edge_todo_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_edge_todo_handle.as<int64_t>()))

{
    m_ops.clear();

    set_uvmesh_xyz_update_rule();
    initialize_xyz();

    set_distance_error_update_rule();
    initialize_distance_error();

    set_curved_edge_length_update_rule();
    initialize_curved_edge_length();

    // Lambdas for priority
    m_valence_improvement = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        const auto [val_before, val_after] =
            wmtk::invariants::ValenceImprovementInvariant::valence_change(
                *m_atdata.uv_mesh_ptr(),
                s);
        return std::vector<long>({val_before - val_after});
    };

    m_triangle_distance_edge_length = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>(
            {-m_distance_error_accessor.scalar_attribute(s.tuple()),
             -m_curved_edge_length_accessor.scalar_attribute(s.tuple())});
    };
}
void ATOperations::set_split_all_attribute_update_None(wmtk::operations::EdgeSplit& op)
{
    op.set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.set_new_attribute_strategy(
        m_atdata.m_uvmesh_xyz_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.set_new_attribute_strategy(
        m_atdata.m_distance_error_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.set_new_attribute_strategy(
        m_atdata.m_curved_edge_length_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.set_new_attribute_strategy(
        m_atdata.m_face_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.set_new_attribute_strategy(
        m_atdata.m_edge_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    op.set_new_attribute_strategy(
        m_atdata.m_edge_todo_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
}

int64_t ATOperations::AT_rgb_split()
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();

    auto rgb_split = std::make_shared<wmtk::operations::composite::RGBSplit>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle,
        m_atdata.m_edge_rgb_state_handle);

    set_split_all_attribute_update_None(rgb_split->split());

    rgb_split->split().add_transfer_strategy(m_uvmesh_xyz_update);
    rgb_split->split().add_transfer_strategy(m_distance_error_update);
    rgb_split->split().add_transfer_strategy(m_curved_edge_length_update);

    rgb_split->add_invariant(std::make_shared<wmtk::RGBSplitInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle.as<int64_t>(),
        m_atdata.m_edge_rgb_state_handle.as<int64_t>()));

    rgb_split->add_invariant(std::make_shared<wmtk::TodoInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_edge_todo_handle.as<int64_t>(),
        1));
    if (m_target_triangle_number > 0) {
        rgb_split->add_invariant(std::make_shared<wmtk::MaxTriangleNumberInvariant>(
            *uv_mesh_ptr,
            m_target_triangle_number));
    }
    m_ops.emplace_back(rgb_split);
    return m_ops.size() - 1;
}

int64_t ATOperations::AT_rgb_split_with_position_optimization()
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();

    auto rgb_split_w_position =
        std::make_shared<wmtk::operations::composite::RGBSplitWithPositionOptimization>(
            *uv_mesh_ptr,
            m_atdata.uv_handle(),
            m_atdata.m_mapping_ptr,
            m_atdata.m_face_rgb_state_handle,
            m_atdata.m_edge_rgb_state_handle);

    set_split_all_attribute_update_None(rgb_split_w_position->split().split());
    rgb_split_w_position->split().split().set_new_attribute_strategy(m_atdata.uv_handle());

    rgb_split_w_position->split().add_transfer_strategy(m_uvmesh_xyz_update);
    rgb_split_w_position->split().add_transfer_strategy(m_distance_error_update);
    rgb_split_w_position->split().add_transfer_strategy(m_curved_edge_length_update);

    rgb_split_w_position->add_invariant(std::make_shared<wmtk::RGBSplitInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle.as<int64_t>(),
        m_atdata.m_edge_rgb_state_handle.as<int64_t>()));

    rgb_split_w_position->add_invariant(std::make_shared<wmtk::TodoInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_edge_todo_handle.as<int64_t>(),
        1));
    if (m_target_triangle_number > 0) {
        rgb_split_w_position->add_invariant(std::make_shared<wmtk::MaxTriangleNumberInvariant>(
            *uv_mesh_ptr,
            m_target_triangle_number));
    }
    m_ops.emplace_back(rgb_split_w_position);
    return m_ops.size() - 1;
}

int64_t ATOperations::AT_rgb_swap()
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();

    auto rgb_swap = std::make_shared<wmtk::operations::composite::RGBSwap>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle,
        m_atdata.m_edge_rgb_state_handle,
        m_atdata.m_edge_todo_handle);

    rgb_swap->add_invariant(std::make_shared<wmtk::RGBSwapInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle.as<int64_t>(),
        m_atdata.m_edge_rgb_state_handle.as<int64_t>()));
    rgb_swap->swap().collapse().add_invariant(
        std::make_shared<MultiMeshLinkConditionInvariant>(*uv_mesh_ptr));
    rgb_swap->add_invariant(std::make_shared<InteriorEdgeInvariant>(*uv_mesh_ptr));
    rgb_swap->add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));


    rgb_swap->swap().split().set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::Mean);
    rgb_swap->swap().collapse().set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::CollapseBasicStrategy::CopyOther);
    rgb_swap->swap().split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    rgb_swap->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_uvmesh_xyz_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);
    {
        // the update strategy that doesn't matter
        rgb_swap->swap().split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
        rgb_swap->swap().collapse().set_new_attribute_strategy(
            m_atdata.m_distance_error_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
        rgb_swap->swap().split().set_new_attribute_strategy(
            m_atdata.m_curved_edge_length_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        rgb_swap->swap().collapse().set_new_attribute_strategy(
            m_atdata.m_curved_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }
    rgb_swap->add_transfer_strategy(m_uvmesh_xyz_update);
    rgb_swap->add_transfer_strategy(m_distance_error_update);
    rgb_swap->add_transfer_strategy(m_curved_edge_length_update);

    rgb_swap->swap().split().set_new_attribute_strategy(
        m_atdata.m_face_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    rgb_swap->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_face_rgb_state_handle,
        wmtk::operations::CollapseBasicStrategy::None);
    rgb_swap->swap().split().set_new_attribute_strategy(
        m_atdata.m_edge_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    rgb_swap->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_edge_rgb_state_handle,
        wmtk::operations::CollapseBasicStrategy::None);
    rgb_swap->swap().split().set_new_attribute_strategy(
        m_atdata.m_edge_todo_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    rgb_swap->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_edge_todo_handle,
        wmtk::operations::CollapseBasicStrategy::None);

    m_ops.emplace_back(rgb_swap);
    return m_ops.size() - 1;
}
} // namespace wmtk::components::operations::internal