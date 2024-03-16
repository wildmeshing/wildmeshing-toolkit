#include "ATOperations.hpp"
#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>


#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>

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
#include <wmtk/components/adaptive_tessellation/operations/RGBSwap.hpp>
#include <wmtk/components/adaptive_tessellation/operations/RGRefine.hpp>

#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <wmtk/Scheduler.hpp>

#include "ATOptions.hpp"

#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>

#include "predicates.h"

#include <fstream>
namespace wmtk::components::operations::internal {
using namespace wmtk::operations;
// using namespace operations::tri_mesh;
using namespace wmtk::operations::composite;
using namespace wmtk::function;
using namespace wmtk::invariants;

ATOperations::ATOperations(
    ATData& atdata,
    double target_distance,
    double target_edge_length,
    double envelope_size,
    double barrier_weight,
    double barrier_triangle_area,
    double distance_weight,
    double amips_weight,
    bool area_weighted_amips)
    : m_atdata(atdata)
    , m_target_distance(target_distance)
    , m_target_edge_length(target_edge_length)
    , m_envelope_size(envelope_size)
    , m_barrier_weight(barrier_weight)
    , m_barrier_triangle_area(barrier_triangle_area)
    , m_distance_weight(distance_weight)
    , m_amips_weight(amips_weight)
    , m_area_weighted_amips(area_weighted_amips)
    , m_uv_accessor(m_atdata.uv_mesh().create_accessor(m_atdata.m_uv_handle.as<double>()))
    , m_uvmesh_xyz_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_uvmesh_xyz_handle.as<double>()))
    , m_distance_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_distance_error_handle.as<double>()))
    , m_amips_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_amips_error_handle.as<double>()))
    , m_3d_edge_length_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_3d_edge_length_handle.as<double>()))
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
    if (m_atdata.funcs()[0]) {
        std::cout << "----- using analytical quadrature" << std::endl;
        m_evaluator_ptr =
            std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
                m_atdata.funcs(),
                image::SAMPLING_METHOD::Analytical);
        m_integral_ptr = std::make_shared<
            wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature>(
            *m_evaluator_ptr);
    } else {
        assert(m_atdata.images()[0]);
        std::cout << "++++ using images sampling quadrature" << std::endl;
        m_evaluator_ptr =
            std::make_shared<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>(
                m_atdata.images(),
                image::SAMPLING_METHOD::Bilinear,
                image::IMAGE_WRAPPING_MODE::CLAMP_TO_EDGE);
        m_integral_ptr =
            std::make_shared<wmtk::components::function::utils::TextureIntegral>(*m_evaluator_ptr);
    }


    set_uvmesh_xyz_update_rule_initialize();

    set_3d_edge_length_update_rule();
    initialize_3d_edge_length();

    set_distance_error_update_rule();
    initialize_distance_error();

    set_3d_amips_error_update_rule();
    initialize_3d_amips_error();

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

    m_high_distance_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>({-m_distance_error_accessor.scalar_attribute(s.tuple())});
        }
        auto other_face = m_atdata.uv_mesh_ptr()->switch_tuple(s.tuple(), PrimitiveType::Triangle);
        return std::vector<double>(
            {-(m_distance_error_accessor.scalar_attribute(s.tuple()) +
               m_distance_error_accessor.scalar_attribute(other_face))});
    };
    m_high_distance_faces_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Triangle);
        return std::vector<double>({-m_distance_error_accessor.scalar_attribute(s.tuple())});
    };

    m_triangle_distance_edge_length = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>(
            {-m_distance_error_accessor.scalar_attribute(s.tuple()),
             -m_curved_edge_length_accessor.scalar_attribute(s.tuple())});
    };

    m_high_amips_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>({-m_amips_error_accessor.scalar_attribute(s.tuple())});
        }
        return std::vector<double>(
            {-(m_amips_error_accessor.scalar_attribute(s.tuple()) +
               m_amips_error_accessor.scalar_attribute(
                   m_atdata.uv_mesh_ptr()->switch_tuple(s.tuple(), PrimitiveType::Triangle)))});
    };
    m_long_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-m_curved_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    m_edge_length_weighted_distance_priority = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>(
                {-m_distance_error_accessor.scalar_attribute(s.tuple()) *
                 m_3d_edge_length_accessor.scalar_attribute(s.tuple())});
        }
        auto other_face = m_atdata.uv_mesh_ptr()->switch_tuple(s.tuple(), PrimitiveType::Triangle);
        return std::vector<double>(
            {-(m_distance_error_accessor.scalar_attribute(s.tuple()) +
               m_distance_error_accessor.scalar_attribute(other_face)) /
             2 * m_3d_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    std::cout << "target edge length " << m_target_distance << std::endl;
    set_energies();
}


void ATOperations::AT_smooth_interior(
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    wmtk::attribute::MeshAttributeHandle uv_handle = m_atdata.uv_handle();

    std::shared_ptr<wmtk::function::LocalNeighborsSumFunction> energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(
            *uv_mesh_ptr,
            uv_handle,
            *function_ptr);

    m_ops.emplace_back(std::make_shared<wmtk::operations::OptimizationSmoothing>(energy));
    m_ops.back()->add_invariant(
        std::make_shared<SimplexInversionInvariant>(*uv_mesh_ptr, uv_handle.as<double>()));
    m_ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(*uv_mesh_ptr));
    m_ops.back()->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
        m_atdata.m_uvmesh_xyz_handle,
        m_envelope_size,
        m_atdata.m_uvmesh_xyz_handle));

    m_ops.back()->add_transfer_strategy(m_uvmesh_xyz_update);
    // {
    m_ops.back()->add_transfer_strategy(m_distance_error_update);
    m_ops.back()->add_transfer_strategy(m_amips_error_update);
    m_ops.back()->add_transfer_strategy(m_3d_edge_length_update);

    m_ops.back()->use_random_priority() = true;
}
void ATOperations::AT_3d_edge_split(std::function<std::vector<double>(const Simplex&)>& priority)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    std::shared_ptr<Mesh> position_mesh_ptr = m_atdata.position_mesh_ptr();

    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*uv_mesh_ptr);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_3d_edge_length_handle.as<double>(),
        m_target_distance));
    split->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
        m_atdata.m_uvmesh_xyz_handle,
        m_envelope_size,
        m_atdata.m_uvmesh_xyz_handle));

    split->set_priority(priority);

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    split->set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_amips_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_curved_edge_length_handle);

    split->add_transfer_strategy(m_uvmesh_xyz_update);

    split->add_transfer_strategy(m_3d_edge_length_update);
    split->add_transfer_strategy(m_distance_error_update);
    split->add_transfer_strategy(m_amips_error_update);
    split->add_transfer_strategy(m_curved_edge_length_update);
    m_ops.emplace_back(split);
}

void ATOperations::AT_edge_split(std::function<std::vector<double>(const Simplex&)>& priority)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    std::shared_ptr<Mesh> position_mesh_ptr = m_atdata.position_mesh_ptr();

    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*uv_mesh_ptr);

    split->set_priority(priority);
    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    split->set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_amips_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_curved_edge_length_handle);

    split->add_transfer_strategy(m_uvmesh_xyz_update);

    split->add_transfer_strategy(m_3d_edge_length_update);
    split->add_transfer_strategy(m_distance_error_update);
    split->add_transfer_strategy(m_amips_error_update);
    split->add_transfer_strategy(m_curved_edge_length_update);
    m_ops.emplace_back(split);
}


void ATOperations::AT_swap_interior(
    std::function<std::vector<double>(const Simplex&)>& priority,
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    auto swap = std::make_shared<TriEdgeSwap>(*uv_mesh_ptr);
    swap->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*uv_mesh_ptr));
    swap->add_invariant(std::make_shared<InteriorEdgeInvariant>(*uv_mesh_ptr));
    swap->add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));
    swap->add_invariant(std::make_shared<wmtk::invariants::EnvelopeInvariant>(
        m_atdata.m_uvmesh_xyz_handle,
        m_envelope_size,
        m_atdata.m_uvmesh_xyz_handle));
    // swap->add_invariant(
    //     std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    swap->add_invariant(std::make_shared<ValenceImprovementInvariant>(*uv_mesh_ptr));
    // swap->set_priority(priority);

    swap->split().set_new_attribute_strategy(m_atdata.uv_handle());
    swap->collapse().set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    swap->split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    swap->collapse().set_new_attribute_strategy(
        m_atdata.m_uvmesh_xyz_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);
    {
        // the update strategy that doesn't matter
        swap->split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_distance_error_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            wmtk::operations::CollapseBasicStrategy::Mean);
        swap->split().set_new_attribute_strategy(
            m_atdata.m_3d_edge_length_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_3d_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(
            m_atdata.m_curved_edge_length_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_curved_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }
    swap->add_transfer_strategy(m_uvmesh_xyz_update);
    swap->add_transfer_strategy(m_amips_error_update);
    swap->add_transfer_strategy(m_distance_error_update);
    swap->add_transfer_strategy(m_3d_edge_length_update);
    swap->add_transfer_strategy(m_curved_edge_length_update);
    m_ops.push_back(swap);
}


void ATOperations::AT_split_boundary()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    int64_t num_edge_meshes = m_atdata.num_edge_meshes();

    // 1) EdgeSplit on boundary
    for (int64_t i = 0; i < num_edge_meshes; ++i) {
        std::shared_ptr<Mesh> edge_meshi_ptr = m_atdata.edge_mesh_i_ptr(i);
        Mesh* sibling_mesh_ptr = m_atdata.sibling_edge_mesh_ptr(edge_meshi_ptr.get());
        // AT_split_single_edge_mesh(edge_meshi_ptr.get());
        if (sibling_mesh_ptr == nullptr) {
            continue;
        }
        // AT_split_single_edge_mesh(sibling_mesh_ptr);
    }
}

void ATOperations::AT_rg_refine(std::function<std::vector<double>(const Simplex&)>& priority)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    wmtk::attribute::MeshAttributeHandle edge_length_handle = m_atdata.m_3d_edge_length_handle;

    auto rg_refine =
        std::make_shared<wmtk::operations::composite::RGRefine>(*uv_mesh_ptr, edge_length_handle);
    rg_refine->split().add_invariant(std::make_shared<TodoLargerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_distance_error_handle.as<double>(),
        m_target_distance));

    rg_refine->set_priority(priority);
    rg_refine->swap().add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));

    rg_refine->split().set_new_attribute_strategy(m_atdata.uv_handle());
    rg_refine->second_split().set_new_attribute_strategy(m_atdata.uv_handle());
    rg_refine->swap().split().set_new_attribute_strategy(m_atdata.uv_handle());
    rg_refine->swap().collapse().set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    rg_refine->split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    rg_refine->second_split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    rg_refine->swap().split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    rg_refine->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_uvmesh_xyz_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    rg_refine->split().set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    rg_refine->second_split().set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    rg_refine->swap().split().set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    rg_refine->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_3d_edge_length_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    // the update strategy that doesn't matter
    rg_refine->split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    rg_refine->second_split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    rg_refine->swap().split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    rg_refine->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_distance_error_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    rg_refine->split().set_new_attribute_strategy(
        m_atdata.m_amips_error_handle,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);
    rg_refine->second_split().set_new_attribute_strategy(
        m_atdata.m_amips_error_handle,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);
    rg_refine->swap().split().set_new_attribute_strategy(
        m_atdata.m_amips_error_handle,
        SplitBasicStrategy::None,
        SplitRibBasicStrategy::Mean);
    rg_refine->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_amips_error_handle,
        wmtk::operations::CollapseBasicStrategy::Mean);

    rg_refine->split().set_new_attribute_strategy(m_atdata.m_curved_edge_length_handle);
    rg_refine->second_split().set_new_attribute_strategy(m_atdata.m_curved_edge_length_handle);
    rg_refine->swap().split().set_new_attribute_strategy(m_atdata.m_curved_edge_length_handle);
    rg_refine->swap().collapse().set_new_attribute_strategy(
        m_atdata.m_curved_edge_length_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    rg_refine->split().add_transfer_strategy(m_uvmesh_xyz_update);
    rg_refine->split().add_transfer_strategy(m_amips_error_update);
    rg_refine->split().add_transfer_strategy(m_distance_error_update);
    rg_refine->split().add_transfer_strategy(m_3d_edge_length_update);
    rg_refine->split().add_transfer_strategy(m_curved_edge_length_update);

    rg_refine->second_split().add_transfer_strategy(m_uvmesh_xyz_update);
    rg_refine->second_split().add_transfer_strategy(m_amips_error_update);
    rg_refine->second_split().add_transfer_strategy(m_distance_error_update);
    rg_refine->second_split().add_transfer_strategy(m_3d_edge_length_update);
    rg_refine->second_split().add_transfer_strategy(m_curved_edge_length_update);


    rg_refine->swap().add_transfer_strategy(m_uvmesh_xyz_update);
    rg_refine->swap().add_transfer_strategy(m_amips_error_update);
    rg_refine->swap().add_transfer_strategy(m_distance_error_update);
    rg_refine->swap().add_transfer_strategy(m_3d_edge_length_update);
    rg_refine->swap().add_transfer_strategy(m_curved_edge_length_update);
    m_ops.push_back(rg_refine);
}

int64_t ATOperations::AT_rgb_split()
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();

    auto rgb_split = std::make_shared<wmtk::operations::composite::RGBSplit>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle,
        m_atdata.m_edge_rgb_state_handle);

    rgb_split->split().set_new_attribute_strategy(m_atdata.m_uv_handle);
    rgb_split->split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    rgb_split->split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    rgb_split->split().set_new_attribute_strategy(m_atdata.m_amips_error_handle);
    rgb_split->split().set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    rgb_split->split().set_new_attribute_strategy(m_atdata.m_curved_edge_length_handle);

    rgb_split->split().add_transfer_strategy(m_uvmesh_xyz_update);
    rgb_split->split().add_transfer_strategy(m_3d_edge_length_update);
    rgb_split->split().add_transfer_strategy(m_distance_error_update);
    rgb_split->split().add_transfer_strategy(m_amips_error_update);
    rgb_split->split().add_transfer_strategy(m_curved_edge_length_update);


    rgb_split->split().set_new_attribute_strategy(
        m_atdata.m_face_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    rgb_split->split().set_new_attribute_strategy(
        m_atdata.m_edge_rgb_state_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    rgb_split->split().set_new_attribute_strategy(
        m_atdata.m_edge_todo_handle,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);

    rgb_split->add_invariant(std::make_shared<wmtk::RGBSplitInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_face_rgb_state_handle.as<int64_t>(),
        m_atdata.m_edge_rgb_state_handle.as<int64_t>()));

    rgb_split->add_invariant(std::make_shared<wmtk::TodoInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_edge_todo_handle.as<int64_t>(),
        1));
    m_ops.emplace_back(rgb_split);
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
            m_atdata.m_amips_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        rgb_swap->swap().collapse().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            wmtk::operations::CollapseBasicStrategy::Mean);
        rgb_swap->swap().split().set_new_attribute_strategy(
            m_atdata.m_3d_edge_length_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        rgb_swap->swap().collapse().set_new_attribute_strategy(
            m_atdata.m_3d_edge_length_handle,
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
    rgb_swap->add_transfer_strategy(m_amips_error_update);
    rgb_swap->add_transfer_strategy(m_distance_error_update);
    rgb_swap->add_transfer_strategy(m_3d_edge_length_update);
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