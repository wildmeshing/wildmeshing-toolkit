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

#include <wmtk/invariants/BoundarySimplexInvariant.hpp>
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
    double barrier_weight,
    double barrier_triangle_area,
    double distance_weight,
    double amips_weight,
    bool area_weighted_amips)
    : m_atdata(atdata)
    , m_target_distance(target_distance)
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
             -m_3d_edge_length_accessor.scalar_attribute(s.tuple())});
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
        return std::vector<double>({-m_3d_edge_length_accessor.scalar_attribute(s.tuple())});
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

    m_ops.back()->add_transfer_strategy(m_uvmesh_xyz_update);
    // {
    m_ops.back()->add_transfer_strategy(m_distance_error_update);
    m_ops.back()->add_transfer_strategy(m_amips_error_update);
    m_ops.back()->add_transfer_strategy(m_3d_edge_length_update);

    m_ops.back()->use_random_priority() = true;
}


void ATOperations::AT_edge_split(
    std::function<std::vector<double>(const Simplex&)>& priority,
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    std::shared_ptr<Mesh> position_mesh_ptr = m_atdata.position_mesh_ptr();

    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*uv_mesh_ptr);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_distance_error_handle.as<double>(),
        m_target_distance));
    // split->add_invariant(
    //     std::make_shared<BoundarySimplexInvariant>(*uv_mesh_ptr, PrimitiveType::Edge));
    // split->add_invariant(
    //     std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    // split->add_invariant(
    //     std::make_shared<StateChanges>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    split->set_priority(priority);
    // split->use_random_priority() = true;

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    split->set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_amips_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);

    split->add_transfer_strategy(m_uvmesh_xyz_update);

    split->add_transfer_strategy(m_3d_edge_length_update);
    split->add_transfer_strategy(m_distance_error_update);
    split->add_transfer_strategy(m_amips_error_update);
    m_ops.emplace_back(split);
}

void ATOperations::AT_boundary_edge_split(
    std::function<std::vector<double>(const Simplex&)>& priority,
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    std::shared_ptr<Mesh> position_mesh_ptr = m_atdata.position_mesh_ptr();

    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*uv_mesh_ptr);
    // split->add_invariant(std::make_shared<TodoAvgEnergyLargerInvariant>(
    //     *uv_mesh_ptr,
    //     m_atdata.m_sum_error_handle.as<double>(),
    //     m_target_distance));
    split->add_invariant(
        std::make_shared<BoundarySimplexInvariant>(*uv_mesh_ptr, PrimitiveType::Edge));
    // split->add_invariant(
    //     std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    split->add_invariant(
        std::make_shared<StateChanges>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    split->set_priority(priority);

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    split->set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_amips_error_handle);

    split->add_transfer_strategy(m_uvmesh_xyz_update);

    // split->add_transfer_strategy(m_edge_length_update);
    split->add_transfer_strategy(m_distance_error_update);
    split->add_transfer_strategy(m_amips_error_update);
    m_ops.emplace_back(split);
}

void ATOperations::AT_face_split(
    std::function<std::vector<double>(const Simplex&)>& priority,
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    std::shared_ptr<Mesh> position_mesh_ptr = m_atdata.position_mesh_ptr();
    wmtk::attribute::MeshAttributeHandle uv_handle = m_atdata.uv_handle();

    auto face_split = std::make_shared<TriFaceSplit>(*uv_mesh_ptr);
    face_split->add_invariant(
        std::make_shared<StateChanges>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    face_split->add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));
    face_split->set_priority(priority);

    face_split->split().set_new_attribute_strategy(m_atdata.uv_handle());
    face_split->collapse().set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    face_split->split().set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    face_split->collapse().set_new_attribute_strategy(
        m_atdata.m_uvmesh_xyz_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);
    {
        // the update strategy that doesn't matter
        face_split->split().set_new_attribute_strategy(m_atdata.m_distance_error_handle);
        face_split->collapse().set_new_attribute_strategy(
            m_atdata.m_distance_error_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
        face_split->split().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        face_split->collapse().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            wmtk::operations::CollapseBasicStrategy::Mean);
    }
    face_split->add_transfer_strategy(m_uvmesh_xyz_update);
    face_split->add_transfer_strategy(m_amips_error_update);
    face_split->add_transfer_strategy(m_distance_error_update);
    m_ops.push_back(face_split);
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
    }
    swap->add_transfer_strategy(m_uvmesh_xyz_update);
    swap->add_transfer_strategy(m_amips_error_update);
    swap->add_transfer_strategy(m_distance_error_update);
    swap->add_transfer_strategy(m_3d_edge_length_update);

    m_ops.push_back(swap);
}

void ATOperations::AT_split_single_edge_mesh(Mesh* edge_meshi_ptr)
{
    auto m_t_handle = edge_meshi_ptr->get_attribute_handle<double>("t", PrimitiveType::Vertex);
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*edge_meshi_ptr);
    split->set_priority(m_long_edges_first);

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_t_handle);

    m_ops.emplace_back(split);
}

void ATOperations::AT_split_boundary()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto uv_handle = m_atdata.uv_handle();
    int64_t num_edge_meshes = m_atdata.num_edge_meshes();

    // 1) EdgeSplit on boundary
    for (int64_t i = 0; i < num_edge_meshes; ++i) {
        std::shared_ptr<Mesh> edge_meshi_ptr = m_atdata.edge_mesh_i_ptr(i);
        Mesh* sibling_mesh_ptr = m_atdata.sibling_edge_mesh_ptr(edge_meshi_ptr.get());
        AT_split_single_edge_mesh(edge_meshi_ptr.get());
        if (sibling_mesh_ptr == nullptr) {
            continue;
        }
        AT_split_single_edge_mesh(sibling_mesh_ptr);
    }
}

void ATOperations::AT_collapse_interior(
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    throw std::runtime_error("AT collapse not implemented");
}

bool ATOperations::single_split_execution(
    wmtk::operations::Operation& edge_split_op,
    std::function<std::vector<double>(const wmtk::simplex::Simplex&)>& edge_priority)
{
    auto& uv_mesh = m_atdata.uv_mesh();
    const auto tups = edge_split_op.mesh().get_all(PrimitiveType::Triangle);
    std::vector<wmtk::simplex::Simplex> edge_simplices;
    for (const auto& tup : tups) {
        // get all the edges simplices of each triangle tuple
        // current edge
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, tup));
        auto next_edge = uv_mesh.switch_tuple(tup, PrimitiveType::Edge);
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, next_edge));
        auto next_next_edge = uv_mesh.switch_tuple(
            uv_mesh.switch_tuple(tup, PrimitiveType::Vertex),
            PrimitiveType::Edge);
        edge_simplices.emplace_back(wmtk::simplex::Simplex(PrimitiveType::Edge, next_next_edge));
    }
    std::stable_sort(
        edge_simplices.begin(),
        edge_simplices.end(),
        [&edge_priority](const auto& s_a, const auto& s_b) {
            return edge_priority(s_a) < edge_priority(s_b);
        });
    assert(edge_simplices.size() > 0);
    auto mods = edge_split_op(edge_simplices[0]);
    return !mods.empty();
}

} // namespace wmtk::components::operations::internal