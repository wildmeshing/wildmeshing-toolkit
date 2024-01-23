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
    double target_edge_length,
    double barrier_weight,
    double barrier_triangle_area,
    double distance_weight,
    double amips_weight,
    bool area_weighted_amips)
    : m_atdata(atdata)
    , m_target_edge_length(target_edge_length)
    , m_barrier_weight(barrier_weight)
    , m_barrier_triangle_area(barrier_triangle_area)
    , m_distance_weight(distance_weight)
    , m_amips_weight(amips_weight)
    , m_area_weighted_amips(area_weighted_amips)
    , m_uv_accessor(m_atdata.uv_mesh().create_accessor(m_atdata.m_uv_handle.as<double>()))

    //.... TODO can be deleted once multimesh transfer strategy is implemented
    , m_uvmesh_xyz_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_uvmesh_xyz_handle.as<double>()))
    , m_pmesh_xyz_accessor(
          m_atdata.position_mesh().create_accessor(m_atdata.m_pmesh_xyz_handle.as<double>()))
    //.....
    , m_distance_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_distance_error_handle.as<double>()))
    , m_sum_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_sum_error_handle.as<double>()))
    , m_barrier_energy_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_barrier_energy_handle.as<double>()))
    , m_amips_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_amips_error_handle.as<double>()))
// , m_edge_priority_accessor(
//       m_atdata.uv_mesh().create_accessor(m_atdata.m_edge_priority_handle.as<double>()))
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
                image::SAMPLING_METHOD::Bicubic,
                image::IMAGE_WRAPPING_MODE::CLAMP_TO_EDGE);
        m_integral_ptr =
            std::make_shared<wmtk::components::function::utils::TextureIntegral>(*m_evaluator_ptr);
    }


    set_uvmesh_xyz_update_rule();
    // initialize_vertex_xyz();
    // set_edge_length_update_rule();
    // initialize_edge_length();
    set_distance_error_update_rule();
    initialize_distance_error();

    set_amips_error_update_rule();
    initialize_amips_error();

    set_sum_error_update_rule();
    initialize_sum_error();


    // set_barrier_energy_update_rule();
    // initialize_barrier_energy();

    // Lambdas for priority
    m_valence_improvement = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        const auto [val_before, val_after] =
            wmtk::invariants::ValenceImprovementInvariant::valence_change(
                *m_atdata.uv_mesh_ptr(),
                s);
        return std::vector<long>({val_before - val_after});
    };

    m_high_error_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>({-m_sum_error_accessor.scalar_attribute(s.tuple())});
        }
        return std::vector<double>(
            {-(m_sum_error_accessor.scalar_attribute(s.tuple()) +
               m_sum_error_accessor.scalar_attribute(
                   m_atdata.uv_mesh_ptr()->switch_face(s.tuple())))});
    };
    m_high_distance_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>({-m_distance_error_accessor.scalar_attribute(s.tuple())});
        }
        auto other_face = m_atdata.uv_mesh_ptr()->switch_face(s.tuple());
        return std::vector<double>(
            {-(m_distance_error_accessor.scalar_attribute(s.tuple()) +
               m_distance_error_accessor.scalar_attribute(other_face))});
    };
    m_high_distance_faces_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Face);
        return std::vector<double>({-m_distance_error_accessor.scalar_attribute(s.tuple())});
    };

    m_high_amips_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>({-m_amips_error_accessor.scalar_attribute(s.tuple())});
        }
        return std::vector<double>(
            {-(m_amips_error_accessor.scalar_attribute(s.tuple()) +
               m_amips_error_accessor.scalar_attribute(
                   m_atdata.uv_mesh_ptr()->switch_face(s.tuple()))) /
             2});
    };
    std::cout << "target edge length " << m_target_edge_length << std::endl;
    set_energies();
}

void ATOperations::set_energies()
{
    m_distance_energy = std::make_shared<wmtk::function::DistanceEnergy>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_integral_ptr,
        m_distance_weight);
    m_distance_nondiff_energy = std::make_shared<wmtk::function::DistanceEnergyNonDiff>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_integral_ptr,
        m_distance_weight);


    m_amips_energy = std::make_shared<wmtk::function::TriangleAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle());
    m_3d_amips_energy = std::make_shared<wmtk::function::PositionMapAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_evaluator_ptr,
        m_amips_weight,
        m_area_weighted_amips);


    ////////// TODO This needs to change/////////
    //////// the integral function doesn't need to be two separate class ///////
    //////// global search for PerTriangleAnalyticalIntegral and DistanceEnergy
    m_sum_energy = std::make_shared<wmtk::function::SumEnergy>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_evaluator_ptr,
        m_integral_ptr,
        m_barrier_weight,
        m_barrier_triangle_area,
        m_distance_weight,
        m_amips_weight,
        m_area_weighted_amips);
}

void ATOperations::set_uvmesh_xyz_update_rule()
{ // 3d vert position update
    auto compute_vertex_position = [&](const Eigen::Vector2d& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        assert(P.rows() == 2);
        Eigen::Vector2d uv = P.col(0);
        return m_evaluator_ptr->uv_to_position(uv);
    };
    m_uvmesh_xyz_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_uvmesh_xyz_handle,
            m_atdata.m_uv_handle,
            compute_vertex_position);
    // initialize this position field
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = m_uv_accessor.vector_attribute(v);
        m_uvmesh_xyz_accessor.vector_attribute(v) = compute_vertex_position(p);
    }
}

//.... TODO can be deleted once multimesh transfer strategy is implemented
void ATOperations::initialize_uvmesh_vertex_xyz()
{
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        auto position_vert = m_atdata.uv_mesh().map_to_parent_tuple(Simplex::vertex(v));
        auto p = m_pmesh_xyz_accessor.vector_attribute(position_vert);
        m_uvmesh_xyz_accessor.vector_attribute(v) = p;
    }
}
//....
///// TODO this should not be used now. Since posiiton is loaded from 3d mesh
void ATOperations::initialize_pmesh_vertex_xyz()
{
    // initialize this position field
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        Eigen::Vector2d uv = m_uv_accessor.vector_attribute(v);
        const auto p = m_evaluator_ptr->uv_to_position(uv);
        auto position_vert = m_atdata.uv_mesh().map_to_parent_tuple(Simplex::vertex(v));
        m_pmesh_xyz_accessor.vector_attribute(position_vert) = p;
    }
}
void ATOperations::set_distance_error_update_rule()
{
    auto compute_distance_error = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) < 0) {
            std::swap(uv1, uv2);
        }
        Eigen::VectorXd error(1);


        error(0) = m_distance_weight * m_integral_ptr->get_error_one_triangle_exact(uv0, uv1, uv2);

        return error;
    };
    m_distance_error_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_distance_error_handle,
            m_atdata.uv_handle(),
            compute_distance_error);
}
void ATOperations::initialize_distance_error()
{
    // initialize distance error values
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Face)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_vertex(f);
        }
        const Eigen::Vector2d v0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d v1 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_vertex(f));
        const Eigen::Vector2d v2 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_vertex(m_atdata.uv_mesh_ptr()->switch_edge(f)));

        double res = m_distance_weight * m_integral_ptr->get_error_one_triangle_exact(v0, v1, v2);

        m_distance_error_accessor.scalar_attribute(f) = res;
    }
    if (false) {
        std::ofstream outputFileposition("AT_box_position_coord.json");
        outputFileposition << static_cast<wmtk::components::function::utils::TextureIntegral&>(
                                  *m_integral_ptr)
                                  .m_jsonData_bary_coord.dump(4);
        outputFileposition.close();
        std::cout << "JSON data written to "
                  << "AT_debug_position_coord.json" << std::endl;
        std::ofstream outputFiletexcoord("AT_box_tex_coord.json");
        outputFiletexcoord << static_cast<wmtk::components::function::utils::TextureIntegral&>(
                                  *m_integral_ptr)
                                  .m_jsonData_texture_coord.dump(4);
        outputFiletexcoord.close();
        std::cout << "JSON data written to "
                  << "AT_debug_tex_coord.json" << std::endl;
    }
}

void ATOperations::set_amips_error_update_rule()
{
    auto compute_amips_error = [&](const Eigen::Matrix<double, 2, 3>& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) <= 0) {
            std::swap(uv1, uv2);
        }
        if (orient2d(uv0.data(), uv1.data(), uv2.data()) <= 0) {
            Eigen::VectorXd infinity(1);
            infinity(0) = std::numeric_limits<double>::infinity();
            return infinity;
        }
        auto p0 = m_evaluator_ptr->uv_to_position(uv0);
        auto p1 = m_evaluator_ptr->uv_to_position(uv1);
        auto p2 = m_evaluator_ptr->uv_to_position(uv2);
        Eigen::VectorXd error(1);
        auto res = m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
        if (m_area_weighted_amips) {
            res *= wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        }
        error(0) = res;
        return error;
    };
    m_amips_error_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_amips_error_handle,
            m_atdata.uv_handle(),
            compute_amips_error);
}
void ATOperations::initialize_amips_error()
{
    // initialize face error values
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Face)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_vertex(f);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_vertex(f));
        const Eigen::Vector2d uv2 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_vertex(m_atdata.uv_mesh_ptr()->switch_edge(f)));
        auto p0 = m_evaluator_ptr->uv_to_position(uv0);
        auto p1 = m_evaluator_ptr->uv_to_position(uv1);
        auto p2 = m_evaluator_ptr->uv_to_position(uv2);
        auto res = wmtk::function::utils::amips(p0, p1, p2);
        res = m_amips_weight * res;
        if (m_area_weighted_amips) {
            res *= wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        }
        m_amips_error_accessor.scalar_attribute(f) = res;
    }
}
void ATOperations::set_sum_error_update_rule()
{ // face error update
    auto compute_sum_error = [&](const Eigen::Matrix<double, 2, 3>& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);

        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) <= 0) {
            std::swap(uv1, uv2);
        }
        if (orient2d(uv0.data(), uv1.data(), uv2.data()) <= 0) {
            Eigen::VectorXd infinity(1);
            infinity(0) = std::numeric_limits<double>::infinity();
            return infinity;
        }
        auto p0 = m_evaluator_ptr->uv_to_position(uv0);
        auto p1 = m_evaluator_ptr->uv_to_position(uv1);
        auto p2 = m_evaluator_ptr->uv_to_position(uv2);
        Eigen::VectorXd error(1);
        double distance_error =
            m_distance_weight * m_integral_ptr->get_error_one_triangle_exact(uv0, uv1, uv2);
        double barrier_error =
            m_barrier_weight *
            wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area);
        double amips_error = m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
        if (m_area_weighted_amips) {
            amips_error *= wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        }
        error(0) = amips_error + distance_error + barrier_error;
        return error;
    };
    m_sum_error_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_sum_error_handle,
            m_atdata.uv_handle(),
            compute_sum_error);
}

void ATOperations::initialize_sum_error()
{
    // initialize face error values
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Face)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_vertex(f);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_vertex(f));
        const Eigen::Vector2d uv2 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_vertex(m_atdata.uv_mesh_ptr()->switch_edge(f)));
        auto p0 = m_evaluator_ptr->uv_to_position(uv0);
        auto p1 = m_evaluator_ptr->uv_to_position(uv1);
        auto p2 = m_evaluator_ptr->uv_to_position(uv2);
        double distance_error =
            m_distance_weight * m_integral_ptr->get_error_one_triangle_exact(uv0, uv1, uv2);
        double barrier_error =
            m_barrier_weight *
            wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area);
        double amips_error = m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
        if (m_area_weighted_amips) {
            amips_error *= wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        }
        m_sum_error_accessor.scalar_attribute(f) = amips_error + distance_error + barrier_error;
    }
}

void ATOperations::set_barrier_energy_update_rule()
{
    auto compute_barrier_energy = [&](const Eigen::Matrix<double, 2, 3>& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) < 0) {
            std::swap(uv1, uv2);
        }

        Eigen::VectorXd error(1);
        error(0) = wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area);
        return error;
    };
    m_barrier_energy_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_barrier_energy_handle,
            m_atdata.uv_handle(),
            compute_barrier_energy);
}

void ATOperations::initialize_barrier_energy()
{ // initialize face error values
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Face)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_vertex(f);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_vertex(f));
        const Eigen::Vector2d uv2 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_vertex(m_atdata.uv_mesh_ptr()->switch_edge(f)));
        auto res = wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area);
        m_barrier_energy_accessor.scalar_attribute(f) = res;
    }
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
    //     m_ops.back()->add_transfer_strategy(m_barrier_energy_update);
    m_ops.back()->add_transfer_strategy(m_amips_error_update);
    //     m_ops.back()->add_transfer_strategy(m_edge_length_update);
    // }
    m_ops.back()->add_transfer_strategy(m_sum_error_update);
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
    // split->add_invariant(std::make_shared<TodoAvgEnergyLargerInvariant>(
    //     *uv_mesh_ptr,
    //     m_atdata.m_sum_error_handle.as<double>(),
    //     m_target_edge_length));
    // split->add_invariant(
    //     std::make_shared<BoundarySimplexInvariant>(*uv_mesh_ptr, PrimitiveType::Edge));
    // split->add_invariant(
    //     std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    split->add_invariant(
        std::make_shared<StateChanges>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    split->set_priority(priority);

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle);
    split->set_new_attribute_strategy(m_atdata.m_distance_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_barrier_energy_handle);
    split->set_new_attribute_strategy(m_atdata.m_amips_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_sum_error_handle);

    split->add_transfer_strategy(m_uvmesh_xyz_update);

    // split->add_transfer_strategy(m_edge_length_update);
    split->add_transfer_strategy(m_distance_error_update);
    // split->add_transfer_strategy(m_barrier_energy_update);
    split->add_transfer_strategy(m_amips_error_update);
    split->add_transfer_strategy(m_sum_error_update);
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
        face_split->split().set_new_attribute_strategy(m_atdata.m_barrier_energy_handle);
        face_split->collapse().set_new_attribute_strategy(
            m_atdata.m_barrier_energy_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
        face_split->split().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        face_split->collapse().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            wmtk::operations::CollapseBasicStrategy::Mean);
        face_split->split().set_new_attribute_strategy(
            m_atdata.m_sum_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        face_split->collapse().set_new_attribute_strategy(
            m_atdata.m_sum_error_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }
    face_split->add_transfer_strategy(m_uvmesh_xyz_update);
    face_split->add_transfer_strategy(m_amips_error_update);
    face_split->add_transfer_strategy(m_sum_error_update);
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
    swap->add_invariant(
        std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
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
        swap->split().set_new_attribute_strategy(m_atdata.m_barrier_energy_handle);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_barrier_energy_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_amips_error_handle,
            wmtk::operations::CollapseBasicStrategy::Mean);
        swap->split().set_new_attribute_strategy(
            m_atdata.m_sum_error_handle,
            SplitBasicStrategy::None,
            SplitRibBasicStrategy::Mean);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_sum_error_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }
    swap->add_transfer_strategy(m_uvmesh_xyz_update);
    swap->add_transfer_strategy(m_amips_error_update);
    swap->add_transfer_strategy(m_sum_error_update);
    swap->add_transfer_strategy(m_distance_error_update);

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
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(*uv_mesh_ptr);
    collapse->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*uv_mesh_ptr));
    // collapse->add_invariant(std::make_shared<InteriorEdgeInvariant>(*uv_mesh_ptr));
    collapse->add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));
    collapse->add_invariant(
        std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));

    collapse->set_priority(m_short_edges_first);

    auto clps_strat = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.uv_handle());
    clps_strat->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(m_atdata.uv_handle(), clps_strat);

    auto clps_strat2 = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.m_uvmesh_xyz_handle);
    clps_strat2->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat2->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);
    collapse->set_new_attribute_strategy(m_atdata.m_uvmesh_xyz_handle, clps_strat2);

    auto clps_strat3 = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.m_pmesh_xyz_handle);
    clps_strat3->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat3->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);
    collapse->set_new_attribute_strategy(m_atdata.m_pmesh_xyz_handle, clps_strat3);


    collapse->add_transfer_strategy(m_uvmesh_xyz_update);
    m_ops.emplace_back(collapse);
}

} // namespace wmtk::components::operations::internal