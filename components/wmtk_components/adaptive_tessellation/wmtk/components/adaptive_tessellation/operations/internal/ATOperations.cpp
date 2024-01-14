#include "ATOperations.hpp"
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>

#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>

#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/Scheduler.hpp>

#include "ATOptions.hpp"

#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/invariants/ValenceImprovementInvariant.hpp>

#include "predicates.h"
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
    double quadrature_weight,
    double amips_weight)
    : m_atdata(atdata)
    , m_barrier_weight(barrier_weight)
    , m_barrier_triangle_area(barrier_triangle_area)
    , m_quadrature_weight(quadrature_weight)
    , m_amips_weight(amips_weight)
    , m_uv_accessor(m_atdata.uv_mesh().create_accessor(m_atdata.m_uv_handle.as<double>()))
    , m_edge_length_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_3d_edge_length_handle.as<double>()))
    , m_xyz_accessor(m_atdata.uv_mesh().create_accessor(m_atdata.m_xyz_handle.as<double>()))
    , m_sum_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_sum_error_handle.as<double>()))
    , m_quadrature_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_quadrature_error_handle.as<double>()))
    , m_amips_error_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_amips_error_handle.as<double>()))
    , m_barrier_energy_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_barrier_energy_handle.as<double>()))


{
    m_ops.clear();

    set_xyz_update_rule();
    initialize_vertex_xyz();
    // set_edge_length_update_rule();
    // initialize_edge_length();
    set_amips_error_update_rule();
    initialize_amips_error();

    set_sum_error_update_rule();
    initialize_sum_error();

    // set_quadrature_error_update_rule();
    // initialize_quadrature_error();
    // set_barrier_energy_update_rule();
    // initialize_barrier_energy();

    // Lambdas for priority
    m_valence_improvement = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        const auto [val_before, val_after] =
            wmtk::invariants::ValenceImprovementInvariant::valence_change(
                *m_atdata.uv_mesh_ptr(),
                s);
        return std::vector<double>({val_before - val_after});
    };


    m_long_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-m_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    m_short_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({m_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);

    m_high_error_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        if (m_atdata.uv_mesh_ptr()->is_boundary(s)) {
            return std::vector<double>({-m_sum_error_accessor.scalar_attribute(s.tuple())});
        }
        return std::vector<double>(
            {-(m_sum_error_accessor.scalar_attribute(s.tuple()) +
               m_sum_error_accessor.scalar_attribute(
                   m_atdata.uv_mesh_ptr()->switch_face(s.tuple()))) /
             2});
    };


    //////////////////////////////////
    // computng bbox diagonal
    bool planar = false; // TODO this needs to be read from the options. For now we work
                         // only on uv mesh
    Eigen::VectorXd bmin(planar ? 2 : 3);
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(planar ? 2 : 3);
    bmax.setConstant(std::numeric_limits<double>::min());
    for (const auto& v : vertices) {
        const auto p = m_xyz_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();
    m_target_edge_length = target_edge_length * bbdiag;

    set_energies();
}

void ATOperations::set_energies()
{
    m_quadrature_energy = std::make_shared<wmtk::function::PerTriangleAnalyticalIntegral>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_atdata.funcs());
    m_amips_energy = std::make_shared<wmtk::function::TriangleAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle());
    m_3d_amips_energy = std::make_shared<wmtk::function::PositionMapAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_atdata.m_evaluator);

    m_sum_energy = std::make_shared<wmtk::function::SumEnergy>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_atdata.m_evaluator,
        m_barrier_weight,
        m_barrier_triangle_area,
        m_quadrature_weight,
        m_amips_weight);
}

void ATOperations::set_xyz_update_rule()
{ // 3d vert position update
    auto compute_vertex_position = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        assert(P.rows() == 2);
        Eigen::Vector2d uv = P.col(0);
        return m_atdata.m_evaluator.uv_to_position(uv);
    };
    m_xyz_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_xyz_handle,
            m_atdata.m_uv_handle,
            compute_vertex_position);
    // initialize this position field
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = m_uv_accessor.vector_attribute(v);
        m_xyz_accessor.vector_attribute(v) = compute_vertex_position(p);
    }
}

void ATOperations::initialize_vertex_xyz()
{
    // initialize this position field
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        Eigen::Vector2d uv = m_uv_accessor.vector_attribute(v);
        m_xyz_accessor.vector_attribute(v) = m_atdata.m_evaluator.uv_to_position(uv);
    }
}

void ATOperations::set_edge_length_update_rule()
{ // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    m_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_3d_edge_length_handle,
            m_atdata.m_xyz_handle,
            compute_edge_length);
}
void ATOperations::initialize_edge_length()
{
    //////////////////////////////////
    // ASSUMES THE XYZ FIELD IS ALREADY INITIALIZED!!!!!!
    //////////////////////////////////
    // initialize edge lengths
    const auto edges = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = m_xyz_accessor.vector_attribute(e);
        const auto p1 = m_xyz_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_vertex(e));

        m_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }
}

void ATOperations::set_quadrature_error_update_rule()
{
    auto compute_quadrature_error = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature
            analytical_quadrature(m_atdata.m_evaluator);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) < 0) {
            std::swap(uv1, uv2);
        }
        Eigen::VectorXd error(1);
        error(0) = analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2);
        return error;
    };
    m_quadrature_error_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_quadrature_error_handle,
            m_atdata.uv_handle(),
            compute_quadrature_error);
}
void ATOperations::initialize_quadrature_error()
{
    // initialize quadrature error values
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Face)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_vertex(f);
        }
        const Eigen::Vector2d v0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d v1 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_vertex(f));
        const Eigen::Vector2d v2 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_vertex(m_atdata.uv_mesh_ptr()->switch_edge(f)));
        wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature
            analytical_quadrature(m_atdata.m_evaluator);

        auto res = analytical_quadrature.get_error_one_triangle_exact(v0, v1, v2);
        m_quadrature_error_accessor.scalar_attribute(f) = res;
    }
}

void ATOperations::set_amips_error_update_rule()
{
    auto compute_amips_error = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) < 0) {
            std::swap(uv1, uv2);
        }
        auto p0 = m_atdata.m_evaluator.uv_to_position(uv0);
        auto p1 = m_atdata.m_evaluator.uv_to_position(uv1);
        auto p2 = m_atdata.m_evaluator.uv_to_position(uv2);
        Eigen::VectorXd error(1);
        error(0) = m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
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
        auto p0 = m_atdata.m_evaluator.uv_to_position(uv0);
        auto p1 = m_atdata.m_evaluator.uv_to_position(uv1);
        auto p2 = m_atdata.m_evaluator.uv_to_position(uv2);
        auto res = wmtk::function::utils::amips(p0, p1, p2);
        m_amips_error_accessor.scalar_attribute(f) = m_amips_weight * res;
    }
}
void ATOperations::set_sum_error_update_rule()
{ // face error update

    auto compute_sum_error = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature
            analytical_quadrature(m_atdata.m_evaluator);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);

        if (orient2d(uv0.data(), uv1.data(), uv2.data()) < 0) {
            std::swap(uv1, uv2);
        }
        auto p0 = m_atdata.m_evaluator.uv_to_position(uv0);
        auto p1 = m_atdata.m_evaluator.uv_to_position(uv1);
        auto p2 = m_atdata.m_evaluator.uv_to_position(uv2);

        Eigen::VectorXd error(1);

        error(0) = m_quadrature_weight *
                       analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2) +
                   m_barrier_weight *
                       wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area) +
                   m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
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
        wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature
            analytical_quadrature(m_atdata.m_evaluator);
        auto p0 = m_atdata.m_evaluator.uv_to_position(uv0);
        auto p1 = m_atdata.m_evaluator.uv_to_position(uv1);
        auto p2 = m_atdata.m_evaluator.uv_to_position(uv2);

        auto res = m_quadrature_weight *
                       analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2) +
                   m_barrier_weight *
                       wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area) +
                   m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
        m_sum_error_accessor.scalar_attribute(f) = res;
    }
}

void ATOperations::set_barrier_energy_update_rule()
{
    auto compute_barrier_energy = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
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

void ATOperations::AT_smooth_interior()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto uv_handle = m_atdata.uv_handle();
    // Energy to optimize
    std::shared_ptr<wmtk::function::PerTriangleTextureIntegralAccuracyFunction> accuracy =
        std::make_shared<wmtk::function::PerTriangleTextureIntegralAccuracyFunction>(
            uv_mesh,
            uv_handle,
            m_atdata.images());

    // std::shared_ptr<wmtk::function::TriangleAMIPS> amips =
    //     std::make_shared<wmtk::function::TriangleAMIPS>(m_atdata.uv_mesh(),
    //     m_atdata.uv_handle());
    std::shared_ptr<wmtk::function::AMIPS> amips =
        std::make_shared<wmtk::function::AMIPS>(m_atdata.uv_mesh(), m_atdata.uv_handle());
    amips->attribute_handle();
    // for (auto& f : uv_mesh.get_all(PrimitiveType::Face)) {
    //     auto val = accuracy->get_value(Simplex::face(f));
    //     std::cout << " has value " << val << std::endl;
    // }

    // MeshAttributeHandle handle = amips->attribute_handle();
    // assert(handle.is_valid());
    std::shared_ptr<wmtk::function::LocalNeighborsSumFunction> energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(
            m_atdata.uv_mesh(),
            m_atdata.uv_handle(),
            *amips);
    for (auto& v : uv_mesh.get_all(PrimitiveType::Vertex)) {
        energy->get_value(Simplex::vertex(v));
        break;
    }
    m_ops.emplace_back(std::make_shared<wmtk::operations::OptimizationSmoothing>(energy));
    m_ops.back()->add_invariant(
        std::make_shared<SimplexInversionInvariant>(uv_mesh, uv_handle.as<double>()));
    m_ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(uv_mesh));
    m_ops.back()->add_transfer_strategy(m_edge_length_update);
    m_ops.back()->use_random_priority() = true;
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

    m_ops.back()->add_transfer_strategy(m_xyz_update);
    m_ops.back()->add_transfer_strategy(m_sum_error_update);
    // {
    //     m_ops.back()->add_transfer_strategy(m_quadrature_error_update);
    //     m_ops.back()->add_transfer_strategy(m_barrier_energy_update);
    m_ops.back()->add_transfer_strategy(m_amips_error_update);
    //     m_ops.back()->add_transfer_strategy(m_edge_length_update);
    // }
    m_ops.back()->use_random_priority() = true;
}


void ATOperations::AT_split_interior(
    std::function<std::vector<double>(const Simplex&)>& priority,
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    wmtk::attribute::MeshAttributeHandle uv_handle = m_atdata.uv_handle();

    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*uv_mesh_ptr);
    split->add_invariant(std::make_shared<TodoAvgEnergyLargerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_sum_error_handle.as<double>(),
        m_target_edge_length));
    // split->add_invariant(std::make_shared<InteriorEdgeInvariant>(uv_mesh));
    split->add_invariant(
        std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    split->set_priority(priority);

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_xyz_handle);
    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_sum_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_quadrature_error_handle);
    split->set_new_attribute_strategy(m_atdata.m_barrier_energy_handle);
    split->set_new_attribute_strategy(m_atdata.m_amips_error_handle);

    split->add_transfer_strategy(m_xyz_update);
    // split->add_transfer_strategy(m_edge_length_update);
    split->add_transfer_strategy(m_sum_error_update);
    // split->add_transfer_strategy(m_quadrature_error_update);
    // split->add_transfer_strategy(m_barrier_energy_update);
    split->add_transfer_strategy(m_amips_error_update);
    m_ops.emplace_back(split);
}

void ATOperations::AT_split_single_edge_mesh(Mesh* edge_meshi_ptr)
{
    auto m_t_handle = edge_meshi_ptr->get_attribute_handle<double>("t", PrimitiveType::Vertex);
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*edge_meshi_ptr);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        m_atdata.uv_mesh(),
        m_atdata.m_3d_edge_length_handle.as<double>(),
        4.0 / 3.0 * m_target_edge_length));
    split->set_priority(m_long_edges_first);

    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_t_handle);

    split->add_transfer_strategy(m_edge_length_update);
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
    collapse->add_invariant(std::make_shared<TodoSmallerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_3d_edge_length_handle.as<double>(),
        4.0 / 5.0 * m_target_edge_length));
    collapse->set_priority(m_short_edges_first);

    auto clps_strat = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.uv_handle());
    clps_strat->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(m_atdata.uv_handle(), clps_strat);

    auto clps_strat2 = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.m_xyz_handle);
    clps_strat2->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat2->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);
    collapse->set_new_attribute_strategy(m_atdata.m_xyz_handle, clps_strat2);

    collapse->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    // collapse->set_new_attribute_strategy(face_error_attribute);

    collapse->add_transfer_strategy(m_xyz_update);
    collapse->add_transfer_strategy(m_edge_length_update);
    // collapse->add_transfer_strategy(face_error_update);
    m_ops.emplace_back(collapse);
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

    swap->split().set_new_attribute_strategy(m_atdata.m_xyz_handle);
    swap->collapse().set_new_attribute_strategy(
        m_atdata.m_xyz_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    swap->split().set_new_attribute_strategy(m_atdata.m_sum_error_handle);

    {
        // the update strategy that doesn't matter
        swap->split().set_new_attribute_strategy(m_atdata.m_quadrature_error_handle);
        swap->split().set_new_attribute_strategy(m_atdata.m_barrier_energy_handle);
        swap->split().set_new_attribute_strategy(m_atdata.m_amips_error_handle);
        swap->split().set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
        swap->collapse().set_new_attribute_strategy(
            m_atdata.m_3d_edge_length_handle,
            wmtk::operations::CollapseBasicStrategy::CopyOther);
    }

    swap->add_transfer_strategy(m_xyz_update);
    swap->add_transfer_strategy(m_sum_error_update);
    swap->add_transfer_strategy(m_amips_error_update);
    // swap->add_transfer_strategy(m_edge_length_update);

    m_ops.push_back(swap);
}

} // namespace wmtk::components::operations::internal