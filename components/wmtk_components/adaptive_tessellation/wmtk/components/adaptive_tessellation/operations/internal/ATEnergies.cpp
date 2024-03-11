#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>

#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>

#include <predicates.h>
#include <iostream>

#include "ATOperations.hpp"

#include <finitediff.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/LineQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/Quadrature.hpp>

namespace wmtk::components::operations::internal {
void ATOperations::set_energies()
{
    m_distance_energy = std::make_shared<wmtk::function::DistanceEnergy>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_integral_ptr,
        1);
    m_distance_nondiff_energy = std::make_shared<wmtk::function::DistanceEnergyNonDiff>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_integral_ptr,
        1);


    m_2d_amips_energy =
        std::make_shared<wmtk::function::AMIPS>(*m_atdata.uv_mesh_ptr(), m_atdata.uv_handle());

    // this amips contains area barrier
    m_3d_amips_energy = std::make_shared<wmtk::function::PositionMapAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_evaluator_ptr,
        m_amips_weight,
        m_area_weighted_amips,
        m_barrier_weight,
        m_barrier_triangle_area);
}

void ATOperations::set_uvmesh_xyz_update_rule_initialize()
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


        error(0) = m_integral_ptr->get_error_one_triangle_exact(uv0, uv1, uv2);

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
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Triangle)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex);
        }
        const Eigen::Vector2d v0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d v1 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex));
        const Eigen::Vector2d v2 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_tuple(
                m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Edge),
                PrimitiveType::Vertex));

        double res = m_integral_ptr->get_error_one_triangle_exact(v0, v1, v2);

        m_distance_error_accessor.scalar_attribute(f) = res;
    }
}

double ATOperations::amips3d_in_double(
    Eigen::Vector2<double>& uv0,
    Eigen::Vector2<double>& uv1,
    Eigen::Vector2<double>& uv2)
{
    // assumes the triangle is not inverted
    if (orient2d(uv0.data(), uv1.data(), uv2.data()) <= 0) {
        std::swap(uv1, uv2);
    }
    if (orient2d(uv0.data(), uv1.data(), uv2.data()) <= 0) {
        Eigen::VectorXd infinity(1);
        return std::numeric_limits<double>::infinity();
    }
    auto p0 = m_evaluator_ptr->uv_to_position(uv0);
    auto p1 = m_evaluator_ptr->uv_to_position(uv1);
    auto p2 = m_evaluator_ptr->uv_to_position(uv2);
    Eigen::VectorXd error(1);
    auto amips = m_amips_weight * wmtk::function::utils::amips(p0, p1, p2);
    auto barrier = m_barrier_weight *
                   wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area);
    if (m_area_weighted_amips) {
        amips *= wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
    }
    return amips + barrier;
}
void ATOperations::set_3d_amips_error_update_rule()
{
    auto compute_amips_error = [&](const Eigen::Matrix<double, 2, 3>& P) -> Eigen::VectorXd {
        assert(P.cols() == 3);
        assert(P.rows() == 2);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);
        Eigen::Vector2<double> uv2 = P.col(2);


        Eigen::VectorXd error(1);

        error(0) = amips3d_in_double(uv0, uv1, uv2);
        return error;
    };
    m_amips_error_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_amips_error_handle,
            m_atdata.uv_handle(),
            compute_amips_error);
}
void ATOperations::initialize_3d_amips_error()
{
    // initialize face error values
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Triangle)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex);
        }
        Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        Eigen::Vector2d uv1 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex));
        Eigen::Vector2d uv2 = m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_tuple(
            m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Edge),
            PrimitiveType::Vertex));
        m_amips_error_accessor.scalar_attribute(f) = amips3d_in_double(uv0, uv1, uv2);
    }
}
void ATOperations::set_3d_edge_length_update_rule()
{
    auto compute_3d_edge_length = [&](const Eigen::Matrix<double, 3, 2>& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 3);
        Eigen::Vector3<double> p0 = P.col(0);
        Eigen::Vector3<double> p1 = P.col(1);

        Eigen::VectorXd error(1);
        error(0) = (p0 - p1).norm();
        return error;
    };
    m_3d_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_3d_edge_length_handle,
            m_atdata.m_uvmesh_xyz_handle,
            compute_3d_edge_length);
}
void ATOperations::initialize_3d_edge_length()
{
    // initialize edge length values
    for (auto& e : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Edge)) {
        const auto p0 = m_uvmesh_xyz_accessor.vector_attribute(e);
        const auto p1 = m_uvmesh_xyz_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(e, PrimitiveType::Vertex));
        m_3d_edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }
}
void ATOperations::set_curved_edge_length_update_rule()
{
    auto compute_curved_edge_length = [&](const Eigen::Matrix<double, 2, 2>& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2);
        Eigen::Vector2<double> uv0 = P.col(0);
        Eigen::Vector2<double> uv1 = P.col(1);

        Eigen::VectorXd error(1);
        error(0) = curved_edge_length_on_displaced_surface(uv0, uv1, m_evaluator_ptr);
        return error;
    };
    m_curved_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_curved_edge_length_handle,
            m_atdata.m_uv_handle,
            compute_curved_edge_length);
}
void ATOperations::initialize_curved_edge_length()
{
    // initialize edge length values
    for (auto& e : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Edge)) {
        const auto uv0 = m_uv_accessor.vector_attribute(e);
        const auto uv1 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(e, PrimitiveType::Vertex));
        m_curved_edge_length_accessor.scalar_attribute(e) =
            curved_edge_length_on_displaced_surface(uv0, uv1, m_evaluator_ptr);
    }
}

double ATOperations::curved_edge_length_on_displaced_surface(
    const Eigen::Vector2d& uv0,
    const Eigen::Vector2d& uv1,
    const std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        m_evaluator_ptr)
{
    auto displacement =
        [&m_evaluator_ptr](const Eigen::Vector2d& x) -> Eigen::Matrix<double, 3, 2> {
        Eigen::Matrix<double, 3, 2> Jac;
        for (int i = 0; i < 3; ++i) {
            auto disp_i = [&i, &m_evaluator_ptr](const Eigen::Vector2d& uv) -> double {
                return m_evaluator_ptr->uv_to_position(uv)(i);
            };
            Eigen::VectorXd grad_i;
            fd::finite_gradient(x, disp_i, grad_i, fd::AccuracyOrder::FOURTH);
            Jac.row(i) = grad_i;
        }
        return Jac;
    };

    wmtk::Quadrature quadrature;
    wmtk::LineQuadrature line_quadrature;
    line_quadrature.get_quadrature(6, quadrature);

    double arc_length = 0;
    for (auto i = 0; i < quadrature.size(); ++i) {
        Eigen::Vector2d quad_point_uv = quadrature.points()(i) * (uv1 - uv0) + uv0;
        Eigen::Matrix<double, 3, 2> Jac = displacement(quad_point_uv);
        arc_length += quadrature.weights()[i] * (Jac * (uv1 - uv0)).norm();
    }
    return arc_length;
}


void ATOperations::initialize_face_rgb_state()
{
    // all faces set to green at level 0
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Triangle)) {
        m_face_rgb_state_accessor.vector_attribute(f) = Eigen::Vector2<int64_t>(0, 0);
    }
}
void ATOperations::initialize_edge_rgb_state()
{
    // all edges set to green at level 0
    for (auto& e : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Edge)) {
        m_edge_rgb_state_accessor.vector_attribute(e) = Eigen::Vector2<int64_t>(0, 0);
    }
}

} // namespace wmtk::components::operations::internal