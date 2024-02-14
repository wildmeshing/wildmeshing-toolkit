#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>

#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/function/utils/amips.hpp>

#include <predicates.h>
#include <iostream>

#include "ATOperations.hpp"
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


    m_amips_energy = std::make_shared<wmtk::function::TriangleAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle());
    m_3d_amips_energy = std::make_shared<wmtk::function::PositionMapAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_evaluator_ptr,
        m_amips_weight,
        m_area_weighted_amips);


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
        auto res = wmtk::function::utils::amips(p0, p1, p2);
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
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Triangle)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex));
        const Eigen::Vector2d uv2 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_tuple(
                m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Edge),
                PrimitiveType::Vertex));
        auto p0 = m_evaluator_ptr->uv_to_position(uv0);
        auto p1 = m_evaluator_ptr->uv_to_position(uv1);
        auto p2 = m_evaluator_ptr->uv_to_position(uv2);
        auto res = wmtk::function::utils::amips(p0, p1, p2);
        // res = m_amips_weight * res;
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
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Triangle)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex));
        const Eigen::Vector2d uv2 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_tuple(
                m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Edge),
                PrimitiveType::Vertex));
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
    for (auto& f : m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Triangle)) {
        if (!m_atdata.uv_mesh_ptr()->is_ccw(f)) {
            f = m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex);
        }
        const Eigen::Vector2d uv0 = m_uv_accessor.vector_attribute(f);
        const Eigen::Vector2d uv1 = m_uv_accessor.vector_attribute(
            m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Vertex));
        const Eigen::Vector2d uv2 =
            m_uv_accessor.vector_attribute(m_atdata.uv_mesh_ptr()->switch_tuple(
                m_atdata.uv_mesh_ptr()->switch_tuple(f, PrimitiveType::Edge),
                PrimitiveType::Vertex));
        auto res = wmtk::function::utils::area_barrier(uv0, uv1, uv2, m_barrier_triangle_area);
        m_barrier_energy_accessor.scalar_attribute(f) = res;
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
} // namespace wmtk::components::operations::internal