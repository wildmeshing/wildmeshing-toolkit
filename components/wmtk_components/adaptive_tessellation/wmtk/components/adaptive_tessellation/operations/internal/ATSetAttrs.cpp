#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include <wmtk/components/adaptive_tessellation/function/simplex/DistanceEnergy.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionAvgDistanceToLimit.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
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


void ATOperations::set_uvmesh_xyz_update_rule()
{ // 3d vert position update
    auto compute_vertex_position = [&](const Eigen::Vector2d& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        assert(P.rows() == 2);
        Eigen::Vector2d uv = P.col(0);
        return m_atdata.evaluator_ptr()->uv_to_position(uv);
    };
    m_uvmesh_xyz_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_uvmesh_xyz_handle,
            m_atdata.m_uv_handle,
            compute_vertex_position);
}

void ATOperations::initialize_xyz()
{
    // initialize this position field
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto uv = m_uv_accessor.vector_attribute(v);
        auto p = m_atdata.evaluator_ptr()->uv_to_position<double>(uv);
        m_uvmesh_xyz_accessor.vector_attribute(v) = p;
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


        error(0) = m_atdata.mapping_ptr()->distance(uv0, uv1, uv2);

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

        double res = m_atdata.mapping_ptr()->distance(v0, v1, v2);

        m_distance_error_accessor.scalar_attribute(f) = res;
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
        error(0) = curved_edge_length_on_displaced_surface(uv0, uv1, m_atdata.evaluator_ptr());
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
            curved_edge_length_on_displaced_surface(uv0, uv1, m_atdata.evaluator_ptr());
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
            auto disp_i = [&i, &x, &m_evaluator_ptr](const Eigen::Vector2d& uv) -> double {
                return m_evaluator_ptr->uv_to_position(x)(i);
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

} // namespace wmtk::components::operations::internal