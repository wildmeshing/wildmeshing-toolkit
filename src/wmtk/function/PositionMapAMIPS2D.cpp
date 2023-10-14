#pragma once
#include <wmtk/utils/triangle_helper_functions.hpp>
#include "PositionMapAMIPS2D.hpp"

namespace wmtk::function {
PositionMapsAMIPS2D::PositionMapsAMIPS2D(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const image::Image& image)
    : AMIPS(mesh, vertex_uv_handle)
    , m_dofs_to_pos(image)
{}

PositionMapsAMIPS2D::PositionMapsAMIPS2D(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const wmtk::image::SamplingAnalyticFunction::FunctionType type,
    const double a,
    const double b,
    const double c)
    : AMIPS(mesh, vertex_uv_handle)
    , m_dofs_to_pos(type, a, b, c)
{}


DScalar PositionMapsAMIPS2D::get_value_autodiff(const Tuple& tuple) const
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_vertex_attribute_handle);

    // TODO curve mesh uv -> t conversion happens here
    Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
    int size = 2;
    Eigen::Matrix<DScalar, 2, 1> dofT = get_T_vector<Eigen::Matrix<DScalar, 2, 1>>(uv0, size);

    Eigen::Vector2d uv1 =
        pos.const_vector_attribute(m_mesh.switch_edge(m_mesh.switch_vertex(tuple)));
    Eigen::Vector2d uv2 =
        pos.const_vector_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(tuple)));

    // return the energy
    return function_eval<DScalar>(dofT, uv1, uv2);
}

template <typename T>
auto PositionMapsAMIPS2D::function_eval(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& uv0,
    const Eigen::Vector2d& uv1,
    const Eigen::Vector2d& uv2) const -> T
{
    Eigen::Matrix<T, 3, 1> pos0 = m_dofs_to_pos.dof_to_pos(uv0);
    Eigen::Matrix<double, 3, 1> pos1 = m_dofs_to_pos.dof_to_pos<double>(uv1);
    Eigen::Matrix<double, 3, 1> pos2 = m_dofs_to_pos.dof_to_pos<double>(uv2);

    return utils::amips(pos0, pos1, pos2);
}
} // namespace wmtk::function
