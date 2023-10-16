#pragma once
#include "AMIPS2D.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/utils/triangle_helper_functions.hpp>
AMIPS2D::AMIPS2D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AMIPS(mesh, vertex_attribute_handle)
{
    // assert(m_vertex_attribute_handle);
    // check the dimension of the position
    // assert(m_mesh.get_attribute_size(m_vertex_attribute_handle) == 2);
}

Eigen::Matrix<double, 3, 2> AMIPS2D::get_target_triangle(double scaling)
{
    const static std::array<double, 6> m_target_triangle = {0., 1., 1. / 2., 0., 0., sqrt(3) / 2.};
    return scaling * Eigen::Matrix<double, 3, 2>::ConstMapType(m_target_triangle.data());
}


DScalar AMIPS2D::get_value_autodiff(const Tuple& tuple) const
{
    return function_eval<DScalar>(tuple);
}

template <typename T>
T AMIPS2D::function_eval(const Tuple& tuple) const
{
    // get_autodiff_value sets the autodiff size if necessary
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_vertex_attribute_handle);

    auto tuple_value = pos.const_vector_attribute(tuple);
    Vector2<T> uv0;
    if constexpr (std::is_same_v < T, DScalar >>) {
        uv0 = as_DScalar(tuple_value);
    } else {
        uv0 = tuple_value;
    }
    constexpr static Primitive PV = PrimitiveType::Vertex;
    constexpr static Primitive PE = PrimitiveType::Edge;

    Eigen::Vector2d uv1 = pos.const_vector_attribute(m_mesh.switch_tuples(tuple, {PE, PV}));
    Eigen::Vector2d uv2 = pos.const_vector_attribute(m_mesh.switch_tuples(tuple, {PV, PE}));

    // return the energy
    return function_eval(uv0, uv1, uv2);
}

template <typename T>
auto AMIPS2D::function_eval(
    const Eigen::Matrix<T, 2, 1>& uv0,
    const Eigen::Vector2d& uv1,
    const Eigen::Vector2d& uv2) const -> T
{
    Eigen::Matrix<T, 2, 2> Dm;
    Dm.row(0) = uv1.template cast<T>() - uv0;
    Dm.row(1) = uv2.template cast<T>() - uv0;
    // Dm << uv1(0) - uv0(0), uv2(0) - uv0(0), uv1(1) - uv0(1), uv2(1) - uv0(1);

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Matrix<double, 3, 2> target_triangle = get_target_triangle(1.0);
    Eigen::Vector2d target0;
    target0 = target_triangle.row(0);
    Ds = target_triangle.bottomRows<2>().rowwise() - target0;

    auto Dsdet = Ds.determinant();
    if (abs(Dsdet) < std::numeric_limits<double>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<double>::infinity());
    }
    Dsinv = Ds.inverse();

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<T, 2, 2> J;
    J = Dm.transpose() * Dsinv.template cast<T>().transpose();

    auto Jdet = J.determinant();
    if (abs(Jdet) < std::numeric_limits<double>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<double>::infinity());
    }
    return (J * J.transpose()).trace() / Jdet;
}
