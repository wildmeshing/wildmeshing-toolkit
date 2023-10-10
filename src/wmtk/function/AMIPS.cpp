#include "AMIPS.hpp"
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_helper_functions.hpp>
using namespace wmtk;
using namespace wmtk::function;

AMIPS::AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AutodiffFunction(mesh, vertex_attribute_handle)
{}
AMIPS_2D::AMIPS_2D(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : AMIPS(mesh, vertex_attribute_handle)
{
    // assert(m_vertex_attribute_handle);
    // check the dimension of the position
    // assert(m_mesh.get_attribute_size(m_vertex_attribute_handle) == 2);
}

AMIPS_3DEmbedded::AMIPS_3DEmbedded(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const image::Image& image)
    : AMIPS(mesh, vertex_uv_handle)
    , m_dofs_to_pos(image)
{}

AMIPS_3DEmbedded::AMIPS_3DEmbedded(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const wmtk::image::SamplingAnalyticFunction::FunctionType type,
    const double a,
    const double b,
    const double c)
    : AMIPS(mesh, vertex_uv_handle)
    , m_dofs_to_pos(type, a, b, c)
{}

Eigen::Matrix<double, 3, 2> AMIPS::get_target_triangle(double scaling)
{
    const static std::array<double, 6> m_target_triangle = {0., 1., 1. / 2., 0., 0., sqrt(3) / 2.};
    return scaling * Eigen::Matrix<double, 3, 2>::ConstMapType(m_target_triangle.data());
}

DScalar AMIPS_2D::get_value_autodiff(const Tuple& tuple) const
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_vertex_attribute_handle);

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
auto AMIPS_2D::function_eval(
    const Eigen::Matrix<T, 2, 1>& uv0,
    const Eigen::Vector2d& uv1,
    const Eigen::Vector2d& uv2) const -> T
{
    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose

    Eigen::Matrix<T, 2, 2> Dm;
    Dm.row(0) = uv1.template cast<T>() - uv0;
    Dm.row(1) = uv2.template cast<T>() - uv0;
    // Dm << uv1(0) - uv0(0), uv2(0) - uv0(0), uv1(1) - uv0(1), uv2(1) - uv0(1);

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Matrix<double, 3, 2> target_triangle = get_target_triangle(1.0);
    Eigen::Vector2d target0, target1, target2;
    target0 = target_triangle.row(0);
    target1 = target_triangle.row(1);
    target2 = target_triangle.row(2);
    Ds.row(0) = target1 - target0;
    Ds.row(1) = target2 - target0;
    // Ds << target1.x() - target0.x(), target2.x() - target0.x(), target1.y() - target0.y(),
    //     target2.y() - target0.y();

    auto Dsdet = Ds.determinant();
    if (abs(Dsdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<double>::infinity());
    }
    Dsinv = Ds.inverse();

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<T, 2, 2> J;
    J = Dm * Dsinv.template cast<T>();
    // J << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
    //     (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
    //     (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
    //     (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Jdet = J.determinant();
    if (abs(Jdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<T>::infinity());
    }
    return (J.transpose() * J).trace() / Jdet;
}

DScalar AMIPS_3DEmbedded::get_value_autodiff(const Tuple& tuple) const
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
auto AMIPS_3DEmbedded::function_eval(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& uv0,
    const Eigen::Vector2d& uv1,
    const Eigen::Vector2d& uv2) const -> T
{
    Eigen::Matrix<T, 3, 1> pos0 = m_dofs_to_pos.dof_to_pos(uv0);
    Eigen::Matrix<double, 3, 1> pos1 = m_dofs_to_pos.dof_to_pos<double>(uv1);
    Eigen::Matrix<double, 3, 1> pos2 = m_dofs_to_pos.dof_to_pos<double>(uv2);
    Eigen::Matrix<T, 3, 1> V2_V1;
    V2_V1 = pos1.template cast<T>() - pos0;
    Eigen::Matrix<T, 3, 1> V3_V1;
    V3_V1 = pos2.template cast<T>() - pos0;

    // tangent bases
    // e1 = (V2 - V1).normalize() (this is buggy due to autodiff normalization implementation)
    // e1 = V2_V1.stableNormalized();
    assert(V2_V1.norm() > 0); // check norm is not 0
    Eigen::Matrix<T, 3, 1> e1 = V2_V1 / V2_V1.norm();
    Eigen::Matrix<T, 3, 1> n = V2_V1.cross(V3_V1);

    // #ifdef DEBUG
    // Eigen::MatrixXd double_n;
    // get_double_vecto(n, 3, double_n);
    // if (double_n.lpNorm<Eigen::Infinity>() < std::numeric_limits<double>::denorm_min()) {
    //     wmtk::logger().critical("n.lpNorm {}", double_n.lpNorm<Eigen::Infinity>());
    //     std::cout << "V1 " << std::endl;
    //     std::cout << std::hexfloat << get_value(pos0(0))<< " " << y1 << v1(2) << std::endl;
    //     std::cout << "V2 " << std::endl;
    //     std::cout << std::hexfloat << v2(0) << " " << v2(1) << v2(2) << std::endl;
    //     std::cout << "V3 " << std::endl;
    //     std::cout << std::hexfloat << v3(0) << " " << v3(1) << v3(2) << std::endl;
    //     assert(false);
    // }
    // #endif
    // n = n.stableNormalized();
    assert(n.norm() > 0); // check norm is not 0
    n = n / n.norm();
    Eigen::Matrix<T, 3, 1> e2 = n.cross(e1);
    // Eigen::Matrix<DScalar, 3, 1> e2_stableNormalized = e2.stableNormalized();
    assert(e2.norm() > 0); // check norm is not 0
    e2 = e2 / e2.norm();

    // project V1, V2, V3 to tangent plane to VT1, VT2, VT3

    Eigen::Matrix<T, 2, 1> VT0, VT1, VT2;
    VT0 << static_cast<T>(0.), static_cast<T>(0.); // the origin
    VT1 << V2_V1.dot(e1), V2_V1.dot(e2);
    VT2 << V3_V1.dot(e1), V3_V1.dot(e2);

    // now construct Dm as before in tangent plane
    // (x2 - x1, y2 - y1, x3 - x1, y2 - y1).transpose
    Eigen::Matrix<T, 2, 2> Dm;
    Dm.row(0) = VT1 - VT0;
    Dm.row(1) = VT2 - VT0;

    T Dmdet = Dm.determinant();
    assert(wmtk::function::get_value(Dmdet) > 0);

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Matrix<double, 3, 2> target_triangle = get_target_triangle(1.0);
    Eigen::Vector2d target0, target1, target2;
    target0 = target_triangle.row(0);
    target1 = target_triangle.row(1);
    target2 = target_triangle.row(2);
    Ds.row(0) = target1 - target0;
    Ds.row(1) = target2 - target0;

    auto Dsdet = Ds.determinant();
    if (abs(Dsdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<double>::infinity());
    }
    Dsinv = Ds.inverse();

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<T, 2, 2> F;
    F = Dm * Dsinv.template cast<T>();

    auto Fdet = F.determinant();
    if (abs(Fdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<T>::infinity());
    }
    return (F.transpose() * F).trace() / Fdet;
}
