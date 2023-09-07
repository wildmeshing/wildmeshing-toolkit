#include "AMIPS.hpp"
#include <wmtk/utils/Logger.hpp>
using namespace wmtk;
using namespace wmtk::energy;

AMIPS::AMIPS(const TriMesh& mesh)
    : DifferentiableEnergy(mesh)
{}
AMIPS_2D::AMIPS_2D(const TriMesh& mesh)
    : AMIPS(mesh)
{}

Eigen::Matrix<double, 3, 2> AMIPS::get_target_triangle(double scaling) const
{
    const static std::array<double, 6> m_target_triangle = {0., 0., 1., 0., 1. / 2., sqrt(3) / 2.};
    return scaling * Eigen::Matrix<double, 3, 2>::ConstMapType(m_target_triangle.data());
}

double AMIPS_2D::energy_eval(const Tuple& tuple) const
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_position_handle);
    Eigen::Vector2d uv1 = pos.vector_attribute(tuple).head<2>();
    Eigen::Vector2d uv2 =
        pos.vector_attribute(m_mesh.switch_edge(m_mesh.switch_vertex(tuple))).head<2>();
    Eigen::Vector2d uv3 =
        pos.vector_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(tuple))).head<2>();

    // return the energy
    return energy_eval<double>(uv1, uv2, uv3);
}
DScalar AMIPS_2D::energy_eval_autodiff(const Tuple& tuple) const
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_position_handle);

    Eigen::Vector2d uv1 = pos.vector_attribute(tuple);
    Eigen::Vector2d uv2 = pos.vector_attribute(m_mesh.switch_edge(m_mesh.switch_vertex(tuple)));
    Eigen::Vector2d uv3 = pos.vector_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(tuple)));

    // return the energy
    return energy_eval<DScalar>(uv1, uv2, uv3);
}
template <typename T>
auto AMIPS_2D::energy_eval(
    const Eigen::Vector2d& uv1,
    const Eigen::Vector2d& uv2,
    const Eigen::Vector2d& uv3) const -> T
{
    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<T, 2, 2> Dm;

    typedef Eigen::Matrix<T, 2, 1> Vec2T;
    Vec2T uv1_;

    get_T_vector<Vec2T>(uv1, 2, uv1_);

    Dm << uv2(0) - uv1_(0), uv3(0) - uv1_(0), uv2(1) - uv1_(1), uv3(1) - uv1_(1);

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Matrix<double, 3, 2> target_triangle = get_target_triangle(1.0);
    Eigen::Vector2d target_A, target_B, target_C;
    target_A = target_triangle.row(0);
    target_B = target_triangle.row(1);
    target_C = target_triangle.row(2);
    Ds << target_B.x() - target_A.x(), target_C.x() - target_A.x(), target_B.y() - target_A.y(),
        target_C.y() - target_A.y();

    auto Dsdet = Ds.determinant();
    if (abs(Dsdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<double>::infinity());
    }
    Dsinv = Ds.inverse();

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<T, 2, 2> F;
    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (abs(Fdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<T>::infinity());
    }
    return (F.transpose() * F).trace() / Fdet;
}

double AMIPS_3DEmbedded::energy_eval(const Tuple& tuple) const
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_position_handle);
    Eigen::Vector2d uv1 = pos.vector_attribute(tuple).head<2>();
    // TODO curve mesh uv -> t conversion happens here
    // check if it's on the curve mesh or not
    // if curve mesh, convert to t
    // else
    int size = 2;
    Eigen::Matrix<double, 2, 1> dofT;
    get_T_vector<Eigen::Matrix<double, 2, 1>>(uv1, size, dofT);

    Eigen::Vector2d uv2 =
        pos.vector_attribute(m_mesh.switch_edge(m_mesh.switch_vertex(tuple))).head<2>();
    Eigen::Vector2d uv3 =
        pos.vector_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(tuple))).head<2>();

    // return the energy
    return energy_eval<double>(dofT, uv2, uv3);
}

DScalar AMIPS_3DEmbedded::energy_eval_autodiff(const Tuple& tuple) const
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_const_accessor(m_position_handle);

    // TODO curve mesh uv -> t conversion happens here
    Eigen::Vector2d uv1 = pos.vector_attribute(tuple);

    int size = 2;
    Eigen::Matrix<DScalar, 2, 1> dofT;
    get_T_vector<Eigen::Matrix<DScalar, 2, 1>>(uv1, size, dofT);

    Eigen::Vector2d uv2 = pos.vector_attribute(m_mesh.switch_edge(m_mesh.switch_vertex(tuple)));
    Eigen::Vector2d uv3 = pos.vector_attribute(m_mesh.switch_vertex(m_mesh.switch_edge(tuple)));

    // return the energy
    return energy_eval<DScalar>(dofT, uv2, uv3);
}
template <typename T>
auto AMIPS_3DEmbedded::energy_eval(
    const Eigen::Matrix<T, Eigen::Dynamic, 1>& dofT,
    const Eigen::Vector2d& uv2,
    const Eigen::Vector2d& uv3) const -> T
{
    Eigen::Matrix<T, 3, 1> pos1 = m_dofs_to_pos.dof_to_pos(dofT);
    Eigen::Matrix<double, 3, 1> pos2 = m_dofs_to_pos.dof_to_pos<double>(uv2);
    Eigen::Matrix<double, 3, 1> pos3 = m_dofs_to_pos.dof_to_pos<double>(uv3);
    Eigen::Matrix<T, 3, 1> V2_V1;
    V2_V1 << pos2(0) - pos1(0), pos2(1) - pos1(1), pos2(2) - pos1(2);
    Eigen::Matrix<T, 3, 1> V3_V1;
    V3_V1 << pos3(0) - pos1(0), pos3(1) - pos1(1), pos3(2) - pos1(2);

    // tangent bases
    // e1 = (V2 - V1).normalize()
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
    //     std::cout << std::hexfloat << get_value(pos1(0))<< " " << y1 << v1(2) << std::endl;
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

    Eigen::Matrix<T, 2, 1> VT1, VT2, VT3;
    VT1 << static_cast<T>(0.), static_cast<T>(0.); // the origin
    VT2 << V2_V1.dot(e1), V2_V1.dot(e2);
    VT3 << V3_V1.dot(e1), V3_V1.dot(e2);

    // now construct Dm as before in tangent plane
    // (x2 - x1, y2 - y1, x3 - x1, y2 - y1).transpose
    Eigen::Matrix<T, 2, 2> Dm;
    Dm << VT2.x() - VT1.x(), VT3.x() - VT1.x(), VT2.y() - VT1.y(), VT3.y() - VT1.y();

    T Dmdet = Dm.determinant();
    assert(wmtk::energy::get_value(Dmdet) > 0);

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Matrix<double, 3, 2> target_triangle = get_target_triangle(1.0);
    Eigen::Vector2d target_A, target_B, target_C;
    target_A = target_triangle.row(0);
    target_B = target_triangle.row(1);
    target_C = target_triangle.row(2);
    Ds << target_B.x() - target_A.x(), target_C.x() - target_A.x(), target_B.y() - target_A.y(),
        target_C.y() - target_A.y();

    auto Dsdet = Ds.determinant();
    if (abs(Dsdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<double>::infinity());
    }
    Dsinv = Ds.inverse();

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<T, 2, 2> F;
    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (abs(Fdet) < std::numeric_limits<Scalar>::denorm_min()) {
        return static_cast<T>(std::numeric_limits<T>::infinity());
    }
    return (F.transpose() * F).trace() / Fdet;
}
