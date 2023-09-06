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

    get_local_vector<Vec2T>(uv1, 2, uv1_);

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
