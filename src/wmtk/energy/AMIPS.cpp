#include "AMIPS.hpp"

double AMIPS_2D::energy_eval(const Tuple& tuple) const override
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_accessor(m_position_handle);

    Eigen::Vector2d uv1 = pos.vector_attribute(tuple);
    Eigen::Vector2d uv2 = pos.vector_attribute(switch_edge(switch_vertex(tuple)));
    Eigen::Vector2d uv3 = pos.vector_attribute(switch_vertex(switch_edge(tuple)));

    // return the energy
    return energy_eval(uv1, uv2, uv3);
}
DScalar AMIPS_2D::energy_eval_autodiff(const Tuple& tuple) const override
{
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = m_mesh.create_accessor(m_position_handle);

    Eigen::Vector2d uv1 = pos.vector_attribute(tuple);
    Eigen::Vector2d uv2 = pos.vector_attribute(switch_edge(switch_vertex(tuple)));
    Eigen::Vector2d uv3 = pos.vector_attribute(switch_vertex(switch_edge(tuple)));

    // return the energy
    return energy_eval_autodiff(uv1, uv2, uv3);
}

static double AMIPS_2D::energy_eval_autodiff(
    const Eigen::Vector2d uv1,
    const Eigen::Vector2d uv2,
    const Eigen::Vector uv3)
{
    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<double, 2, 2> Dm;

    Dm << uv2(0) - uv1(0), uv3(0) - uv1(0), uv2(1) - uv1(1), uv3(1) - uv1(1);

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Vector2d target_A, target_B, target_C;
    target_A << 0., 0.;
    target_B << 1., 0.;
    target_C << 1. / 2., sqrt(3) / 2.;
    Ds << target_B.x() - target_A.x(), target_C.x() - target_A.x(), target_B.y() - target_A.y(),
        target_C.y() - target_A.y();

    auto Dsdet = Ds.determinant();
    if (std::abs(Dsdet) < std::numeric_limits<AMIPS::Scalar>::denorm_min()) {
        return std::numeric_limits<double>::infinity();
    }
    Dsinv = Ds.inverse();

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<double, 2, 2> F;
    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (std::abs(Fdet) < std::numeric_limits<double>::denorm_min()) {
        return std::numeric_limits<double>::infinity();
    }
    return (F.transpose() * F).trace() / Fdet;
}

static DScalar AMIPS_2D::energy_eval_autodiff(
    const Eigen::Vector2d uv1,
    const Eigen::Vector2d uv2,
    const Eigen::Vector2d uv3)
{
    AMIPS::DScalar x0(0, input_triangle[state.idx * 2]), y0(1, input_triangle[state.idx * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<AMIPS::DScalar, 2, 2> Dm;

    Dm << input_triangle[(i * 2 + 2) % 6] - x0, input_triangle[(i * 2 + 4) % 6] - x0,
        input_triangle[(i * 2 + 3) % 6] - y0, input_triangle[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<AMIPS::DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv;
    Ds << target_triangle[(i * 2 + 2) % 6] - target_triangle[(i * 2 + 0) % 6],
        target_triangle[(i * 2 + 4) % 6] - target_triangle[(i * 2 + 0) % 6],
        target_triangle[(i * 2 + 3) % 6] - target_triangle[(i * 2 + 1) % 6],
        target_triangle[(i * 2 + 5) % 6] - target_triangle[(i * 2 + 1) % 6];

    auto Dsdet = Ds.determinant();
    if (std::abs(Dsdet) < std::numeric_limits<AMIPS::Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::infinity();
        return;
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<AMIPS::Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::infinity();
        return;
    }
    AMIPS::DScalar AMIPS_function = (F.transpose() * F).trace() / Fdet;
}