#include "AMIPS2D_autodiff.h"
inline DECLARE_DIFFSCALAR_BASE();

DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::AMIPS_autodiff(
    const std::array<double, 6>& T)
{
    // typedef Eigen::Vector2d Gradient; // wrt one vert
    // typedef Eigen::Matrix2d Hessian; // wrt one vertx

    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    using Scalar = typename DScalar::Scalar;
    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T[0]), y0(1, T[1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T[2] - x0, T[4] - x0, T[3] - y0, T[5] - y0;
    // reference equilateral triangle: 0,0, 0,2, 1,sqr(3)
    Eigen::Matrix2d Ds, Dsinv;
    Ds << 2., 1., 0., sqrt(3);
    Dsinv = Ds.inverse();
    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F;
    // Dminv = Dm.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    // define of energy = tr(F.T@F)/det(F)

    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<Scalar>::denorm_min()) {
        return DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }
    DScalar AMIPS = (F.transpose() * F).trace() / F.determinant();
    return AMIPS;
}

DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::AMIPS_autodiff_customize_target(
    const std::array<double, 6>& T1,
    const std::array<double, 6>& T2,
    int i)
{
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    using Scalar = typename DScalar::Scalar;
    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T2[i * 2]), y0(1, T2[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T2[(i * 2 + 2) % 6] - x0, T2[(i * 2 + 4) % 6] - x0, T2[(i * 2 + 3) % 6] - y0,
        T2[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv; // = scale(T);
    // Ds << 2., 1., 0., sqrt(3);
    //  (-1,0), (2,0.5), (0,8)
    Ds << T1[(i * 2 + 2) % 6] - T1[(i * 2 + 0) % 6], T1[(i * 2 + 4) % 6] - T1[(i * 2 + 0) % 6],
        T1[(i * 2 + 3) % 6] - T1[(i * 2 + 1) % 6], T1[(i * 2 + 5) % 6] - T1[(i * 2 + 1) % 6];
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<Scalar>::denorm_min()) {
        return DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }
    DScalar AMIPS = (F.transpose() * F).trace() / Fdet;
    return AMIPS;
}

DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::SymDi_autodiff(
    const std::array<double, 6>& T)
{
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    using Scalar = typename DScalar::Scalar;
    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T[0]), y0(1, T[1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T[2] - x0, T[4] - x0, T[3] - y0, T[5] - y0;

    // reference equilateral triangle scaling over reference: 0,0, 0,2, 1,sqr(3) -> S = sqr(3)
    // auto scale = [](const auto& T) {
    //     Eigen::Vector3d ac;
    //     ac << T[4] - T[0], T[5] - T[1], 0.0;
    //     Eigen::Vector3d ab;
    //     ab << T[2] - T[0], T[3] - T[1], 0.0;
    //     double S = ((ac.cross(ab)).norm()) / 2.;
    //     double r = sqrt(S);
    //     assert(r > 0);
    //     wmtk::logger().info("r {}", r);
    //     Eigen::Matrix2d Ds;
    //     Ds << 2 * r * sqrt(1 / sqrt(3)), r * sqrt(1 / sqrt(3)), 0, r * sqrt(sqrt(3));

    //     Eigen::Vector3d edge1;
    //     edge1 << Ds.col(0)(0), Ds.col(0)(1), 0.0;

    //     Eigen::Vector3d edge2;
    //     edge2 << Ds.col(1)(0), Ds.col(1)(1), 0.0;
    //     wmtk::logger().info("new {} , S {}", (edge1.cross(edge2).norm()) / 2., S);

    //     return Ds;
    // };

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F;
    Eigen::Matrix2d Ds, Dsinv; // = scale(T);
    Ds << 2., 1., 0., sqrt(3);
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<Scalar>::denorm_min()) {
        return DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }
    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    DScalar SymDi = (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    return SymDi;
}

/**
 * @brief
 *
 * @param T1 target traingle stacked in std::array<double,6>
 * @param T2 input traingle stacked in std::array<double,6>
 * @param i the index of vertex that will be smoothed
 * @return DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>
 */
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::SymDi_autodiff_customize_target(
    const std::array<double, 6>& T1,
    const std::array<double, 6>& T2,
    int i)
{
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    using Scalar = typename DScalar::Scalar;
    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T2[i * 2]), y0(1, T2[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T2[(i * 2 + 2) % 6] - x0, T2[(i * 2 + 4) % 6] - x0, T2[(i * 2 + 3) % 6] - y0,
        T2[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv; // = scale(T);
    // Ds << 2., 1., 0., sqrt(3);
    //  (-1,0), (2,0.5), (0,8)
    Ds << T1[(i * 2 + 2) % 6] - T1[(i * 2 + 0) % 6], T1[(i * 2 + 4) % 6] - T1[(i * 2 + 0) % 6],
        T1[(i * 2 + 3) % 6] - T1[(i * 2 + 1) % 6], T1[(i * 2 + 5) % 6] - T1[(i * 2 + 1) % 6];
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<Scalar>::denorm_min()) {
        return DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    DScalar SymDi = (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    return SymDi;
}