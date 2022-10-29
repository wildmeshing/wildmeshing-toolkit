#include "AMIPS2D_autodiff.h"
inline DECLARE_DIFFSCALAR_BASE();
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::AMIPS_autodiff(
    const std::array<double, 6>& T)
{
    // typedef Eigen::Vector2d Gradient; // wrt one vert
    // typedef Eigen::Matrix2d Hessian; // wrt one vertx

    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;

    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T[0]), y0(1, T[1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T[2] - x0, T[4] - x0, T[3] - y0, T[5] - y0;

    // reference equilateral triangle: 0,0, 0,2, 1,sqr(3)
    Eigen::Matrix2d Ds;
    Ds << 2., 1., 0., sqrt(3);
    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F, Dminv;
    Dminv = Dm.inverse();
    F << (Ds(0, 0) * Dminv(0, 0) + Ds(0, 1) * Dminv(1, 0)),
        (Ds(0, 0) * Dminv(0, 1) + Ds(0, 1) * Dminv(1, 1)),
        (Ds(1, 0) * Dminv(0, 0) + Ds(1, 1) * Dminv(1, 0)),
        (Ds(1, 0) * Dminv(0, 1) + Ds(1, 1) * Dminv(1, 1));
    // define of energy = tr(F.T@F)/det(F)
    DScalar AMIPS = (F.transpose() * F).trace() / F.determinant();
    return AMIPS;
}

DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::AMIPS_autodiff_customize_target(
    const std::array<double, 6>& T1,
    const std::array<double, 6>& T2,
    int i)
{
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T2[i * 2]), y0(1, T2[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T2[(i * 2 + 2) % 6] - x0, T2[(i * 2 + 4) % 6] - x0, T2[(i * 2 + 3) % 6] - y0,
        T2[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F, Dminv;
    Dminv = Dm.inverse();
    Eigen::Matrix2d Ds; // = scale(T);
    // Ds << 2., 1., 0., sqrt(3);
    //  (-1,0), (2,0.5), (0,8)
    Ds << T1[(i * 2 + 2) % 6] - T1[(i * 2 + 0) % 6], T1[(i * 2 + 4) % 6] - T1[(i * 2 + 0) % 6],
        T1[(i * 2 + 3) % 6] - T1[(i * 2 + 1) % 6], T1[(i * 2 + 5) % 6] - T1[(i * 2 + 1) % 6];

    F << (Ds(0, 0) * Dminv(0, 0) + Ds(0, 1) * Dminv(1, 0)),
        (Ds(0, 0) * Dminv(0, 1) + Ds(0, 1) * Dminv(1, 1)),
        (Ds(1, 0) * Dminv(0, 0) + Ds(1, 1) * Dminv(1, 0)),
        (Ds(1, 0) * Dminv(0, 1) + Ds(1, 1) * Dminv(1, 1));

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    DScalar AMIPS = (F.transpose() * F).trace() / F.determinant();
    return AMIPS;
}

DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::SymDi_autodiff(
    const std::array<double, 6>& T)
{
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
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
    Eigen::Matrix<DScalar, 2, 2> F, Dminv;
    Dminv = Dm.inverse();
    Eigen::Matrix2d Ds; // = scale(T);
    Ds << 2., 1., 0., sqrt(3);

    F << (Ds(0, 0) * Dminv(0, 0) + Ds(0, 1) * Dminv(1, 0)),
        (Ds(0, 0) * Dminv(0, 1) + Ds(0, 1) * Dminv(1, 1)),
        (Ds(1, 0) * Dminv(0, 0) + Ds(1, 1) * Dminv(1, 0)),
        (Ds(1, 0) * Dminv(0, 1) + Ds(1, 1) * Dminv(1, 1));

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    DScalar SymDi = (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    return SymDi;
}

DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::SymDi_autodiff_customize_target(
    const std::array<double, 6>& T1,
    const std::array<double, 6>& T2,
    int i)
{
    typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
    DiffScalarBase::setVariableCount(2);
    DScalar x0(0, T2[i * 2]), y0(1, T2[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << T2[(i * 2 + 2) % 6] - x0, T2[(i * 2 + 4) % 6] - x0, T2[(i * 2 + 3) % 6] - y0,
        T2[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<DScalar, 2, 2> F, Dminv;
    Dminv = Dm.inverse();
    Eigen::Matrix2d Ds; // = scale(T);
    // Ds << 2., 1., 0., sqrt(3);
    //  (-1,0), (2,0.5), (0,8)
    Ds << T1[(i * 2 + 2) % 6] - T1[(i * 2 + 0) % 6], T1[(i * 2 + 4) % 6] - T1[(i * 2 + 0) % 6],
        T1[(i * 2 + 3) % 6] - T1[(i * 2 + 1) % 6], T1[(i * 2 + 5) % 6] - T1[(i * 2 + 1) % 6];

    F << (Ds(0, 0) * Dminv(0, 0) + Ds(0, 1) * Dminv(1, 0)),
        (Ds(0, 0) * Dminv(0, 1) + Ds(0, 1) * Dminv(1, 1)),
        (Ds(1, 0) * Dminv(0, 0) + Ds(1, 1) * Dminv(1, 0)),
        (Ds(1, 0) * Dminv(0, 1) + Ds(1, 1) * Dminv(1, 1));

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    DScalar SymDi = (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    return SymDi;
}