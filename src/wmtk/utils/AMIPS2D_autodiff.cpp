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
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> wmtk::SymDi_Autodiff(
    const std::array<double, 6>& T)
{
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
    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();
    DScalar SymDi = (F * F.transpose()).trace() + (F_inv * F_inv.transpose()).trace();
    return SymDi;
}