#pragma once
#include "Energy2d.h"

namespace wmtk {

AMIPS::DScalar AMIPS::energy_function(std::array<double, 6>& T1, std::array<double, 6>& T2, int i)
{
    DiffScalarBase::setVariableCount(2);
    AMIPS::DScalar x0(0, T2[i * 2]), y0(1, T2[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<AMIPS::DScalar, 2, 2> Dm;
    Dm << T2[(i * 2 + 2) % 6] - x0, T2[(i * 2 + 4) % 6] - x0, T2[(i * 2 + 3) % 6] - y0,
        T2[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Ds@Dm.inv
    Eigen::Matrix<AMIPS::DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv;
    Ds << T1[(i * 2 + 2) % 6] - T1[(i * 2 + 0) % 6], T1[(i * 2 + 4) % 6] - T1[(i * 2 + 0) % 6],
        T1[(i * 2 + 3) % 6] - T1[(i * 2 + 1) % 6], T1[(i * 2 + 5) % 6] - T1[(i * 2 + 1) % 6];

    auto Dsdet = Ds.determinant();
    if (std::abs(Dsdet) < std::numeric_limits<AMIPS::Scalar>::denorm_min()) {
        return AMIPS::DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<AMIPS::Scalar>::denorm_min()) {
        return AMIPS::DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }
    AMIPS::DScalar AMIPS_function = (F.transpose() * F).trace() / Fdet;
    return AMIPS_function;
}

SymDi::DScalar SymDi::energy_function(std::array<double, 6>& T1, std::array<double, 6>& T2, int i)
{
    DiffScalarBase::setVariableCount(2);
    SymDi::DScalar x0(0, T2[i * 2]), y0(1, T2[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<SymDi::DScalar, 2, 2> Dm;
    Dm << T2[(i * 2 + 2) % 6] - x0, T2[(i * 2 + 4) % 6] - x0, T2[(i * 2 + 3) % 6] - y0,
        T2[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<SymDi::DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv;
    Ds << T1[(i * 2 + 2) % 6] - T1[(i * 2 + 0) % 6], T1[(i * 2 + 4) % 6] - T1[(i * 2 + 0) % 6],
        T1[(i * 2 + 3) % 6] - T1[(i * 2 + 1) % 6], T1[(i * 2 + 5) % 6] - T1[(i * 2 + 1) % 6];

    auto Dsdet = Ds.determinant();
    if (std::abs(Dsdet) < std::numeric_limits<SymDi::Scalar>::denorm_min()) {
        return SymDi::DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<SymDi::Scalar>::denorm_min()) {
        return SymDi::DScalar(
            std::numeric_limits<double>::max(),
            Eigen::Vector2d(0, 0),
            Eigen::Matrix2d::Zero());
    }

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    SymDi::DScalar SymDi = (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    return SymDi;
}
} // namespace wmtk