#include "Energy2d.h"

namespace wmtk {
void AMIPS::eval(State& state) const
{
    DiffScalarBase::setVariableCount(2);
    auto input_triangle = state.input_triangle;
    auto target_triangle = state.target_triangle;
    for (auto i = 0; i < 6; i++) target_triangle[i] = state.scaling * target_triangle[i];
    int i = state.idx;

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
        state.value = std::numeric_limits<double>::max();
        return;
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));

    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<AMIPS::Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::max();
        return;
    }
    AMIPS::DScalar AMIPS_function = (F.transpose() * F).trace() / Fdet;
    state.value = AMIPS_function.getValue();
    state.gradient = AMIPS_function.getGradient();
    state.hessian = AMIPS_function.getHessian();
}

void SymDi::eval(State& state) const
{
    DiffScalarBase::setVariableCount(2);

    auto input_triangle = state.input_triangle;
    auto target_triangle = state.target_triangle;
    for (auto i = 0; i < 6; i++) target_triangle[i] = state.scaling * target_triangle[i];
    int i = state.idx;

    SymDi::DScalar x0(0, input_triangle[i * 2]), y0(1, input_triangle[i * 2 + 1]);

    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<SymDi::DScalar, 2, 2> Dm;
    Dm << input_triangle[(i * 2 + 2) % 6] - x0, input_triangle[(i * 2 + 4) % 6] - x0,
        input_triangle[(i * 2 + 3) % 6] - y0, input_triangle[(i * 2 + 5) % 6] - y0;

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<SymDi::DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv;
    Ds << target_triangle[(i * 2 + 2) % 6] - target_triangle[(i * 2 + 0) % 6],
        target_triangle[(i * 2 + 4) % 6] - target_triangle[(i * 2 + 0) % 6],
        target_triangle[(i * 2 + 3) % 6] - target_triangle[(i * 2 + 1) % 6],
        target_triangle[(i * 2 + 5) % 6] - target_triangle[(i * 2 + 1) % 6];

    auto Dsdet = Ds.determinant();
    if (std::abs(Dsdet) < std::numeric_limits<SymDi::Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::max();
        return;
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<SymDi::Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::max();
        return;
    }

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    SymDi::DScalar SymDi_function =
        (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    state.value = SymDi_function.getValue();
    state.gradient = SymDi_function.getGradient();
    state.hessian = SymDi_function.getHessian();
}
} // namespace wmtk