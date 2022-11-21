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
        state.value = std::numeric_limits<double>::infinity();
        return;
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<SymDi::Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::infinity();
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

void TwoAndAHalf::eval(State& state) const
{
    DiffScalarBase::setVariableCount(2);

    auto double_displacement = [this](double u, double v) {
        return this->displacement(DScalar(u), DScalar(v)).getValue();
    };

    auto input_triangle = state.input_triangle;
    auto target_triangle = state.target_triangle;
    for (auto i = 0; i < 6; i++) target_triangle[i] = state.scaling * target_triangle[i];
    int i = state.idx;

    DScalar x0(0, input_triangle[i * 2]), y0(1, input_triangle[i * 2 + 1]);

    // 2.5D to 3D (for now just lifting the plane to z=1.0)
    // For simplicity always have V1 be the vertex that's the variable and origin
    // Also need to preserve the orientation
    Eigen::Matrix<DScalar, 3, 1> V1(x0, y0, displacement(x0, y0));
    Eigen::Matrix<double, 3, 1> V2(
        input_triangle[(i * 2 + 2) % 6],
        input_triangle[(i * 2 + 3) % 6],
        double_displacement(input_triangle[(i * 2 + 2) % 6], input_triangle[(i * 2 + 3) % 6]));
    Eigen::Matrix<double, 3, 1> V3(
        input_triangle[(i * 2 + 4) % 6],
        input_triangle[(i * 2 + 5) % 6],
        double_displacement(input_triangle[(i * 2 + 4) % 6], input_triangle[(i * 2 + 5) % 6]));

    // calculate the tangent basis
    Eigen::Matrix<DScalar, 3, 1> V2_V1(
        input_triangle[(i * 2 + 2) % 6] - x0,
        input_triangle[(i * 2 + 3) % 6] - y0,
        DScalar(0.0));
    Eigen::Matrix<DScalar, 3, 1> V3_V1(
        input_triangle[(i * 2 + 4) % 6] - x0,
        input_triangle[(i * 2 + 5) % 6] - y0,
        DScalar(0.0));

    Eigen::Matrix<DScalar, 3, 1> e1; // e1 = (V2 - V1).normalize()
    e1 = V2_V1 / V2_V1.norm(); // check norm is not 0
    Eigen::Matrix<DScalar, 3, 1> n;
    n = e1.cross(V3_V1);
    n = n / n.norm();
    Eigen::Matrix<DScalar, 3, 1> e2;
    e2 = n.cross(e1);
    e2 = e2 / e2.norm();

    // project V1, V2, V3 to tangent plane to VT1, VT2, VT3
    Eigen::Matrix<double, 2, 1> VT1;
    Eigen::Matrix<DScalar, 2, 1> VT2, VT3;
    VT1 << 0., 0.; // the origin
    VT2 << V2_V1.dot(e1), (DScalar)0.;
    VT3 << V3_V1.dot(e1), V3_V1.dot(e2);

    // now construct Dm as before in tangent plane
    // (x0 - x1, y0 - y1, x0 - x2, y0 - y2).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << VT2(0, 0) - VT1(0, 0), VT3(0, 0) - VT1(0, 0), VT2(1, 0) - VT1(1, 0),
        VT3(1, 0) - VT1(1, 0);

    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv;
    Ds << target_triangle[(i * 2 + 2) % 6] - target_triangle[(i * 2 + 0) % 6],
        target_triangle[(i * 2 + 4) % 6] - target_triangle[(i * 2 + 0) % 6],
        target_triangle[(i * 2 + 3) % 6] - target_triangle[(i * 2 + 1) % 6],
        target_triangle[(i * 2 + 5) % 6] - target_triangle[(i * 2 + 1) % 6];

    auto Dsdet = Ds.determinant();
    if (std::abs(Dsdet) < std::numeric_limits<Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::infinity();
        return;
    }
    Dsinv = Ds.inverse();

    F << (Dm(0, 0) * Dsinv(0, 0) + Dm(0, 1) * Dsinv(1, 0)),
        (Dm(0, 0) * Dsinv(0, 1) + Dm(0, 1) * Dsinv(1, 1)),
        (Dm(1, 0) * Dsinv(0, 0) + Dm(1, 1) * Dsinv(1, 0)),
        (Dm(1, 0) * Dsinv(0, 1) + Dm(1, 1) * Dsinv(1, 1));
    auto Fdet = F.determinant();
    if (std::abs(Fdet.getValue()) < std::numeric_limits<Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::infinity();
        return;
    }

    // define of energy = F.frobeniusnormsquare + F.inverse().frobeniusnormsquare
    auto F_inv = F.inverse();

    DScalar SymDi_function = (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    state.value = SymDi_function.getValue();
    state.gradient = SymDi_function.getGradient();
    state.hessian = SymDi_function.getHessian();
}
} // namespace wmtk