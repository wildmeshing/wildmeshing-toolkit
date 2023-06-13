#include "Energy2d.h"

namespace wmtk {
void AMIPS::eval(State& state, DofsToPositions& dof_to_positions) const
{
    DiffScalarBase::setVariableCount(2);
    auto [x1, y1] = dof_to_positions.eval(state.dofx);
    auto input_triangle = std::array{x1.getValue(),
                                     y1.getValue(),
                                     state.two_opposite_vertices(0, 0),
                                     state.two_opposite_vertices(0, 1),
                                     state.two_opposite_vertices(0, 2),
                                     state.two_opposite_vertices(0, 3)};
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

void SymDi::eval(State& state, DofsToPositions& dof_to_positions) const
{
    DiffScalarBase::setVariableCount(2);

    auto [x1, y1] = dof_to_positions.eval(state.dofx);
    auto input_triangle = std::array{x1.getValue(),
                                     y1.getValue(),
                                     state.two_opposite_vertices(0, 0),
                                     state.two_opposite_vertices(0, 1),
                                     state.two_opposite_vertices(0, 2),
                                     state.two_opposite_vertices(0, 3)};
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

    DScalar x1(0, input_triangle[i * 2]), y1(1, input_triangle[i * 2 + 1]);
    // For simplicity always have V1 be the vertex that's the variable and origin
    // Also need to preserve the orientation
    DScalar z1 = displacement(x1, y1);
    double z2 =
        double_displacement(input_triangle[(i * 2 + 2) % 6], input_triangle[(i * 2 + 3) % 6]);
    double z3 =
        double_displacement(input_triangle[(i * 2 + 4) % 6], input_triangle[(i * 2 + 5) % 6]);

    // calculate the tangent basis
    Eigen::Matrix<DScalar, 3, 1> V2_V1(
        input_triangle[(i * 2 + 2) % 6] - x1,
        input_triangle[(i * 2 + 3) % 6] - y1,
        z2 - z1);
    Eigen::Matrix<DScalar, 3, 1> V3_V1(
        input_triangle[(i * 2 + 4) % 6] - x1,
        input_triangle[(i * 2 + 5) % 6] - y1,
        z3 - z1);

    Eigen::Matrix<DScalar, 3, 1> e1; // e1 = (V2 - V1).normalize()
    e1 = V2_V1.stableNormalized();
    //assert(V2_V1.norm() != 0); // check norm is not 0
    // e1 = V2_V1 / V2_V1.norm();
    Eigen::Matrix<DScalar, 3, 1> n;
    n = V2_V1.cross(V3_V1);
    n.stableNormalized();
    if (n.lpNorm<Eigen::Infinity>() < std::numeric_limits<double>::denorm_min()) {
        std::cout << "V1 " << std::endl;
        std::cout << std::hexfloat << input_triangle[i * 2] << " " << input_triangle[i * 2 + 1]
                  << z1 << std::endl;
        std::cout << "V2 " << std::endl;
        std::cout << std::hexfloat << input_triangle[i * 2 + 2] << " " << input_triangle[i * 2 + 3]
                  << z2 << std::endl;
        std::cout << "V3 " << std::endl;
        std::cout << std::hexfloat << input_triangle[i * 2 + 4] << " " << input_triangle[i * 2 + 5]
                  << z3 << std::endl;
        assert(false);
    }

    // assert(n.norm() != 0); // check norm is not 0
    // n = n / n.norm();
    Eigen::Matrix<DScalar, 3, 1> e2;
    e2 = n.cross(e1);
    e2.stableNormalized();
    //assert(e2.norm() != 0); // check norm is not 0
    // e2 = e2 / e2.norm();

    // project V1, V2, V3 to tangent plane to VT1, VT2, VT3
    Eigen::Matrix<double, 2, 1> VT1;
    Eigen::Matrix<DScalar, 2, 1> VT2, VT3;
    VT1 << 0., 0.; // the origin
    VT2 << V2_V1.dot(e1), (DScalar)0.;
    VT3 << V3_V1.dot(e1), V3_V1.dot(e2);

    // now construct Dm as before in tangent plane
    // (x2 - x1, y2 - y1, x3 - x1, y2 - y1).transpose
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

    DScalar TwoAndAHalf_function =
        (F.transpose() * F).trace() + (F_inv.transpose() * F_inv).trace();
    state.value = TwoAndAHalf_function.getValue();
    state.gradient = TwoAndAHalf_function.getGradient();
    state.hessian = TwoAndAHalf_function.getHessian();

    wmtk::logger().info("/// idx {} value {} grad {}", state.idx, state.value, state.gradient);
}

void EdgeLengthEnergy::eval(State& state, DofsToPositions& dof_to_positions) const
{
    DiffScalarBase::setVariableCount(2);
    assert(state.scaling > 0);

    double l_squared = std::pow(state.scaling, 2);
    assert(l_squared > 0);
    //// trying this. so just minimizing for one ring edge length
    l_squared = 0;
    ////
    assert(state.two_opposite_vertices.rows() == 1);
    auto [x1, y1] = dof_to_positions.eval(state.dofx);
    DScalar total_energy = DScalar(0);

    Eigen::Matrix<DScalar, 3, 1> v1 = m_displ->get(x1, y1);
    DScalar v2u = DScalar(state.two_opposite_vertices(0, 0));
    DScalar v2v = DScalar(state.two_opposite_vertices(0, 1));
    Eigen::Matrix<DScalar, 3, 1> v2 = m_displ->get(v2u, v2v);
    DScalar v3u = DScalar(state.two_opposite_vertices(0, 2));
    DScalar v3v = DScalar(state.two_opposite_vertices(0, 3));
    Eigen::Matrix<DScalar, 3, 1> v3 = m_displ->get(v3u, v3v);

    Eigen::Matrix<DScalar, 3, 1> V2_V1 = v2 - v1;
    Eigen::Matrix<DScalar, 3, 1> V3_V1 = v3 - v1;
    Eigen::Matrix<DScalar, 3, 1> V3_V2 = v3 - v2;

    // check if area is either inverted or smaller than certain A_hat
    // this is 3d area!!!
    DScalar area;
    area = (V2_V1.cross(V3_V1)).squaredNorm();
    assert(area.getValue() > 0);
    // energies for edge length
    total_energy += pow((V2_V1.squaredNorm() - l_squared), 2);
    total_energy += pow((V3_V1.squaredNorm() - l_squared), 2);
    total_energy += pow((V3_V2.squaredNorm() - l_squared), 2);

    // energy barrier for small triangle only when A < A_hat
    // check if area is either inverted or smaller than certain A_hat
    // wmtk::logger().info("----current area {}", area.getValue());
    double A_hat = 1e-6; // this is arbitrary now
    assert(A_hat > 0);

    if (area < A_hat) {
        assert((area / A_hat) < 1.0);
        DScalar barrier_energy = -(area - A_hat) * (area - A_hat) * log(area / A_hat);
        // wmtk::logger().info("----current barrier energy {}", barrier_energy.getValue());
        total_energy += barrier_energy;
    }

    state.value = total_energy.getValue();
    state.gradient = total_energy.getGradient();
    state.hessian = total_energy.getHessian();
}

void EdgeAccuracyEnergy::eval(State& state, DofsToPositions& dof_to_positions) const
// measure edge quadrature. For each one-ring triangle only takes one edge, which will cover all the
// one-ring edges without repeat
{
    DiffScalarBase::setVariableCount(2);
    DScalar total_energy = DScalar(0);
    assert(state.two_opposite_vertices.rows() == 1);
    auto [x1, y1] = dof_to_positions.eval(state.dofx);

    Eigen::Matrix<DScalar, 2, 1> v1, v2;
    v1 << x1, y1;
    v2 << DScalar(state.two_opposite_vertices(0, 0)), DScalar(state.two_opposite_vertices(0, 1));

    total_energy = m_displ->get_error_per_edge(v1, v2);
    state.value = total_energy.getValue();
    state.gradient = total_energy.getGradient();
    state.hessian = total_energy.getHessian();
}

void AreaAccuracyEnergy::eval(State& state, DofsToPositions& dof_to_positions) const
// measure edge quadrature. For each one-ring triangle only takes one edge, which will cover all the
// one-ring edges without repeat
{
    lagrange::enable_fpe();
    DiffScalarBase::setVariableCount(2);
    DScalar total_energy = DScalar(0);
    assert(state.two_opposite_vertices.rows() == 1);
    auto [x1, y1] = dof_to_positions.eval(state.dofx);
    Eigen::Matrix<DScalar, 3, 2, Eigen::RowMajor> triangle;
    triangle << x1, y1, DScalar(state.two_opposite_vertices(0, 0)),
        DScalar(state.two_opposite_vertices(0, 1)), DScalar(state.two_opposite_vertices(0, 2)),
        DScalar(state.two_opposite_vertices(0, 3));

    // is in order primary_vert, p2, p3
    DScalar v2u = DScalar(state.two_opposite_vertices(0, 0));
    DScalar v2v = DScalar(state.two_opposite_vertices(0, 1));

    DScalar v3u = DScalar(state.two_opposite_vertices(0, 2));
    DScalar v3v = DScalar(state.two_opposite_vertices(0, 3));


    // check if area is either inverted or smaller than certain A_hat
    DScalar dblarea = (v2u - x1) * (v3v - y1) - (v2v - y1) * (v3u - x1);
    DScalar area;
    area = dblarea * dblarea;
    // wmtk::logger().info("----current area {}", area.getValue());
    double A_hat = 1e-6; // this is arbitrary now
    assert(A_hat > 0);

    if (area < A_hat) {
        assert((area / A_hat) < 1.0);
        DScalar barrier_energy = -(area - A_hat) * (area - A_hat) * log(area / A_hat);
        // wmtk::logger().info("----current barrier energy {}", barrier_energy.getValue());
        total_energy += barrier_energy;
    }

    // total_energy = m_displ->get_error_per_triangle(triangle);
    total_energy += m_texture_integral.get_error_one_triangle(triangle);
    state.value = total_energy.getValue();
    state.gradient = total_energy.getGradient();
    state.hessian = total_energy.getHessian();
}

void QuadricEnergy::eval(State& state, DofsToPositions& dof_to_positions) const
// measure edge quadrature. For each one-ring triangle only takes one edge, which will cover all the
// one-ring edges without repeat
{
    DiffScalarBase::setVariableCount(2);
    assert(state.two_opposite_vertices.rows() == 1);
    auto [x1, y1] = dof_to_positions.eval(state.dofx);

    auto p = m_displ->get(x1, y1);

    const auto& q = m_facet_quadrics[state.idx];
    DScalar energy = p.transpose() * q.A().cast<DScalar>() * p -
                     DScalar(2.0) * p.dot(q.b().cast<DScalar>()) + DScalar(q.c);

    DScalar v2u = DScalar(state.two_opposite_vertices(0, 0));
    DScalar v2v = DScalar(state.two_opposite_vertices(0, 1));
    Eigen::Matrix<DScalar, 3, 1> v2 = m_displ->get(v2u, v2v);
    DScalar v3u = DScalar(state.two_opposite_vertices(0, 2));
    DScalar v3v = DScalar(state.two_opposite_vertices(0, 3));
    Eigen::Matrix<DScalar, 3, 1> v3 = m_displ->get(v3u, v3v);
    Eigen::Matrix<DScalar, 3, 1> V2_V1;
    V2_V1 = v2 - p;
    Eigen::Matrix<DScalar, 3, 1> V3_V1;
    V3_V1 = v3 - p;
    // check if area is either inverted or smaller than certain A_hat
    DScalar area;
    area = (V2_V1.cross(V3_V1)).squaredNorm();
    // wmtk::logger().info("----current area {}", area.getValue());
    double A_hat = 1; // this is arbitrary now
    assert(A_hat > 0);
    if (area <= 0) {
        energy += std::numeric_limits<double>::infinity();
    }
    if (area < A_hat) {
        assert((area / A_hat) < 1.0);
        DScalar barrier_energy = -(area - A_hat) * (area - A_hat) * log(area / A_hat);
        // wmtk::logger().info("----current barrier energy {}", barrier_energy.getValue());
        energy += barrier_energy;
    }

    state.value = energy.getValue();
    state.gradient = energy.getGradient();
    state.hessian = energy.getHessian();
}

// use finite difference to calculate gradient and hessian
// use fixed tangent plane
void AMIPS3D::eval(State& state, DofsToPositions& dof_to_positions) const
{
    DiffScalarBase::setVariableCount(2);

    auto target_triangle = state.target_triangle;
    for (auto i = 0; i < 6; i++) target_triangle[i] = state.scaling * target_triangle[i];
    assert(state.two_opposite_vertices.rows() == 1);
    auto [x1, y1] = dof_to_positions.eval(state.dofx);

    //////// check if the uv triangle is flipped or degenrate
    // lambda function for checking if triangle is degenerate
    auto triangle_2d_area = [](Eigen::Vector2d A, Eigen::Vector2d B, Eigen::Vector2d C) -> double {
        Eigen::Matrix<double, 2, 1> B_A = B - A;
        Eigen::Matrix<double, 2, 1> C_A = C - A;
        double area = (0.5) * abs(B_A.x() * C_A.y() - B_A.y() * C_A.x());
        return area;
    };
    auto is_degenerate_lambda = [&]() -> bool {
        Eigen::Vector2d A = Eigen::Vector2d(x1.getValue(), y1.getValue());
        Eigen::Vector2d B =
            Eigen::Vector2d(state.two_opposite_vertices(0, 0), state.two_opposite_vertices(0, 1));
        Eigen::Vector2d C =
            Eigen::Vector2d(state.two_opposite_vertices(0, 2), state.two_opposite_vertices(0, 3));
        auto res = igl::predicates::orient2d(A, B, C);
        if (res != igl::predicates::Orientation::POSITIVE) {
            wmtk::logger().info("inverted or colinear vertices are {} {} {}", A, B, C);
            return true;
        } else {
            // if (triangle_2d_area(A, B, C) < 1e-10) return true;
            return false;
        }
    };
    assert(!is_degenerate_lambda());
    /////////

    const Eigen::Matrix<DScalar, 3, 1> v1 = m_displ->get(x1, y1);
    const DScalar v2u = DScalar(state.two_opposite_vertices(0, 0));
    const DScalar v2v = DScalar(state.two_opposite_vertices(0, 1));
    const Eigen::Matrix<DScalar, 3, 1> v2 = m_displ->get(v2u, v2v);
    const DScalar v3u = DScalar(state.two_opposite_vertices(0, 2));
    const DScalar v3v = DScalar(state.two_opposite_vertices(0, 3));
    const Eigen::Matrix<DScalar, 3, 1> v3 = m_displ->get(v3u, v3v);


    Eigen::Matrix<DScalar, 3, 1> V2_V1 = v2 - v1;
    Eigen::Matrix<DScalar, 3, 1> V3_V1 = v3 - v1;
    ////////////////////////
    // tangent bases
    Eigen::Matrix<DScalar, 3, 1> e1; // e1 = (V2 - V1).normalize()
    // e1 = V2_V1.stableNormalized();
    assert(V2_V1.norm() > 0); // check norm is not 0
    e1 = V2_V1 / V2_V1.norm();
    Eigen::Matrix<DScalar, 3, 1> n;
    n = V2_V1.cross(V3_V1);

    if (n.lpNorm<Eigen::Infinity>().getValue() < std::numeric_limits<double>::denorm_min()) {
        wmtk::logger().critical("n.lpNorm {}", n.lpNorm<Eigen::Infinity>().getValue());
        std::cout << "V1 " << std::endl;
        std::cout << std::hexfloat << x1 << " " << y1 << v1(2) << std::endl;
        std::cout << "V2 " << std::endl;
        std::cout << std::hexfloat << v2(0) << " " << v2(1) << v2(2) << std::endl;
        std::cout << "V3 " << std::endl;
        std::cout << std::hexfloat << v3(0) << " " << v3(1) << v3(2) << std::endl;
        assert(false);
    }
    // n = n.stableNormalized();
    assert(n.norm() > 0); // check norm is not 0
    n = n / n.norm();
    Eigen::Matrix<DScalar, 3, 1> e2;
    e2 = n.cross(e1);
    // Eigen::Matrix<DScalar, 3, 1> e2_stableNormalized = e2.stableNormalized();
    assert(e2.norm() > 0); // check norm is not 0
    e2 = e2 / e2.norm();


    // project V1, V2, V3 to tangent plane to VT1, VT2, VT3

    Eigen::Matrix<DScalar, 2, 1> VT1, VT2, VT3;
    VT1 << DScalar(0.), DScalar(0.); // the origin
    VT2 << V2_V1.dot(e1), DScalar(0.);
    VT3 << V3_V1.dot(e1), V3_V1.dot(e2);

    // now construct Dm as before in tangent plane
    // (x2 - x1, y2 - y1, x3 - x1, y2 - y1).transpose
    Eigen::Matrix<DScalar, 2, 2> Dm;
    Dm << VT2.x() - VT1.x(), VT3.x() - VT1.x(), VT2.y() - VT1.y(), VT3.y() - VT1.y();
    assert(Dm.determinant().getValue() > 0);
    // define of transform matrix F = Dm@Ds.inv
    Eigen::Matrix<DScalar, 2, 2> F;

    Eigen::Matrix2d Ds, Dsinv;
    Eigen::Vector2d target_A, target_B, target_C;
    target_A << target_triangle[0], target_triangle[1];
    target_B << target_triangle[2], target_triangle[3];
    target_C << target_triangle[4], target_triangle[5];
    Ds << target_B.x() - target_A.x(), target_C.x() - target_A.x(), target_B.y() - target_A.y(),
        target_C.y() - target_A.y();

    auto Dsdet = Ds.determinant();
    // no inversion
    assert(Dsdet > 0);
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
    assert(Fdet.getValue() > 0);
    if (std::abs(Fdet.getValue()) < std::numeric_limits<Scalar>::denorm_min()) {
        state.value = std::numeric_limits<double>::infinity();
        return;
    }

    DScalar AMIPS_function = (F.transpose() * F).trace() / Fdet;
    state.value = AMIPS_function.getValue();
    state.gradient = AMIPS_function.getGradient();
    state.hessian = AMIPS_function.getHessian();
}

} // namespace wmtk
