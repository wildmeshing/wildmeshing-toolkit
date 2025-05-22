#include "local_tet_joint_opt.hpp"
#include <fstream>
#include <iostream>

namespace wmtk {
namespace operations {
namespace utils {

// Implementation of SymmetricDirichletEnergy operator
inline double SymmetricDirichletEnergy::operator()(const Eigen::Matrix3d& F, Eigen::Matrix3d* dE_dF)
    const
{
    const Eigen::Matrix3d Finv = F.inverse();
    const double energy = 0.5 * (F.squaredNorm() + Finv.squaredNorm());
    if (dE_dF) {
        *dE_dF = F - Finv.transpose();
    }
    return energy;
}

// Implementation of precompute_reference
void precompute_reference(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    std::vector<TetPrecomp>& P)
{
    const int m = static_cast<int>(T.rows());
    P.resize(m);
    for (int ti = 0; ti < m; ++ti) {
        const int i0 = T(ti, 0), i1 = T(ti, 1), i2 = T(ti, 2), i3 = T(ti, 3);
        Eigen::Matrix3d X;
        X.col(0) = V.row(i1) - V.row(i0);
        X.col(1) = V.row(i2) - V.row(i0);
        X.col(2) = V.row(i3) - V.row(i0);
        TetPrecomp pc;
        pc.Xinv = X.inverse();
        pc.volume = std::abs(X.determinant()) / 6.0;
        P[ti] = pc;
    }
}

// Implementation of compute_energy_and_gradient_fast
template <typename DerivedVParam, typename DerivedT, typename EnergyFunctor>
double compute_energy_and_gradient_fast(
    const Eigen::MatrixBase<DerivedVParam>& V_param,
    const Eigen::MatrixBase<DerivedT>& T,
    const std::vector<TetPrecomp>& P,
    Eigen::MatrixXd& grad,
    const EnergyFunctor& energy_functor)
{
    const int n = static_cast<int>(V_param.rows());
    grad.setZero(n, 3);

    double total_E = 0.0;
    const int m = static_cast<int>(T.rows());

    for (int ti = 0; ti < m; ++ti) {
        const int i0 = T(ti, 0), i1 = T(ti, 1), i2 = T(ti, 2), i3 = T(ti, 3);
        const TetPrecomp& pc = P[ti];

        // Current edge matrix Xp
        Eigen::Matrix3d Xp;
        Xp.col(0) = V_param.row(i1) - V_param.row(i0);
        Xp.col(1) = V_param.row(i2) - V_param.row(i0);
        Xp.col(2) = V_param.row(i3) - V_param.row(i0);

        const Eigen::Matrix3d F = Xp * pc.Xinv; // deformation gradient

        Eigen::Matrix3d dE_dF;
        const double Ei = energy_functor(F, &dE_dF);
        total_E += Ei;

        const Eigen::Matrix3d dE_dXp = dE_dF * pc.Xinv.transpose();

        // Scatter to vertex gradients
        grad.row(i1) += dE_dXp.col(0).transpose();
        grad.row(i2) += dE_dXp.col(1).transpose();
        grad.row(i3) += dE_dXp.col(2).transpose();
        grad.row(i0) -= (dE_dXp.col(0) + dE_dXp.col(1) + dE_dXp.col(2)).transpose();
    }
    return total_E / m;
}

void local_tet_joint_opt(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T_before,
    const Eigen::MatrixXi& T_after,
    Eigen::MatrixXd& V_param,
    const std::vector<int>& constraint_vids)
{
    // Precompute reference data for all tetrahedra
    std::vector<TetPrecomp> P;
    precompute_reference(V, T_before, P);
    Eigen::MatrixXd grad;
    double energy =
        compute_energy_and_gradient_fast(V_param, T_before, P, grad, SymmetricDirichletEnergy());
    std::cout << "energy: " << energy << std::endl;
    std::cout << "grad: \n" << grad << std::endl;
    // Set z-axis gradient to zero for all constraint vertices
    for (const int vid : constraint_vids) {
        if (vid >= 0 && vid < grad.rows()) {
            // Zero out the z-component (third column) of the gradient
            grad(vid, 2) = 0.0;
        } else {
            std::cout << "vid: " << vid << " is out of range" << std::endl;
        }
    }

    std::cout << "After zeroing z-gradient for constraint vertices:" << std::endl;
    std::cout << "grad: \n" << grad << std::endl;
    // Compute energy and gradient for the entire mesh
}

// Explicit template instantiations
template double
compute_energy_and_gradient_fast<Eigen::MatrixXd, Eigen::MatrixXi, SymmetricDirichletEnergy>(
    const Eigen::MatrixBase<Eigen::MatrixXd>& V_param,
    const Eigen::MatrixBase<Eigen::MatrixXi>& T,
    const std::vector<TetPrecomp>& P,
    Eigen::MatrixXd& grad,
    const SymmetricDirichletEnergy& energy_functor);

} // namespace utils
} // namespace operations
} // namespace wmtk
