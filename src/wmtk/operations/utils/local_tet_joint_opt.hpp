#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace wmtk {
namespace operations {
namespace utils {

// Energy functor interface for local tet optimization
class SymmetricDirichletEnergy
{
public:
    SymmetricDirichletEnergy() = default;

    // E = 0.5*(||F||_F^2 + ||F^{-1}||_F^2)
    double operator()(const Eigen::Matrix3d& F, Eigen::Matrix3d* dE_dF) const;
};

// Pre-computed data for each tetrahedron
struct TetPrecomp
{
    Eigen::Matrix3d Xinv; // inverse of reference edge matrix
    double volume; // |det(X)| / 6 (optional weight)
};

// Pre-compute reference data for all tetrahedra
void precompute_reference(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    std::vector<TetPrecomp>& P);

// Compute energy and gradient for the entire mesh
template <
    typename DerivedVParam,
    typename DerivedT,
    typename EnergyFunctor = SymmetricDirichletEnergy>
double compute_energy_and_gradient_fast(
    const Eigen::MatrixBase<DerivedVParam>& V_param,
    const Eigen::MatrixBase<DerivedT>& T,
    const std::vector<TetPrecomp>& P,
    Eigen::MatrixXd& grad,
    const EnergyFunctor& energy_functor = EnergyFunctor());

} // namespace utils
} // namespace operations
} // namespace wmtk
