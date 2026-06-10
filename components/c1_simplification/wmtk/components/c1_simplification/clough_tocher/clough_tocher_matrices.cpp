#include "clough_tocher_matrices.hpp"
#include "clough_tocher_autogen_matrix_coeffs.hpp"


std::array<Eigen::Matrix<double, 3, 3>, 3> CT_subtri_bound_matrices()
{
    double bound_coeffs[3][3][3];
    CT_interpolant::CT_tri_bounds_coeffs(bound_coeffs);

    std::array<Eigen::Matrix<double, 3, 3>, 3> CT_bound_coeffs;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                CT_bound_coeffs[i](j, k) = bound_coeffs[i][j][k];
            }
        }
    }

    return CT_bound_coeffs;
}

std::array<Eigen::Matrix<double, 10, 12>, 3> CT_subtri_matrices()
{
    double CT_coeffs[3][10][12];
    CT_interpolant::CT_sub_tri_matrices(CT_coeffs);

    std::array<Eigen::Matrix<double, 10, 12>, 3> CT_matrices;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 10; ++j) {
            for (int k = 0; k < 12; ++k) {
                CT_matrices[i](j, k) = CT_coeffs[i][j][k];
            }
        }
    }

    return CT_matrices;
}
