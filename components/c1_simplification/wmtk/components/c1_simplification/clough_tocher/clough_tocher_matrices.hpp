#pragma once
#include <wmtk/Types.hpp>

std::array<Eigen::Matrix<double, 3, 3>, 3> CT_subtri_bound_matrices();
std::array<Eigen::Matrix<double, 10, 12>, 3> CT_subtri_matrices();
