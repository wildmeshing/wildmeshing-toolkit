/*  This is an example on how automatic differentiation can be used in
    a multidimensional Newton iteration, which eliminates need for
    (often painful) symbolic derivatives.

    Copyright (c) 2009 by Wenzel Jakob

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <iostream>
#include "Logger.hpp"
#include "autodiff.h"
// DECLARE_DIFFSCALAR_BASE();
namespace wmtk {

/**
energy is wrt triangle, G and H wrt first vert of the triagle
*/
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> AMIPS_autodiff(const std::array<double, 6>& T);
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> AMIPS_autodiff_customize_target(
    const std::array<double, 6>& T1,
    const std::array<double, 6>& T2,
    int i = 0);
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> SymDi_autodiff(const std::array<double, 6>& T);
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> SymDi_autodiff_customize_target(
    const std::array<double, 6>& T1,
    const std::array<double, 6>& T2,
    int i = 0);
} // namespace wmtk