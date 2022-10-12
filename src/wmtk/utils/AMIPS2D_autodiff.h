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

#include <Eigen/Cholesky>
#include <iostream>
#include "autodiff.h"

DECLARE_DIFFSCALAR_BASE();
/**
energy is wrt triangle, G and H wrt first vert of the triagle
*/
DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> AMIPS_autodiff(const std::array<double, 6>& T)
{
    // typedef Eigen::Vector2d Gradient; // wrt one vert
    // typedef Eigen::Matrix2d Hessian; // wrt one vert
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