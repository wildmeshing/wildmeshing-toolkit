// This file is part of TriWild, a software for generating linear/curved triangle meshes.
//
// Copyright (C) 2019 Yixin Hu <yixin.hu@nyu.edu>
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include <Eigen/Dense>
#include <array>

namespace wmtk {
    double AMIPS2D_energy(const std::array<double, 6>& T);
    void AMIPS2D_jacobian(const std::array<double, 6>& T, Eigen::Vector2d& J);
    void AMIPS2D_hessian(const std::array<double, 6>& T, Eigen::Matrix2d& H);

    // TODO add support for features
    // double AMIPS2D_energy(const feature::FeatureElement &feature, const double t, const std::array<double, 6>& T);
    // double AMIPS2D_jacobian(const feature::FeatureElement &feature, const double t, const std::array<double, 6>& T);
    // double AMIPS2D_hessian(const feature::FeatureElement &feature, const double t, const std::array<double, 6>& T);
}