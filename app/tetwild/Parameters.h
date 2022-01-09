//
// Created by Yixin Hu on 10/12/21.
//

#pragma once
#include "common.h"

namespace tetwild {
struct Parameters
{
    double epsr = 1e-3; // relative error bound (wrt diagonal)
    double eps = -1.; // absolute error bound
    double lr = 5e-2; // target edge length (relative)
    double l = -1.;
    double diag_l = -1.;
    Vector3f min = Vector3f::Zero();
    Vector3f max = Vector3f::Ones();

    double splitting_l2 = -1.; // the lower bound length (squared) for edge split
    double collapsing_l2 = std::numeric_limits<double>::max(); // the upper bound length (squared) for edge collapse

    void init(const Vector3d& min_, const Vector3d& max_)
    {
        min = min_;
        max = max_;
        diag_l = (max - min).norm();
        if (l > 0)
            lr = l / diag_l;
        else
            l = lr * diag_l;
        splitting_l2 = l * l * (16 / 9.);
        collapsing_l2 = l * l * (16 / 25.);

        if (eps > 0)
            epsr = eps / diag_l;
        else
            eps = epsr * diag_l;
    }
};
} // namespace tetwild
