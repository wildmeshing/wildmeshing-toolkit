//
// Created by Yixin Hu on 10/12/21.
//

#pragma once
#include "common.h"

namespace tetwild {
struct Parameters
{
    double epsr = 1 / 1000.;
    double eps;
    double lr = 1 / 20.;
    double l = -1;
    double diag_l;
    Vector3d min, max;

    double splitting_l2;
    double collapsing_l2;

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
