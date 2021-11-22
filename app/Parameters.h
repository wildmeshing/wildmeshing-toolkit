//
// Created by Yixin Hu on 10/12/21.
//

#pragma once
#include "common.h"

namespace tetwild
{
	struct Parameters {
        double epsr;
        double eps;
        double lr;
        double l;
        double diag_l;
        Vector3f min, max;

        double splitting_l2;
        double collapsing_l2;

        void init() {
            splitting_l2 = l * l * (16 / 9.);
            collapsing_l2 = l * l * (16 / 25.);
        }
    };
} // namespace tetwild
