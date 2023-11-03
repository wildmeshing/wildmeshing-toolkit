#pragma once
#include "common.h"

namespace wmtk{
    struct Parameters{
        double epsr;
        double eps;
        double lr;
        double l;
        double diag_l;
        Vector3f min, max;
    };
}

