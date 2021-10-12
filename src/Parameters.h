//
// Created by Yixin Hu on 10/12/21.
//

#ifndef WILDMESHING_TOOLKIT_PARAMETERS_H
#define WILDMESHING_TOOLKIT_PARAMETERS_H
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

#endif //WILDMESHING_TOOLKIT_PARAMETERS_H
