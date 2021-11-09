//
// Created by Yixin Hu on 11/9/21.
//

#ifndef FLOATTETWILD_AMIPS_H
#define FLOATTETWILD_AMIPS_H

#include "common.h"

namespace wmtk{
    Scalar AMIPS_energy(const std::array<Scalar, 12>& T);
    void AMIPS_jacobian(const std::array<Scalar, 12>& T, Vector3& result_0);
    void AMIPS_hessian(const std::array<Scalar, 12>& T, Matrix3& result_0);
}


#endif //FLOATTETWILD_AMIPS_H
