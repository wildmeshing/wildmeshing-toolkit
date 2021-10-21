//
// Created by Yixin Hu on 10/12/21.
//

#pragma once
#include "Mesh.h"

namespace wmtk{
    double get_quality(TetMesh& mesh, int t_id);
    double get_quality(TetMesh& mesh, const Vector3f& v1, const Vector3f& v2, const Vector3f& v3, const Vector3f& v4);

    //private:
    Scalar AMIPS_energy(const std::array<double, 12>& T);
    void AMIPS_jacobian(const std::array<double, 12>& T, Vector3f& result_0);
    void AMIPS_hessian(const std::array<double, 12>& T, Matrix3f& result_0);
}
