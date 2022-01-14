#pragma once
#include "Mesh.h"

namespace wmtk{
    double get_quality(TetMesh& mesh, int t_id); // why does it need the mesh? this could be separate I think
    double get_quality(TetMesh& mesh, const Vector3d& v1, const Vector3d& v2, const Vector3d& v3, const Vector3d& v4);

    //private:
    Scalar AMIPS_energy(const std::array<double, 12>& T);
    void AMIPS_jacobian(const std::array<double, 12>& T, Vector3d& result_0);
    void AMIPS_hessian(const std::array<double, 12>& T, Matrix3f& result_0);
}
