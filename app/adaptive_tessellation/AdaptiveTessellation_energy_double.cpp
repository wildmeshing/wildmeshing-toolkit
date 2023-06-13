#include <igl/read_triangle_mesh.h>
#include <igl/writeDMAT.h>
#include <igl/write_triangle_mesh.h>
#include "AdaptiveTessellation.h"
using namespace adaptive_tessellation;
using namespace wmtk;

double AdaptiveTessellation::get_amips3d_error_for_face(
    const Eigen::Vector2d& A,
    const Eigen::Vector2d& B,
    const Eigen::Vector2d& C) const
{
    const Eigen::Matrix<double, 3, 1>& p1 = mesh_parameters.m_displacement->get(A(0), A(1));
    const Eigen::Matrix<double, 3, 1>& p2 = mesh_parameters.m_displacement->get(B(0), B(1));
    const Eigen::Matrix<double, 3, 1>& p3 = mesh_parameters.m_displacement->get(C(0), C(1));
    return wmtk::amips3d_error(p1, p2, p3);
}