#include "SYMDIR.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/symdir.hpp>
namespace wmtk::function {
SYMDIR::SYMDIR(const TriMesh& uv_mesh, const MeshAttributeHandle<double>& uv_attribute_handle)
    : TriangleAutodiffFunction(uv_mesh, uv_attribute_handle)
{
    //  TODO: make reference to be the equilateral triangle
    A << DScalar(0.), DScalar(0.), DScalar(0.);
    B << DScalar(1.), DScalar(0.), DScalar(0.);
    C << DScalar(0.5), DScalar(sqrt(3) / 2.), DScalar(0.);
}

SYMDIR::SYMDIR(
    const TriMesh& mesh,
    const TriMesh& uv_mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle,
    const MeshAttributeHandle<double>& uv_attribute_handle)
    : TriangleAutodiffFunction(uv_mesh, uv_attribute_handle)
{
    // TODO:???? make reference from mesh and vertex_attribute_handle
    A << DScalar(0.), DScalar(0.), DScalar(0.);
    B << DScalar(1.), DScalar(0.), DScalar(0.);
    C << DScalar(0.5), DScalar(sqrt(3) / 2.), DScalar(0.);
}

SYMDIR::~SYMDIR() = default;

using DScalar = typename AutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
DScalar SYMDIR::eval(const simplex::Simplex& domain_simplex, const std::array<DSVec, 3>& coords)
    const
{
    switch (embedded_dimension()) {
    case 2: {
        DSVec2 a = coords[0], b = coords[1], c = coords[2];
        return utils::symdir(A, B, C, a, b, c);
    }
    default: throw std::runtime_error("Symmetric Dirichlet energy is only defined in 2d");
    }
}

} // namespace wmtk::function
