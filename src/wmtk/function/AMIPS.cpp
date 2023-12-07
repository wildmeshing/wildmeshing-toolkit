#include "AMIPS.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/amips.hpp>
namespace wmtk::function {
AMIPS::AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : TriangleAutodiffFunction(mesh, vertex_attribute_handle)
{}

AMIPS::~AMIPS() = default;

using DScalar = typename AutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;
DScalar AMIPS::eval(const simplex::Simplex& domain_simplex, const std::array<DSVec, 3>& coords)
    const
{
    switch (embedded_dimension()) {
    case 2: {
        DSVec2 a = coords[0], b = coords[1], c = coords[2];
        return utils::amips(a, b, c);
    }
    case 3: {
        DSVec3 a = coords[0], b = coords[1], c = coords[2];
        return utils::amips(a, b, c);
    }
    default: throw std::runtime_error("AMIPS only supports 2D and 3D meshes");
    }
}

} // namespace wmtk::function
