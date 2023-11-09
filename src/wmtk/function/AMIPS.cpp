#include "AMIPS.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/amips.hpp>
namespace wmtk::function {
AMIPS::AMIPS(const TriMesh& mesh, const MeshAttributeHandle<double>& vertex_attribute_handle)
    : TriangleAutodiffFunction(mesh, vertex_attribute_handle)
{}

AMIPS::~AMIPS() = default;

using DScalar = typename AutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;
DScalar AMIPS::eval(DSVec& coordinate0, DSVec& coordinate1, DSVec& coordinate2) const
{
    if (embedded_dimension() != 2 && embedded_dimension() != 3) {
        throw std::runtime_error("AMIPS only supports 2D and 3D meshes");
    }
    if (embedded_dimension() == 2) {
        coordinate0.conservativeResize(2);
        DSVec2 coordinate0_2d = coordinate0;
        coordinate1.conservativeResize(2);
        DSVec2 coordinate1_2d = coordinate1;
        coordinate2.conservativeResize(2);
        DSVec2 coordinate2_2d = coordinate2;
        return utils::amips(coordinate0_2d, coordinate1_2d, coordinate2_2d);
    } else {
        coordinate0.conservativeResize(3);
        coordinate1.conservativeResize(3);
        coordinate2.conservativeResize(3);
        DSVec3 coordinate0_3d = coordinate0, coordinate1_3d = coordinate1,
               coordinate2_3d = coordinate2;
        return utils::amips(coordinate0_3d, coordinate1_3d, coordinate2_3d);
    }
}

} // namespace wmtk::function
