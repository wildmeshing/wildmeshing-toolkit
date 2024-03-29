#include "TriangleAMIPS.hpp"

#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace wmtk::function {
TriangleAMIPS::TriangleAMIPS(
    const TriMesh& mesh,
    const attribute::MeshAttributeHandle& vertex_attribute_handle)
    : PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_attribute_handle)
{}

TriangleAMIPS::~TriangleAMIPS() = default;

using DScalar = typename PerSimplexAutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;

DScalar TriangleAMIPS::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(coords.size() == 3);

    assert(
        domain_simplex.primitive_type() ==
        PrimitiveType::Triangle); // "TriangleAMIPS only supports faces meshes"

    switch (embedded_dimension()) {
    case 2: {
        DSVec2 a = coords[0], b = coords[1], c = coords[2];
        return utils::amips(a, b, c);
    }
    case 3: {
        DSVec3 a = coords[0], b = coords[1], c = coords[2];
        return utils::amips(a, b, c);
    }
    default: assert(false); // "TriangleAMIPS only supports 2D and 3D meshes"
    }

    return DScalar();
}

} // namespace wmtk::function
