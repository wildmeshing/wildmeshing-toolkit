#include "TriangleAreaAD.hpp"

#include <wmtk/Primitive.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace wmtk::function {
TriangleAreaAD::TriangleAreaAD(
    const TriMesh& mesh,
    const attribute::MeshAttributeHandle& vertex_attribute_handle)
    : PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_attribute_handle)
{}

TriangleAreaAD::~TriangleAreaAD() = default;

using DScalar = typename PerSimplexAutodiffFunction::DScalar;
using DSVec2 = Eigen::Vector2<DScalar>;
using DSVec3 = Eigen::Vector3<DScalar>;

DScalar TriangleAreaAD::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(coords.size() == 3);

    if (domain_simplex.primitive_type() != PrimitiveType::Face)
        throw std::runtime_error("TriangleAreaAD only supports faces meshes");

    switch (embedded_dimension()) {
    case 2: {
        DSVec2 a = coords[0], b = coords[1], c = coords[2];
        return wmtk::utils::triangle_signed_2d_area(a, b, c);
    }
    case 3: {
        DSVec3 a = coords[0], b = coords[1], c = coords[2];
        return wmtk::utils::triangle_3d_area(a, b, c);
    }
    default: throw std::runtime_error("TriangleAreaAD only supports 2D and 3D meshes");
    }
}

} // namespace wmtk::function
