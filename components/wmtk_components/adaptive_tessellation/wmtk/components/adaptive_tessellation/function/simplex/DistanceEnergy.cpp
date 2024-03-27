#include "DistanceEnergy.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
DistanceEnergy::DistanceEnergy(
    const Mesh& mesh,
    const wmtk::attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::IntegralBase> integral_ptr,
    double weight)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_integral_ptr(integral_ptr)
    , m_weight(weight)
{}

DistanceEnergy::~DistanceEnergy() = default;
DScalar DistanceEnergy::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);

    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    return m_weight * m_integral_ptr->average_area_integral_over_triangle<DScalar>(a, b, c);
}

} // namespace wmtk::function
