#include "PerTriangleTextureIntegralAccuracyFunction.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureMapAvgDistanceToLimit.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
PerTriangleTextureIntegralAccuracyFunction::PerTriangleTextureIntegralAccuracyFunction(
    const Mesh& mesh,
    const wmtk::attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        pos_evaluator_ptr)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator_ptr(pos_evaluator_ptr)
{}

PerTriangleTextureIntegralAccuracyFunction::~PerTriangleTextureIntegralAccuracyFunction() = default;

DScalar PerTriangleTextureIntegralAccuracyFunction::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);
    wmtk::components::function::utils::TextureMapAvgDistanceToLimit texture_integral(
        *m_pos_evaluator_ptr);
    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    return texture_integral.distance(a, b, c);
}

} // namespace wmtk::function
