#include "PerTriangleAnalyticalIntegral.hpp"
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
PerTriangleAnalyticalIntegral::PerTriangleAnalyticalIntegral(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        pos_evaluator_ptr)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator_ptr(pos_evaluator_ptr)
{}
PerTriangleAnalyticalIntegral::~PerTriangleAnalyticalIntegral() = default;

DScalar PerTriangleAnalyticalIntegral::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);
    wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
        *m_pos_evaluator_ptr);
    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    return analytical_quadrature.get_error_one_triangle_exact(a, b, c);
}
} // namespace wmtk::function