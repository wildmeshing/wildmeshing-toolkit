#include "PerTriangleAnalyticalIntegral.hpp"
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>

namespace image = wmtk::components::adaptive_tessellation::image;

namespace wmtk::function {
PerTriangleAnalyticalIntegral::PerTriangleAnalyticalIntegral(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& vertex_uv_handle,
    const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs,
    const image::SAMPLING_METHOD sampling_method)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator(funcs, sampling_method)
{}

DScalar PerTriangleAnalyticalIntegral::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);
    wmtk::components::adaptive_tessellation::function::utils::AnalyticalFunctionTriangleQuadrature
        analytical_quadrature(m_pos_evaluator);
    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    return analytical_quadrature.get_error_one_triangle_exact(a, b, c);
}
} // namespace wmtk::function