#include "SumEnergy.hpp"
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
SumEnergy::SumEnergy(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& vertex_uv_handle,
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator pos_evaluator,
    const double lambda,
    const image::SAMPLING_METHOD sampling_method)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator(pos_evaluator)
    , m_lambda(lambda)
{}
SumEnergy::~SumEnergy() = default;

DScalar SumEnergy::eval(const simplex::Simplex& domain_simplex, const std::vector<DSVec>& coords)
    const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);
    wmtk::components::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
        m_pos_evaluator);

    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    // DSVec3 p0 = m_pos_evaluator.uv_to_position(a);
    // DSVec3 p1 = m_pos_evaluator.uv_to_position(b);
    // DSVec3 p2 = m_pos_evaluator.uv_to_position(c);
    DScalar amips = m_lambda * utils::amips(a, b, c);
    DScalar quadrature =
        (1 - m_lambda) * analytical_quadrature.get_error_one_triangle_exact(a, b, c);
    return amips + quadrature;
}
} // namespace wmtk::function