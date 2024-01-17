#include "PositionMapAMIPS.hpp"
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
PositionMapAMIPS::PositionMapAMIPS(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        pos_evaluator_ptr,
    const image::SAMPLING_METHOD sampling_method)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator_ptr(pos_evaluator_ptr)
{}
PositionMapAMIPS::~PositionMapAMIPS() = default;

DScalar PositionMapAMIPS::eval(
    const simplex::Simplex& domain_simplex,
    const std::vector<DSVec>& coords) const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);

    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    DSVec3 p0 = m_pos_evaluator_ptr->uv_to_position(a);
    DSVec3 p1 = m_pos_evaluator_ptr->uv_to_position(b);
    DSVec3 p2 = m_pos_evaluator_ptr->uv_to_position(c);

    return utils::amips(p0, p1, p2);
}
} // namespace wmtk::function