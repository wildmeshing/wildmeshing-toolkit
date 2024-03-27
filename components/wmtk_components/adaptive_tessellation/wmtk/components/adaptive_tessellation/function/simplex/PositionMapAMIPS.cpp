#include "PositionMapAMIPS.hpp"
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionNumericalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
PositionMapAMIPS::PositionMapAMIPS(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        pos_evaluator_ptr,
    double amips_weight,
    bool amips_area_weighted,
    double barrier_weight,
    double barrier_area)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator_ptr(pos_evaluator_ptr)
    , m_amips_weight(amips_weight)
    , m_amips_area_weighted(amips_area_weighted)
    , m_barrier_weight(barrier_weight)
    , m_barrier_area(barrier_area)
{
    logger().warn("PositionMapAMIPS::PositionMapAMIPS amips weighted {}", m_amips_weight);

    if (m_barrier_weight != 0.) {
        logger().warn("PositionMapAMIPS includes barrier weights {}", m_barrier_weight);
        logger().warn("                                with area {}", m_barrier_area);
    } else {
        logger().warn("PositionMapAMIPS NO barrier weights");
    }

    if (m_amips_area_weighted) {
        logger().warn("PositionMapAMIPS 2d area weighted amips");
    } else {
        logger().warn("PositionMapAMIPS NO 2d area weighted amips");
    }
}
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
    DScalar barrier =
        m_barrier_weight * wmtk::function::utils::area_barrier(a, b, c, m_barrier_area);
    DScalar amips = m_amips_weight * utils::amips(p0, p1, p2);
    if (m_amips_area_weighted) {
        amips = amips * wmtk::utils::triangle_unsigned_2d_area(a, b, c);
    }
    return amips + barrier;
}
} // namespace wmtk::function