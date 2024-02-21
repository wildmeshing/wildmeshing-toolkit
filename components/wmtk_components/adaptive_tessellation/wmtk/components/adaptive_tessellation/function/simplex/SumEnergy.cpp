#include "SumEnergy.hpp"
#include <wmtk/components/adaptive_tessellation/function/utils/AnalyticalFunctionTriangleQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/area_barrier.hpp>
#include <wmtk/function/utils/amips.hpp>
#include <wmtk/utils/triangle_areas.hpp>

namespace image = wmtk::components::image;

namespace wmtk::function {
SumEnergy::SumEnergy(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        pos_evaluator,
    std::shared_ptr<wmtk::components::function::utils::IntegralBase> integral_ptr,
    const double distance_energy_weight,
    std::shared_ptr<wmtk::function::PositionMapAMIPS> amips_energy,
    const image::SAMPLING_METHOD sampling_method)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator_ptr(pos_evaluator)
    , m_integral_ptr(integral_ptr)
    , m_distance_energy_weight(distance_energy_weight)
    , m_3d_amips_energy(amips_energy)
{
    logger().warn("SumEnergy:: distance_energy_weight {}", m_distance_energy_weight);
    if (m_3d_amips_energy) {
        logger().warn("SumEnergy:: 3d amips with barrier is ON ");
    } else {
        logger().warn("SumEnergy:: 3d amips with barrier is OFF ");
    }
}
SumEnergy::~SumEnergy() = default;

DScalar SumEnergy::eval(const simplex::Simplex& domain_simplex, const std::vector<DSVec>& coords)
    const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);
    DSVec2 a = coords[0], b = coords[1], c = coords[2];

    DScalar energy =
        m_distance_energy_weight * m_integral_ptr->get_error_one_triangle_exact(a, b, c);
    if (m_3d_amips_energy) {
        energy += m_3d_amips_energy->eval(domain_simplex, coords);
    }

    return energy;
}
} // namespace wmtk::function