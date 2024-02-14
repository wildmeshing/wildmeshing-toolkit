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
    wmtk::components::function::utils::ThreeChannelPositionMapEvaluator pos_evaluator,
    std::shared_ptr<wmtk::components::function::utils::IntegralBase> integral_ptr,
    const double barrier_weight,
    const double barrier_area_constant,
    const double quadrature_weight,
    const double amips_weight,
    const bool amips_area_weighted,
    const image::SAMPLING_METHOD sampling_method)
    : wmtk::function::PerSimplexAutodiffFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_pos_evaluator(pos_evaluator)
    , m_integral_ptr(integral_ptr)
    , m_barrier_weight(barrier_weight)
    , m_barrier_area(barrier_area_constant)
    , m_quadrature_weight(quadrature_weight)
    , m_amips_weight(amips_weight)
    , m_amips_area_weighted(amips_area_weighted)
{
    std::cout << "SumEnergy::SumEnergy mbarrier_weight " << m_barrier_weight << std::endl;
    std::cout << "SumEnergy::SumEnergy mbarrier_area " << m_barrier_area << std::endl;
    std::cout << "SumEnergy::SumEnergy mquadrature_weight " << m_quadrature_weight << std::endl;
    std::cout << "SumEnergy::SumEnergy mamips_weight " << m_amips_weight << std::endl;
    std::cout << "SumEnergy::SumEnergy mamips_area_weighted " << m_amips_area_weighted << std::endl;
}
SumEnergy::~SumEnergy() = default;

DScalar SumEnergy::eval(const simplex::Simplex& domain_simplex, const std::vector<DSVec>& coords)
    const
{
    assert(embedded_dimension() == 2);
    assert(coords.size() == 3);
    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    DSVec3 p0 = m_pos_evaluator.uv_to_position(a);
    DSVec3 p1 = m_pos_evaluator.uv_to_position(b);
    DSVec3 p2 = m_pos_evaluator.uv_to_position(c);
    DScalar barrier =
        m_barrier_weight * wmtk::function::utils::area_barrier(a, b, c, m_barrier_area);
    DScalar quadrature =
        m_quadrature_weight * m_integral_ptr->get_error_one_triangle_exact(a, b, c);
    DScalar amips = m_amips_weight * utils::amips(p0, p1, p2);
    if (m_amips_area_weighted) {
        amips = amips * wmtk::utils::triangle_3d_area(p0, p1, p2);
    }

    return barrier + quadrature + amips;
}
} // namespace wmtk::function