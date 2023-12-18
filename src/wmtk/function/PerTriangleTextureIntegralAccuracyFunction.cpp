#include "PerTriangleTextureIntegralAccuracyFunction.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/amips.hpp>
#include "utils/TextureIntegral.hpp"

namespace image = wmtk::components::adaptive_tessellation::image;

namespace wmtk::function {
PerTriangleTextureIntegralAccuracyFunction::PerTriangleTextureIntegralAccuracyFunction(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const std::array<image::Image, 3>& images,
    const image::SAMPLING_METHOD sampling_method,
    const image::IMAGE_WRAPPING_MODE wrapping_mode)
    : TriangleAutodiffFunction(mesh, vertex_uv_handle)
    , m_pos_evaluator(images, sampling_method, wrapping_mode)
{}

PerTriangleTextureIntegralAccuracyFunction::~PerTriangleTextureIntegralAccuracyFunction() = default;

DScalar PerTriangleTextureIntegralAccuracyFunction::eval(
    const simplex::Simplex& domain_simplex,
    const std::array<DSVec, 3>& coords) const
{
    assert(embedded_dimension() == 2);
    utils::TextureIntegral texture_integral(m_pos_evaluator);
    DSVec2 a = coords[0], b = coords[1], c = coords[2];
    return texture_integral.get_error_one_triangle_exact(a, b, c);
}

} // namespace wmtk::function
