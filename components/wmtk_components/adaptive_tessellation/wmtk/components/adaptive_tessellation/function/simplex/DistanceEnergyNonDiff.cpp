#include "DistanceEnergyNonDiff.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/TextureIntegral.hpp>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/SimplexGetter.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace wmtk::function::utils {
template <int64_t NV, int64_t DIM>
std::array<double, NV * DIM> unbox(
    const std::vector<std::decay_t<typename attribute::ConstMapResult<double>>>& data,
    const int64_t index)
{
    std::array<double, NV * DIM> res;
    assert(data.size() == NV);

    const size_t start = index < 0 ? 0 : index;

    for (size_t i = 0; i < NV; ++i) {
        const size_t ii = (i + start) % NV;
        assert(data[ii].size() == DIM);

        for (size_t j = 0; j < DIM; ++j) {
            res[DIM * i + j] = data[ii][j];
        }
    }

    return res;
}
} // namespace wmtk::function::utils

namespace image = wmtk::components::image;

namespace wmtk::function {
DistanceEnergyNonDiff::DistanceEnergyNonDiff(
    const Mesh& mesh,
    const wmtk::attribute::MeshAttributeHandle& vertex_uv_handle,
    std::shared_ptr<wmtk::components::function::utils::IntegralBase> integral_ptr,
    double weight)
    : wmtk::function::PerSimplexFunction(mesh, PrimitiveType::Vertex, vertex_uv_handle)
    , m_integral_ptr(integral_ptr)
    , m_weight(weight)
{}

DistanceEnergyNonDiff::~DistanceEnergyNonDiff() = default;
double DistanceEnergyNonDiff::get_value(const simplex::Simplex& domain_simplex) const
{
    assert(embedded_dimension() == 2);
    wmtk::attribute::Accessor<double> accessor =
        mesh().create_const_accessor(attribute_handle().as<double>());
    auto [attrs, index] = utils::get_simplex_attributes(
        mesh(),
        accessor,
        m_primitive_type,
        domain_simplex,
        std::optional<Tuple>());
    auto uvs = utils::unbox<3, 2>(attrs, index);
    Eigen::Vector2d a(uvs[0], uvs[1]);
    Eigen::Vector2d b(uvs[2], uvs[3]);
    Eigen::Vector2d c(uvs[4], uvs[5]);
    return m_weight * m_integral_ptr->get_error_one_triangle_exact(a, b, c);
}

} // namespace wmtk::function
