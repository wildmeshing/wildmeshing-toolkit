#include "PositionMapAMIPS2D.hpp"
#include <wmtk/Types.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace wmtk::function {
PositionMapAMIPS2D::PositionMapAMIPS2D(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const image::Image& image)
    : TriangleAMIPS(mesh, vertex_uv_handle)
    , m_pos_evaluator(image)
{}

PositionMapAMIPS2D::PositionMapAMIPS2D(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const wmtk::image::SamplingAnalyticFunction::FunctionType type,
    const double a,
    const double b,
    const double c)
    : TriangleAMIPS(mesh, vertex_uv_handle)
    , m_pos_evaluator(type, a, b, c)
{}


auto PositionMapAMIPS2D::get_value_autodiff(const Tuple& simplex) const -> DScalar
{
    // get_autodiff_value sets the autodiff size if necessary
    // get the uv coordinates of the triangle
    ConstAccessor<double> pos = mesh().create_const_accessor(get_coordinate_attribute_handle());

    const Tuple& tuple = simplex;
    auto tuple_value = pos.const_vector_attribute(tuple);

    Vector2<DScalar> uv0;
    uv0 = utils::as_DScalar<DScalar>(tuple_value);

    constexpr static PrimitiveType PV = PrimitiveType::Vertex;
    constexpr static PrimitiveType PE = PrimitiveType::Edge;

    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh().switch_tuples(tuple, {PE, PV}));
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh().switch_tuples(tuple, {PV, PE}));

    Vector3<DScalar> pos0 = m_pos_evaluator.uv_to_pos(uv0);
    Eigen::Vector3d pos1 = m_pos_evaluator.uv_to_pos(uv1);
    Eigen::Vector3d pos2 = m_pos_evaluator.uv_to_pos(uv2);

    return utils::amips(pos0, pos1, pos2);
}

} // namespace wmtk::function
