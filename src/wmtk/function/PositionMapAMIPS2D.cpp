#include "PositionMapAMIPS2D.hpp"
#include <wmtk/Types.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>
#include <wmtk/function/utils/amips.hpp>

namespace wmtk::function {
PositionMapAMIPS2D::PositionMapAMIPS2D(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const image::Image& image)
    : AMIPS(mesh, vertex_uv_handle)
    , m_pos_evaluator(image)
{}

PositionMapAMIPS2D::PositionMapAMIPS2D(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const image::SamplingAnalyticFunction::FunctionType type,
    const double a,
    const double b,
    const double c)
    : AMIPS(mesh, vertex_uv_handle)
    , m_pos_evaluator(type, a, b, c)
{}

DScalar PositionMapAMIPS2D::eval(
    const Simplex& domain_simplex,
    const std::array<DSVec, 3>& coordinates) const
{
    // assert the coordinates are uv coordinates
    assert(domain_simplex.dimension() == 2);

    DSVec2 a = coordinates[0], b = coordinates[1], c = coordinates[2];
    DSVec3 pos0 = m_pos_evaluator.uv_to_pos(a);
    DSVec3 pos1 = m_pos_evaluator.uv_to_pos(b);
    DSVec3 pos2 = m_pos_evaluator.uv_to_pos(c);
    return utils::amips(pos0, pos1, pos2);
}

} // namespace wmtk::function
