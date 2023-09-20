#include "VertexSmoothUsingDifferentiableEnergy.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/utils/Optimization.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/triangle_helper_functions.hpp>
#include "VertexSmooth.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>::initialize_invariants(
    const TriMesh& m)
{
    smooth_settings.initialize_invariants(m);
    smooth_settings.invariants.add(
        std::make_shared<TriangleInversionInvariant>(m, smooth_settings.position));
}
} // namespace wmtk::operations

namespace wmtk::operations::tri_mesh {
VertexSmoothUsingDifferentiableEnergy::VertexSmoothUsingDifferentiableEnergy(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexSmooth(m, t, settings.smooth_settings)
    , m_uv_pos_accessor{m.create_accessor<double>(settings.smooth_settings.position)}
    , m_settings{settings}
{}

std::string VertexSmoothUsingDifferentiableEnergy::name() const
{
    return "tri_mesh_vertex_smooth_using_differentiable_energy";
}

bool VertexSmoothUsingDifferentiableEnergy::before() const
{
    if (!mesh().is_valid_slow(input_tuple())) {
        return false;
    }
    return true;
}

bool VertexSmoothUsingDifferentiableEnergy::execute()
{
    const Eigen::Vector2d p = m_uv_pos_accessor.vector_attribute(input_tuple());
    OperationSettings<tri_mesh::VertexSmooth> op_settings;
    tri_mesh::VertexSmooth smooth_op(mesh(), input_tuple(), m_settings.smooth_settings);
    if (!smooth_op()) {
        return false;
    }

    const Tuple tup = smooth_op.return_tuple();
    assert(mesh().is_valid_slow(tup));
    // start scope
    // auto scope = mesh().create_scope();

    // // fix boundary curve
    // if (!m_settings.smooth_settings.smooth_boundary && mesh().is_boundary_vertex(tup)) {
    //     // do curve mesh smoothing

    // } else {
    //     Optimization opt(
    //         input_tuple(),
    //         m_uv_pos_accessor,
    //         *m_settings.energy.get(),
    //         mesh(),
    //         m_settings.smooth_settings.invariants,
    //         m_settings.second_order,
    //         m_settings.line_search);
    //     opt.optimize2d(p);
    //     // double step_size = m_settings.step_size;
    //     // while (m_settings.line_search && !m_settings.smooth_settings.invariants.after(
    //     //                                      PrimitiveType::Face,
    //     //                                      modified_primitives(PrimitiveType::Face))) {
    //     //     step_size /= 2;
    //     //     opt.optimize2d(m_uv_pos_accessor.vector_attribute(smooth_op.return_tuple()),
    //     //     step_size);
    //     // }
    // }

    return true;
}


} // namespace wmtk::operations::tri_mesh
