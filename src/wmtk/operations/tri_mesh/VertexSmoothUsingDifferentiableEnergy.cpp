#include "VertexSmoothUsingDifferentiableEnergy.hpp"
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/invariants/TriangleInversionInvariant.hpp>
#include <wmtk/utils/Optimization.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/triangle_helper_functions.hpp>

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy>::initialize_invariants(
    const TriMesh& m)
{
    base_settings.initialize_invariants(m);
    base_settings.invariants.add(std::make_shared<TriangleInversionInvariant>(m, uv_position));
}
} // namespace wmtk::operations

namespace wmtk::operations::tri_mesh {
VertexSmoothUsingDifferentiableEnergy::VertexSmoothUsingDifferentiableEnergy(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexAttributesUpdateBase(m, t, settings.base_settings)
    , m_uv_pos_accessor{m.create_accessor<double>(settings.uv_position)}
    , m_settings{settings}
{}

std::string VertexSmoothUsingDifferentiableEnergy::name() const
{
    return "tri_mesh_vertex_smooth_using_differentiable_energy";
}

bool VertexSmoothUsingDifferentiableEnergy::execute()
{
    assert(mesh().is_valid_slow(input_tuple()));
    if (!tri_mesh::VertexAttributesUpdateBase::execute()) {
        return false;
    }

    const Tuple tup = tri_mesh::VertexAttributesUpdateBase::return_tuple();
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
