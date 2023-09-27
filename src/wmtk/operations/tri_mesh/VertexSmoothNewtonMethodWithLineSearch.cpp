#include "VertexSmoothNewtonMethodWithLineSearch.hpp"

namespace wmtk::operations {

namespace tri_mesh {
VertexSmoothNewtonMethodWithLineSearch::VertexSmoothNewtonMethodWithLineSearch(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexSmoothNewtonMethod(m, t, settings)
{}

bool VertexSmoothNewtonMethodWithLineSearch::execute()
{
    tri_mesh::VertexSmoothNewtonMethod smooth_op(mesh(), input_tuple(), m_settings);
    Eigen::Vector2d p = m_uv_pos_accessor.vector_attribute(input_tuple());
    if (!smooth_op()) {
        // line search
        Tuple tup = smooth_op.return_tuple();
        double step_size = 1;
        double minimum_step_size = 1e-6;
        if (!m_settings.smooth_settings.smooth_boundary && mesh().is_boundary_vertex(tup)) {
        } else {
            Eigen::Vector2d search_dir = Eigen::Vector2d::Zero();
            search_dir = -m_settings.energy->get_hessian(tup).ldlt().solve(
                m_settings.energy->get_gradient(tup));
            Eigen::Vector2d new_pos = p + search_dir;
            while (!m_settings.smooth_settings.invariants.after(
                       PrimitiveType::Face,
                       smooth_op.modified_primitives(PrimitiveType::Face)) &&
                   (step_size > minimum_step_size)) {
                step_size /= 2;
                search_dir = -m_settings.energy->get_hessian(tup).ldlt().solve(
                    m_settings.energy->get_gradient(tup));
                new_pos = p + search_dir;

                m_uv_pos_accessor.vector_attribute(tup) = new_pos;
            }
        }
    }

    return true;
}
} // namespace tri_mesh

} // namespace wmtk::operations