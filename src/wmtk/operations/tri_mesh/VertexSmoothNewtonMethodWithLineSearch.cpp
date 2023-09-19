#include "VertexSmoothNewtonMethodWithLineSearch.hpp"

namespace wmtk::operations {
namespace tri_mesh {
VertexSmoothNewtonMethodWithLineSearch::VertexSmoothNewtonMethodWithLineSearch(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothOptimization>& settings)
    : VertexSmoothNewTonMethod(m, t, settings)
{}

bool VertexSmoothNewtonMethodWithLineSearch::execute()
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

    // fix boundary curve
    if (!m_settings.smooth_settings.smooth_boundary && mesh().is_boundary_vertex(tup)) {
        // do curve mesh smoothing

    } else {
        Optimization opt(
            input_tuple(),
            m_uv_pos_accessor,
            *m_settings.energy.get(),
            mesh(),
            m_settings.second_order,
            m_settings.line_search);
        opt.step_size = m_settings.step_size;
        opt.execute();
        if (!opt.success) {
            return false;
        }
        m_uv_pos_accessor.set_vector_attribute(input_tuple(), opt.x);
    }

    return true;
}
} // namespace tri_mesh

} // namespace wmtk::operations