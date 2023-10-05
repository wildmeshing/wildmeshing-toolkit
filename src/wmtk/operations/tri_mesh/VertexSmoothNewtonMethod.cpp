#include "VertexSmoothNewtonMethod.hpp"
#include <wmtk/utils/Logger.hpp>
namespace wmtk::operations::tri_mesh {
VertexSmoothNewtonMethod::VertexSmoothNewtonMethod(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexSmoothUsingDifferentiableEnergy(m, t, settings)
{}
std::string VertexSmoothNewtonMethod::name() const
{
    return "tri_mesh_vertex_smooth_newton_method";
}

bool VertexSmoothNewtonMethod::execute()
{
    const Eigen::Vector2d p = m_uv_pos_accessor.vector_attribute(input_tuple());
    if (!tri_mesh::VertexSmoothUsingDifferentiableEnergy::execute()) {
        wmtk::logger().debug("execute failed");
        return false;
    }

    const Tuple tup = tri_mesh::VertexAttributesUpdateBase::return_tuple();
    assert(mesh().is_valid_slow(tup));
    // check if it is a boundary vertex
    if (!m_settings.smooth_boundary && mesh().is_boundary_vertex(tup)) {
    }
    // do curve mesh smoothing
    else {
        Eigen::Vector2d search_dir = Eigen::Vector2d::Zero();
        search_dir =
            -m_settings.energy->get_hessian(tup).ldlt().solve(m_settings.energy->get_gradient(tup));
        Eigen::Vector2d new_pos = p + search_dir;
        m_uv_pos_accessor.vector_attribute(tup) = new_pos;
    }
    m_output_tuple = resurrect_tuple(input_tuple());
    return true;
}
} // namespace wmtk::operations::tri_mesh