#include "VertexSmoothGradientDescent.hpp"
#include <wmtk/utils/Logger.hpp>
namespace wmtk::operations::tri_mesh::internal {
VertexSmoothGradientDescent::VertexSmoothGradientDescent(
    Mesh& m,
    const Tuple& t,
    const OperationSettings<VertexSmoothUsingDifferentiableEnergy>& settings)
    : VertexSmoothUsingDifferentiableEnergy(m, t, settings)
{}
std::string VertexSmoothGradientDescent::name() const
{
    return "tri_mesh_vertex_smooth_newton_method";
}


template <int Dim>
Eigen::Vector<double, Dim> VertexSmoothGradientDescent::get_descent_direction(
    optimization::FunctionInterface<Dim>& f) const
{
    return -f.get_gradient();
}

bool VertexSmoothGradientDescent::execute()
{
    auto accessor = coordinate_accessor();
    auto interface = get_function_interface<2>(accessor);

    auto pos = interface.get_coordinate();
    Eigen::Vector2d next_pos = pos + m_settings.step_size * get_descent_direction(interface);
    interface.store(next_pos);

    if (!tri_mesh::VertexSmoothUsingDifferentiableEnergy::execute()) {
        wmtk::logger().debug("execute failed");
        return false;
    }
    return true;
}
} // namespace wmtk::operations::tri_mesh::internal
