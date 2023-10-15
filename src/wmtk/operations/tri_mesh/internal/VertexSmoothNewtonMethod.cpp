#include "VertexSmoothNewtonMethod.hpp"
#include <wmtk/utils/Logger.hpp>
namespace wmtk::operations::tri_mesh::internal {
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
template <int Dim>
Eigen::Vector<double, Dim> VertexSmoothNewtonMethod::get_descent_direction(
    optimization::FunctionInterface<Dim>& f) const
{
    return -f.get_hessian().ldlt().solve(f.get_gradient());
}
template <>
Eigen::Vector<double, 2> VertexSmoothNewtonMethod::get_descent_direction<2>(
    optimization::FunctionInterface<2>& f) const;
template <>
Eigen::Vector<double, 3> VertexSmoothNewtonMethod::get_descent_direction<3>(
    optimization::FunctionInterface<3>& f) const;


bool VertexSmoothNewtonMethod::execute()
{
    auto accessor = coordinate_accessor();
    auto interface = get_function_interface<2>(accessor);

    auto pos = interface.get_coordinate();
    Eigen::Vector2d next_pos = pos + m_settings.step_size * get_descent_direction(interface);
    interface.store(next_pos);


    if (!tri_mesh::VertexSmoothUsingDifferentiableEnergy::execute()) {
        return false;
    }
    return true;
}

} // namespace wmtk::operations::tri_mesh::internal
