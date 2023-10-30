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


Eigen::VectorXd VertexSmoothGradientDescent::get_descent_direction(
    function::utils::DifferentiableFunctionEvaluator& f) const
{
    return -f.get_gradient();
}

bool VertexSmoothGradientDescent::execute()
{
    auto accessor = coordinate_accessor();
    auto evaluator = get_function_evaluator(accessor);

    auto pos = evaluator.get_coordinate();
    Eigen::Vector2d next_pos = pos + m_settings.step_size * get_descent_direction(evaluator);
    evaluator.store(next_pos);

    if (!tri_mesh::VertexSmoothUsingDifferentiableEnergy::execute()) {
        wmtk::logger().debug("execute failed");
        return false;
    }
    return true;
}
std::vector<double> VertexSmoothGradientDescent::priority() const
{
    double gradnorm = m_settings.energy->get_gradient(input_tuple()).norm();
    std::vector<double> r;
    r.emplace_back(-gradnorm);
    return r;
}
} // namespace wmtk::operations::tri_mesh::internal
