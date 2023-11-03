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
Eigen::VectorXd VertexSmoothNewtonMethod::get_descent_direction(
    function::utils::DifferentiableFunctionEvaluator& f) const
{
    return -f.get_hessian().ldlt().solve(f.get_gradient());
}


bool VertexSmoothNewtonMethod::execute()
{
    auto accessor = coordinate_accessor();
    auto evaluator = get_function_evaluator(accessor);

    auto pos = evaluator.get_coordinate().eval();
    double value = evaluator.get_value(pos);
    auto dir = get_descent_direction(evaluator);
    Eigen::Vector2d next_pos = pos + m_settings.step_size * dir;
    double new_value = evaluator.get_value(next_pos);

    /*
    spdlog::info(
        "Went from f({},{})={} to f({},{})={}    ====== +={} * {},{}",
        pos.x(),
        pos.y(),
        value,
        next_pos.x(),
        next_pos.y(),
        new_value,
        m_settings.step_size,
        dir.x(),
        dir.y());

    */
    evaluator.store(next_pos);


    if (!tri_mesh::VertexSmoothUsingDifferentiableEnergy::execute()) {
        return false;
    }
    return true;
}
std::vector<double> VertexSmoothNewtonMethod::priority() const
{
    double gradnorm = m_settings.energy->get_gradient(input_tuple()).norm();
    std::vector<double> r;
    r.emplace_back(-gradnorm);
    return r;
}

} // namespace wmtk::operations::tri_mesh::internal
