#include "VertexSmoothNewtonMethod.hpp"
#include <wmtk/utils/Logger.hpp>
namespace wmtk::operations::tri_mesh::internal {
VertexSmoothNewtonMethod::VertexSmoothNewtonMethod(
    Mesh& m,
    const Simplex& t,
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
    // auto evaluator = get_function_evaluator(accessor);

    // spdlog::info("evaluator {}", (long)&evaluator);

    // assert(mesh().is_ccw(input_tuple()));
    // assert(mesh().is_valid_slow(evaluator.simplex().tuple()));

    // assert(&accessor.mesh() == &mesh());
    // auto pos = evaluator.get_coordinate().eval();
    // double value = evaluator.get_value(pos);
    // auto dir = get_descent_direction(evaluator);
    // Eigen::Vector2d next_pos = pos + m_settings.step_size * dir;
    // double new_value = evaluator.get_value(next_pos);

    Eigen::VectorXd pos = accessor.vector_attribute(input_tuple());
    Simplex input_simplex(PrimitiveType::Vertex, input_tuple());
    double value = m_settings.energy->get_value(input_simplex);
    Eigen::MatrixXd hessian = m_settings.energy->get_hessian(input_simplex);
    Eigen::VectorXd gradient = m_settings.energy->get_gradient(input_simplex);
    Eigen::VectorXd dir = -hessian.ldlt().solve(gradient);

    Eigen::VectorXd next_pos = pos + m_settings.step_size * dir;

    accessor.vector_attribute(input_tuple()) = next_pos;
    double new_value = m_settings.energy->get_value(Simplex(PrimitiveType::Vertex, input_tuple()));
    // evaluator.store(next_pos);


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
