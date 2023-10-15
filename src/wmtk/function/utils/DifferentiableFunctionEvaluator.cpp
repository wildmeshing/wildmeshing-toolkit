#include "DifferentiableFunctionEvaluator.hpp"
namespace wmtk::function::utils {

DifferentiableFunctionEvaluator::DifferentiableFunctionEvaluator(
    const function::DifferentiableFunction& function,
    Accessor<double>& accessor,
    const Tuple& tuple)
    : FunctionEvaluator(function, accessor, tuple)
{}

auto DifferentiableFunctionEvaluator::function() const -> const function::DifferentiableFunction&
{
    return static_cast<const DifferentiableFunction&>(FunctionEvaluator::function());
}

auto DifferentiableFunctionEvaluator::get_gradient() const -> Vector
{
    return function().get_gradient_sum(tuple(), top_level_cofaces());
}
auto DifferentiableFunctionEvaluator::get_hessian() const -> Matrix
{
    return function().get_hessian_sum(tuple(), top_level_cofaces());
}
auto DifferentiableFunctionEvaluator::get_gradient(double v) -> Vector
{
    store(v);
    return get_gradient();
}
auto DifferentiableFunctionEvaluator::get_hessian(double v) -> Matrix
{
    store(v);
    return get_hessian();
}
} // namespace wmtk::function::utils
