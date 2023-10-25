#include "DifferentiableFunctionEvaluator.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
#include <wmtk/simplex/upper_level_cofaces.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>


namespace wmtk::function::utils {
DifferentiableFunctionEvaluator::DifferentiableFunctionEvaluator(
    const function::LocallyDifferentiableFunction& function,
    Accessor<double>& accessor,
    const Simplex& simplex)
    : FunctionEvaluator(function, accessor, simplex)
{
    m_upper_level_cofaces = compute_upper_level_cofaces();
}

auto DifferentiableFunctionEvaluator::function() const
    -> const function::LocallyDifferentiableFunction&
{
    return static_cast<const LocallyDifferentiableFunction&>(FunctionEvaluator::function());
}

auto DifferentiableFunctionEvaluator::get_value(double v) -> double
{
    store(v);
    return get_value();
}

auto DifferentiableFunctionEvaluator::get_value() const -> double
{
    return function().get_value_sum(
        wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
            upper_level_cofaces(),
            function_simplex_type()));
}

auto DifferentiableFunctionEvaluator::get_gradient() const -> Vector
{
    return function().get_gradient_sum(
        wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
            upper_level_cofaces(),
            function_simplex_type()));
}

auto DifferentiableFunctionEvaluator::get_hessian() const -> Matrix
{
    return function().get_hessian_sum(
        wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
            upper_level_cofaces(),
            function_simplex_type()));
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

const std::vector<Tuple>& DifferentiableFunctionEvaluator::upper_level_cofaces() const
{
    return m_upper_level_cofaces;
}
std::vector<Tuple> DifferentiableFunctionEvaluator::compute_upper_level_cofaces() const
{
    return simplex::upper_level_cofaces_tuples(mesh(), simplex(), function_simplex_type());
}

std::vector<Tuple> DifferentiableFunctionEvaluator::compute_top_level_cofaces() const
{
    return simplex::top_level_cofaces_tuples(mesh(), simplex());
}
} // namespace wmtk::function::utils