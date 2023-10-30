#include "DifferentiableFunctionEvaluator.hpp"
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>


namespace wmtk::function::utils {
DifferentiableFunctionEvaluator::DifferentiableFunctionEvaluator(
    const function::DifferentiableFunction& function,
    Accessor<double>& accessor,
    const Simplex& simplex)
    : FunctionEvaluator(function, accessor, simplex)
    , m_function(function)
{
    // m_cofaces_single_dimension = compute_cofaces_single_dimension();
}

auto DifferentiableFunctionEvaluator::function() const -> const function::DifferentiableFunction&
{
    return m_function;
    // return static_cast<const DifferentiableFunction&>(FunctionEvaluator::function());
}


// auto DifferentiableFunctionEvaluator::get_value() const -> double
//{
//     return get_value(simplex());
//    // return function().get_value_sum(
//    //     wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
//    //         cofaces_single_dimension(),
//    //         function_simplex_type()));
//}

auto DifferentiableFunctionEvaluator::get_gradient() const -> Vector
{
    return function().get_hessian(simplex());
    // return function().get_gradient_sum(
    //     wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
    //         cofaces_single_dimension(),
    //         function_simplex_type()));
}

auto DifferentiableFunctionEvaluator::get_hessian() const -> Matrix
{
    return function().get_hessian(simplex());
    // return function().get_hessian_sum(
    //     wmtk::simplex::utils::tuple_vector_to_homogeneous_simplex_vector(
    //         cofaces_single_dimension(),
    //         function_simplex_type()));
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

// const std::vector<Tuple>& DifferentiableFunctionEvaluator::cofaces_single_dimension() const
//{
//     return m_cofaces_single_dimension;
// }
// std::vector<Tuple> DifferentiableFunctionEvaluator::compute_cofaces_single_dimension() const
//{
//     return simplex::cofaces_single_dimension_tuples(mesh(), simplex(), function_simplex_type());
// }
//
// std::vector<Tuple> DifferentiableFunctionEvaluator::compute_top_dimension_cofaces() const
//{
//     return simplex::top_dimension_cofaces_tuples(mesh(), simplex());
// }
} // namespace wmtk::function::utils
