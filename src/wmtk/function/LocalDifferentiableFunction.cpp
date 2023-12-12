#include "LocalDifferentiableFunction.hpp"
#include <wmtk/Tuple.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include "simplex/PerSimplexDifferentiableFunction.hpp"
namespace wmtk::function {
LocalDifferentiableFunction::LocalDifferentiableFunction(
    std::shared_ptr<PerSimplexDifferentiableFunction> function)
    : LocalFunction(std::move(function))
{}

LocalDifferentiableFunction::~LocalDifferentiableFunction() = default;

Eigen::VectorXd LocalDifferentiableFunction::get_gradient(const Simplex& variable_simplex) const
{
    return per_simplex_function().get_gradient_sum(
        get_local_neighborhood_domain_simplices(variable_simplex),
        variable_simplex);
}
Eigen::MatrixXd LocalDifferentiableFunction::get_hessian(const Simplex& variable_simplex) const
{
    return per_simplex_function().get_hessian_sum(
        get_local_neighborhood_domain_simplices(variable_simplex),
        variable_simplex);
}

const PerSimplexDifferentiableFunction& LocalDifferentiableFunction::per_simplex_function() const
{
    return static_cast<const PerSimplexDifferentiableFunction&>(
        LocalFunction::per_simplex_function());
}
std::shared_ptr<PerSimplexDifferentiableFunction>
LocalDifferentiableFunction::per_simplex_function_ptr() const
{
    return std::static_pointer_cast<PerSimplexDifferentiableFunction>(
        LocalFunction::per_simplex_function_ptr());
}

MeshAttributeHandle<double> LocalDifferentiableFunction::get_coordinate_attribute_handle() const
{
    return per_simplex_function().get_coordinate_attribute_handle();
}
} // namespace wmtk::function
