
#include "LocalFunction.hpp"
#include <wmtk/simplex/cofaces_single_dimension.hpp>

namespace wmtk::function {

LocalFunction::LocalFunction(std::shared_ptr<PerSimplexFunction> function)
    : m_function(std::move(function))
{}

LocalFunction::~LocalFunction() = default;

const PerSimplexFunction& LocalFunction::per_simplex_function() const
{
    return *per_simplex_function_ptr();
}

std::shared_ptr<PerSimplexFunction> LocalFunction::per_simplex_function_ptr() const
{
    return m_function;
}

const Mesh& LocalFunction::mesh() const
{
    return per_simplex_function().mesh();
}
std::vector<Simplex> LocalFunction::get_local_neighborhood_domain_simplices(
    const Simplex& variable_simplex) const
{
    return wmtk::simplex::cofaces_single_dimension_simplices(
        m_function->mesh(),
        variable_simplex,
        get_domain_simplex_type());
}

double LocalFunction::get_value(const Simplex& variable_simplex) const
{
    return per_simplex_function().get_value_sum(
        get_local_neighborhood_domain_simplices(variable_simplex));
}

PrimitiveType LocalFunction::get_domain_simplex_type() const
{
    return per_simplex_function().get_domain_simplex_type();
}

} // namespace wmtk::function
