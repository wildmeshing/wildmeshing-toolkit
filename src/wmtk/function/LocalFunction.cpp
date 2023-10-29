
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
std::vector<Tuple> LocalFunction::get_local_neighborhood_tuples(const Simplex& simplex) const
{
    return wmtk::simplex::cofaces_single_dimension_tuples(
        m_function->mesh(),
        simplex,
        per_simplex_function().get_simplex_type());
}

double LocalFunction::get_value(const Simplex& simplex) const
{
    return per_simplex_function().get_value_sum(get_local_neighborhood_tuples(simplex));
}

PrimitiveType LocalFunction::get_simplex_type() const
{
    return per_simplex_function().get_simplex_type();
}

double LocalFunction::get_value(const Tuple& simplex) const
{
    return get_value(Simplex(get_simplex_type(), simplex));
}

} // namespace wmtk::function
