#include "FunctionEvaluator.hpp"
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
namespace wmtk::function::utils {

FunctionEvaluator::FunctionEvaluator(
    const function::Function& function,
    Accessor<double>& accessor,
    const Simplex& simplex)
    : m_function(function)
    , m_accessor(accessor)
    , m_simplex(simplex)
{
    assert(accessor.mesh().is_valid_slow(simplex.tuple()));
}

FunctionEvaluator::FunctionEvaluator(const FunctionEvaluator&) = default;
FunctionEvaluator::FunctionEvaluator(FunctionEvaluator&&) = default;
// FunctionEvaluator& FunctionEvaluator::operator=(const FunctionEvaluator&) = default;
// FunctionEvaluator& FunctionEvaluator::operator=(FunctionEvaluator&&) = default;

void FunctionEvaluator::store(double v)
{
    m_accessor.scalar_attribute(tuple()) = v;
}


double FunctionEvaluator::get_value() const
{
    return m_function.get_value(m_simplex);
}
auto FunctionEvaluator::get_value(double v) -> double
{
    store(v);
    return get_value();
}
} // namespace wmtk::function::utils
