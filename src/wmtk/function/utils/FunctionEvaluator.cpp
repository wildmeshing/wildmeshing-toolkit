#include "FunctionEvaluator.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
namespace wmtk::function::utils {

FunctionEvaluator::FunctionEvaluator(
    const function::Function& function,
    Accessor<double>& accessor,
    const Tuple& tuple)
    : m_function(function)
    , m_accessor(accessor)
    , m_tuple(tuple)
{
    m_top_level_cofaces = compute_top_level_cofaces();
}


void FunctionEvaluator::store(double v)
{
    m_accessor.scalar_attribute(m_tuple) = v;
}


double FunctionEvaluator::get_value() const
{
    return m_function.get_value_sum(m_top_level_cofaces);
}
auto FunctionEvaluator::get_value(double v) -> double
{
    store(v);
    return get_value();
}

const std::vector<Tuple>& FunctionEvaluator::top_level_cofaces() const
{
    return m_top_level_cofaces;
}
std::vector<Tuple> FunctionEvaluator::compute_top_level_cofaces() const
{
    return simplex::top_level_cofaces_tuples(mesh(), Simplex(PrimitiveType::Vertex, tuple()));
}
} // namespace wmtk::function::utils
