#include "FunctionNumericalInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

FunctionNumericalInvariant::FunctionNumericalInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func)
    : Invariant(func->mesh())
    , m_func(func)
    , m_type(type)

{}

bool FunctionNumericalInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    double after = 0;
    for (const auto& t : top_dimension_tuples_after)
        after += m_func->get_value(simplex::Simplex(m_type, t));

    if (std::isnan(after) || std::isinf(after)) return false;
    return true;
}
} // namespace wmtk::invariants