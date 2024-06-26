#include "SmallerFunctionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

SmallerFunctionInvariant::SmallerFunctionInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func,
    const double max_value)
    : Invariant(func->mesh())
    , m_func(func)
    , m_max_value(max_value)
    , m_type(type)
{}

bool SmallerFunctionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    auto max = [&](const std::vector<Tuple>& tuples) {
        double value = std::numeric_limits<double>::lowest();
        for (const auto& t : tuples)
            value = std::max(value, m_func->get_value(simplex::Simplex(m_type, t)));

        return value;
    };
    const double after = max(top_dimension_tuples_after);

    if (after < m_max_value) {
        return true;
    }

    const double before = mesh().parent_scope(max, top_dimension_tuples_before);

    return after < before;
}
} // namespace wmtk::invariants