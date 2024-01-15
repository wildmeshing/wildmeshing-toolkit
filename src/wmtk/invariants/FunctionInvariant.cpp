#include "FunctionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

FunctionInvariant::FunctionInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func,
    bool accept_equal)
    : Invariant(func->mesh(), false, true, true)
    , m_func(func)
    , m_type(type)
    , m_accept_equal(accept_equal)
{}

bool FunctionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    auto sum = [&](const std::vector<Tuple>& tuples) {
        double _before = 0;
        for (const auto& t : tuples) _before += m_func->get_value(simplex::Simplex(m_type, t));

        return _before;
    };


    const double before = mesh().parent_scope(sum, top_dimension_tuples_before);
    const double after = sum(top_dimension_tuples_after);

    if (std::isnan(after) || std::isinf(after)) return false;

    if (m_accept_equal) return after <= before;

    return after < before;
}
} // namespace wmtk::invariants
