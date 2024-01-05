#include "MaxFunctionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

MaxFunctionInvariant::MaxFunctionInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func)
    : Invariant(func->mesh())
    , m_func(func)
    , m_type(type)
{}

bool MaxFunctionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    const double before = mesh().parent_scope([&]() {
        double _before = std::numeric_limits<double>::lowest();
        for (const auto& t : top_dimension_tuples_before)
            _before = std::max(_before, m_func->get_value(simplex::Simplex(m_type, t)));

        return _before;
    });


    double after = std::numeric_limits<double>::lowest();
    for (const auto& t : top_dimension_tuples_after)
        after = std::max(after, m_func->get_value(simplex::Simplex(m_type, t)));

    return after < before;
}
} // namespace wmtk::invariants