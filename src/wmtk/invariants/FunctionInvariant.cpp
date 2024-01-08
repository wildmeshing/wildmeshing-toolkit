#include "FunctionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

FunctionInvariant::FunctionInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func)
    : Invariant(func->mesh())
    , m_func(func)
    , m_type(type)
{}

bool FunctionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    const double before = mesh().parent_scope([&]() {
        double _before = 0;
        for (const auto& t : top_dimension_tuples_before)
            _before += m_func->get_value(simplex::Simplex(m_type, t));

        return _before;
    });


    double after = 0;
    for (const auto& t : top_dimension_tuples_after)
        after += m_func->get_value(simplex::Simplex(m_type, t));

    if (std::isnan(after) || std::isinf(after)) return false;

    return after < before;
}
} // namespace wmtk::invariants