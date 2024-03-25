#include "MaxFunctionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

MaxFunctionInvariant::MaxFunctionInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func)
    : Invariant(func->mesh(), false, true, true)
    , m_func(func)
    , m_type(type)
{}

bool MaxFunctionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    auto max = [&](const std::vector<Tuple>& tuples) {
        double value = std::numeric_limits<double>::lowest();
        for (const auto& t : tuples)
            value = std::max(value, m_func->get_value(simplex::Simplex(m_type, t)));

        return value;
    };
    const double before = mesh().parent_scope(max, top_dimension_tuples_before);


    const double after = max(top_dimension_tuples_after);

    // debug code
    // if (!(after < before)) {
    //     std::cout << "after check: before: " << before << ", after: " << after << std::endl;
    //     std::cout << "before tet size: " << top_dimension_tuples_before.size()
    //               << ", after tet size: " << top_dimension_tuples_after.size() << std::endl;
    // }

    return after < before;
}
} // namespace wmtk::invariants
