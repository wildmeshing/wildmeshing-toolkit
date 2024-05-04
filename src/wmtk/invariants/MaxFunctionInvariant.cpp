#include "MaxFunctionInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>

namespace wmtk::invariants {

MaxFunctionInvariant::MaxFunctionInvariant(
    const PrimitiveType type,
    const std::shared_ptr<function::PerSimplexFunction>& func,
    const std::optional<TypedAttributeHandle<Rational>>& coordinate)
    : Invariant(func->mesh(), coordinate.has_value(), true, true)
    , m_func(func)
    , m_type(type)
    , m_coordinate_handle(coordinate)
{
    if (true) {
    }
}

bool MaxFunctionInvariant::before(const simplex::Simplex& simplex) const
{
    if (m_coordinate_handle) {
        const auto accessor = m_func->mesh().create_const_accessor(m_coordinate_handle.value());
        const auto coord = accessor.const_vector_attribute(simplex.tuple());
        was_v0_rounded = true;
        for (int64_t i = 0; i < accessor.dimension(); ++i) {
            if (!coord[i].is_rounded()) {
                was_v0_rounded = false;
                break;
            }
        }
    }
    return true;
}

bool MaxFunctionInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (m_coordinate_handle && !was_v0_rounded) return true;

    auto max = [&](const std::vector<Tuple>& tuples) {
        double value = std::numeric_limits<double>::lowest();
        for (const auto& t : tuples) {
            value = std::max(value, m_func->get_value(simplex::Simplex(mesh(), m_type, t)));
        }


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
