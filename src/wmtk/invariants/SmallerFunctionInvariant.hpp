#pragma once

#include <memory>
#include "Invariant.hpp"

namespace wmtk {

namespace function {
class PerSimplexFunction;
}

namespace invariants {

class SmallerFunctionInvariant : public Invariant
{
public:
    SmallerFunctionInvariant(
        const PrimitiveType type,
        const std::shared_ptr<function::PerSimplexFunction>& func,
        const double max_value);

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    std::shared_ptr<function::PerSimplexFunction> m_func;
    const double m_max_value;
    const PrimitiveType m_type;
};
} // namespace invariants
} // namespace wmtk