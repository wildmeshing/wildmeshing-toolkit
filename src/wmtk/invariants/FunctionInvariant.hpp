#pragma once

#include <memory>
#include "Invariant.hpp"

namespace wmtk {

namespace function {
class PerSimplexFunction;
}

namespace invariants {

class FunctionInvariant : public Invariant
{
public:
    FunctionInvariant(
        const PrimitiveType type,
        const std::shared_ptr<function::PerSimplexFunction>& func,
        bool accept_equal = false);

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    std::shared_ptr<function::PerSimplexFunction> m_func;
    const PrimitiveType m_type;
    bool m_accept_equal;
};
} // namespace invariants
} // namespace wmtk