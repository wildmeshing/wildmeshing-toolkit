#pragma once

#include <memory>
#include "Invariant.hpp"

namespace wmtk {

namespace function {
class PerSimplexFunction;
}

namespace invariants {

class FunctionNumericalInvariant : public Invariant
{
public:
    FunctionNumericalInvariant(
        const PrimitiveType type,
        const std::shared_ptr<function::PerSimplexFunction>& func);

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    std::shared_ptr<function::PerSimplexFunction> m_func;
    const PrimitiveType m_type;
};
} // namespace invariants
} // namespace wmtk