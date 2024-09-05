#pragma once

#include <memory>
#include <optional>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/utils/Rational.hpp>
#include "Invariant.hpp"

namespace wmtk {

namespace function {
class PerSimplexFunction;
}

namespace invariants {

class MaxFunctionInvariant : public Invariant
{
public:
    MaxFunctionInvariant(
        const PrimitiveType type,
        const std::shared_ptr<function::PerSimplexFunction>& func,
        const std::optional<TypedAttributeHandle<Rational>>& coordinate = {});

    bool before(const simplex::Simplex& simplex) const override;

    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    std::shared_ptr<function::PerSimplexFunction> m_func;
    const PrimitiveType m_type;
    const std::optional<TypedAttributeHandle<Rational>> m_coordinate_handle;
    mutable bool was_v0_rounded;
};
} // namespace invariants
} // namespace wmtk