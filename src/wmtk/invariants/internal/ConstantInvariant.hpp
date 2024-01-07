#pragma once

#include <memory>

#include <wmtk/invariants/Invariant.hpp>

namespace wmtk::invariants::internal {

class ConstantInvariant : public Invariant
{
public:
    ConstantInvariant(const Mesh& m, bool before, bool after);

    bool before(const simplex::Simplex& t) const;
    bool after(
        const std::vector<Tuple>& top_dimension_tuples_before,
        const std::vector<Tuple>& top_dimension_tuples_after) const override;

private:
    bool m_before;
    bool m_after;
};
} // namespace wmtk::invariants::internal
