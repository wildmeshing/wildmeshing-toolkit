#include "ConstantInvariant.hpp"


namespace wmtk::invariants::internal {


ConstantInvariant::ConstantInvariant(const Mesh& m, bool before, bool after)

    : Invariant(m)
    , m_before(before)
    , m_after(after)
{}
bool ConstantInvariant::before(const simplex::Simplex& t) const
{
    return m_before;
}
bool ConstantInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    return m_after;
}

} // namespace wmtk::invariants::internal
