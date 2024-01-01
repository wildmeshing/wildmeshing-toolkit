#include "NewAttributeStrategy.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/utils/Rational.hpp>

namespace wmtk::operations {

NewAttributeStrategy::~NewAttributeStrategy() = default;


const Mesh& NewAttributeStrategy::mesh() const
{
    return const_cast<const Mesh&>(const_cast<NewAttributeStrategy*>(this)->mesh());
}
void NewAttributeStrategy::set_simplex_predicate(SimplexPredicateType&& f)
{
    m_simplex_predicate = std::move(f);
}

void NewAttributeStrategy::set_simplex_predicate(BasicSimplexPredicate f)
{
    switch (optype) {
    default: [[fallthrough]];
    case BasicSimplexPredicate::Default: [[fallthrough]];
    case BasicSimplexPredicate::None: set_simplex_predicate({}); break;
    case BasicSimplexPredicate::IsInterior:
        set_simplex_predicate(
            [&](const simplex::Simplex& s) -> bool { return mesh().is_boundary(s); });
        break;
    }
}

std::bitset<2> NewAttributeStrategy::evaluate_predicate(
    PrimitiveType pt,
    const std::array<Tuple, 2>& simplices)
{
    if (!bool(m_simplex_predicate)) {
        std::bitset<2> pred(0);
        return pred;
    }

    auto old_pred = mesh().parent_scope([&]() {
        std::bitset<2> pred(0);
        if (bool(m_simplex_predicate)) {
            for (size_t j = 0; j < 2; ++j) {
                pred[j] = m_simplex_predicate(simplex::Simplex(pt, simplices[j]));
            }
        }
        return pred;
    });

    return old_pred;
}
} // namespace wmtk::operations
