#include "AttributeUpdateStrategyCollection.hpp"
namespace wmtk::operations {


bool AttributeUpdateStrategyCollection::matches_handle(
    const wmtk::attribute::MeshAttributeHandleVariant& attr,
    const simplex::Simplex& s) const
{
    for (auto&& s : m_strategies) {
        if (s->matches_handle(attr, s)) {
            return true;
        }
    }
    return false;
}
bool AttributeUpdateStrategyCollection::run(const simplex::Simplex& s) override
{
    for (auto&& s : m_strategies) {
        if (s->primitive_type(attr, s)) {
            s->run(s);
        }
    }
    return false;
}
} // namespace wmtk::operations
