#include "TopologicalTransferStrategy.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::operations {

TopologicalTransferStrategy::TopologicalTransferStrategy(
    const attribute::MeshAttributeHandle& my_handle,
    FunctorType&& f)
    : m_functor(f)
{}

void TopologicalTransferStrategy::run(const simplex::Simplex& s) const
{
    if (m_functor) {
        m_functor(s);
    }
}

} // namespace wmtk::operations