#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::operations {

/**
 * Perform a transfer strategy
 */
class TopologicalTransferStrategy
{
public:
    using FunctorType = std::function<void(const simplex::Simplex&)>;

    TopologicalTransferStrategy(
        const attribute::MeshAttributeHandle& my_handle,
        FunctorType&& f = nullptr);

    void run(const simplex::Simplex& s) const;

private:
    FunctorType m_functor;
};

} // namespace wmtk::operations