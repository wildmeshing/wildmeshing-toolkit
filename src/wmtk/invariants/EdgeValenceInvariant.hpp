#pragma once

#include "Invariant.hpp"

namespace wmtk::invariants {

/**
 * @brief for edge swap in tetmesh. Checks that the valence of the edge is equal to the given
 * valence.
 *
 */
class EdgeValenceInvariant : public Invariant
{
public:
    EdgeValenceInvariant(const Mesh& m, int64_t valence);
    using Invariant::Invariant;

    bool before(const simplex::Simplex& t) const override;

private:
    int64_t m_valence;
};

} // namespace wmtk::invariants