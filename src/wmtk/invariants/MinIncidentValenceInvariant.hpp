#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
/**
 * Invariant for minimum valence on both incident vertices of an edge.
 */
class MinIncidentValenceInvariant : public MeshInvariant
{
public:
    MinIncidentValenceInvariant(const Mesh& m, long min_valence);
    using MeshInvariant::MeshInvariant;
    bool before(const Tuple& t) const override;

private:
    const MeshAttributeHandle<double> m_coordinate_handle;
    long m_min_valence;
};
} // namespace wmtk
