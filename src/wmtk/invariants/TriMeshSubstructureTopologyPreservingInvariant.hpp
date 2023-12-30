#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {

class TriMeshSubstructureTopologyPreservingInvariant : public Invariant
{
public:
    TriMeshSubstructureTopologyPreservingInvariant(
        const Mesh& m,
        const MeshAttributeHandle<long>& substructure_edge_tag_handle,
        const long substructure_tag_value);
    bool before(const Simplex& input_simplex) const override;

private:
    MeshAttributeHandle<long> m_substructure_edge_tag_handle;
    long m_substructure_tag_value;
};

} // namespace wmtk::invariants
