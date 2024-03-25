#pragma once

#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {

class TriMeshSubstructureTopologyPreservingInvariant : public Invariant
{
public:
    TriMeshSubstructureTopologyPreservingInvariant(
        const Mesh& m,
        const TypedAttributeHandle<int64_t>& substructure_edge_tag_handle,
        const int64_t substructure_tag_value);
    bool before(const simplex::Simplex& input_simplex) const override;

private:
    TypedAttributeHandle<int64_t> m_substructure_edge_tag_handle;
    int64_t m_substructure_tag_value;
};

} // namespace wmtk::invariants
