#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "Invariant.hpp"

namespace wmtk::invariants {

class TetMeshSubstructureTopologyPreservingInvariant : public Invariant
{
public:
    TetMeshSubstructureTopologyPreservingInvariant(
        const Mesh& m,
        const TypedAttributeHandle<int64_t>& substructure_face_tag_handle,
        const TypedAttributeHandle<int64_t>& substructure_edge_tag_handle,
        const int64_t substructure_tag_value);
    bool before(const simplex::Simplex& input_simplex) const override;

private:
    TypedAttributeHandle<int64_t> m_substructure_face_tag_handle;
    TypedAttributeHandle<int64_t> m_substructure_edge_tag_handle;
    int64_t m_substructure_tag_value;
};

} // namespace wmtk::invariants
