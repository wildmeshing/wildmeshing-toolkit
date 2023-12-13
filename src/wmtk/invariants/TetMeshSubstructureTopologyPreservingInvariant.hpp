#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk::invariants {

class TetMeshSubstructureTopologyPreservingInvariant : public MeshInvariant
{
public:
    TetMeshSubstructureTopologyPreservingInvariant(
        const Mesh& m,
        const MeshAttributeHandle<long>& substructure_face_tag_handle,
        const MeshAttributeHandle<long>& substructure_edge_tag_handle,
        const long substructure_tag_value);
    bool before(const Tuple& t) const override;

private:
    MeshAttributeHandle<long> m_substructure_face_tag_handle;
    MeshAttributeHandle<long> m_substructure_edge_tag_handle;
    long m_substructure_tag_value;
};

} // namespace wmtk::invariants
