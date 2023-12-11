#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk::invariants {

class SubstructureTopologyPreservingInvariant : public MeshInvariant
{
public:
    SubstructureTopologyPreservingInvariant(
        const Mesh& m,
        const MeshAttributeHandle<long>& substructure_face_tag_handle,
        const MeshAttributeHandle<long>& substructure_edge_tag_handle,
        const long substructure_tag_value);
    bool before(const Tuple& t) const override;

    bool before_tri(const Tuple& t) const;

    bool before_tet(const Tuple& t) const;

private:
    MeshAttributeHandle<long> m_substructure_face_tag_handle;
    MeshAttributeHandle<long> m_substructure_edge_tag_handle;
    long m_substructure_tag_value;
};

} // namespace wmtk::invariants
