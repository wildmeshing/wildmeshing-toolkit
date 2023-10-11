#pragma once

#include <wmtk/attribute/AttributeHandle.hpp>
#include "MeshInvariant.hpp"

namespace wmtk {
class OffsetDiffTagInvariant : public MeshInvariant
{
public:
    // NOTE: this takes in the threshold squared rather than the threshold itself
    OffsetDiffTagInvariant(
        const Mesh& m,
        const MeshAttributeHandle<long>& vertex_tag,
        const MeshAttributeHandle<long>& edge_tag,
        const MeshAttributeHandle<long>& split_todo,
        long input_tag_value,
        long embedding_tag_value,
        long offset_tag_value);
    using MeshInvariant::MeshInvariant;
    bool before(const Tuple& t) const override;

private:
    const MeshAttributeHandle<long> m_vertex_tag_handle;
    const MeshAttributeHandle<long> m_edge_tag_handle;
    const MeshAttributeHandle<long> m_split_todo_handle;
    long m_input_tag_value;
    long m_embedding_tag_value;
    long m_offset_tag_value;
};
} // namespace wmtk
