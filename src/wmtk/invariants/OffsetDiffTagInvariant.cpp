#include "OffsetDiffTagInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
OffsetDiffTagInvariant::OffsetDiffTagInvariant(
    const Mesh& m,
    const MeshAttributeHandle<long>& vertex_tag_handle,
    const MeshAttributeHandle<long>& edge_tag_handle,
    long input_tag_value,
    long embedding_tag_value,
    long offset_tag_value)
    : MeshInvariant(m)
    , m_vertex_tag_handle(vertex_tag_handle)
    , m_edge_tag_handle(edge_tag_handle)
    , m_input_tag_value(input_tag_value)
    , m_embedding_tag_value(embedding_tag_value)
    , m_offset_tag_value(offset_tag_value)
{}
bool OffsetDiffTagInvariant::before(const Tuple& t) const
{
    ConstAccessor<long> vertex_tag_accessor = mesh().create_accessor(m_vertex_tag_handle);

    long vt0 = vertex_tag_accessor.const_vector_attribute(t)(0);
    long vt1 = vertex_tag_accessor.const_vector_attribute(mesh().switch_vertex(t))(0);
    if ((vt0 == m_input_tag_value && vt1 == m_embedding_tag_value) ||
        (vt1 == m_input_tag_value && vt0 == m_embedding_tag_value)) {
        return true;
    }
    return false;
}
} // namespace wmtk
