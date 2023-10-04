#include "SplitScallfoldInvariant.hpp"
#include <wmtk/Mesh.hpp>

namespace wmtk {
SplitScallfoldInvariant::SplitScallfoldInvariant(
    const Mesh& m,
    const MeshAttributeHandle<double>& position_handle,
    const MeshAttributeHandle<long>& vertex_tag_handle,
    double threshold_squared,
    long input_tag_value,
    long embedding_tag_value,
    long offset_tag_value)
    : MeshInvariant(m)
    , m_position_handle(position_handle)
    , m_vertex_tag_handle(vertex_tag_handle)
    , m_threshold_squared(threshold_squared)
    , m_input_tag_value(input_tag_value)
    , m_embedding_tag_value(embedding_tag_value)
    , m_offset_tag_value(offset_tag_value)
{}
bool SplitScallfoldInvariant::before(const Tuple& t) const
{
    ConstAccessor<long> vertex_tag_accessor = mesh().create_accessor(m_vertex_tag_handle);
    long vt0 = vertex_tag_accessor.const_vector_attribute(t)(0);
    long vt1 = vertex_tag_accessor.const_vector_attribute(mesh().switch_vertex(t))(0);
    if (vt0 == m_input_tag_value || vt1 == m_input_tag_value) {
        return false;
    }

    ConstAccessor<double> position_accessor = mesh().create_accessor(m_position_handle);
    auto p0 = position_accessor.const_vector_attribute(t);
    auto p1 = position_accessor.const_vector_attribute(mesh().switch_vertex(t));
    const double l_squared = (p1 - p0).squaredNorm();
    return l_squared > m_threshold_squared;
}
} // namespace wmtk
