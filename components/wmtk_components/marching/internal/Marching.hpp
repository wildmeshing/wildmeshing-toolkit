#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class Marching
{
    bool m_lock_boundary;
    MeshAttributeHandle<double> m_position_handle; // record position
    MeshAttributeHandle<int64_t> m_vertex_tag; // record vertex tag value
    MeshAttributeHandle<int64_t> m_edge_tag; // record edge vertex tag value
    MeshAttributeHandle<int64_t> m_filter_tag; // record filter value. 0 means should be ignored
    int64_t m_input_tag_value; // the value used to those simplicity you want to seperate
    int64_t m_embedding_tag_value; // the scalffold simplicities' tag value
    int64_t m_split_tag_value; // when you split a simplicity, you will set m_split_tag_value to the
                               // new simplicity

public:
    Marching(
        MeshAttributeHandle<double>& position_handle,
        MeshAttributeHandle<int64_t>& vertex_tag,
        MeshAttributeHandle<int64_t>& edge_tag,
        MeshAttributeHandle<int64_t>& filter_tag,
        const int64_t input_tag_value,
        const int64_t embedding_tag_value,
        const int64_t split_tag_value,
        const bool lock_boundary = true);

    void process(TriMesh& m_mesh);
};

} // namespace wmtk::components::internal
