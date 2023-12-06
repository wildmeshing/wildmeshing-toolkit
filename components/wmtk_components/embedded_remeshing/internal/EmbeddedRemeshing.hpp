#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class EmbeddedRemeshing
{
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();
    bool m_lock_boundary = true;

    double m_inflate_len;
    long m_input_tag_value;
    long m_embedding_tag_value;
    long m_offset_tag_value;
    MeshAttributeHandle<long> m_vertex_tag_handle;
    MeshAttributeHandle<long> m_edge_tag_handle;
    MeshAttributeHandle<long> m_face_tag_handle;
    MeshAttributeHandle<long> m_todo_vertex_handle;
    MeshAttributeHandle<double> m_pos_handle;

public:
    EmbeddedRemeshing(
        MeshAttributeHandle<long>& vertex_tag_handle,
        MeshAttributeHandle<long>& edge_tag_handle,
        MeshAttributeHandle<long>& face_tag_handle,
        MeshAttributeHandle<long>& todo_vertex_handle,
        MeshAttributeHandle<double>& pos_handle,
        const long input_tag_value,
        const long embedding_tag_value,
        const long offset_tag_value,
        const double inflate_len,
        const double length,
        const bool lock_boundary);

    void tri_split_offset_and_scalffold(TriMesh& mesh);
    void tri_collapse_scalffold(TriMesh& mesh);
    void tri_swap_scalffold(TriMesh& mesh);
    void remeshing(TriMesh& mesh, const long iterations);
    void remeshing(TetMesh& mesh, const long iterations);
};

} // namespace wmtk::components::internal
