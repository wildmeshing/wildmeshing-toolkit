#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class RegularSpace
{
    TriMesh& m_mesh;
    bool m_lock_boundary = true;

    MeshAttributeHandle<double> m_position_handle;
    MeshAttributeHandle<long> m_vertex_tag;
    MeshAttributeHandle<long> m_edge_tag;
    long m_input_tag_value;
    long m_embedding_tag_value;
    long m_split_tag_value;
    int m_dimension;

    Scheduler m_scheduler;

public:
    RegularSpace(
        TriMesh& mesh,
        MeshAttributeHandle<double> position_handle,
        MeshAttributeHandle<long> vertex_tag,
        MeshAttributeHandle<long> edge_tag,
        long input_tag_value,
        long embedding_tag_value,
        long split_tag_value,
        int m_dimension = 2,
        const bool lock_boundary = true);

    void process_in_1d();
    void process_in_2d();
    void process_in_3d();

    void process();
};

} // namespace wmtk::components::internal
