#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class RegularSpace
{
    TriMesh& m_mesh;
    bool m_lock_boundary;

    MeshAttributeHandle<double> m_position_handle;
    MeshAttributeHandle<long> m_vertex_tag;
    MeshAttributeHandle<long> m_edge_tag;
    long m_input_tag_value;
    long m_embedding_tag_value;
    long m_split_tag_value;
    int m_dimension;

    Scheduler m_scheduler;

    void process_in_0d();
    void process_in_1d();
    // we don't need this function I thought, since we can't convert TriMesh to the Mesh. That's
    // said TetMesh can't be passes into this class as a TriMesh object.
    void process_in_2d();

public:
    RegularSpace(
        TriMesh& mesh,
        MeshAttributeHandle<double>& position_handle,
        MeshAttributeHandle<long>& vertex_tag,
        MeshAttributeHandle<long>& edge_tag,
        const long input_tag_value,
        const long embedding_tag_value,
        const long split_tag_value,
        const int dimension = 1,
        const bool lock_boundary = true);

    void process();
};

} // namespace wmtk::components::internal
