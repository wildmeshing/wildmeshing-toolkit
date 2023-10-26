#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class Marching
{
    TriMesh& m_mesh;
    bool m_lock_boundary;

    MeshAttributeHandle<double> m_position_handle; // record position
    MeshAttributeHandle<long> m_vertex_tag; // record vertex tag value
    MeshAttributeHandle<long> m_edge_tag; // record edge vertex tag value
    long m_input_tag_value; // the value used to those simplicity you want to seperate
    long m_embedding_tag_value; // the scalffold simplicities' tag value
    long m_split_tag_value; // when you split a simplicity, you will set m_split_tag_value to the
                            // new simplicity
    int m_dimension; // operate dimension, 0 for vertex, 1 for edge, 2 for face, 3 for tet

    Scheduler m_scheduler;

    void process_in_2d();
    void process_in_3d();

public:
    Marching(
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
