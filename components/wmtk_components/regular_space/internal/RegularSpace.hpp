#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

/*
 * This class is used to seperate mesh and make sure there are no direct connection
 * between independent simplicity collection
 */
class RegularSpace
{
    TriMesh& m_mesh;
    // bool m_lock_boundary;// TODO unused variable

    MeshAttributeHandle<double> m_position_handle; // record position
    MeshAttributeHandle<long> m_vertex_tag; // record vertex tag value
    MeshAttributeHandle<long> m_edge_tag; // record edge vertex tag value
    long m_input_tag_value; // the value used to those simplicity you want to seperate
    long m_embedding_tag_value; // the scalffold simplicities' tag value
    long m_split_tag_value; // when you split a simplicity, you will set m_split_tag_value to the
                            // new simplicity
    int m_dimension; // operate dimension, 0 for vertex, 1 for edge, 2 for face, 3 for tet

    Scheduler m_scheduler;

    /*
     * seperate edges end with two ends with the same attribute(m_input_tag_value)
     *
     * If you have serveral vertices in a mesh, you don't want
     * those specific vertices with the same attribute directly connect with each
     * other, you should use process_in_0d
     */
    void process_in_0d();
    /*
     * seperate the face if there the three vertices and edges all tagged with the input_value
     * seperate the edge end with two vertices tagged with input_value but the edge itself is not
     * marked as the input_value
     *
     * If you have serveral segments in a mesh, you don't want
     * those specific segments with the same attribute directly connect with each
     * other, you should use process_in_1d
     */
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
