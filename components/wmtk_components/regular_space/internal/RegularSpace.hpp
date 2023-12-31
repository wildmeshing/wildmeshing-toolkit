#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

/*
 * This class is used to seperate mesh and make sure there are no direct connection
 * between independent simplicity collection
 */
class RegularSpace
{
    MeshAttributeHandle<double> m_position_handle; // record position
    MeshAttributeHandle<int64_t> m_vertex_tag; // record vertex tag value
    MeshAttributeHandle<int64_t> m_edge_tag; // record edge vertex tag value
    int64_t m_input_tag_value; // the value used to those simplicity you want to seperate
    int64_t m_embedding_tag_value; // the scalffold simplicities' tag value
    int64_t m_split_tag_value; // when you split a simplicity, you will set m_split_tag_value to the
                               // new simplicity
    // int m_dimension; // operate dimension, 0 for vertex, 1 for edge, 2 for face, 3 for tet

public:
    RegularSpace(
        MeshAttributeHandle<double>& position_handle,
        MeshAttributeHandle<int64_t>& vertex_tag,
        MeshAttributeHandle<int64_t>& edge_tag,
        const int64_t input_tag_value,
        const int64_t embedding_tag_value,
        const int64_t split_tag_value);

    /*
     * seperate edges end with two ends with the same attribute(m_input_tag_value)
     *
     * If you have serveral vertices in a mesh, you don't want
     * those specific vertices with the same attribute directly connect with each
     * other, you should use process_in_0d
     */
    void process_vertex_simplicity_in_2d(TriMesh& m_mesh);
    /*
     * seperate the face if there the three vertices and edges all tagged with the input_value
     * seperate the edge end with two vertices tagged with input_value but the edge itself is not
     * marked as the input_value
     *
     * If you have serveral segments in a mesh, you don't want
     * those specific segments with the same attribute directly connect with each
     * other, you should use process_in_1d
     */
    void process_edge_simplicity_in_2d(TriMesh& m_mesh);
    void process_vertex_simplicity_in_3d(TetMesh& m_mesh);
    void process_edge_simplicity_in_3d(TetMesh& m_mesh);
};

} // namespace wmtk::components::internal
