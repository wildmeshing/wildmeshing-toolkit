#pragma once

#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class Marching
{
public:
    Marching(
        TriMesh& mesh,
        MeshAttributeHandle<double>& position_handle,
        MeshAttributeHandle<long>& vertex_tag,
        MeshAttributeHandle<long>& edge_tag,
        MeshAttributeHandle<long>& filter_tag,
        const long input_tag_value,
        const long embedding_tag_value,
        const long split_tag_value);

    void process();

private:
    TriMesh& m_mesh;

    MeshAttributeHandle<double> m_position_handle; // record position
    MeshAttributeHandle<long> m_vertex_tag; // record vertex tag value
    MeshAttributeHandle<long> m_edge_tag; // record edge vertex tag value
    MeshAttributeHandle<long> m_filter_tag; // record filter value. 0 means should be ignored
    long m_input_tag_value; // the value used to those simplicity you want to seperate
    long m_embedding_tag_value; // the scalffold simplicities' tag value
    long m_split_tag_value; // when you split a simplicity, you will set m_split_tag_value to the
                            // new simplicity

    std::unique_ptr<attribute::AttributeInitializationHandle<double>> m_pos_attribute;
};

} // namespace wmtk::components::internal
