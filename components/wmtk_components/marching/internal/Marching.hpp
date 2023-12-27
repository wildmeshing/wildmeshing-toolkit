#pragma once

#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class Marching
{
public:
    Marching(
        TriMesh& mesh,
        std::tuple<MeshAttributeHandle<long>, long, long>& vertex_tags,
        std::tuple<std::string, long>& output_vertex_tag,
        std::vector<std::tuple<MeshAttributeHandle<long>, long>>& filter_tags);

    void process();

private:
    TriMesh& m_mesh;

    std::tuple<MeshAttributeHandle<long>, long, long>& m_vertex_tags;
    std::vector<std::tuple<MeshAttributeHandle<long>, long>> m_edge_filter_tags;

    std::unique_ptr<attribute::AttributeInitializationHandle<double>> m_pos_attribute;

    std::tuple<std::string, long>& m_output_vertex_tag;
};

} // namespace wmtk::components::internal
