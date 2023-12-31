#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

class Marching
{
public:
    Marching(
        Mesh& mesh,
        std::tuple<MeshAttributeHandle<int64_t>, int64_t, int64_t>& vertex_tags,
        std::tuple<std::string, int64_t>& output_vertex_tag,
        std::vector<std::tuple<MeshAttributeHandle<int64_t>, int64_t>>& filter_tags);

    void process();

private:
    Mesh& m_mesh;

    std::tuple<MeshAttributeHandle<int64_t>, int64_t, int64_t>& m_vertex_tags;
    std::vector<std::tuple<MeshAttributeHandle<int64_t>, int64_t>> m_edge_filter_tags;

    std::unique_ptr<attribute::AttributeInitializationHandle<double>> m_pos_attribute;

    std::tuple<std::string, int64_t>& m_output_vertex_tag;
};

} // namespace wmtk::components::internal
