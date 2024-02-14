#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

class Marching
{
public:
    Marching(
        Mesh& mesh,
        attribute::MeshAttributeHandle& vertex_label,
        const std::vector<int64_t>& input_values,
        const int64_t output_value,
        std::vector<attribute::MeshAttributeHandle>& filter_labels,
        const std::vector<int64_t>& filter_values,
        const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes);

    void process();

private:
    Mesh& m_mesh;

    attribute::MeshAttributeHandle m_vertex_label;
    std::vector<int64_t> m_input_values;
    int64_t m_output_value;

    std::vector<attribute::MeshAttributeHandle> m_filter_labels;
    std::vector<int64_t> m_filter_values;

    std::vector<attribute::MeshAttributeHandle> m_pass_through_attributes;
};

} // namespace wmtk::components::internal
