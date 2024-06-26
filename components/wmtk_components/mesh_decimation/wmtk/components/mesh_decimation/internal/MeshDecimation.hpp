#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

class MeshDecimation
{
public:
    MeshDecimation(
        Mesh& mesh,
        attribute::MeshAttributeHandle constrainted_cell_tag_handle,
        double target_len,
        const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes);

    void process();

private:
    Mesh& m_mesh;

    attribute::MeshAttributeHandle m_constrainted_cell_tag_handle;
    double m_target_len;

    std::vector<attribute::MeshAttributeHandle> m_pass_through_attributes;
};

} // namespace wmtk::components::internal
