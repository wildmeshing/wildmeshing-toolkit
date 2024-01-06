#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

/*
 * This class is used to seperate mesh and make sure there are no direct connection
 * between independent simplicity collection
 */
class RegularSpace
{
public:
    RegularSpace(
        Mesh& mesh,
        const std::vector<attribute::MeshAttributeHandle>& label_attributes,
        const std::vector<int64_t>& values,
        const std::vector<attribute::MeshAttributeHandle>& pass_through_attributes);

    void regularize_tags();

private:
    Mesh& m_mesh;

    std::vector<attribute::MeshAttributeHandle> m_label_attributes;
    std::vector<int64_t> m_values;

    std::vector<attribute::MeshAttributeHandle> m_pass_through_attributes;

    void clear_attributes();
};

} // namespace wmtk::components::internal
