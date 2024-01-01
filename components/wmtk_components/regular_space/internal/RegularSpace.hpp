#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::internal {

/*
 * This class is used to seperate mesh and make sure there are no direct connection
 * between independent simplicity collection
 */
class RegularSpace
{
    Mesh& m_mesh;

    std::unique_ptr<attribute::AttributeInitializationHandle<double>> m_pos_attribute;


public:
    RegularSpace(Mesh& mesh);

    void regularize_tags(const std::vector<std::tuple<std::string, int64_t, int64_t>>& tags);
};

} // namespace wmtk::components::internal
