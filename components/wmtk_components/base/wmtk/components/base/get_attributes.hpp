#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components::base {

std::vector<attribute::MeshAttributeHandle> get_attributes(
    const Mesh& m,
    const std::vector<std::string>& attribute_names);

}