
#pragma once
#include <vector>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk::io::internal {

    std::vector<wmtk::attribute::MeshAttributeHandle> get_attribute_handles(const Mesh& m);

}
