#pragma once

#include <memory>
#include <string_view>
namespace wmtk {
class TriMesh;
}
namespace wmtk::components::multimesh {
class MeshCollection;
}
using namespace wmtk;

std::shared_ptr<wmtk::TriMesh> fuse_eigen(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::string_view& position_attribute_name,
    const std::string_view& tag_format);
