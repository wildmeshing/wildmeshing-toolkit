
#pragma once


#include <wmtk/TriMesh.hpp>


using namespace wmtk;


namespace wmtk::components::multimesh {
class MeshCollection;
}

std::shared_ptr<wmtk::TriMesh> fuse(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::map<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>>& to_fuse,
    // const std::map<std::array<int64_t, 2>, std::vector<std::array<Tuple, 2>>>& to_fuse,
    const std::string_view& position_attribute_name);
