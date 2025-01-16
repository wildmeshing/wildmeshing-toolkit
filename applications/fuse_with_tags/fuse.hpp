
#pragma once
#include <vector>


#include <wmtk/TriMesh.hpp>


using namespace wmtk;


namespace wmtk::components::multimesh {
class MeshCollection;
}

std::pair<
    std::shared_ptr<wmtk::TriMesh>,
    std::vector<std::tuple<std::shared_ptr<Mesh>, std::string>>>
fuse(
    wmtk::components::multimesh::MeshCollection& mc,
    const std::map<std::array<int64_t, 2>, std::vector<std::array<int64_t, 2>>>& to_fuse,
    // const std::map<std::array<int64_t, 2>, std::vector<std::array<Tuple, 2>>>& to_fuse,
    const std::string_view& position_attribute_name);
