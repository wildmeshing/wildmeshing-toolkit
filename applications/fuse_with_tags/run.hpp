#pragma once

#include <map>
#include <array>
#include <vector>
#include <cstdint>
#include <string_view>
namespace wmtk {
class Mesh;
namespace components::multimesh {
class MeshCollection;
class NamedMultiMesh;
} // namespace components::multimesh
} // namespace wmtk
  //
struct Params {
    wmtk::components::multimesh::MeshCollection& collection;
    std::string_view output_name;
    std::string_view tag_format;
    std::string_view position_attribute_name;
    std::map<std::array<int64_t,2>, std::vector<std::array<int64_t,2>>> alignments;
};

wmtk::components::multimesh::NamedMultiMesh& run(
        Params &params);
