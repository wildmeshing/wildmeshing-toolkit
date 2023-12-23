#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct RegularSpaceOptions
{
    std::string type; // regular space
    std::string input; // mesh input dir
    std::string output; // mesh output dir
    std::string pos_attribute_name;
    std::string vertex_tag_handle_name;
    std::string edge_tag_handle_name;
    int dimension; // 0-vertex 1-edge 2-face 3-tet
    long input_value;
    long embedding_value;
    long split_value = -1;
    bool lock_boundary = true;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    RegularSpaceOptions,
    type,
    input,
    output,
    pos_attribute_name,
    vertex_tag_handle_name,
    edge_tag_handle_name,
    dimension,
    input_value,
    embedding_value,
    split_value,
    lock_boundary);

} // namespace internal
} // namespace components
} // namespace wmtk