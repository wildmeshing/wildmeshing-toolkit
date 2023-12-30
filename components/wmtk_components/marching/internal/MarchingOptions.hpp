#pragma once


#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct MarchingOptions
{
    std::string type; // marching
    std::string input; // mesh input dir
    std::string output; // mesh output dir
    std::string pos_attribute_name;
    std::string vertex_tag_handle_name;
    std::string edge_tag_handle_name;
    std::string face_filter_handle_name;
    std::string tet_filter_handle_name;
    int dimension; // 2-2D 3-3D
    int64_t input_value;
    int64_t embedding_value;
    int64_t split_value = -1;
    bool lock_boundary = true;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    MarchingOptions,
    type,
    input,
    output,
    pos_attribute_name,
    vertex_tag_handle_name,
    edge_tag_handle_name,
    face_filter_handle_name,
    tet_filter_handle_name,
    dimension,
    input_value,
    embedding_value,
    split_value,
    lock_boundary);

} // namespace internal
} // namespace components
} // namespace wmtk