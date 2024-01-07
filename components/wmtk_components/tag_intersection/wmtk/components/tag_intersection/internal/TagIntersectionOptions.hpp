#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <vector>
#include <wmtk/PrimitiveType.hpp>

namespace wmtk::components::internal {

struct TagIntersectionAttributes
{
    std::vector<std::string> vertex_labels;
    std::vector<std::string> edge_labels;
    std::vector<std::string> face_labels;
    std::vector<std::string> tetrahedron_labels;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TagIntersectionAttributes,
    vertex_labels,
    edge_labels,
    face_labels,
    tetrahedron_labels);

struct TagIntersectionValues
{
    std::vector<int64_t> vertex_values;
    std::vector<int64_t> edge_values;
    std::vector<int64_t> face_values;
    std::vector<int64_t> tetrahedron_values;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TagIntersectionValues,
    vertex_values,
    edge_values,
    face_values,
    tetrahedron_values);

struct TagIntersectionOptions
{
    std::string input;
    std::string output;
    TagIntersectionAttributes attributes;
    TagIntersectionValues values;
    TagIntersectionAttributes output_attributes;
    TagIntersectionValues output_values;
    std::vector<std::string> pass_through;
};

void to_json(nlohmann::json& j, TagIntersectionOptions& o);

void from_json(const nlohmann::json& j, TagIntersectionOptions& o);

} // namespace wmtk::components::internal