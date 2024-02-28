#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct MeshDecimationOptions
{
    std::string input;
    std::string output;
    double target_len;
    std::string cell_constraint_tag_name;
    std::vector<std::string> attributes;
    std::vector<std::string> pass_through;
};

void to_json(nlohmann::json& j, MeshDecimationOptions& o);

void from_json(const nlohmann::json& j, MeshDecimationOptions& o);

} // namespace internal
} // namespace components
} // namespace wmtk