#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct MeshDecimationOptions
{
    std::string input;
    std::string output;
    int64_t constrait_value;
    double target_len;
    std::string cell_constrait_tag_name;
    std::vector<std::string> pass_through;
};

void to_json(nlohmann::json& j, MeshDecimationOptions& o);

void from_json(const nlohmann::json& j, MeshDecimationOptions& o);

} // namespace internal
} // namespace components
} // namespace wmtk