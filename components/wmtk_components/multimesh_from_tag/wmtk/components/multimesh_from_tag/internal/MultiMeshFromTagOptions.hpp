#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>
#include <vector>

namespace wmtk::components::internal {

struct MultiMeshFromTagOptions
{
    std::string input;
    std::string output;
    std::string substructure_label;
    int64_t substructure_value;
    std::vector<std::string> pass_through;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    MultiMeshFromTagOptions,
    input,
    output,
    substructure_label,
    substructure_value,
    pass_through);

} // namespace wmtk::components::internal