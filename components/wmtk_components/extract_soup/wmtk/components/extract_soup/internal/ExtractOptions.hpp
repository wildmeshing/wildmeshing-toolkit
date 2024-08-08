#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct ExtractOptions
{
    std::string input; // input
    std::string output; // output
    double delta_x;
    bool mode;
    std::string volumetric_encoded_file; // encoded_file
    int level;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ExtractOptions,
    input,
    output,
    delta_x,
    mode,
    volumetric_encoded_file,
    level);

} // namespace internal
} // namespace components
} // namespace wmtk