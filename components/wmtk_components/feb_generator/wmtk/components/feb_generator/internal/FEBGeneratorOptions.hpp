#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct FEBGeneratorOptions
{
    std::string input; // input
    std::string settings_input; // json files for selections
    std::string output_folder; // output folder
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(FEBGeneratorOptions, input, settings_input, output_folder);

} // namespace internal
} // namespace components
} // namespace wmtk