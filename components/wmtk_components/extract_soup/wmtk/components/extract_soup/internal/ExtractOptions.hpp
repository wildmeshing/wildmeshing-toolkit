#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct ExtractOptions
{
    std::string name; // input
    std::filesystem::path file; // output
    bool mode;
    std::string volumetric_encoded_file; // encoded_file
    int level;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ExtractOptions, name, file, mode, volumetric_encoded_file,level);

} // namespace internal
} // namespace components
} // namespace wmtk