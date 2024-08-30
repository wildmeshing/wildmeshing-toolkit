#pragma once

#include <wmtk/components/utils/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct OutputAttributes
{
    std::string position;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputAttributes, position);

struct OutputOptions
{
    std::string input;
    std::filesystem::path file;
    OutputAttributes attributes;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputOptions, input, file, attributes);

} // namespace internal
} // namespace components
} // namespace wmtk