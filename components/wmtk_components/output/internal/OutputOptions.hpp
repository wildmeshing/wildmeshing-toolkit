#pragma once

#include <wmtk_components/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct OutputOptions
{
    std::string type;
    std::string input;
    long cell_dimension = -1;
    std::filesystem::path file;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(OutputOptions, type, input, cell_dimension, file);

} // namespace internal
} // namespace components
} // namespace wmtk