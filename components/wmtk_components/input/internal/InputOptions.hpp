#pragma once

#include <wmtk_components/json_utils.hpp>

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct InputOptions
{
    std::string type;
    std::string name;
    std::filesystem::path file;
    long cell_dimension = -1;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(InputOptions, type, name, file, cell_dimension);

} // namespace internal
} // namespace components
} // namespace wmtk