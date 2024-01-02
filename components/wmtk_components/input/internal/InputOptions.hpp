#pragma once

#include <wmtk_components/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct InputOptions
{
    std::string type;
    std::string name;
    std::filesystem::path file;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(InputOptions, type, name, file);

} // namespace internal
} // namespace components
} // namespace wmtk