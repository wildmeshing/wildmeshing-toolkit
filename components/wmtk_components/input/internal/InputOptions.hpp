#pragma once

#include <wmtk_components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct InputOptions
{
    std::string name;
    std::filesystem::path file;
    bool ignore_z;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(InputOptions, name, file, ignore_z);

} // namespace internal
} // namespace components
} // namespace wmtk