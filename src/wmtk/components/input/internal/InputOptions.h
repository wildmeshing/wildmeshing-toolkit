#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct InputOptions
{
    std::string name;
    std::string type;
    std::filesystem::path file;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(InputOptions, name, type, file);

} // namespace internal
} // namespace components
} // namespace wmtk