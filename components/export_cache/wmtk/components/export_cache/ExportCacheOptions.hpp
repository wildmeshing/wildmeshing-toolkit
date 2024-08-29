#pragma once

#include <wmtk/components/base/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct ExportCacheOptions
{
    std::filesystem::path folder;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ExportCacheOptions, folder);

} // namespace internal
} // namespace components
} // namespace wmtk