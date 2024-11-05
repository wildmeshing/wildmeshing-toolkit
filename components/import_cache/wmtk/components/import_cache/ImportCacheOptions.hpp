#pragma once

#include <wmtk/components/utils/json_utils.hpp>

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct ImportCacheOptions
{
    std::filesystem::path folder;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(ImportCacheOptions, folder);

} // namespace internal
} // namespace components
} // namespace wmtk