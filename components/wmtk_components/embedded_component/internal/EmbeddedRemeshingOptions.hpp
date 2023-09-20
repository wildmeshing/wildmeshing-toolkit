#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

// this enum just for a test, we definitely should discuss the resolution design later
enum Resolution_Level { LOW, MID, HIGH };

struct EmbeddedRemeshingOptions
{
    std::string type;
    std::string input;
    std::string output;
    Resolution_Level resolute_level;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EmbeddedRemeshingOptions, type, input, output,resolute_level);

} // namespace internal
} // namespace components
} // namespace wmtk