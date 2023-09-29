#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

// this enum just for a test, we definitely should discuss the resolution design later
enum Resolution_Level { LOW, MID, HIGH };

struct EmbeddingOptions
{
    std::string input_file;
    std::string type;
    std::string input;
    std::string output;
    std::string tag_name;
    long input_tag_value;
    long embedding_tag_value;
    Resolution_Level resolute_level;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    EmbeddingOptions,
    input_file,
    type,
    input,
    output,
    tag_name,
    input_tag_value,
    embedding_tag_value,
    resolute_level);

} // namespace internal
} // namespace components
} // namespace wmtk