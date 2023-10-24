#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct EmbeddingOptions
{
    std::string input_file;
    std::string type;
    std::string input;
    std::string output;
    std::string tag_name;
    long input_tag_value;
    long embedding_tag_value;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    EmbeddingOptions,
    input_file,
    type,
    input,
    output,
    tag_name,
    input_tag_value,
    embedding_tag_value);

} // namespace internal
} // namespace components
} // namespace wmtk