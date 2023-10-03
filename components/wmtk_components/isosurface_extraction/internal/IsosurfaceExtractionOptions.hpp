#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct IsosurfaceExtractionOptions
{
    std::string type;
    std::string input;
    std::string output;
    double inflate_abs = -1;
    double inflate_rel = -1;
    double resolution_rate = -1;
    long input_tag_value = 0;
    long embedding_tag_value = 1;
    long offset_tag_value = 2;
    long iteration_times = -1;
    bool lock_boundary = true;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsosurfaceExtractionOptions,
    type,
    input,
    output,
    inflate_abs,
    inflate_rel,
    resolution_rate,
    iteration_times,
    lock_boundary);

} // namespace internal
} // namespace components
} // namespace wmtk