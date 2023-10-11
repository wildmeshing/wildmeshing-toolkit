#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct RegularSpaceOptions
{
    std::string type; // regular space
    std::string input; // mesh input dir
    std::string output; // mesh output dir
    int demension; // 0-vertex 1-edge 2-face 3-tet
    std::map<std::string, long> tags_value;
    long split_value = -1;
    bool lock_boundary = true;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    RegularSpaceOptions,
    type,
    input,
    output,
    demension,
    tags_value,
    split_value,
    lock_boundary);

} // namespace internal
} // namespace components
} // namespace wmtk