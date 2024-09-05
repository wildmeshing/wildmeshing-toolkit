#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct DelaunayOptions
{
    std::string input;
    std::string position;
    std::string output;
    int64_t cell_dimension;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DelaunayOptions, input, output, position, cell_dimension);

} // namespace internal
} // namespace components
} // namespace wmtk