#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct WindingNumberOptions
{
    std::string input;
    std::string filtering_base;
    std::string output;
    double threshold;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(WindingNumberOptions, input, filtering_base, output, threshold);

} // namespace internal
} // namespace components
} // namespace wmtk