#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components::internal {

struct EdgeInsOptions
{
    std::string edges;
    std::string triangles;
    std::string output;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(EdgeInsOptions, edges, triangles, output);


} // namespace wmtk::components::internal