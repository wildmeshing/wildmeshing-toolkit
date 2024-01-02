#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct IsotropicRemeshingOptions
{
    std::string type;
    std::string input;
    std::string output;
    double length_abs = -1;
    double length_rel = -1;
    int64_t iterations = -1;
    bool lock_boundary = true;
    bool preserve_childmesh_topology = false;
    bool preserve_childmesh_geometry = false;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    IsotropicRemeshingOptions,
    type,
    input,
    output,
    length_abs,
    length_rel,
    iterations,
    lock_boundary,
    preserve_childmesh_topology,
    preserve_childmesh_geometry);

} // namespace internal
} // namespace components
} // namespace wmtk