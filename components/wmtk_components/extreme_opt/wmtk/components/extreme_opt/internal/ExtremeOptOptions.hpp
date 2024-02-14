#pragma once

#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

namespace wmtk {
namespace components {
namespace internal {

struct ExtremeOptOptions
{
    std::string mesh_name = "";
    double length_rel = -1;
    long iterations = 0;
    bool lock_boundary = true;
    bool smooth_boundary = true;
    bool check_inversion = true;
    bool do_split = true;
    bool do_collapse = true;
    bool collapse_optimize_E_max = true;
    bool do_swap = true;
    bool swap_optimize_E_max = true;
    bool do_smooth = true;
    bool uniform_reference = true; // use equilateral triangle as reference
    bool debug_output = true;
    std::string debug_dir = "";
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    ExtremeOptOptions,
    mesh_name,
    length_rel,
    iterations,
    lock_boundary,
    smooth_boundary,
    check_inversion,
    do_split,
    do_collapse,
    collapse_optimize_E_max,
    do_swap,
    swap_optimize_E_max,
    do_smooth,
    uniform_reference,
    debug_output,
    debug_dir);

} // namespace internal
} // namespace components
} // namespace wmtk