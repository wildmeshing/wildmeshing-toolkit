#include <wmtk/RationalPositions.h>
#include <wmtk/utils/Profiler.hpp>

#include <wmtk/utils/Logger.hpp>

#include <mutex>
#include <utility>

namespace wmtk {

size_t RationalPositions::round_all_vertices()
{
    if (m_all_rounded.load(std::memory_order_relaxed)) {
        return 0;
    }
    WMTK_PROF_SCOPE_NOSIZE("round_all_vertices");

    size_t reclaimed = 0, still_unrounded = 0;
    for (const size_t vid : all_vertex_ids()) {
        if (vertex_is_rounded(vid)) {
            continue;
        }
        if (WMTK_PROF_TIMED("round/round_vertex", round_vertex(vid))) {
            ++reclaimed;
        } else {
            ++still_unrounded;
        }
    }

    if (still_unrounded == 0) {
        m_all_rounded.store(true, std::memory_order_relaxed);
    }
    if (reclaimed > 0 || still_unrounded > 0) {
        logger().info(
            "rounding sweep: reclaimed {}, still unrounded {}",
            reclaimed,
            still_unrounded);
    }
    return reclaimed;
}

bool RationalPositions::round_and_check_all_rounded()
{
    round_all_vertices();
    return m_all_rounded.load(std::memory_order_relaxed);
}

size_t RationalPositions::count_unrounded() const
{
    size_t n = 0;
    for (const size_t vid : all_vertex_ids()) {
        if (!vertex_is_rounded(vid)) ++n;
    }
    return n;
}

namespace {
std::mutex unrounded_output_mutex;
UnroundedOutput unrounded_output;
} // namespace

UnroundedOutput take_unrounded_output()
{
    std::lock_guard lock(unrounded_output_mutex);
    return std::exchange(unrounded_output, UnroundedOutput());
}

void check_unrounded_output(
    const std::string& component,
    const size_t unrounded,
    const bool memory,
    const bool throw_on_fail)
{
    if (unrounded == 0) return;
    {
        std::lock_guard lock(unrounded_output_mutex);
        unrounded_output = {component, unrounded};
    }
    logger().error(
        "{}: Not all vertices rounded! {} vertices keep exact positions: {}",
        component,
        unrounded,
        memory ? "returned in the mesh's V_rational, for a following memory call"
               : "the written mesh holds their rounding only");
    if (throw_on_fail) {
        log_and_throw_error("{}: Not all vertices rounded! (throw_on_fail)", component);
    }
}

} // namespace wmtk
