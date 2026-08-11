#include <wmtk/RationalPositions.h>

#include <wmtk/utils/Logger.hpp>

namespace wmtk {

size_t RationalPositions::round_all_vertices()
{
    if (m_all_rounded.load(std::memory_order_relaxed)) {
        return 0;
    }

    size_t reclaimed = 0, still_unrounded = 0;
    for (const size_t vid : all_vertex_ids()) {
        if (vertex_is_rounded(vid)) {
            continue;
        }
        if (round_vertex(vid)) {
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

} // namespace wmtk
