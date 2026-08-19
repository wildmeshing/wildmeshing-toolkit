#include "SimWildMesh.h"

#include <cassert>

namespace wmtk::components::simwild {

bool SimWildMesh::swap_before_interior(const std::vector<size_t>& tids)
{
    assert(!tids.empty());
    auto& cache = swap_tag_cache.local();
    cache.tet_tags = m_tet_attribute[tids.front()].tags;
    for (const size_t tid : tids) {
        if (m_tet_attribute[tid].tags != cache.tet_tags) {
            // No surface face means one tag-homogeneous region. Disagreement here says the
            // tag/surface invariant was already broken, so refuse instead of moving material.
            log_and_throw_error(
                "swap_before_interior: tag disagreement in supposedly interior tets {} vs {}",
                tid,
                tids.front());
        }
    }
    return true;
}

bool SimWildMesh::swap_before_surface(
    const std::vector<size_t>& tids,
    size_t a,
    size_t b,
    size_t c,
    size_t d)
{
    // The old surface faces split the edge ring into two tag-homogeneous arcs. A ring vertex
    // strictly inside an arc identifies its tag after the diagonal flip; c and d sit on the
    // interface and therefore identify neither side.
    auto& cache = swap_tag_cache.local();
    cache.ring_tags.clear();
    for (const size_t tid : tids) {
        const CellTag& tags = m_tet_attribute[tid].tags;
        for (const size_t v : oriented_tet_vids(tid)) {
            if (v == a || v == b || v == c || v == d) continue;
            const auto [it, inserted] = cache.ring_tags.try_emplace(v, tags);
            if (!inserted && it->second != tags) return false;
        }
    }
    return true;
}

bool SimWildMesh::swap_after_cells(const std::vector<size_t>& tids, const bool is_surface_flip)
{
    const auto& cache = swap_tag_cache.local();
    if (!is_surface_flip) {
        for (const size_t tid : tids) m_tet_attribute[tid].tags = cache.tet_tags;
        return true;
    }

    for (const size_t tid : tids) {
        const CellTag* tag = nullptr;
        for (const size_t v : oriented_tet_vids(tid)) {
            const auto it = cache.ring_tags.find(v);
            if (it == cache.ring_tags.end()) continue;
            if (tag != nullptr && *tag != it->second) return false;
            tag = &it->second;
        }
        if (tag == nullptr) return false;
        m_tet_attribute[tid].tags = *tag;
    }
    return true;
}

} // namespace wmtk::components::simwild
