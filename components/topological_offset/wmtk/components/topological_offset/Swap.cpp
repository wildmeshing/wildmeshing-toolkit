
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace wmtk::components::topological_offset {

/**
 * The swap operations themselves are wmtk::TetOptimizerMesh's. What is left here is the offset's
 * own: keeping the region tags and construction labels consistent across a swap.
 */

bool TopoOffsetTetMesh::swap_capture_tag(const std::vector<size_t>& tids)
{
    std::map<CellTag, size_t> tag_count;
    std::set<int> labels;
    for (const size_t t : tids) {
        tag_count[m_tet_attribute[t].tag]++;
        labels.insert(m_tet_attribute[t].label);
    }
    // Region membership is read from the construction label, and a swap reuses recycled tet slots
    // carrying whatever label was there before, so the label must be carried across explicitly. A
    // ring spanning two labels has a region boundary running through it, and is refused.
    if (labels.size() > 1) {
        return false;
    }
    m_swap_label.local() = *labels.begin();
    // A face between differently tagged tets is the offset surface, so a ring spanning two tags
    // has that surface running through it: refused, because one tag for the whole ring moves it.
    // Do not re-add "majority tag wins"; measured worse -- see git history of this file.
    if (tag_count.size() > 1) {
        return false;
    }

    size_t max_count = 0;
    CellTag max_tag;
    for (const auto& [tag, count] : tag_count) {
        if (count > max_count) {
            max_count = count;
            max_tag = tag;
        }
    }
    m_swap_tag.local() = max_tag;
    return true;
}

bool TopoOffsetTetMesh::swap_before_interior(const std::vector<size_t>& tids)
{
    return swap_capture_tag(tids);
}

bool TopoOffsetTetMesh::swap_before_surface(
    const std::vector<size_t>& tids,
    const size_t a,
    const size_t b,
    const size_t c,
    const size_t d)
{
    if (!swap_capture_tag(tids)) {
        return false;
    }

    // The flip replaces surface faces (a,b,c) and (a,b,d) with (a,c,d),(b,c,d). Both must belong
    // to the same tracked surface: a mixed flip would hand a triangle of the input complex to the
    // offset surface, or the reverse, moving the line where one meets the other.
    const size_t fid_abc = std::get<1>(tuple_from_face(std::array<size_t, 3>{{a, b, c}}));
    const size_t fid_abd = std::get<1>(tuple_from_face(std::array<size_t, 3>{{a, b, d}}));
    if (fid_abc == static_cast<size_t>(-1) || fid_abd == static_cast<size_t>(-1)) {
        return false;
    }
    if (m_face_attribute[fid_abc].m_surface_class != m_face_attribute[fid_abd].m_surface_class) {
        return false;
    }
    // A flip across a junction would detach the new diagonal from one of the boundaries the old
    // faces lay on: refuse when the two faces' boundary masks differ.
    if (face_mask({{a, b, c}}) != face_mask({{a, b, d}})) {
        return false;
    }

    // Non-offset surface flips are not refused categorically: the shared swap checks both new
    // triangles with surface_triangle_is_outside(), which dispatches through the face's boundary
    // mask to the per-tag envelopes, and that envelope is the geometric constraint. The
    // class-match and mask-match refusals above are the topology half. No offset criterion is
    // captured here; placement accuracy belongs to Phase B.
    return true;
}

bool TopoOffsetTetMesh::swap_after_cells(const std::vector<size_t>& tids, bool is_surface_flip)
{
    const CellTag& tag = m_swap_tag.local();
    const int label = m_swap_label.local();
    for (const size_t t : tids) {
        m_tet_attribute[t].tag = tag;
        m_tet_attribute[t].label = label;
    }

    // No offset-criterion acceptance for surface flips: the shared swap has already checked both
    // new triangles against the face's envelope (see swap_before_surface()). `is_surface_flip`
    // stays a parameter because the base reports it, but nothing here branches on it.
    (void)is_surface_flip;
    ++iter_cnt_swap;
    return true;
}

} // namespace wmtk::components::topological_offset
