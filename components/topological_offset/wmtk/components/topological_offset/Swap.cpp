
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace wmtk::components::topological_offset {

/**
 * The swap operations themselves -- the 3-2, 4-4 and 5-6 edge swaps, the 2-3 face swap, their
 * drivers, the face-attribute tracker and the surface diagonal flip -- are
 * wmtk::TetOptimizerMesh's. What is left here is only what is the offset's own: keeping the
 * region tags consistent, and keeping the offset surface faithful to the implicit offset field
 * of the input complex.
 */

bool TopoOffsetTetMesh::swap_capture_tag(const std::vector<size_t>& tids)
{
    std::map<CellTag, size_t> tag_count;
    std::set<int> labels;
    for (const size_t t : tids) {
        tag_count[m_tet_attribute[t].tag]++;
        labels.insert(m_tet_attribute[t].label);
    }
    // The construction LABEL is what region membership is read from (cell_in_region(),
    // cell_is_offset_band(), cell_is_input_complex()), and a swap reuses recycled tet slots
    // whose labels belong to whatever was there before. So it has to be carried across
    // explicitly, exactly as the tag is -- and for the same reason the tag rule below refuses a
    // mixed ring, a ring spanning two labels has a region boundary running through it and
    // cannot be given one label without moving that boundary.
    if (labels.size() > 1) {
        return false;
    }
    m_swap_label.local() = *labels.begin();
    // Refuse any swap whose ring spans more than one tag.
    //
    // The rule used to be "at most two tags, and the majority wins". That is what tore the
    // offset region: a face between differently tagged tets IS the offset surface, so a ring
    // spanning two tags has the surface running through it, and collapsing all the new cells
    // onto the majority tag moves that surface -- merging the two regions wherever the
    // minority lost. Measured, the swap pass alone was enough to make the region
    // non-manifold; with this it is not.
    //
    // Conservative: a genuine surface flip could keep both tags by assigning each new cell the
    // tag of the side it lands on. That is worth doing, and would recover the flips this
    // refuses, but it needs the side-of-surface test to be exact.
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

    // The flip replaces the two current surface faces (a,b,c) and (a,b,d) with (a,c,d),(b,c,d).
    // Both must belong to the SAME tracked surface: a mixed flip would hand a triangle of the
    // input complex to the offset boundary, or the reverse, moving the line where one meets the
    // other.
    const size_t fid_abc = std::get<1>(tuple_from_face(std::array<size_t, 3>{{a, b, c}}));
    const size_t fid_abd = std::get<1>(tuple_from_face(std::array<size_t, 3>{{a, b, d}}));
    if (fid_abc == static_cast<size_t>(-1) || fid_abd == static_cast<size_t>(-1)) {
        return false;
    }
    if (m_face_attribute[fid_abc].m_surface_class != m_face_attribute[fid_abd].m_surface_class) {
        return false;
    }
    // A flip across a JUNCTION would detach the new diagonal from one of the boundaries the
    // old faces lay on: refuse when the two faces' boundary masks differ. Currently unreachable
    // for region faces -- non-offset flips are refused categorically below -- so this is
    // defense-in-depth there, but it also catches an offset-class flip whose corners straddle
    // a junction curve.
    if (face_mask({{a, b, c}}) != face_mask({{a, b, d}})) {
        return false;
    }

    // TETWILD PARITY: no categorical refusal of non-offset surface flips, and no
    // offset-criterion capture for offset ones.
    //
    // The categorical refusal of input-complex/region flips rested, by its own comment, on "a
    // gap, not a principle": unlike a split or a collapse, a flip had no criterion for whether
    // the re-triangulated surface still represents the input. TetWild answers that with its
    // envelope -- it re-triangulates its own input surface on nothing but the eps tube -- and
    // the same answer now applies here: the shared swap checks both new triangles with
    // surface_triangle_is_outside(), which dispatches through the face's boundary mask to the
    // per-tag envelopes (or, for an all-offset face in Phase A, to the offset envelope). The
    // class-match and mask-match refusals above keep a flip from moving a junction or trading
    // geometry between tracked surfaces, which is the topology half of the contract.
    //
    // Note the reach of this is narrow for interior region boundaries: swap_capture_tag()
    // refuses any ring spanning two tags, and an interior region-boundary face always has
    // differently-tagged tets on its two sides, so those flips are structurally refused above
    // regardless. What this admits is flips within one tag -- the domain wall, and the offset
    // surface itself, held by their envelopes.
    //
    // The Phi-criterion swap acceptance (capture here, compare in swap_after_cells()) is gone
    // with the collapse one -- see collapse_after_connectivity() for the argument. The envelope
    // is the constraint; the criterion belongs to Phase B.
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

    // TETWILD PARITY: no offset-criterion acceptance for surface flips. The shared swap has
    // already checked both new triangles against the face's envelope (see
    // swap_before_surface()); a second criterion on top of that is what this used to be, and
    // it is gone with the collapse one -- see collapse_after_connectivity(). `is_surface_flip`
    // stays a parameter because the base reports it, but nothing here branches on it now.
    (void)is_surface_flip;
    ++iter_cnt_swap;
    return true;
}

} // namespace wmtk::components::topological_offset
