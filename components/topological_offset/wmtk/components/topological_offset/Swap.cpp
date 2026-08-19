
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

    if (m_face_attribute[fid_abc].m_surface_class != OFFSET_SURFACE_CLASS) {
        // Input-complex flip. Refused: re-triangulating a non-planar quad of the input complex
        // MOVES that surface, and the input complex is frozen categorically. This used to
        // return true and leave it to the shared operation's envelope check, but the envelope
        // is a tolerance -- it permits drift up to eps -- and the input complex is the geometry
        // the offset distance is measured against, so it may not drift at all.
        return false;
    }

    // The bar swap_after_cells() compares against, captured before the flip.
    if (m_offset_potential) {
        double worst = 0.;
        for (const size_t v : {a, b, c, d}) {
            for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(v))) {
                worst = std::max(worst, face_criterion_rel(f));
            }
        }
        m_swap_offset_rel_before.local() = worst;
    }

    // An OFFSET-surface flip is allowed unconditionally. It used to be gated on
    // offset_swap_normal_deviation_ok(), which refused a flip whose new diagonal made the
    // sampled field normals disagree more than the old one did -- a proxy for "this flip cuts
    // across a feature", needed only because the loop's own criterion was blind to it. The
    // face-sampled Phi residual sees it directly and refines instead. A flip does not move any
    // vertex, so it cannot change where the offset surface passes through space except through
    // the diagonal it picks, which is exactly what the criterion now measures.
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

    // A SWAP IS ACCEPTED BY THE SAME CRITERION THE SMOOTHING MINIMISES -- see
    // collapse_after_connectivity() for the argument, which is identical. A surface flip moves no
    // vertex, so it cannot change where the offset passes through space except through the
    // diagonal it picks; but that diagonal is exactly what decides whether the two triangles
    // follow the level set or cut across it, and the criterion is what can tell.
    //
    // Only a surface flip is checked. An interior swap is within one label by construction (the
    // ring would otherwise span a region boundary, which swap_capture_tag refuses), so it leaves
    // the offset surface untouched and the check would be pure cost.
    if (is_surface_flip && m_offset_potential) {
        std::set<size_t> seen;
        double after = 0.;
        for (const size_t t : tids) {
            const std::array<size_t, 4> vs = oriented_tet_vids(t);
            for (int skip = 0; skip < 4; ++skip) {
                std::array<size_t, 3> fv;
                int k = 0;
                for (int j = 0; j < 4; ++j) {
                    if (j != skip) fv[k++] = vs[j];
                }
                const auto [f, unused_tid] = tuple_from_face(fv);
                if (!f.is_valid(*this)) continue;
                if (!seen.insert(f.fid(*this)).second) continue;
                if (!face_is_offset_surface_live(f)) continue;
                after = std::max(after, face_criterion_rel(f));
            }
        }
        if (after > m_swap_offset_rel_before.local()) {
            ++iter_cnt_swap_offset_reject;
            return false;
        }
    }
    ++iter_cnt_swap;
    return true;
}

} // namespace wmtk::components::topological_offset
