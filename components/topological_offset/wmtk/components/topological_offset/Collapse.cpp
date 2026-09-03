
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& t)
{
    // The collapse length gate lives in the shared pass, which filters the candidate list against
    // collapsing_l2 scaled by the endpoints' sizing scalars.
    if (!TetOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // The survivor's own sizing scalar, for sizing_collapse_min = false: the base collapse
    // overwrites it with the min of the two, and collapse_edge_after() puts it back.
    // collapse_cache is the base's, filled by the call above; v2 survives.
    m_collapse_survivor_sizing.local() =
        m_vertex_attribute[collapse_cache.local().v2_id].m_sizing_scalar;
    // Applied unconditionally, where the base asks only when both endpoints already sit on a
    // tracked simplex: the offset region is a thin shell, so a collapse with one endpoint in the
    // interior can still pinch its two sides together while every tracked surface survives.
    if (!substructure_link_condition(t)) {
        return false;
    }
    return true;
}

bool TopoOffsetTetMesh::collapse_edge_after(const Tuple& t)
{
    if (!TetOptimizerMesh::collapse_edge_after(t)) {
        return false;
    }
    const size_t v2_id = collapse_cache.local().v2_id;
    if (!m_offset_params.sizing_collapse_min) { // see collapse_edge_before()
        m_vertex_attribute[v2_id].m_sizing_scalar = m_collapse_survivor_sizing.local();
    }
    // deform_others: every surviving cell at the survivor changed shape (v1 became v2); their
    // rest is stale.
    for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
        stamp_rest_cell(tid);
    }
    return true;
}

bool TopoOffsetTetMesh::collapse_before_vertex(
    const size_t v1_id,
    const size_t v2_id,
    const double edge_length)
{
    // Diagnostic: the flattest cell this collapse is about to reshape, read back by
    // record_flatness() in collapse_after_vertex().
    {
        double f = 1.;
        for (const size_t tid : get_one_ring_tids_for_vertex(v1_id)) {
            f = std::min(f, tet_flatness(tid));
        }
        m_collapse_parent_flatness.local() = f;
    }

    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse removes; it merges into v2, which keeps its position. A
    // vertex on any tracked boundary may be removed provided it merges onto a vertex of the same
    // class, the result stays inside its tags' envelopes, and the substructure link condition
    // survives. That is TetWild's rule for its input surface, applied uniformly.

    // Never both surfaces on one vertex: such a vertex sits at distance 0 from the input complex
    // and is asked to sit at target_distance from it at once. Refused here, and asserted
    // independently by check_no_vertex_on_both_surfaces().
    {
        const bool input = VE[v1_id].m_is_on_input || VE[v2_id].m_is_on_input;
        const bool offset = VE[v1_id].m_is_on_offset || VE[v2_id].m_is_on_offset;
        if (input && offset) {
            return false;
        }
    }

    // The front is always length-limited, whatever the pass says: it deliberately has no
    // envelope while it moves, so its sizing field is the only thing bounding its resolution.
    if (!m_collapse_limit_length && VE[v1_id].m_is_on_offset) {
        return false;
    }

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not leave
    // the particular surface it belongs to, and each class is checked separately.
    if (VE[v1_id].m_is_on_input && !VE[v2_id].m_is_on_input) {
        return false;
    }
    if (VE[v1_id].m_is_on_offset && !VE[v2_id].m_is_on_offset) {
        return false;
    }
    if (VE[v1_id].m_is_on_region && !VE[v2_id].m_is_on_region) {
        return false;
    }

    // open boundary: an order-2 vertex may not merge into a lower-order one.
    if (edge_length > 0 && m_vertex_attribute[v1_id].m_order == 2 &&
        m_vertex_attribute[v2_id].m_order < 2) {
        return false;
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // No offset criterion gates a collapse inside the loop: the offset envelope holds the
    // surface, so containment is the shared pass's job. Coarsening keeps an absolute bar,
    // because it runs after the loop and trades elements for nothing but the promise that the
    // result is still good. As in 2D.
    if (m_coarsen_mode && m_offset_potential) {
        double after = 0.;
        for (const Tuple& f : offset_surface_faces_live_at(v2_id)) {
            after = std::max(after, face_criterion_rel(f));
        }
        if (after > 1.0) {
            ++iter_cnt_collapse_offset_reject;
            return false;
        }
    }
    return true;
}

void TopoOffsetTetMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    // Diagnostic. Runs after the collapse is committed, so what it sees is real; the survivor's
    // ring is every cell the collapse reshaped.
    for (const size_t tid : get_one_ring_tids_for_vertex(v2_id)) {
        if (tet_amips(tid) >= kNeedleQuality) report_needle("COLLAPSE", tid, -1.);
        record_flatness("COLLAPSE", m_collapse_parent_flatness.local(), tid);
    }

    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;
    // Churn: v1 is the vertex being removed, so if a split created it this collapse undoes that
    // split. Same epoch means the collapse pass immediately following its own split pass took it
    // straight back out.
    {
        const uint32_t born = m_vertex_extra.at(v1_id).m_born_epoch;
        if (born != 0) {
            ++iter_cnt_recollapsed;
            if (born == m_op_epoch) ++iter_cnt_recollapsed_same_pass;
        }
    }

    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
    m_vertex_extra[v2_id].m_is_on_region =
        m_vertex_extra.at(v1_id).m_is_on_region || m_vertex_extra.at(v2_id).m_is_on_region;
    // The survivor now carries both vertices' geometry, so it lies on the union of their
    // boundaries. See VertexExtra::m_boundary_mask.
    m_vertex_extra[v2_id].m_boundary_mask |= m_vertex_extra.at(v1_id).m_boundary_mask;

    // The base calls this only once a collapse has actually gone through.
    ++iter_cnt_collapse;
}

} // namespace wmtk::components::topological_offset
