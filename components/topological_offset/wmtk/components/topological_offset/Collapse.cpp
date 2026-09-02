
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
    // collapsing_l2 scaled by the endpoints' sizing scalars. Do not re-add a copy here: it could
    // never fire, and in coarsen_mesh(), where the base clears the limit on purpose, it is wrong.

    if (!TetOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // Applied unconditionally, where the base asks only when both endpoints already sit on a
    // tracked simplex: the offset region is a thin shell, so a collapse with one endpoint in the
    // interior can still pinch its two sides together while every tracked surface survives.
    if (!substructure_link_condition(t)) {
        ++m_offset_collapse_refusals.sublink;
        return false;
    }
    return true;
}

bool TopoOffsetTetMesh::collapse_before_vertex(
    const size_t v1_id,
    const size_t v2_id,
    const double edge_length)
{
    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse removes; it merges into v2, which keeps its position.
    // Decimating the input surface to whatever the envelope tolerates is the contract, so there
    // is no blanket freeze on input-complex vertices. Beyond the rules below, a collapse is
    // constrained by substructure_link_condition() in collapse_edge_before(), the shared pass's
    // containment check against v2's tag envelopes, and collapse_after_connectivity().

    // Never both surfaces on one vertex: it would sit at distance 0 and at target_distance from
    // the input complex at once, so the offset surface through it could never converge. A
    // collapse is the only thing that could manufacture one, since collapse_after_vertex() ORs
    // the flags. Asserted independently by check_no_vertex_on_both_surfaces().
    {
        const bool input = VE[v1_id].m_is_on_input_complex || VE[v2_id].m_is_on_input_complex;
        const bool offset = VE[v1_id].m_is_on_offset || VE[v2_id].m_is_on_offset;
        if (input && offset) {
            ++m_offset_collapse_refusals.invariant;
            return false;
        }
    }

    // Do not re-add the Botsch-Kobbelt 4/3 long-edge refusal; measured worse -- see git history.

    // The base only knows that both endpoints are on some tracked surface. A vertex may not
    // leave the particular surface it belongs to: an input-complex vertex carries input
    // geometry, and an offset-boundary vertex carries the offset.
    if (edge_length > 0 && VE[v1_id].m_is_on_region && !VE[v2_id].m_is_on_region) {
        ++m_offset_collapse_refusals.class_region;
        return false;
    }
    if (edge_length > 0 && VE[v1_id].m_is_on_offset && !VE[v2_id].m_is_on_offset) {
        ++m_offset_collapse_refusals.class_offset;
        return false;
    }

    // open boundary
    if (edge_length > 0 && m_vertex_attribute[v1_id].m_order == 2 &&
        m_vertex_attribute[v2_id].m_order < 2) {
        ++m_offset_collapse_refusals.order2;
        return false;
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // No offset criterion gates a collapse inside the loop: Phase A holds the offset surface in
    // m_offset_envelope, so containment is the shared pass's job, and Phase B re-places the
    // surface every round.
    //
    // Coarsening keeps an absolute bar, because it runs after the loop and trades elements for
    // nothing but the promise that the result is still good. face_criterion_rel() is the max of
    // the AMIPS and offset-residual criteria, each normalized so 1.0 is its own tolerance.
    if (m_coarsen_mode && m_offset_potential) {
        double after = 0.;
        for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(v2_id))) {
            after = std::max(after, face_criterion_rel(f));
        }
        if (after > 1.0) {
            ++iter_cnt_collapse_offset_reject;
            return false;
        }
    }
    ++iter_cnt_collapse;
    return true;
}

void TopoOffsetTetMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;
    // Churn counters. v1 is the vertex being removed, so if a split created it this collapse
    // undoes that split; the same epoch means the collapse pass immediately after its own split
    // pass took it straight back out.
    {
        const uint32_t born = m_vertex_extra.at(v1_id).m_born_epoch;
        if (born != 0) {
            ++iter_cnt_recollapsed;
            if (m_op_epoch == born) ++iter_cnt_recollapsed_same_pass;
        }
    }
    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_region =
        m_vertex_extra.at(v1_id).m_is_on_region || m_vertex_extra.at(v2_id).m_is_on_region;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
    // Cannot collide with m_is_on_offset above: collapse_before_vertex() refused the merge that
    // would have put both on this vertex.
    m_vertex_extra[v2_id].m_is_on_input_complex = m_vertex_extra.at(v1_id).m_is_on_input_complex ||
                                                  m_vertex_extra.at(v2_id).m_is_on_input_complex;
    // Boundary-mask bits merge the same way the flags do: conservatively, onto the survivor. The
    // shared collapse's containment check ran before this OR, against v2's own mask -- the same
    // deliberate pre-OR staleness m_is_on_region has at that point.
    m_vertex_extra[v2_id].m_boundary_mask |= m_vertex_extra.at(v1_id).m_boundary_mask;
}

} // namespace wmtk::components::topological_offset
