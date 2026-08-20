
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& t)
{
    // DIAGNOSTIC, off by default. See ab_no_collapse_after_first_round: the band is in a
    // one-for-one stalemate -- split creates an on-offset vertex and collapse removes one, net
    // -53 over six rounds -- so this switches the collapse half off from round 2 to establish
    // whether that stalemate is what holds the residual up. It gives up everything coarsening
    // provides and is not a fix.
    if (m_ab_collapses_disabled) {
        return false;
    }

    // THE 4/5 LENGTH GATE IS NOT HERE, AND MUST NOT BE RE-ADDED HERE.
    //
    // It lives in the shared collapse pass, applied to the CANDIDATE LIST -- an edge longer than
    // collapsing_l2 * sizing_ratio^2 (l * 4/5 * s, the mean of the endpoints' sizing scalars) is
    // filtered by is_weight_up_to_date and never offered to this function. That is TetWild's
    // placement, restored in origin/main b2f216e3ba, and it is scaled by the sizing field, which
    // is what an offset run needs: the sizing field is driven toward min_sizing_scalar, so an
    // UNSCALED 0.8*l gate would never fire in exactly the refined regions where the churn lives.
    //
    // This file briefly carried its own copy behind WMTK_OFFSET_COLLAPSE_LENGTH_GATE while the
    // placement was being evaluated. Removed once the shared gate landed: it is the same test on
    // an already-filtered list, so it could never fire -- except in coarsen_mesh(), where the
    // base clears m_collapse_limit_length on purpose and a length gate is exactly wrong. Doing
    // the ring work only to refuse it is also the slower half of the same decision.
    //
    // WHAT THE GATE DOES NOT FIX, since it will be asked again: 4/3 and 4/5 do not compose. An
    // edge split at exactly its threshold yields children of 2/3 * l * s, already inside the
    // collapse band the moment they are created; only a parent longer than 8/5 * l * s produces
    // children the gate protects. Measured before the gate, 68.4% of all splits were undone by
    // the collapse pass that ran immediately after the split pass that created them; with it,
    // prism tau=0.01 converged in the same 6 rounds with 32% fewer operations and 24% less wall.
    // Getting the remainder into the band is the interleaved smoothing pass's job.

    if (!TetOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // Unconditionally, where the base asks only when BOTH endpoints already sit on a tracked
    // simplex. That rule is right for tetwild and simwild; here the offset region is a thin
    // shell, and a collapse with one endpoint in the interior can still pinch its two sides
    // together while every tracked surface survives intact.
    return substructure_link_condition(t);
}

bool TopoOffsetTetMesh::collapse_before_vertex(
    const size_t v1_id,
    const size_t v2_id,
    const double edge_length)
{
    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse REMOVES (it merges into v2, which keeps its position). If
    // v1 belongs to the input complex or the domain boundary, removing it deletes a simplex of
    // a set that must not change at all, so the collapse is refused outright -- including the
    // input-onto-input case the surface rule below would otherwise allow. That case is not
    // hypothetical: it is what lets the shared engine decimate the input surface down to
    // whatever m_envelope tolerates.
    if (vertex_is_frozen(v1_id)) {
        return false;
    }

    // BOTSCH-KOBBELT'S OTHER LEG -- refusing a collapse that would CREATE an edge over
    // 4/3 * l * s -- was ported here from uday-offset3d c82383e3d7 and REMOVED after measurement.
    // Do not port it again without new evidence.
    //
    // On prism at tau = 0.01, a single convex model that converges in 6 rounds without it, the
    // guard alone did not converge: 4 rounds, 358k operations, 4468s, and 57909 tets over
    // stop_energy at once with the max energy RISING to 3.06e20 and the worst tet inverted. Run
    // with the 4/5 gate as well it was worse -- it threw in round 3, with one trapped sliver
    // (volume 1.45e-18, all four vertices on the offset surface at the sizing floor) whose six
    // edges were each either too long for 4/5 or refused by this guard, so no exit existed.
    // Rounds 1-4 looked like the best thing here -- 77k splits against the baseline's 271k at
    // the same residual, same-pass churn 29.3% against 68.4% -- which is exactly why the
    // measurement had to run to the end.
    //
    // Its cost is also not proportional to what it saves: 20.3M refusals over 4 rounds, each one
    // a candidate whose ring work was done and thrown away, so round 4 cut operations ~70% and
    // still spent MORE wall clock than the baseline.
    //
    // The 4/5 gate it was meant to complete is unconditional now and lives in the shared pass --
    // see collapse_edge_before().

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not
    // leave the particular surface it belongs to: an input-complex vertex carries input
    // geometry, and an offset-boundary vertex carries the offset.
    if (edge_length > 0 && VE[v1_id].m_is_on_input && !VE[v2_id].m_is_on_input) {
        return false;
    }
    if (edge_length > 0 && VE[v1_id].m_is_on_offset && !VE[v2_id].m_is_on_offset) {
        return false;
    }

    // open boundary
    if (edge_length > 0 && m_vertex_attribute[v1_id].m_order == 2 &&
        m_vertex_attribute[v2_id].m_order < 2) {
        return false;
    }

    // The bar collapse_after_connectivity() compares against: how bad the offset surface around
    // this edge already is. Captured here because the removed vertex's faces are gone afterwards.
    if (m_offset_potential) {
        double worst = 0.;
        for (const size_t v : {v1_id, v2_id}) {
            for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(v))) {
                worst = std::max(worst, face_criterion_rel(f));
            }
        }
        m_collapse_offset_rel_before.local() = worst;
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // A COLLAPSE IS ACCEPTED BY THE SAME CRITERION THE SMOOTHING MINIMISES.
    //
    // The smoother places an offset vertex by minimising w (Phi - c)^2 and the loop converges
    // when the Phi residual is inside tolerance everywhere on the offset surface, vertices and
    // face interiors alike. Every other operation has to answer to that same measure, or it can
    // undo in one collapse what the smoother spent an iteration achieving -- and it did:
    // measured on topological_offset_3d_convex, collapse alone took the offset surface from 1172
    // faces to 326 while the vertices it left behind sat at 0.95% of delta, which is what a
    // vertex-only view of the world calls perfect.
    //
    // Length gates cannot express this. They ask whether an edge is short relative to a sizing
    // target, which is a statement about the MESH; the criterion asks whether the surface is
    // still the offset, which is a statement about the GEOMETRY, and only the second one is what
    // the run is for. So the guard is the criterion itself, flat: after the collapse, no offset
    // face at the surviving vertex may be over tolerance.
    //
    // NON-DEGRADING, mirroring the AMIPS gate a few lines up rather than imposing an absolute
    // bar. The first version of this WAS absolute -- refuse if any offset face at the survivor
    // is over tolerance -- and that is a different rule from the one every other operation
    // obeys: it refuses a collapse for the state the mesh is already IN rather than for the
    // change the collapse makes. Early in a run most offset faces are over tolerance, so it
    // froze them, and in 2D it made a degenerate face permanent -- collapse is what removes
    // one, and its neighbour being over tolerance refused the removal forever, pinning AMIPS at
    // MAX_ENERGY for the rest of the run.
    // TWO BARS, because the coarsening pass is asking a different question from the main loop.
    //
    // In the LOOP the rule mirrors the AMIPS gate: refuse an operation that makes things worse.
    // That is right there, because the loop is still working -- most of the mesh is over
    // tolerance early on and an absolute bar would freeze it.
    //
    // COARSENING is not working, it is banking. It runs after the loop has finished and trades
    // elements for nothing except the promise that the result is still good, so the bar is
    // ABSOLUTE: both criteria -- AMIPS and the offset residual, which is what
    // face_criterion_rel() returns the max of -- must be inside tolerance afterwards. A collapse
    // that leaves anything over tolerance is not a saving, it is a regression with fewer
    // elements.
    if (m_offset_potential) {
        double after = 0.;
        for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(v2_id))) {
            after = std::max(after, face_criterion_rel(f));
        }
        const double bar = m_coarsen_mode ? 1.0 : m_collapse_offset_rel_before.local();
        if (after > bar) {
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
    // CHURN: v1 is the vertex being REMOVED. If a split created it, this collapse is undoing
    // that split. Same epoch means the collapse pass that immediately follows its own split
    // pass took it straight back out -- work the mesh never got to keep. A larger age means it
    // survived at least one further iteration before something decided against it.
    {
        const uint32_t born = m_vertex_extra.at(v1_id).m_born_epoch;
        if (born != 0) {
            ++iter_cnt_recollapsed;
            if (m_op_epoch == born) ++iter_cnt_recollapsed_same_pass;
        }
    }
    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
    // Boundary-mask bits merge the same way the flags do: conservatively, onto the survivor.
    // (The containment check in the shared collapse ran before this OR, against v2's own mask
    // -- the same deliberate pre-OR staleness m_is_on_input has at that point.)
    m_vertex_extra[v2_id].m_boundary_mask |= m_vertex_extra.at(v1_id).m_boundary_mask;
}

} // namespace wmtk::components::topological_offset
