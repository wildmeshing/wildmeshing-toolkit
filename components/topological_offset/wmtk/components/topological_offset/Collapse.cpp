
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

    // THE 4/5 LENGTH GATE, IN THE SPLIT'S UNITS. Toggleable while it is being evaluated.
    //
    // The base applies its length gate only in m_coarsen_mode, and unscaled -- there
    // collapsing_l2 is the USER's target length, deliberately not the sizing field (see
    // OptimizerParameters, coarsen_unbounded). That is the right question for a coarsening pass
    // and the wrong one here: an offset run drives m_sizing_scalar toward min_sizing_scalar, so
    // in exactly the refined regions where the churn lives every edge is far below an unscaled
    // 0.8*l and the gate would never fire.
    //
    // So this mirrors the SPLIT gate instead, term for term, and inverts the comparison:
    //
    //     split    refuses  length2 <  splitting_l2  * sizing_ratio^2   (l * 4/3 * s)
    //     collapse refuses  length2 >  collapsing_l2 * sizing_ratio^2   (l * 4/5 * s)
    //
    // with sizing_ratio the mean of the endpoints' scalars, as there. The band between them is
    // the length the optimizer is asking for at this location, and an edge inside it is left
    // alone by both passes.
    //
    // WHAT THIS DOES NOT FIX: 4/3 and 4/5 do not compose. An edge split at exactly its
    // threshold yields children of 2/3 * l * s, which is below 4/5 -- inside the collapse band
    // the moment they are created. Only a parent longer than 8/5 * l * s produces children the
    // gate protects. Getting the rest into the band is the interleaved smoothing pass's job.
    // Measured before this gate: 68.4% of all splits were undone by the collapse pass that ran
    // immediately after the split pass that created them.
    static const bool kCollapseLengthGate =
        offset_experiment_flag("WMTK_OFFSET_COLLAPSE_LENGTH_GATE", false);
    if (kCollapseLengthGate) {
        const size_t v1 = t.vid(*this);
        const size_t v2 = t.switch_vertex(*this).vid(*this);
        const double sizing_ratio =
            (m_vertex_attribute[v1].m_sizing_scalar + m_vertex_attribute[v2].m_sizing_scalar) / 2;
        if (get_length2(t) > m_params.collapsing_l2 * sizing_ratio * sizing_ratio) {
            ++iter_cnt_collapse_length_gate;
            return false;
        }
    }

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

    // BOTSCH-KOBBELT'S OTHER LEG: refuse a collapse that would CREATE an over-long edge.
    // Toggleable while it is being evaluated, like the 4/5 gate in collapse_edge_before.
    //
    // The 4/5 gate asks whether the edge being REMOVED is short enough to deserve removing.
    // That question cannot see the churn the measurement actually found, because 4/3 and 4/5 do
    // not compose: an edge split at exactly its threshold yields children at 2/3 * l * s, which
    // are already inside the collapse band the instant they are born, and the gate waves them
    // through. Only a parent past 8/5 * l * s makes children the gate protects. Botsch-Kobbelt,
    // whose hysteresis constants 4/3 and 4/5 are, is stable because its collapse asks the OTHER
    // question too -- would the merge produce an edge over 4/3? -- and merging two children
    // recreates exactly the parent that was just split, so that is the question that catches
    // this. The port never had this leg.
    //
    // Same units as the split gate, term for term: the created edge (v2,nb) is over-long when
    // its length2 exceeds splitting_l2 * sbar^2 with sbar the mean of the two endpoints'
    // sizing scalars -- so an edge this refuses to create is exactly an edge the split pass
    // would immediately split back.
    //
    // AN EDGE THAT WAS ALREADY OVER-LONG IS EXEMPT (the `after2 > before2` test, not a flat
    // refusal): the collapse may leave a bad edge no worse than it found it. That is what keeps
    // degenerate cleanup possible -- when v1 ~ v2 the two lengths are equal and nothing is
    // refused -- and it means the guard only ever blocks a genuine lengthening.
    //
    // VALENCE ESCAPE, and it is load-bearing. Uday measured the guard alone trapping slivers:
    // splits keep pumping valence into vertices whose relieving collapses the guard refuses,
    // reaching valence 240 with 32 vertices over the split threshold and an 18x runtime hit
    // from the retry grind. A collapse deletes the tets around (v1,v2) and is precisely the
    // operation that relieves valence, so relief takes precedence and the guard stands down.
    // Self-limiting rather than unbounded: with the escape the global max sits exactly at the
    // threshold, none over.
    static const bool kCreatedEdgeGuard =
        offset_experiment_flag("WMTK_OFFSET_CREATED_EDGE_GUARD", false);
    if (kCreatedEdgeGuard) {
        const auto thr = static_cast<size_t>(std::max(0, m_params.split_high_valence_threshold));
        const auto valence = [&](const size_t v) { return get_one_ring_tids_for_vertex(v).size(); };
        const auto ring = get_one_ring_vids_for_vertex(v1_id);
        bool relieve = thr > 0 && (valence(v1_id) > thr || valence(v2_id) > thr);
        if (thr > 0 && !relieve) {
            for (const size_t nb : ring) {
                if (valence(nb) > thr) {
                    relieve = true;
                    break;
                }
            }
        }
        if (relieve) {
            ++iter_cnt_collapse_valence_escape;
        } else {
            const Vector3d p1 = m_vertex_attribute[v1_id].m_posf;
            const Vector3d p2 = m_vertex_attribute[v2_id].m_posf;
            for (const size_t nb : ring) {
                if (nb == v1_id || nb == v2_id) continue;
                const double after2 = (p2 - m_vertex_attribute[nb].m_posf).squaredNorm();
                const double sbar = 0.5 * (m_vertex_attribute[v2_id].m_sizing_scalar +
                                           m_vertex_attribute[nb].m_sizing_scalar);
                if (after2 <= m_params.splitting_l2 * sbar * sbar) continue; // not over-long
                const double before2 = (p1 - m_vertex_attribute[nb].m_posf).squaredNorm();
                if (after2 > before2) { // creates or lengthens an over-long edge
                    ++iter_cnt_collapse_created_edge_gate;
                    return false;
                }
            }
        }
    }

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
