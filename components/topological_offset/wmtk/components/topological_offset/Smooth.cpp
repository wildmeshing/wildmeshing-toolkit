
#include "TopoOffsetTetMesh.h"

#include <set>
#include <vector>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    const size_t vid = t.vid(*this);
    ++m_move_stats[size_t(vertex_class(vid))].attempted;

    // THE BASE'S smooth_before MINUS ITS BOUNDING-BOX REFUSAL, which is why this does not call
    // it. The base rounds the vertex and then refuses outright any vertex with a non-empty
    // on_bbox_faces, i.e. it FREEZES the domain wall. That is a stronger constraint than any
    // envelope and it is the second half of what made the wall a degeneracy trap: a tet resting
    // on it had its longest edge unsplittable (EdgeSplittingTet.cpp, now lifted) AND its three
    // wall vertices unmovable, so nothing could relieve the sliver from either side.
    //
    // Here the wall is a region boundary held in m_envelope like every other one, so its
    // vertices are smoothed and the containment check decides whether the move survives --
    // exactly the contract the input complex already gets. What keeps the wall a wall is that
    // check, not immobility.
    //
    // Rounding still has to happen, and its failure still refuses the move; the two outcomes the
    // base folded into one false are separated here because they mean different things.
    const bool rounded_now = round(t);
    if (!m_vertex_attribute[vid].m_is_rounded && !rounded_now) {
        ++m_move_stats[size_t(vertex_class(vid))].refused_before;
        ++m_smooth_trace.before_unrounded;
        return false;
    }
    // AN INPUT-COMPLEX VERTEX IS SMOOTHED, exactly as TetWild smooths a surface vertex.
    //
    // This used to refuse it: "the input complex must stay exactly where it is, it is the
    // geometry the offset is measured against". The second half is true and the first half does
    // not follow from it. What the offset is measured against is m_input_complex_bvh and
    // m_offset_potential, both built ONCE from the input as loaded and never rebuilt -- so the
    // distance field does not care where the mesh elements representing the complex end up. All
    // freezing them bought was a fixed set of vertices; what it cost is that every element
    // touching the input complex was unimprovable, and the faces pinned between two frozen
    // vertices could never reach stop_energy.
    //
    // TetWild's mechanism is the right one and it is already here: the vertex is smoothed by the
    // shared solver and held inside its tags' boundary envelopes -- the intersection of them at
    // a junction -- which smoothing_containment_envelope() answers for it, with the projection
    // step smoothing_mode = "projected" performs. That is a tolerance the input surface may
    // drift within, which is exactly what TetWild's own input surface gets -- not a licence to
    // move anywhere.
    //
    // Measured before this change: 14758 of 229276 smoothing attempts refused here.
    return true;
}

/**
 * @brief Every vertex goes through the shared smoother. Nothing is dispatched anywhere else.
 *
 * This used to fork: an offset-surface vertex was placed by smooth_after_offset_surface(), a
 * blend of a quadric fit to sampled offset planes and the offset-surface Laplacian, guarded by
 * its own hand-rolled inversion bisection; everything else got the shared two-stage AMIPS
 * smoother. The fork existed only because the Euclidean distance to the input complex has no
 * usable gradient, so the offset could not be an ENERGY -- and the price was that the one
 * surface the whole component exists to place was the one surface that bypassed the shared
 * smoother's line search, its exact inversion test and its accept checks.
 *
 * The smooth offset potential removes the reason: the offset is now the term
 * smoothing_extra_energy() adds to this vertex's objective, so it is minimised by the same
 * solver as AMIPS and defended by the same checks. See OffsetPotential.
 */
bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    if (m_vertex_extra[vid].m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
    } else {
        ++m_smooth_trace.interior_attempted;
    }
    // HOW FAR IT ACTUALLY GOT. Captured around the base call because that is what runs the
    // solver, the projection and every accept gate; on any refusal the base leaves the vertex
    // where it started, so a false answer contributes nothing. See MoveStats.
    const Vector3d before = m_vertex_attribute[vid].m_posf;
    // Both halves of the discriminator have to be read BEFORE the move: how many tracked faces
    // the containment check will have to work with, and where the vertex started relative to the
    // walls it claims. See WallMoveStats.
    const bool on_wall = !m_vertex_attribute[vid].on_bbox_faces.empty();
    const bool zero_tracked = on_wall && get_surface_faces_for_vertex(vid).faces().empty();
    const double dev_before = on_wall ? wall_offplane_deviation(vid) : 0.;

    const bool ok = TetOptimizerMesh::smooth_after(t);
    if (!ok) {
        return false;
    }

    // Phi's lower strata -- wires and isolated points -- used to need a dedicated point check
    // here, because the base's containment walks tracked FACES and a wire vertex has none
    // carrying its geometry. The per-tag envelopes closed that hole structurally: a wire or
    // isolated point only arises where two or more selected tags meet, so its vertex carries
    // several boundary-mask bits and smoothing_containment_envelope() hands the base an
    // IntersectionEnvelope whose is_outside(face) test -- and, before that, the pull toward the
    // most-violated member tube -- holds it at the junction.
    m_move_stats[size_t(vertex_class(vid))].add((m_vertex_attribute[vid].m_posf - before).norm());
    if (on_wall) {
        m_wall_moves.note(zero_tracked, dev_before, wall_offplane_deviation(vid));
    }
    return true;
}

void TopoOffsetTetMesh::log_smooth_trace() const
{
    const auto& s = m_smooth_trace;
    logger().info(
        "\tsmooth trace: attempted {} | before: bbox {}, unrounded {}, on-input {} | "
        "offset-surface vertices {}, interior {} ({})",
        s.attempted.load(),
        s.before_bbox.load(),
        s.before_unrounded.load(),
        s.before_on_input.load(),
        s.offset_attempted.load(),
        s.interior_attempted.load(),
        m_smooth_rejects.to_string());
}

bool TopoOffsetTetMesh::is_offset_face(const Tuple& f) const
{
    return is_offset_face(f.fid(*this));
}

bool TopoOffsetTetMesh::is_offset_face(const size_t fid) const
{
    return face_is_offset(fid);
}

std::vector<TopoOffsetTetMesh::Tuple> TopoOffsetTetMesh::get_offset_surface_faces_for_vertex(
    const Tuple& t) const
{
    std::vector<Tuple> result;
    std::set<size_t> seen_fids;

    const size_t vid = t.vid(*this);
    for (const size_t tid : get_one_ring_tids_for_vertex(t)) {
        const auto tet_vids = oriented_tet_vids(tid);

        // the 3 faces of the tet incident to vid are those obtained by omitting one of the
        // *other* 3 vertices; omitting vid itself gives the one face that does not contain it
        for (int skip = 0; skip < 4; ++skip) {
            if (tet_vids[skip] == vid) {
                continue;
            }

            std::array<size_t, 3> face_vids;
            int k = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) {
                    face_vids[k++] = tet_vids[j];
                }
            }

            const auto [face_tuple, unused_tid] = tuple_from_face(face_vids);
            const size_t fid = face_tuple.fid(*this);
            if (!seen_fids.insert(fid).second) {
                continue;
            }

            if (is_offset_face(face_tuple)) {
                result.push_back(face_tuple);
            }
        }
    }
    return result;
}

} // namespace wmtk::components::topological_offset
