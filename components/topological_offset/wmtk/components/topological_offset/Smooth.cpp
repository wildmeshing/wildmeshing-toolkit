
#include "TopoOffsetTetMesh.h"

#include <set>
#include <vector>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    ++m_smooth_trace.attempted;
    // Read before the base call: the base folds "on the bounding box" and "could not be
    // rounded to doubles" into one false, and those mean completely different things.
    const bool on_bbox = !m_vertex_attribute[t.vid(*this)].on_bbox_faces.empty();
    // rounds the vertex and refuses the bounding box
    if (!TetOptimizerMesh::smooth_before(t)) {
        if (on_bbox) {
            ++m_smooth_trace.before_bbox;
        } else {
            ++m_smooth_trace.before_unrounded;
        }
        return false;
    }
    // the input complex must stay exactly where it is: it is the geometry the offset is
    // measured against, not something to be improved
    if (m_vertex_extra[t.vid(*this)].m_is_on_input) {
        ++m_smooth_trace.before_on_input;
        return false;
    }
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
    if (m_vertex_extra[t.vid(*this)].m_is_on_offset) {
        ++m_smooth_trace.offset_attempted;
    } else {
        ++m_smooth_trace.interior_attempted;
    }
    return TetOptimizerMesh::smooth_after(t);
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
