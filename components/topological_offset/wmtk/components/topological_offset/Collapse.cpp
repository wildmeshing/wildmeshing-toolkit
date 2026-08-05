
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>

namespace wmtk::components::topological_offset {

double TopoOffsetTetMesh::face_normal_deviation(const Tuple& f) const
{
    const std::array<Tuple, 3> fv = get_face_vertices(f);
    const Vector3d p_a = m_vertex_attribute[fv[0].vid(*this)].m_posf;
    const Vector3d p_b = m_vertex_attribute[fv[1].vid(*this)].m_posf;
    const Vector3d p_c = m_vertex_attribute[fv[2].vid(*this)].m_posf;

    const Vector3d ab = p_b - p_a;
    const Vector3d ac = p_c - p_a;
    const Vector3d cross = ab.cross(ac);
    const double cross_norm = cross.norm();
    if (cross_norm < 1e-12) return 0.; // degenerate triangle, nothing to measure
    const Vector3d face_normal = cross / cross_norm;

    const Vector3d centroid = (p_a + p_b + p_c) / 3.;
    const Vector3d nearest = m_input_complex_bvh.nearest_point(centroid);
    const Vector3d diff = centroid - nearest;
    const double dist = diff.norm();
    if (dist < 1e-12) return 0.; // right on the input complex, no normal to compare against
    const Vector3d input_normal = diff / dist;

    // orientation independent: a triangle whose plane is parallel to the ideal offset plane
    // counts as aligned regardless of which way get_face_vertices() happened to wind it
    const double s = std::clamp(face_normal.cross(input_normal).norm(), -1., 1.);
    return (180. / M_PI) * std::asin(s);
}

double TopoOffsetTetMesh::max_offset_surface_normal_deviation_at_vertex(size_t vid) const
{
    double max_nd = 0.;
    for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(vid))) {
        max_nd = std::max(max_nd, face_normal_deviation(f));
    }
    return max_nd;
}

bool TopoOffsetTetMesh::is_offset_surface_edge(const Tuple& e) const
{
    const size_t v0 = e.vid(*this);
    const size_t v1 = e.switch_vertex(*this).vid(*this);

    for (const Tuple& tet : get_incident_tets_for_edge(e)) {
        const std::array<size_t, 4> vs = oriented_tet_vids(tet);

        // the two vertices of the tet that are not part of the edge are each the third
        // corner of one of the two faces of this tet that contain the edge
        for (const size_t w : vs) {
            if (w == v0 || w == v1) continue;
            const auto [face_tuple, unused_fid] = tuple_from_face({{v0, v1, w}});
            if (is_offset_surface_face(face_tuple)) {
                return true;
            }
        }
    }
    return false;
}

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& t)
{
    const size_t v1_id = t.vid(*this); // removed by the collapse
    const size_t v2_id = t.switch_vertex(*this).vid(*this); // survives, and keeps its position

    if (m_vertex_attribute[v1_id].m_is_on_surface || m_vertex_attribute[v2_id].m_is_on_surface) {
        // don't touch the input surface
        return false;
    }

    const auto& VA = m_vertex_attribute;

    ///check if on bbox/surface/boundary
    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) return false;
        for (int on_bbox : VA[v1_id].on_bbox_faces)
            if (std::find(
                    VA[v2_id].on_bbox_faces.begin(),
                    VA[v2_id].on_bbox_faces.end(),
                    on_bbox) == VA[v2_id].on_bbox_faces.end()) {
                return false;
            }
    }

    const int v1_label = VA[v1_id].label;
    const int v2_label = VA[v2_id].label;
    const int e_label = m_edge_attribute[t.eid(*this)].label;
    // if (v1_label == 2 && v2_label == 2 && e_label != 2) {
    //     // if the edge is between two vertices of the same label, then the edge must have the same
    //     // label as well. Otherwise, don't collapse it.
    //     return false;
    // }
    if (v1_label == 2 && e_label != 2) {
        // do not collapse away from the offset
        return false;
    }
    const bool v1_on_offset_surface = !get_offset_surface_faces_for_vertex(t).empty();
    const bool v2_on_offset_surface =
        !get_offset_surface_faces_for_vertex(t.switch_vertex(*this)).empty();
    if (v1_on_offset_surface && v2_on_offset_surface && !is_offset_surface_edge(t)) {
        // both endpoints are on the offset surface, but the edge connecting them isn't -- it
        // cuts across the surface rather than running along it, so collapsing it would pinch
        // two separate patches of the offset boundary together
        return false;
    }

    // OffsetCollapseBeforeInvariant analogue: don't collapse onto a vertex whose surviving
    // offset-surface patch is already poorly aligned with the input complex -- the reference
    // checks the survivor's cofaces pre-collapse, which is what this reduces to since v2's
    // position and its own incident faces are untouched by the collapse.
    if (max_offset_surface_normal_deviation_at_vertex(v2_id) >= m_max_normal_deviation_deg) {
        return false;
    }

    auto& cache = edge_collapse_cache.local();

    // NormalDeviationAfterInvariant analogue setup: remember how bad the offset surface
    // already was around this edge, so collapse_edge_after() only blocks a collapse that
    // makes a *good* patch worse, not one that was already over the threshold.
    cache.nd_before = std::max(
        max_offset_surface_normal_deviation_at_vertex(v1_id),
        max_offset_surface_normal_deviation_at_vertex(v2_id));

    cache.v1_id = v1_id;
    cache.edge_labels.clear();
    cache.face_labels.clear();

    // Snapshot edge/face labels around v1, keyed by vertex ids rather than eid()/fid(): those
    // are derived from whichever incident tet currently has the lowest id, and collapsing
    // removes the tets that touched the collapsed edge, which can hand that "lowest id" role
    // to a different surviving tet whose slot in m_edge_attribute/m_face_attribute was never
    // written for this particular edge/face. Restoring by vertex-id key in
    // collapse_edge_after() below fixes that back up.
    //
    // Edges/faces that touch v1 are additionally dropped when their tet is a "wing" tet (one
    // that also contains v2): those tets are removed by the collapse entirely, so a v1-w
    // edge/face coming only from a wing tet has no valid v2-w counterpart to reparent onto --
    // it just vanishes with the tet. Skipping them here keeps collapse_edge_after() from
    // reparenting them onto the *different*, already-correct edge/face that happens to share
    // the same vertex set (the wing tet's face opposite v1, for instance, is exactly {v2, w0,
    // w1} and is cached separately below since it does not touch v1).
    for (const Tuple& tet : get_one_ring_tets_for_vertex(t)) {
        const size_t tid = tet.tid(*this);
        const std::array<size_t, 4> tet_vids = oriented_tet_vids(tid);
        const bool is_wing_tet =
            std::find(tet_vids.begin(), tet_vids.end(), v2_id) != tet_vids.end();

        for (int i = 0; i < 6; ++i) {
            const Tuple e = tuple_from_edge(tid, i);
            const size_t ev0 = e.vid(*this);
            const size_t ev1 = e.switch_vertex(*this).vid(*this);
            if (is_wing_tet && (ev0 == v1_id || ev1 == v1_id)) continue;
            cache.edge_labels[simplex::Edge(ev0, ev1)] = m_edge_attribute[e.eid(*this)].label;
        }
        for (int i = 0; i < 4; ++i) {
            const Tuple f = tuple_from_face(tid, i);
            const std::array<Tuple, 3> fv = get_face_vertices(f);
            const size_t fv0 = fv[0].vid(*this);
            const size_t fv1 = fv[1].vid(*this);
            const size_t fv2 = fv[2].vid(*this);
            if (is_wing_tet && (fv0 == v1_id || fv1 == v1_id || fv2 == v1_id)) continue;
            cache.face_labels[simplex::Face(fv0, fv1, fv2)] = m_face_attribute[f.fid(*this)].label;
        }
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_edge_after(const Tuple& t)
{
    auto& cache = edge_collapse_cache.local();
    const size_t v1_id = cache.v1_id;
    const size_t v2_id = t.vid(*this); // survivor

    // Pass 1: edges/faces that never touched v1 -- restore directly. This must run before
    // pass 2 below, since pass 2 only fills in slots pass 1 left untouched.
    for (const auto& [edge, label] : cache.edge_labels) {
        if (edge.vertices()[0] == v1_id || edge.vertices()[1] == v1_id) {
            continue; // handled in pass 2
        }
        const Tuple e = tuple_from_edge({{edge.vertices()[0], edge.vertices()[1]}});
        m_edge_attribute[e.eid(*this)].label = label;
    }
    for (const auto& [face, label] : cache.face_labels) {
        const auto& fv = face.vertices();
        if (fv[0] == v1_id || fv[1] == v1_id || fv[2] == v1_id) {
            continue; // handled in pass 2
        }
        const auto [face_tuple, global_fid] = tuple_from_face({{fv[0], fv[1], fv[2]}});
        m_face_attribute[global_fid].label = label;
    }

    // Pass 2: edges/faces that had v1 as a corner (and, per the wing-tet filtering in
    // collapse_edge_before, are guaranteed to have a valid reparented counterpart) now live at
    // v2 instead. Their eid()/fid() slot was never written for this identity before, so
    // without this it silently reads back whatever default (label 0) was already there,
    // breaking anything that trusts edge/face labels near the collapse -- e.g. flood_fill()'s
    // connectivity graph, which only follows edges with label != 0.
    //
    // Only fill in slots pass 1 left at the default: v1 and v2 could already have shared a
    // vertex before the collapse (e.g. edge (v2, w) pre-existing alongside (v1, w)), in which
    // case pass 1 already wrote the correct, independent value for that slot and it must win.
    for (const auto& [edge, label] : cache.edge_labels) {
        const size_t a = edge.vertices()[0];
        const size_t b = edge.vertices()[1];
        if (a != v1_id && b != v1_id) continue; // handled in pass 1
        const size_t other = (a == v1_id) ? b : a;
        if (other == v2_id) continue; // the collapsed edge itself -- gone, not reparented

        const Tuple e = tuple_from_edge({{v2_id, other}});
        EdgeAttributes& ea = m_edge_attribute[e.eid(*this)];
        if (ea.label == 0) ea.label = label;
    }
    for (const auto& [face, label] : cache.face_labels) {
        const auto& fv = face.vertices();
        std::array<size_t, 2> others;
        int k = 0;
        bool touches_v1 = false;
        bool touches_v2 = false;
        for (const size_t v : fv) {
            if (v == v1_id) {
                touches_v1 = true;
            } else {
                if (v == v2_id) touches_v2 = true;
                others[k++] = v;
            }
        }
        if (!touches_v1) continue; // handled in pass 1
        if (touches_v2) continue; // also had v2 as a corner -- gone, not reparented

        const auto [face_tuple, global_fid] = tuple_from_face({{v2_id, others[0], others[1]}});
        FaceAttributes& fa = m_face_attribute[global_fid];
        if (fa.label == 0) fa.label = label;
    }

    // NormalDeviationAfterInvariant analogue: only reject a move that degrades an
    // already-good offset surface patch -- if it was already over the threshold before, don't
    // block a collapse from fixing (or merely not fixing) it.
    if (cache.nd_before < m_max_normal_deviation_deg) {
        const double nd_after = max_offset_surface_normal_deviation_at_vertex(t.vid(*this));
        if (nd_after >= m_max_normal_deviation_deg) {
            return false;
        }
    }

    return true;
}

void TopoOffsetTetMesh::collapse_all_edges()
{
    const std::vector<Tuple> edges = get_edges();
    std::vector<Tuple> new_edges; // required out-param, unused after the call

    for (const Tuple& e : edges) {
        if (!e.is_valid(*this)) {
            continue; // may already be gone from an earlier collapse
        }

        new_edges.clear();
        if (collapse_edge(e, new_edges)) {
            continue; // removed v0, kept v1
        }

        // v0 could not be removed (e.g. it is an input-complex vertex) -- try collapsing the
        // other way instead, which removes v1 and keeps v0
        new_edges.clear();
        collapse_edge(e.switch_vertex(*this), new_edges);
    }
}

} // namespace wmtk::components::topological_offset
