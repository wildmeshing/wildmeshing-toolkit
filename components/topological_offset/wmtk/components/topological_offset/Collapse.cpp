
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

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& t)
{
    const size_t v1_id = t.vid(*this); // removed by the collapse
    const size_t v2_id = t.switch_vertex(*this).vid(*this); // survives, and keeps its position

    if (m_vertex_attribute[v1_id].m_is_on_surface || m_vertex_attribute[v2_id].m_is_on_surface) {
        // don't touch the input surface
        return false;
    }

    const int v1_label = m_vertex_attribute[v1_id].label;
    const int v2_label = m_vertex_attribute[v2_id].label;
    const int e_label = m_edge_attribute[t.eid(*this)].label;
    if (v1_label == v2_label && v1_label != e_label) {
        // if the edge is between two vertices of the same label, then the edge must have the same
        // label as well. Otherwise, don't collapse it.
        return false;
    }
    if (v1_label == 2 && e_label != 2) {
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
    for (const Tuple& tet : get_one_ring_tets_for_vertex(t)) {
        const size_t tid = tet.tid(*this);
        for (int i = 0; i < 6; ++i) {
            const Tuple e = tuple_from_edge(tid, i);
            const size_t ev0 = e.vid(*this);
            const size_t ev1 = e.switch_vertex(*this).vid(*this);
            cache.edge_labels[simplex::Edge(ev0, ev1)] = m_edge_attribute[e.eid(*this)].label;
        }
        for (int i = 0; i < 4; ++i) {
            const Tuple f = tuple_from_face(tid, i);
            const std::array<Tuple, 3> fv = get_face_vertices(f);
            cache.face_labels[simplex::Face(fv[0].vid(*this), fv[1].vid(*this), fv[2].vid(*this))] =
                m_face_attribute[f.fid(*this)].label;
        }
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_edge_after(const Tuple& t)
{
    auto& cache = edge_collapse_cache.local();
    const size_t v1_id = cache.v1_id;

    for (const auto& [edge, label] : cache.edge_labels) {
        if (edge.vertices()[0] == v1_id || edge.vertices()[1] == v1_id) {
            continue; // incident to the removed vertex -- this edge no longer exists
        }
        const Tuple e = tuple_from_edge({{edge.vertices()[0], edge.vertices()[1]}});
        m_edge_attribute[e.eid(*this)].label = label;
    }

    for (const auto& [face, label] : cache.face_labels) {
        const auto& fv = face.vertices();
        if (fv[0] == v1_id || fv[1] == v1_id || fv[2] == v1_id) {
            continue;
        }
        const auto [face_tuple, global_fid] = tuple_from_face({{fv[0], fv[1], fv[2]}});
        m_face_attribute[global_fid].label = label;
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

void TopoOffsetTetMesh::collapse_all_edges(double min_edge_len_ratio)
{
    const double min_len = min_edge_len_ratio * m_params.target_distance;
    const double min_len_sq = min_len * min_len;

    const std::vector<Tuple> edges = get_edges();
    std::vector<Tuple> new_edges; // required out-param, unused after the call

    for (const Tuple& e : edges) {
        if (!e.is_valid(*this)) {
            continue; // may already be gone from an earlier collapse
        }

        const size_t v0 = e.vid(*this);
        const size_t v1 = e.switch_vertex(*this).vid(*this);
        const double len_sq =
            (m_vertex_attribute[v0].m_posf - m_vertex_attribute[v1].m_posf).squaredNorm();
        // if (len_sq >= min_len_sq) {
        //     continue;
        // }

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
