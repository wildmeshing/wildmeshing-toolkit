#include "SimWildMesh.h"

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>

namespace wmtk::components::simwild {

bool SimWildMesh::split_before_cells(const Tuple& edge_tuple, const std::vector<Tuple>& parents)
{
    auto& cache = split_tag_cache.local();
    cache.tets.clear();

    const simplex::Edge edge = simplex_from_edge(edge_tuple);
    for (const Tuple& parent : parents) {
        const simplex::Tet tet = simplex_from_tet(parent);
        cache.tets[tet.opposite_edge(edge)] = m_tet_attribute.at(parent.tid(*this));
    }
    return true;
}

bool SimWildMesh::split_after_cells(
    const size_t v1,
    const size_t v2,
    const size_t v_new,
    const std::vector<Tuple>& children)
{
    auto& cache = split_tag_cache.local();
    cache.v_new = v_new;

    const auto children1 = get_incident_tets_for_edge(v1, v_new);
    const simplex::Edge edge1(v1, v_new);
    for (const Tuple& child : children1) {
        const simplex::Edge opposite = simplex_from_tet(child).opposite_edge(edge1);
        const auto it = cache.tets.find(opposite);
        if (it == cache.tets.end()) return false;
        m_tet_attribute[child.tid(*this)] = it->second;
    }

    const auto children2 = get_incident_tets_for_edge(v2, v_new);
    const simplex::Edge edge2(v2, v_new);
    for (const Tuple& child : children2) {
        const simplex::Edge opposite = simplex_from_tet(child).opposite_edge(edge2);
        const auto it = cache.tets.find(opposite);
        if (it == cache.tets.end()) return false;
        m_tet_attribute[child.tid(*this)] = it->second;
    }

    return children1.size() + children2.size() == children.size();
}

bool SimWildMesh::split_face_before(const Tuple& t)
{
    auto& c = m_face_split_cache;
    c.v = {
        {t.vid(*this),
         t.switch_vertex(*this).vid(*this),
         t.switch_edge(*this).switch_vertex(*this).vid(*this)}};
    Vector3d centroid = Vector3d::Zero();
    c.sizing_scalar = 0;
    for (const size_t v : c.v) {
        if (!m_vertex_attribute[v].m_is_rounded) {
            return false; // the new vertex is placed in doubles
        }
        centroid += m_vertex_attribute[v].m_posf / 3;
        c.sizing_scalar += m_vertex_attribute[v].m_sizing_scalar / 3;
    }
    c.position = m_face_split_position.value_or(centroid);
    c.partition_id = m_vertex_attribute[c.v[0]].partition_id;
    c.on_bbox_faces = wmtk::set_intersection(
        wmtk::set_intersection(
            m_vertex_attribute[c.v[0]].on_bbox_faces,
            m_vertex_attribute[c.v[1]].on_bbox_faces),
        m_vertex_attribute[c.v[2]].on_bbox_faces);
    std::array<size_t, 3> sorted = c.v;
    std::sort(sorted.begin(), sorted.end());
    c.face = m_face_attribute[std::get<1>(tuple_from_face(sorted))];

    c.tets.clear();
    c.faces.clear();
    std::vector<Tuple> parents = {t};
    if (const auto opp = t.switch_tetrahedron(*this)) {
        parents.push_back(*opp);
    }
    for (const Tuple& parent : parents) {
        size_t off = size_t(-1);
        for (const size_t v : oriented_tet_vids(parent)) {
            if (v != c.v[0] && v != c.v[1] && v != c.v[2]) {
                off = v;
            }
        }
        c.tets.emplace_back(off, m_tet_attribute[parent.tid(*this)]);
        for (int k = 0; k < 3; ++k) {
            std::array<size_t, 3> f = {{c.v[k], c.v[(k + 1) % 3], off}};
            std::sort(f.begin(), f.end());
            c.faces.emplace_back(f, m_face_attribute[std::get<1>(tuple_from_face(f))]);
        }
    }
    if (m_track_feature_edges) {
        std::vector<size_t> tids;
        for (const Tuple& parent : parents) {
            tids.push_back(parent.tid(*this));
        }
        feature_edges_cache(tids, c.changed_edges);
    }
    return true;
}

bool SimWildMesh::split_face_after(const Tuple& t)
{
    const auto& c = m_face_split_cache;
    // the new vertex: the corner of t's tet that is neither on the old face nor off it
    size_t v_new = size_t(-1);
    for (const size_t v : oriented_tet_vids(t)) {
        bool old = v == c.v[0] || v == c.v[1] || v == c.v[2];
        for (const auto& [off, _] : c.tets) {
            old = old || v == off;
        }
        if (!old) {
            v_new = v;
        }
    }
    if (v_new == size_t(-1)) {
        return false;
    }

    VertexAttributes va;
    va.m_posf = c.position;
    va.m_is_rounded = true;
    va.set_pos_to_posf();
    va.m_is_on_surface = c.face.m_is_surface_fs;
    va.m_order = va.m_is_on_surface ? 1 : 0;
    va.on_bbox_faces = c.on_bbox_faces;
    va.m_sizing_scalar = c.sizing_scalar;
    va.partition_id = c.partition_id;
    m_vertex_attribute[v_new] = va;
    if (m_track_feature_points) {
        m_feature_point_attribute[v_new].m_feature_point_id = NO_FEATURE_POINT;
    }

    // The tets' other faces keep their attributes (their ids follow the tets); the face's three
    // pieces take its attributes, each measured along the face with one corner replaced by the
    // new vertex; the faces inside the old tets are new.
    for (const auto& [f, attrs] : c.faces) {
        m_face_attribute[std::get<1>(tuple_from_face(f))] = attrs;
    }
    const int o = c.face.orientation_along(c.v);
    for (int k = 0; k < 3; ++k) {
        std::array<size_t, 3> piece = c.v;
        piece[k] = v_new;
        std::array<size_t, 3> sorted = piece;
        std::sort(sorted.begin(), sorted.end());
        auto& attrs = m_face_attribute[std::get<1>(tuple_from_face(sorted))];
        attrs = c.face;
        attrs.set_orientation_along(piece, o);
    }
    for (const auto& [off, _] : c.tets) {
        for (const size_t v : c.v) {
            std::array<size_t, 3> f = {{v, v_new, off}};
            std::sort(f.begin(), f.end());
            m_face_attribute[std::get<1>(tuple_from_face(f))].reset();
        }
    }

    // the children of each tet: the tets at the new vertex with that tet's corner off the face
    for (const size_t tid : get_one_ring_tids_for_vertex(v_new)) {
        const auto vs = oriented_tet_vids(tid);
        bool found = false;
        for (const auto& [off, attrs] : c.tets) {
            if (std::find(vs.begin(), vs.end(), off) != vs.end()) {
                m_tet_attribute[tid] = attrs;
                found = true;
            }
        }
        if (!found) {
            return false;
        }
        const Tuple child = tuple_from_tet(tid);
        if (is_inverted(child)) {
            return false; // split_face rolls back
        }
        m_tet_attribute[tid].m_quality = get_quality(child);
    }
    // no edge goes: each keeps its tags in the children, and the new ones, at v_new, have none
    if (m_track_feature_edges) {
        feature_edges_restore_region(get_one_ring_tids_for_vertex(v_new), c.changed_edges);
    }
    m_last_face_split_vertex = v_new;
    return true;
}

bool SimWildMesh::split_tet_before(const Tuple& t)
{
    auto& c = m_tet_split_cache;
    c.v = oriented_tet_vids(t);
    Vector3d centroid = Vector3d::Zero();
    c.sizing_scalar = 0;
    for (const size_t v : c.v) {
        if (!m_vertex_attribute[v].m_is_rounded) {
            return false; // the new vertex is placed in doubles
        }
        centroid += m_vertex_attribute[v].m_posf / 4;
        c.sizing_scalar += m_vertex_attribute[v].m_sizing_scalar / 4;
    }
    c.position = centroid;
    c.partition_id = m_vertex_attribute[c.v[0]].partition_id;
    c.on_bbox_faces = m_vertex_attribute[c.v[0]].on_bbox_faces;
    for (int k = 1; k < 4; ++k) {
        c.on_bbox_faces =
            wmtk::set_intersection(c.on_bbox_faces, m_vertex_attribute[c.v[k]].on_bbox_faces);
    }
    c.tet = m_tet_attribute[t.tid(*this)];
    c.faces.clear();
    for (int k = 0; k < 4; ++k) {
        std::array<size_t, 3> f = {{c.v[(k + 1) % 4], c.v[(k + 2) % 4], c.v[(k + 3) % 4]}};
        std::sort(f.begin(), f.end());
        c.faces.emplace_back(f, m_face_attribute[std::get<1>(tuple_from_face(f))]);
    }
    if (m_track_feature_edges) {
        feature_edges_cache({t.tid(*this)}, c.changed_edges);
    }
    return true;
}

bool SimWildMesh::split_tet_after(const Tuple& t)
{
    const auto& c = m_tet_split_cache;
    // the new vertex: the corner of t's tet that is not one of the old tet's
    size_t v_new = size_t(-1);
    for (const size_t v : oriented_tet_vids(t)) {
        if (std::find(c.v.begin(), c.v.end(), v) == c.v.end()) {
            v_new = v;
        }
    }
    if (v_new == size_t(-1)) {
        return false;
    }

    VertexAttributes va;
    va.m_posf = c.position;
    va.m_is_rounded = true;
    va.set_pos_to_posf();
    va.m_is_on_surface = false; // inside the old tet
    va.m_order = 0;
    va.on_bbox_faces = c.on_bbox_faces;
    va.m_sizing_scalar = c.sizing_scalar;
    va.partition_id = c.partition_id;
    m_vertex_attribute[v_new] = va;
    if (m_track_feature_points) {
        m_feature_point_attribute[v_new].m_feature_point_id = NO_FEATURE_POINT;
    }

    // the tet's four faces keep their attributes (their ids follow the tets); the six faces
    // inside it are new
    for (const auto& [f, attrs] : c.faces) {
        m_face_attribute[std::get<1>(tuple_from_face(f))] = attrs;
    }
    for (int i = 0; i < 4; ++i) {
        for (int j = i + 1; j < 4; ++j) {
            std::array<size_t, 3> f = {{c.v[i], c.v[j], v_new}};
            std::sort(f.begin(), f.end());
            m_face_attribute[std::get<1>(tuple_from_face(f))].reset();
        }
    }

    for (const size_t tid : get_one_ring_tids_for_vertex(v_new)) {
        m_tet_attribute[tid] = c.tet;
        const Tuple child = tuple_from_tet(tid);
        if (is_inverted(child)) {
            return false; // split_tet rolls back
        }
        m_tet_attribute[tid].m_quality = get_quality(child);
    }
    // no edge goes: each keeps its tags in the children, and the new ones, at v_new, have none
    if (m_track_feature_edges) {
        feature_edges_restore_region(get_one_ring_tids_for_vertex(v_new), c.changed_edges);
    }
    return true;
}

bool SimWildMesh::split_adjust_position(const size_t v_new, const std::vector<Tuple>& children)
{
    if (!m_voronoi_split_fn || !m_vertex_attribute[v_new].m_is_rounded) return true;

    const auto& shared_cache = split_cache.local();
    const size_t v1 = shared_cache.v1_id;
    const size_t v2 = shared_cache.v2_id;
    auto& p = m_vertex_attribute[v_new].m_posf;

    Vector3d p0 = m_vertex_attribute[v1].m_posf;
    Vector3d p1 = m_vertex_attribute[v2].m_posf;
    if (m_voronoi_split_fn(p0) >= 0) std::swap(p0, p1);

    for (int i = 0; i < 20; ++i) {
        p = 0.5 * (p0 + p1);
        m_vertex_attribute[v_new].set_pos_to_posf(); // p aliases m_posf

        bool inverted = false;
        for (const Tuple& child : children) {
            if (is_inverted(child)) {
                inverted = true;
                break;
            }
        }
        if (inverted || (p1 - p0).squaredNorm() < 1e-20) break;

        if (m_voronoi_split_fn(p) < 0) {
            p0 = p;
        } else {
            p1 = p;
        }
    }

    for (const Tuple& child : children) {
        if (!is_inverted(child)) continue;
        logger().warn("Voronoi split inverted a cell; reverting to the TetWild midpoint");
        p = 0.5 * (m_vertex_attribute[v1].m_posf + m_vertex_attribute[v2].m_posf);
        m_vertex_attribute[v_new].set_pos_to_posf(); // p aliases m_posf
        break;
    }
    return true;
}

} // namespace wmtk::components::simwild
