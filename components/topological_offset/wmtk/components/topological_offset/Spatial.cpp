#include <igl/point_simplex_squared_distance.h>
#include <limits>
#include <queue>
#include "Circle.hpp"
#include "Sphere.hpp"
#include "TopoOffsetTetMesh.h"
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {


bool TopoOffsetTetMesh::tet_is_in_offset_conservative(const size_t t_id, const double threshold_r)
    const
{
    std::queue<Sphere> q;
    q.emplace(*this, t_id);

    bool initial = true;
    while (!q.empty()) {
        Sphere sph = q.front();
        q.pop();

        if (sph.radius() < threshold_r) {
            if (initial) {
                logger().warn(
                    "Initial approximating sphere for tet is smaller than threshold radius. "
                    "Tet not considered for offset growth. Decrease relative_ball_threshold.");
            }
            return false; // sphere too small --> conservatively, tet outside offset
        }
        initial = false;

        const bool overlap = sph.overlaps_tet(*this, t_id);
        if (!overlap) {
            continue;
        }

        const double d = m_input_complex_bvh.dist(sph.center());
        if ((d - sph.radius()) > m_params.target_distance) {
            return false; // sphere outside --> tet outside offset
        }
        if ((d + sph.radius()) < m_params.target_distance) {
            continue; // sphere inside --> check next circle
        }

        sph.refine(q); // undetermined --> refine further
    }

    return true; // all spheres inside --> tet inside offset
}


bool TopoOffsetTetMesh::offset_tet_consistent_topology(const size_t t_id) const
{
    // collect vertices in input/offset
    auto vs = oriented_tet_vids(t_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_attribute[vs[i]].label == 1) { // bad
            log_and_throw_error("Input adjacent tet given to offset_tet_consistent_topology");
        }
        if (m_vertex_attribute[vs[i]].label == 2) { // otherwise if label is 2 (offset)
            vs_in.push_back(vs[i]);
        }
    }

    bool offset_consistent;
    if (vs_in.size() == 3) { // must have exactly one face in input
        int num_faces_in_input = 0;
        for (int i = 0; i < 4; i++) {
            num_faces_in_input +=
                (m_face_attribute[tuple_from_face(t_id, i).fid(*this)].label == 2);
        }
        offset_consistent = (num_faces_in_input == 1);
    } else if (vs_in.size() == 4) { // must have two or three faces, no isolated edges in input
        std::vector<Tuple> faces_in_input;
        for (int i = 0; i < 4; i++) {
            Tuple f = tuple_from_face(t_id, i);
            if (m_face_attribute[f.fid(*this)].label == 2) {
                faces_in_input.push_back(f);
            }
        }
        if (faces_in_input.size() == 3) {
            offset_consistent = true;
        } else if (faces_in_input.size() == 2) { // must guarantee no floating edge
            std::vector<size_t> face1_vs;
            face1_vs.push_back(faces_in_input[0].vid(*this));
            face1_vs.push_back(faces_in_input[0].switch_vertex(*this).vid(*this));
            face1_vs.push_back(
                faces_in_input[0].switch_edge(*this).switch_vertex(*this).vid(*this));
            std::sort(face1_vs.begin(), face1_vs.end());
            std::vector<size_t> face2_vs;
            face2_vs.push_back(faces_in_input[1].vid(*this));
            face2_vs.push_back(faces_in_input[1].switch_vertex(*this).vid(*this));
            face2_vs.push_back(
                faces_in_input[1].switch_edge(*this).switch_vertex(*this).vid(*this));
            std::sort(face2_vs.begin(), face2_vs.end());
            std::vector<size_t> shared_edge_vids = set_intersection(face1_vs, face2_vs);
            simplex::Tet tet_simp = simplex_from_tet(t_id);
            simplex::Edge oppo_edge =
                tet_simp.opposite_edge(simplex::Edge(shared_edge_vids[0], shared_edge_vids[1]));
            size_t opp_eid = tuple_from_edge(oppo_edge.vertices()).eid(*this);
            offset_consistent = (m_edge_attribute[opp_eid].label == 0);
        } else { // no bueno
            offset_consistent = false;
        }
    } else { // topology would be changed
        offset_consistent = false;
    }

    if (!offset_consistent) {
        return false;
    }

    // check if any topologies would be changed
    for (const int64_t& tag : m_tet_attribute[t_id].tag) {
        if (!tag_tet_consistent_topology(t_id, tag)) {
            return false;
        }
    }
    return true;
}


bool TopoOffsetTetMesh::tag_tet_consistent_topology(size_t t_id, int64_t tag) const
{
    // collect boundary vertices
    auto vs = oriented_tet_vids(t_id);
    std::vector<size_t> boundary_vs;
    for (const size_t& v : vs) {
        auto one_ring_tids = get_one_ring_tids_for_vertex(v);
        bool v_in = false;
        bool first_in = m_tet_attribute[one_ring_tids[0]].tag.count(tag) != 0;
        for (const size_t& tid : one_ring_tids) {
            bool t_in = m_tet_attribute[tid].tag.count(tag) != 0;
            if (t_in != first_in) {
                v_in = true;
                break;
            }
        }
        if (v_in) {
            boundary_vs.push_back(v);
        }
    }

    // collect boundary edges
    std::map<simplex::Edge, bool> boundary_edges;
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            simplex::Edge e(vs[i], vs[j]);
            Tuple etup = tuple_from_edge(e.vertices());
            auto incident_tids = get_incident_tids_for_edge(etup);
            bool e_in = false;
            bool first_in = m_tet_attribute[incident_tids[0]].tag.count(tag) != 0;
            for (const size_t& tid : incident_tids) {
                bool t_in = m_tet_attribute[tid].tag.count(tag) != 0;
                if (t_in != first_in) {
                    e_in = true;
                    break;
                }
            }
            boundary_edges[e] = e_in;
        }
    }

    // collect bounndary faces
    std::map<simplex::Face, bool> boundary_faces;
    for (int i = 0; i < 4; i++) {
        simplex::Face f(vs[i], vs[(i + 1) % 4], vs[(i + 2) % 4]);
        auto [ftup, fid] = tuple_from_face(f);

        auto other = ftup.switch_tetrahedron(*this);
        if (other) {
            bool t1_in = m_tet_attribute[ftup.tid(*this)].tag.count(tag) != 0;
            bool t2_in = m_tet_attribute[other.value().tid(*this)].tag.count(tag) != 0;
            boundary_faces[f] = (t1_in && !t2_in) || (!t1_in && t2_in);
        } else {
            boundary_faces[f] = m_tet_attribute[ftup.tid(*this)].tag.count(tag) != 0;
        }
    }

    int num_boundary_faces = 0;
    for (const auto& pair : boundary_faces) {
        num_boundary_faces += (pair.second ? 1 : 0);
    }

    // check criteria (same logic as offset_tet_consistent_topology)
    if (boundary_vs.size() == 3) { // must have exactly one face in input
        return (num_boundary_faces == 1);
    } else if (boundary_vs.size() == 4) { // must have two or three faces, no isolated edges
        if (num_boundary_faces == 3) {
            return true;
        } else if (num_boundary_faces == 2) { // must guarantee no floating edge
            std::vector<simplex::Face> boundary_f_simps;
            for (const auto& pair : boundary_faces) {
                if (pair.second) {
                    boundary_f_simps.push_back(pair.first);
                }
            }
            simplex::Tet tet_simp(vs[0], vs[1], vs[2], vs[3]);
            simplex::Edge opp_edge(
                tet_simp.opposite_vertex(boundary_f_simps[0]).id(),
                tet_simp.opposite_vertex(boundary_f_simps[1]).id());
            return (!boundary_edges[opp_edge]);
        } else { // no bueno
            return false;
        }
    } else { // topology would be changed
        return false;
    }
}


bool TopoOffsetTriMesh::tri_is_in_offset_conservative(const size_t f_id, const double threshold_r)
    const
{
    std::queue<Circle> q;
    q.emplace(*this, f_id);

    bool initial = true;
    while (!q.empty()) {
        Circle circ = q.front();
        q.pop();

        if (circ.radius() < threshold_r) {
            if (initial) {
                logger().warn(
                    "Initial approximating circle for tri is smaller than threshold radius. "
                    "Tri not considered for offset growth. Decrease relative_ball_threshold.");
            }
            return false; // circle too small --> tri outside offset
        }
        initial = false;

        const bool overlap = circ.overlaps_tri(*this, f_id);
        if (!overlap) {
            continue;
        }

        const double d = m_input_complex_bvh.dist(circ.center());
        if ((d - circ.radius()) > m_params.target_distance) {
            return false; // circle is outside --> tri outside offset
        }
        if ((d + circ.radius()) < m_params.target_distance) {
            continue; // circle is inside --> check next circle
        }

        circ.refine(q); // undetermined --> subdivide
    }

    return true; // all circles inside --> tri inside offset
}


bool TopoOffsetTriMesh::offset_tri_consistent_topology(const size_t f_id) const
{
    bool offset_consistent;

    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> vs_in;
    for (const size_t& v : vs) {
        if (m_vertex_attribute[v].label == 1) {
            log_and_throw_error(
                "Input adjacent tri (id {}) given to offset_tri_consistent_topology",
                f_id);
        }
        if (m_vertex_attribute[v].label == 2) {
            vs_in.push_back(v);
        }
    }

    if (vs_in.size() == 3) { // must have exactly two edges in
        int num_es_in = 0;
        for (int i = 0; i < 3; i++) {
            num_es_in += (m_edge_attribute[tuple_from_edge(f_id, i).eid(*this)].label == 2);
        }
        // logger().info("\t{}", num_es_in == 2);
        offset_consistent = (num_es_in == 2);
    } else if (vs_in.size() == 2) { // must have exactly one edge in (between two 'in' verts)
        simplex::Edge e(vs_in[0], vs_in[1]);
        offset_consistent = (m_edge_attribute[edge_id_from_simplex(e)].label == 2);
    } else { // 0 or 1 vertex in, topology would be changed
        offset_consistent = false;
    }

    if (!offset_consistent) { // definitely cannot add
        return false;
    }

    // check if any topologies changed by adding this tri to offset
    for (const int64_t& id : m_face_attribute[f_id].tag) {
        if (!tag_tri_consistent_topology(f_id, id)) {
            return false;
        }
    }
    return true;
}


bool TopoOffsetTriMesh::tag_tri_consistent_topology(size_t f_id, int64_t tag) const
{
    // collect boundary vertices
    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> boundary_vs;
    for (const size_t& v : vs) {
        auto one_ring_fids = get_one_ring_fids_for_vertex(v);
        bool v_in = false;
        bool first_in = m_face_attribute[one_ring_fids[0]].tag.count(tag) != 0;
        for (const size_t& fid : one_ring_fids) {
            bool f_in = m_face_attribute[fid].tag.count(tag) != 0;
            if (f_in != first_in) {
                v_in = true;
                break;
            }
        }
        if (v_in) {
            boundary_vs.push_back(v);
        }
    }

    // collect boundary edges
    std::map<simplex::Edge, bool> boundary_edges;
    for (int i = 0; i < 3; i++) {
        simplex::Edge e(vs[i], vs[(i + 1) % 3]);
        auto [etup, e_id] = tuple_from_edge(e.vertices());

        auto other = etup.switch_face(*this);
        if (other) {
            bool f1_in = m_face_attribute[etup.fid(*this)].tag.count(tag) != 0;
            bool f2_in = m_face_attribute[other.value().fid(*this)].tag.count(tag) != 0;
            boundary_edges[e] = (f1_in && !f2_in) || (!f1_in && f2_in);
        } else {
            boundary_edges[e] = m_face_attribute[etup.fid(*this)].tag.count(tag) != 0;
        }
    }

    // check criteria (same logic as offset_tri_consistent_topology)
    if (boundary_vs.size() == 3) {
        int num_boundary_edges = 0;
        for (const auto& pair : boundary_edges) {
            if (pair.second) {
                num_boundary_edges++;
            }
        }
        return (num_boundary_edges == 2);
    } else if (boundary_vs.size() == 2) {
        simplex::Edge e(boundary_vs[0], boundary_vs[1]);
        return boundary_edges[e];
    } else {
        return false;
    }
}


} // namespace wmtk::components::topological_offset
