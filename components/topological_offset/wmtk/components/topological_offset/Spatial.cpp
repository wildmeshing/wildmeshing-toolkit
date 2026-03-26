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


bool TopoOffsetTetMesh::tet_consistent_topology(const size_t t_id) const
{
    // collect vertices in input/offset
    auto vs = oriented_tet_vids(t_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_attribute[vs[i]].label != 0) {
            vs_in.push_back(vs[i]);
        }
    }

    if (vs_in.size() == 3) { // must have exactly one face in input
        int num_faces_in_input = 0;
        for (int i = 0; i < 4; i++) {
            num_faces_in_input +=
                (m_face_attribute[tuple_from_face(t_id, i).fid(*this)].label != 0);
        }
        return (num_faces_in_input == 1);
    } else if (vs_in.size() == 4) { // must have one or two faces, no isolated edges in input
        std::vector<Tuple> faces_in_input;
        for (int i = 0; i < 4; i++) {
            Tuple f = tuple_from_face(t_id, i);
            if (m_face_attribute[f.fid(*this)].label != 0) {
                faces_in_input.push_back(f);
            }
        }
        if (faces_in_input.size() == 3) {
            return true;
        } else if (faces_in_input.size() == 2) { // must guarantee no floating edge
            std::vector<size_t> potential_edge; // will have two verts. logic
            for (const size_t& v_id : vs_in) {
                bool in1 =
                    (v_id == faces_in_input[0].vid(*this)) ||
                    (v_id == faces_in_input[0].switch_vertex(*this).vid(*this)) ||
                    (v_id == faces_in_input[0].switch_edge(*this).switch_vertex(*this).vid(*this));
                bool in2 =
                    (v_id == faces_in_input[1].vid(*this)) ||
                    (v_id == faces_in_input[1].switch_vertex(*this).vid(*this)) ||
                    (v_id == faces_in_input[1].switch_edge(*this).switch_vertex(*this).vid(*this));
                if (in1 && in2) potential_edge.push_back(v_id);
            }
            size_t opp_eid = tuple_from_edge({{potential_edge[0], potential_edge[1]}}).eid(*this);
            return (m_edge_attribute[opp_eid].label == 0);
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


bool TopoOffsetTriMesh::tri_consistent_topology(const size_t f_id) const
{
    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 3; i++) {
        if (m_vertex_attribute[vs[i]].label != 0) {
            vs_in.push_back(vs[i]);
        }
    }

    if (vs_in.size() == 3) { // must have exactly two edges in
        int num_es_in = 0;
        for (int i = 0; i < 3; i++) {
            num_es_in += (m_edge_attribute[tuple_from_edge(f_id, i).eid(*this)].label != 0);
        }
        // logger().info("\t{}", num_es_in == 2);
        return (num_es_in == 2);
    } else if (vs_in.size() == 2) { // must have exactly one edge in (between two 'in' verts)
        simplex::Edge e(vs_in[0], vs_in[1]);
        return (m_edge_attribute[edge_id_from_simplex(e)].label != 0);
    } else { // 0 or 1 vertex in, topology would be changed
        return false;
    }
}


} // namespace wmtk::components::topological_offset