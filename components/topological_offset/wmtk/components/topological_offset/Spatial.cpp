#include <igl/point_simplex_squared_distance.h>
#include <limits>
#include <queue>
#include "Circle.hpp"
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {


double TopoOffsetTriMesh::dist_to_input_complex(const Vector2d& q) const
{
    Vector2d closest_point;
    double sq_dist;
    m_input_complex_bvh.nearest_facet(q, closest_point, sq_dist);
    return sqrt(sq_dist);
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

        const double d_to_complex = dist_to_input_complex(circ.center());
        if ((d_to_complex - circ.radius()) > m_params.target_distance) {
            return false; // circle is outside --> tri outside offset
        }
        if ((d_to_complex + circ.radius()) < m_params.target_distance) {
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