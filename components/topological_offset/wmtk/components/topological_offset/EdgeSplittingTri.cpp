#include <set>
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {

//// TriMesh splitting

// v1 must have label 1/2, v2 has label 0
void TopoOffsetTriMesh::edge_split_binary_search(const size_t v1, const size_t v2, Vector2d& p_new)
    const
{
    const Vector2d v1_pos = m_vertex_attribute[v1].m_posf;
    const Vector2d v2_pos = m_vertex_attribute[v2].m_posf;
    edge_split_binary_search(v1_pos, v2_pos, p_new);
}
void TopoOffsetTriMesh::edge_split_binary_search(
    const Vector2d& v1_pos,
    const Vector2d& v2_pos,
    Vector2d& p_new) const
{
    const double eps = m_params.edge_search_term_len;
    Vector2d p1 = v1_pos;
    Vector2d p2 = v2_pos;
    while ((p2 - p1).norm() > eps) {
        Vector2d p = (p1 + p2) / 2.0;
        double dist = m_input_complex_bvh.dist(p);
        if (dist < m_params.target_distance) {
            p1 = p;
        } else {
            p2 = p;
        }
    }
    p_new = (p1 + p2) / 2.0;
}


void TopoOffsetTriMesh::edge_split_log_root_find(const size_t v1, const size_t v2, Vector2d& p_new)
    const
{
    const double eps = m_params.edge_search_term_len;
    const Vector2d v1_pos = m_vertex_attribute[v1].m_posf;
    const Vector2d v2_pos = m_vertex_attribute[v2].m_posf;
    const Vector2d v_hat = (v2_pos - v1_pos).normalized();
    const double l_max = (v2_pos - v1_pos).norm();
    if (l_max < eps) {
        logger().warn("near degenerate edge given to edge_split_root_find. splitting at midpoint");
        p_new = (v1_pos + v2_pos) / 2.0;
        return;
    }

    double l_curr = eps;
    Vector2d p1 = v1_pos;
    Vector2d p2 = v1_pos + (l_curr * v_hat);
    double f2 = m_input_complex_bvh.dist(p2);
    while (f2 < m_params.target_distance) {
        l_curr *= 2.0;
        p2 = v1_pos + (l_curr * v_hat);
        f2 = m_input_complex_bvh.dist(p2);
        if (l_curr > l_max) {
            if (f2 < m_params.target_distance) { // entire edge is (likely) within offset.
                logger().warn(
                    "edge (likely) entirely in offset for root finding edge split. Splitting edge "
                    "at 99\% of length");
                p_new = v1_pos + (0.99 * l_max * v_hat);
                return;
            } else { // zero is (likely) between last two chunks. use binary search here
                p1 = v1_pos + (0.5 * l_curr * v_hat);
                p2 = v2_pos;
                edge_split_binary_search(p1, p2, p_new);
                return;
            }
        }
    }
    p1 = v1_pos + (0.5 * l_curr * v_hat);
    edge_split_binary_search(p1, p2, p_new);
}


void TopoOffsetTriMesh::edge_split_sphere_tracing(const size_t v1, const size_t v2, Vector2d& p_new)
    const
{
    const double eps = m_params.edge_search_term_len;
    const Vector2d v1_pos = m_vertex_attribute[v1].m_posf;
    const Vector2d v2_pos = m_vertex_attribute[v2].m_posf;
    const double L = (v2_pos - v1_pos).norm();
    const Vector2d u_hat = (v2_pos - v1_pos) / L;
    const double D = m_params.target_distance;

    // if near degenerate, fall to midpoint split
    if (L < eps) {
        p_new = (v1_pos + v2_pos) * 0.5;
        return;
    }

    p_new = v1_pos;
    double t = 0;
    double dp = m_input_complex_bvh.dist(p_new);
    while (D - dp > eps) { // note: guaranteed that D - dp > 0
        t += D - dp;
        if (t >= L) { // entire edge is in offset. return split at 0.99 edge length
            p_new = v1_pos + (0.99 * L * u_hat);
            return;
        }
        p_new = v1_pos + (t * u_hat);
        dp = m_input_complex_bvh.dist(p_new);
    }

    // snap solution point away from v1 and v2 if too close
    if ((p_new - v1_pos).norm() < (0.01 * L)) {
        p_new = v1_pos + (0.01 * L * u_hat);
    } else if ((p_new - v2_pos).norm() < (0.01 * L)) {
        p_new = v1_pos + (0.99 * L * u_hat);
    }
}


bool TopoOffsetTriMesh::split_edge_before(const Tuple& t)
{
    // load and clear cache
    auto& cache = edge_split_cache.local();
    cache.existing_eattr.clear();
    cache.opp_v_fattr.clear();

    size_t e_id = t.eid(*this);

    // new vertex
    cache.v1_id = t.vid(*this);
    cache.v2_id = t.switch_vertex(*this).vid(*this);
    Vector2d p1 = m_vertex_attribute[cache.v1_id].m_posf;
    Vector2d p2 = m_vertex_attribute[cache.v2_id].m_posf;
    Vector2d p_new;
    if (m_edge_split_mode == EdgeSplitMode::Midpoint) {
        p_new = (p1 + p2) / 2.0;
    } else if (m_edge_split_mode == EdgeSplitMode::BinarySearch) {
        if ((m_vertex_attribute[cache.v1_id].label == 0) &&
            (m_vertex_attribute[cache.v2_id].label != 0)) {
            edge_split_binary_search(cache.v2_id, cache.v1_id, p_new);
        } else if (
            (m_vertex_attribute[cache.v1_id].label != 0) &&
            (m_vertex_attribute[cache.v2_id].label == 0)) {
            edge_split_binary_search(cache.v1_id, cache.v2_id, p_new);
        } else {
            log_and_throw_error(
                "Invalid edge [{}] for binary search split. Both vertices in/out of offset/input "
                "complex.",
                e_id);
        }
    } else if (m_edge_split_mode == EdgeSplitMode::Initial) {
        // determine split distance
        double edge_len = (p1 - p2).norm();
        double split_dist = (edge_len / 2.0);
        if (m_params.target_distance < (edge_len / 2.0)) {
            split_dist = (m_params.target_distance / 2.0); // hacky. will be split again later
        }

        // set split point
        if ((m_vertex_attribute[cache.v1_id].label == 0) &&
            (m_vertex_attribute[cache.v2_id].label == 1)) {
            p_new = ((p1 - p2) * (split_dist / edge_len)) + p2;
        } else if (
            (m_vertex_attribute[cache.v1_id].label == 1) &&
            (m_vertex_attribute[cache.v2_id].label == 0)) {
            p_new = ((p2 - p1) * (split_dist / edge_len)) + p1;
        } else {
            log_and_throw_error(
                "Invalid edge [{}] for initial edge split. Both vertices in/out of input "
                "complex.",
                e_id);
        }
    } else if (m_edge_split_mode == EdgeSplitMode::LogRootFind) {
        if ((m_vertex_attribute[cache.v1_id].label == 0) &&
            (m_vertex_attribute[cache.v2_id].label != 0)) {
            edge_split_log_root_find(cache.v2_id, cache.v1_id, p_new);
        } else if (
            (m_vertex_attribute[cache.v1_id].label != 0) &&
            (m_vertex_attribute[cache.v2_id].label == 0)) {
            edge_split_log_root_find(cache.v1_id, cache.v2_id, p_new);
        } else {
            log_and_throw_error(
                "Invalid edge [{}] for log root finding edge split. Both vertices in/out of "
                "offset/input complex.",
                e_id);
        }
    } else if (m_edge_split_mode == EdgeSplitMode::SphereTracing) {
        if ((m_vertex_attribute[cache.v1_id].label == 0) &&
            (m_vertex_attribute[cache.v2_id].label != 0)) {
            edge_split_sphere_tracing(cache.v2_id, cache.v1_id, p_new);
        } else if (
            (m_vertex_attribute[cache.v1_id].label != 0) &&
            (m_vertex_attribute[cache.v2_id].label == 0)) {
            edge_split_sphere_tracing(cache.v1_id, cache.v2_id, p_new);
        } else {
            log_and_throw_error(
                "Invalid edge [{}] for log root finding edge split. Both vertices in/out of "
                "offset/input complex.",
                e_id);
        }
    } else {
        log_and_throw_error("Invalid edge split mode.");
    }
    cache.new_v = VertexAttributes2d(p_new);
    cache.new_v.label = m_edge_attribute[e_id].label;

    // split edge attribute
    cache.split_eattr = m_edge_attribute[e_id];

    // per-opp vert attributes
    std::vector<size_t> opp_v_ids;
    opp_v_ids.push_back(t.switch_edge(*this).switch_vertex(*this).vid(*this));
    auto other = t.switch_face(*this);
    if (other) {
        opp_v_ids.push_back(other.value().switch_edge(*this).switch_vertex(*this).vid(*this));
    }
    for (const size_t opp_v_id : opp_v_ids) {
        Tuple ftup = tuple_from_simplex(simplex::Face(opp_v_id, cache.v1_id, cache.v2_id));
        size_t f_id = ftup.fid(*this);

        simplex::Edge e1(cache.v1_id, opp_v_id);
        size_t e1_id = tuple_from_edge(cache.v1_id, opp_v_id, f_id).eid(*this);
        simplex::Edge e2(cache.v2_id, opp_v_id);
        size_t e2_id = tuple_from_edge(cache.v2_id, opp_v_id, f_id).eid(*this);

        cache.existing_eattr[e1] = m_edge_attribute[e1_id];
        cache.existing_eattr[e2] = m_edge_attribute[e2_id];
        cache.opp_v_fattr[opp_v_id] = m_face_attribute[f_id];
    }

    return true;
}


bool TopoOffsetTriMesh::split_edge_after(const Tuple& t)
{
    if (!TriMesh::split_edge_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = edge_split_cache.local();
    size_t v_id = get_vertices().size() - 1;
    // std::vector<size_t> opp_vids;
    // for (const auto& pair : cache.opp_v_fattr) {
    //     opp_vids.push_back(pair.first);
    // }
    // size_t v_id = edge_split_get_new_vid(cache.v1_id, cache.v2_id, opp_vids);
    m_vertex_attribute[v_id] = cache.new_v;

    // split edge attributes
    for (const auto& pair : cache.opp_v_fattr) {
        size_t opp_v_id = pair.first;

        Tuple ftup_1 = tuple_from_simplex(simplex::Face(cache.v1_id, v_id, opp_v_id));
        size_t f1_id = ftup_1.fid(*this);
        m_edge_attribute[tuple_from_edge(cache.v1_id, v_id, f1_id).eid(*this)] = cache.split_eattr;

        Tuple ftup_2 = tuple_from_simplex(simplex::Face(cache.v2_id, v_id, opp_v_id));
        size_t f2_id = ftup_2.fid(*this);
        m_edge_attribute[tuple_from_edge(cache.v2_id, v_id, f2_id).eid(*this)] = cache.split_eattr;

        break;
    }

    // per existing edge attributes
    for (const auto& pair : cache.existing_eattr) {
        size_t e_id = edge_id_from_simplex(pair.first);
        m_edge_attribute[e_id] = pair.second;
    }

    // new edges and faces
    for (const auto& pair : cache.opp_v_fattr) {
        size_t opp_v_id = pair.first;
        FaceAttributes2d f_attr = pair.second;

        size_t f1_id = tuple_from_simplex(simplex::Face(cache.v1_id, v_id, opp_v_id)).fid(*this);
        m_face_attribute[f1_id] = f_attr;
        size_t f2_id = tuple_from_simplex(simplex::Face(cache.v2_id, v_id, opp_v_id)).fid(*this);
        m_face_attribute[f2_id] = f_attr;
        size_t new_e_id = edge_id_from_simplex(simplex::Edge(opp_v_id, v_id));
        m_edge_attribute[new_e_id].label = pair.second.label;
    }

    return true;
}


bool TopoOffsetTriMesh::split_face_before(const Tuple& t)
{
    auto& cache = face_split_cache.local();
    cache.existing_eattr.clear();

    // face id, retain attribute
    size_t f_id = t.fid(*this);
    cache.split_fattr = m_face_attribute[f_id];

    // vertices (new vertex attributes too)
    cache.v1_id = t.vid(*this);
    cache.v2_id = t.switch_vertex(*this).vid(*this);
    cache.v3_id = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    Vector2d p1 = m_vertex_attribute[cache.v1_id].m_posf;
    Vector2d p2 = m_vertex_attribute[cache.v2_id].m_posf;
    Vector2d p3 = m_vertex_attribute[cache.v3_id].m_posf;
    cache.new_v = VertexAttributes2d((p1 + p2 + p3) / 3);
    cache.new_v.label = cache.split_fattr.label;

    // existing edges
    simplex::Edge e1(cache.v1_id, cache.v2_id);
    simplex::Edge e2(cache.v2_id, cache.v3_id);
    simplex::Edge e3(cache.v1_id, cache.v3_id);
    cache.existing_eattr[e1] = m_edge_attribute[edge_id_from_simplex(e1)];
    cache.existing_eattr[e2] = m_edge_attribute[edge_id_from_simplex(e2)];
    cache.existing_eattr[e3] = m_edge_attribute[edge_id_from_simplex(e3)];

    return true;
}


bool TopoOffsetTriMesh::split_face_after(const Tuple& t)
{
    if (!TriMesh::split_face_after(t)) {
        return false;
    }

    auto& cache = face_split_cache.local();
    size_t v_id = get_vertices().size() - 1;
    m_vertex_attribute[v_id] = cache.new_v;

    // existing edges
    for (const auto& pair : cache.existing_eattr) {
        size_t e_id = edge_id_from_simplex(pair.first);
        m_edge_attribute[e_id] = pair.second;
    }

    // all internal edges and faces
    std::array<size_t, 3> vs = {{cache.v1_id, cache.v2_id, cache.v3_id}};
    for (int i = 0; i < 3; i++) {
        // edge
        size_t e_id = edge_id_from_simplex(simplex::Edge(vs[i], v_id));
        m_edge_attribute[e_id].label = cache.split_fattr.label;

        // face
        size_t f_id = tuple_from_simplex(simplex::Face(vs[i], vs[(i + 1) % 3], v_id)).fid(*this);
        m_face_attribute[f_id] = cache.split_fattr;
    }

    return true;
}

} // namespace wmtk::components::topological_offset
