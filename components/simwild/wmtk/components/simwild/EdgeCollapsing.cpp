#include "SimWildMesh.h"

#include <algorithm>
#include <set>

namespace wmtk::components::simwild {

bool SimWildMesh::collapse_before_vertex(size_t v1, size_t v2, double edge_length) const
{
    if (edge_length <= 0 || m_vertex_attribute[v1].m_order != 2) return true;
    return m_vertex_attribute[v2].m_order >= 2;
}

bool SimWildMesh::collapse_quality_allowed(size_t v1, double quality, double ring_max) const
{
    return !m_collapse_check_quality ||
           TetOptimizerMesh::collapse_quality_allowed(v1, quality, ring_max);
}

bool SimWildMesh::collapse_is_order_2_edge(const std::array<size_t, 2>& e)
{
    // These extra curve-envelope checks are SimWild's preserve-topology policy. TetWild's
    // shared surface link condition is already gated by the same flag in the common core.
    return m_params.preserve_topology && is_order_2_edge(e);
}

bool SimWildMesh::collapse_after_connectivity(
    size_t v1,
    size_t v2,
    const std::vector<std::array<size_t, 2>>& boundary_edges)
{
    m_vertex_attribute[v2].m_order =
        std::max(m_vertex_attribute[v1].m_order, m_vertex_attribute[v2].m_order);

    if (collapse_cache.local().edge_length <= 0) return true;
    for (const auto& vids : boundary_edges) {
        const std::array<Vector3d, 2> pts{
            {m_vertex_attribute.at(vids[0]).m_posf, m_vertex_attribute.at(vids[1]).m_posf}};
        if (m_order_2_edge_envelope->is_outside(pts)) return false;
    }
    return true;
}

void SimWildMesh::collapse_after_vertex(size_t, size_t v2)
{
    // A SimWild surface is not inherited geometry: it is precisely the interface between
    // unlike cell tags. Re-derive the affected faces after connectivity and tag data settle;
    // OR-merging the two old face flags can leave a homogeneous face marked as an interface.
    std::set<size_t> affected_vertices;
    affected_vertices.insert(v2);

    const auto tids = get_one_ring_tids_for_vertex(v2);
    for (const size_t tid : tids) {
        if (!tuple_from_tet(tid).is_valid(*this)) continue;
        for (int j = 0; j < 4; ++j) {
            const Tuple face = tuple_from_face(tid, j);
            const size_t fid = face.fid(*this);
            const auto opposite = face.switch_tetrahedron(*this);
            const bool is_interface =
                opposite.has_value() &&
                m_tet_attribute[tid].tags != m_tet_attribute[opposite->tid(*this)].tags;
            m_face_attribute[fid].m_is_surface_fs = is_interface;
            const auto fvs = get_face_vids(face);
            affected_vertices.insert(fvs.begin(), fvs.end());
        }
    }

    for (const size_t vid : affected_vertices) {
        bool on_interface = false;
        for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
            if (!tuple_from_tet(tid).is_valid(*this)) continue;
            const auto tet = oriented_tet_vids(tid);
            for (int j = 0; j < 4; ++j) {
                const Tuple face = tuple_from_face(tid, j);
                const auto fvs = get_face_vids(face);
                if (std::find(fvs.begin(), fvs.end(), vid) == fvs.end()) continue;
                if (m_face_attribute[face.fid(*this)].m_is_surface_fs) {
                    on_interface = true;
                    break;
                }
            }
            if (on_interface) break;
        }
        m_vertex_attribute[vid].m_is_on_surface = on_interface;
    }
    for (const size_t vid : affected_vertices) {
        m_vertex_attribute[vid].m_order =
            m_vertex_attribute[vid].m_is_on_surface ? compute_vertex_order(vid) : 0;
    }
}

void SimWildMesh::simplify()
{
    compute_vertex_partition_morton();
    if (m_params.debug_output) write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    logger().info("===== Simplify =====");

    m_envelope->use_exact = false;
    m_envelope->init(m_V_envelope, m_F_envelope, m_sim_params.eps_simplify);

    m_collapse_check_quality = false;
    collapse_all_edges();
    if (m_params.debug_output) write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    m_collapse_check_quality = true;

    logger().warn("Update envelope");
    MatrixXd V(vert_capacity(), 3);
    V.setZero();
    for (size_t i = 0; i < vert_capacity(); ++i) {
        const Tuple v = tuple_from_vertex(i);
        if (v.is_valid(*this)) V.row(i) = m_vertex_attribute.at(v.vid(*this)).m_posf;
    }

    const auto surf_faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
    MatrixXi F(surf_faces.size(), 3);
    for (size_t i = 0; i < surf_faces.size(); ++i) {
        F.row(i) = Vector3i(int(surf_faces[i][0]), int(surf_faces[i][1]), int(surf_faces[i][2]));
    }

    const bool use_exact = m_envelope->use_exact;
    m_envelope = nullptr;
    m_V_envelope.clear();
    m_F_envelope.clear();
    if (V.size() > 0 && F.size() > 0) {
        init_envelope(V, F, use_exact);
    } else {
        logger().warn("No surface faces left after simplification, skip re-building envelope");
    }
    logger().info("===== Simplification done =====");
}

} // namespace wmtk::components::simwild
