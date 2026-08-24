#include "SimWildMesh.h"

#include <algorithm>
#include <set>

namespace wmtk::components::simwild {

bool SimWildMesh::collapse_before_vertex(size_t v1, size_t v2, double edge_length)
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

void SimWildMesh::collapse_after_vertex(size_t v1, size_t v2)
{
    m_vertex_attribute[v2].m_sizing_scalar =
        std::min(m_vertex_attribute[v1].m_sizing_scalar, m_vertex_attribute[v2].m_sizing_scalar);
}

void SimWildMesh::simplify()
{
    compute_vertex_partition_morton();
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
    logger().info("===== Simplify =====");

    m_envelope->use_exact = false;
    m_envelope->init(m_V_envelope, m_F_envelope, m_sim_params.eps_simplify);

    m_collapse_check_quality = false;
    collapse_all_edges();
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
    m_collapse_check_quality = true;

    logger().warn("Update envelope");
    MatrixXd V(vert_capacity(), 3);
    V.setZero();
    for (size_t i = 0; i < vert_capacity(); ++i) {
        const Tuple v = tuple_from_vertex(i);
        if (v.is_valid(*this)) {
            V.row(i) = m_vertex_attribute.at(v.vid(*this)).m_posf;
        }
    }

    const auto surf_faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
    MatrixXi F(surf_faces.size(), 3);
    for (size_t i = 0; i < surf_faces.size(); ++i) {
        F.row(i) = Vector3i(int(surf_faces[i][0]), int(surf_faces[i][1]), int(surf_faces[i][2]));
    }

    m_envelope = nullptr;
    m_V_envelope.clear();
    m_F_envelope.clear();
    if (V.size() > 0 && F.size() > 0) {
        init_envelope(V, F);
    } else {
        logger().warn("No surface faces left after simplification, skip re-building envelope");
    }
    logger().info("===== Simplification done =====");
}

} // namespace wmtk::components::simwild
