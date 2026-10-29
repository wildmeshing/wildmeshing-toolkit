#include "SimWildMesh.h"

#include <algorithm>
#include <set>

namespace wmtk::components::simwild {

// The order-2 edges are held by the base: tagged at init (tag_order2_edges()), every edge a
// collapse keeps must stay in their tube, so the chord check that ran here only under
// preserve_topology is gone.

/// An order-2 vertex collapses only onto order >= 2: the order-2 curves never take in a vertex
/// of a sheet. Not for the geometry -- the tube check alone holds the curves in place -- but for
/// the surface and volume history maps, which follow each vertex in its own piece of the complex:
/// without the rule, two overlapping spheres (tracking-example-simwild-volume) mapped 24 surface
/// points off the surface and 62 points into another region.
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

void SimWildMesh::simplify()
{
    compute_vertex_partition_morton();
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
    logger().info("===== Simplify =====");

    // The same switch as tetwild's: the sampled envelope for the simplification alone, the
    // exact one dominating its cost on a large input. Otherwise the simplification keeps the
    // mode use_sample_envelope gave m_envelope. Nothing to restore afterwards: the envelope is
    // rebuilt below, by init_envelope, in that mode.
    if (m_sim_params.simplify_use_sample_envelope) {
        m_envelope->use_exact = false;
    }
    m_envelope->init(m_V_envelope, m_F_envelope, m_sim_params.eps_simplify);
    init_label_envelopes(m_sim_params.eps_simplify, m_envelope->use_exact);

    /**
     * Not checking the quality here can lead to horrible elements that take forever to fix in the
     * optimization stage. The quality check reduces the amount of simplification, but the overall
     * convergence is faster because the optimizer does not have to fix the mess.
     */
    // m_collapse_check_quality = false;
    collapse_all_edges();
    if (m_params.debug_output) {
        write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
    }
    // m_collapse_check_quality = true;

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
    m_F_envelope_labels.clear();
    if (V.size() > 0 && F.size() > 0) {
        init_envelope(V, F);
    } else {
        logger().warn("No surface faces left after simplification, skip re-building envelope");
    }
    init_label_envelopes(m_envelope_eps, !m_sim_params.use_sample_envelope);
    logger().info("===== Simplification done =====");
}

} // namespace wmtk::components::simwild
