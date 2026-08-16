
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& t)
{
    if (!TetOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // Unconditionally, where the base asks only when BOTH endpoints already sit on a tracked
    // simplex. That rule is right for tetwild and simwild; here the offset region is a thin
    // shell, and a collapse with one endpoint in the interior can still pinch its two sides
    // together while every tracked surface survives intact.
    return substructure_link_condition(t);
}

bool TopoOffsetTetMesh::collapse_before_vertex(
    const size_t v1_id,
    const size_t v2_id,
    const double edge_length)
{
    const auto& VE = m_vertex_extra;

    // v1 is the vertex the collapse REMOVES (it merges into v2, which keeps its position). If
    // v1 belongs to the input complex or the domain boundary, removing it deletes a simplex of
    // a set that must not change at all, so the collapse is refused outright -- including the
    // input-onto-input case the surface rule below would otherwise allow. That case is not
    // hypothetical: it is what lets the shared engine decimate the input surface down to
    // whatever m_envelope tolerates.
    if (vertex_is_frozen(v1_id)) {
        return false;
    }

    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not
    // leave the particular surface it belongs to: an input-complex vertex carries input
    // geometry, and an offset-boundary vertex carries the offset.
    if (edge_length > 0 && VE[v1_id].m_is_on_input && !VE[v2_id].m_is_on_input) {
        return false;
    }
    if (edge_length > 0 && VE[v1_id].m_is_on_offset && !VE[v2_id].m_is_on_offset) {
        return false;
    }

    // open boundary
    if (edge_length > 0 && m_vertex_attribute[v1_id].m_order == 2 &&
        m_vertex_attribute[v2_id].m_order < 2) {
        return false;
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t,
    const std::vector<std::array<size_t, 2>>&)
{
    // NO NORMAL-DEVIATION GUARD, in either hook. Both used to be here, and both were the paper's
    // Sec. 5.3.3 Step 2 rule: refuse a collapse that leaves the offset surface's FIELD normals
    // disagreeing across a face by more than sigma_max. They existed because the criterion the
    // loop converged on was measured at VERTICES only, and a vertex criterion cannot see a
    // surface that has decimated to a few large triangles cutting across the offset -- so the
    // decimation had to be forbidden operation by operation instead of measured.
    //
    // The face-sampled Phi residual measures it directly (see offset_face_samples()), and feeds
    // face_criterion_rel(), so a band too coarse to be the offset now REFINES instead of having
    // to be protected. Same substitution 2D made when its collapse_edge_after() guard went.
    //
    // This is the one deletion here that changes what the collapse pass may do rather than only
    // where a computation lives, so it is the one to look at first if the 3D offset decimates.
    ++iter_cnt_collapse;
    return true;
}

void TopoOffsetTetMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    if (m_vertex_extra.at(v1_id).m_is_on_offset) ++iter_cnt_collapse_offset_removed;
    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
}

} // namespace wmtk::components::topological_offset
