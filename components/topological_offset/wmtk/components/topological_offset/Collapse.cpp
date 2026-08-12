
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

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

    // max over all 4 samples, not just the centroid: a face straddling a feature has samples
    // on both sides of it, and only the max catches the one that disagrees with the face's own
    // (necessarily flat) normal -- averaging or using a single sample would miss it.
    double max_dev = 0.;
    for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
        if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
        // orientation independent: a triangle whose plane is parallel to the sample's implied
        // plane counts as aligned regardless of which way get_face_vertices() winds it
        const double c = std::clamp(face_normal.cross(s.normal).norm(), -1., 1.);
        max_dev = std::max(max_dev, (180. / M_PI) * std::asin(c));
    }
    return max_dev;
}

double TopoOffsetTetMesh::max_offset_surface_normal_deviation_at_vertex(size_t vid) const
{
    double max_nd = 0.;
    for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(vid))) {
        max_nd = std::max(max_nd, face_normal_deviation(f));
    }
    return max_nd;
}

double TopoOffsetTetMesh::collapse_normal_deviation(
    const size_t v_from,
    const size_t v_to,
    const size_t remove_vid) const
{
    const Vector3d p0 = m_vertex_attribute[v_from].m_posf;
    const Vector3d p1 = m_vertex_attribute[v_to].m_posf;

    const Vector3d e_dir = (p1 - p0).normalized();

    double min_angle = std::numeric_limits<double>::max();
    double max_angle = std::numeric_limits<double>::lowest();
    for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(remove_vid))) {
        for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
            // orientation dependent (0-180): unlike face_normal_deviation, the edge direction
            // has a genuine sign, so a sample pointing "with" vs "against" it are different
            const double dot = std::clamp(e_dir.dot(s.normal), -1., 1.);
            const double angle = (180. / M_PI) * std::acos(dot);
            min_angle = std::min(min_angle, angle);
            max_angle = std::max(max_angle, angle);
        }
    }
    if (min_angle > max_angle) return 0.; // no samples found at all
    return max_angle - min_angle;
}

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

    // OffsetCollapseBeforeInvariant analogue: don't collapse if the offset-target normal field
    // sampled around the survivor disagrees with itself (relative to the collapse direction)
    // by more than the threshold -- that disagreement is the signature of a feature edge
    // nearby, and collapsing across it would flatten/cut through the feature.
    if (collapse_normal_deviation(v1_id, v2_id, v1_id) >=
        m_offset_params.max_normal_deviation_deg) {
        return false;
    }

    // NormalDeviationAfterInvariant analogue setup: remember how bad the offset surface
    // already was around this edge, so the `after` hook only blocks a collapse that makes a
    // *good* patch worse, not one that was already over the threshold.
    m_collapse_nd_before.local() = std::max(
        max_offset_surface_normal_deviation_at_vertex(v1_id),
        max_offset_surface_normal_deviation_at_vertex(v2_id));

    return true;
}

bool TopoOffsetTetMesh::collapse_after_connectivity(
    const size_t,
    const size_t v2_id,
    const std::vector<std::array<size_t, 2>>&)
{
    // NormalDeviationAfterInvariant analogue: only reject a move that degrades an
    // already-good offset surface patch -- if it was already over the threshold before, don't
    // block a collapse from fixing (or merely not fixing) it.
    if (m_collapse_nd_before.local() < m_offset_params.max_normal_deviation_deg) {
        const double nd_after = max_offset_surface_normal_deviation_at_vertex(v2_id);
        if (nd_after >= m_offset_params.max_normal_deviation_deg) {
            return false;
        }
    }
    return true;
}

void TopoOffsetTetMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
}

} // namespace wmtk::components::topological_offset
