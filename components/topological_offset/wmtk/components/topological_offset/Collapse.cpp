
#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace wmtk::components::topological_offset {

double TopoOffsetTetMesh::face_normal_deviation(const Tuple& f) const
{
    // Paper Definition 5: the maximum angle between the OFFSET normal at the element's center
    // and the OFFSET normal at other points within the element.
    //
    //     sigma(t) = max over p_i in t of angle( n(p_c), n(p_i) ),  p_i = (1-u)*p_v + u*p_c
    //
    // Both terms are FIELD normals; the face's own geometric normal does not appear. That is the
    // whole point and it is what makes the quantity converge: n() is continuous away from the
    // input complex's features, so shrinking a face brings its samples closer together and drives
    // sigma to zero, which is something the sizing field can actually satisfy.
    //
    // This used to compare the face's own normal against the sample normals, i.e. misorientation.
    // Refinement does not fix misorientation -- a smaller triangle in the same plane is just as
    // misoriented -- so the sizing field could never drive it down, and update_sizing_field()
    // would refine around such a face forever. The 2D side hit exactly that and was corrected to
    // Definition 5; see the normal-deviation section of .claude/CLAUDE.md.
    //
    // offset_surface_samples() already produces the paper's sample set: index 0 is the centroid
    // p_c, indices 1-3 are (1-u)*p_v + u*p_c with u = 0.1, and each carries the field normal.
    const std::array<OffsetSurfaceSample, 4> samples = offset_surface_samples(f);
    const Vector3d& n_c = samples[0].normal;
    if (n_c.squaredNorm() < 1e-20) return 0.; // centroid sits on the complex: no direction

    double max_dev = 0.;
    for (int i = 1; i < 4; ++i) {
        const Vector3d& n_i = samples[i].normal;
        if (n_i.squaredNorm() < 1e-20) continue; // degenerate sample: no normal direction
        // A genuine angle between two field normals, both pointing outward from the complex, so
        // there is no orientation ambiguity to fold away here.
        const double c = std::clamp(n_c.dot(n_i), -1., 1.);
        max_dev = std::max(max_dev, (180. / M_PI) * std::acos(c));
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
    // Paper Sec. 5.3.3, Step 2: "a collapse is only performed if the user-defined maximum normal
    // deviation is not exceeded." Taken literally that would freeze any patch already over
    // sigma_max -- usually a genuine feature, where no refinement will ever align the field
    // normals -- so the bar is the WORSE of the paper's threshold and where the patch already was.
    //
    // The previous rule here was "only block a collapse that makes a GOOD patch bad", i.e. the
    // guard switched off entirely once nd_before crossed sigma_max. That is a ratchet: the instant
    // a patch crosses the threshold it is unprotected, and the next collapses take it to 90. In 2D
    // it decimated the offset from 429 to 80 elements with only 8 collapses refused; using the
    // worse-of bar took that run from not-converging to converging, with 58 then 54 refused.
    const double bar =
        std::max(m_offset_params.max_normal_deviation_deg, m_collapse_nd_before.local());
    if (max_offset_surface_normal_deviation_at_vertex(v2_id) > bar) {
        ++iter_cnt_collapse_nd_reject;
        return false;
    }
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
