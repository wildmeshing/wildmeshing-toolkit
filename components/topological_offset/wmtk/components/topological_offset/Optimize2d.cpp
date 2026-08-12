#include "TopoOffsetTriMesh.h"

#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace wmtk::components::topological_offset {

/**
 * The 2D optimization phase. The operations themselves -- split, collapse, the quality-driven
 * edge flip, and the driver that sequences them -- are wmtk::TriOptimizerMesh's. What is here
 * is only what the offset knows: where its two tracked surfaces are, and how a vertex on the
 * offset boundary is allowed to move.
 */

void TopoOffsetTriMesh::label_offset_boundary()
{
    // Face quality, which the shared operations read and keep up to date from here on.
    for (const Tuple& f : get_faces()) {
        const size_t fid = f.fid(*this);
        m_face_attribute[fid].m_quality = get_quality(fid);
    }

    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        m_edge_attribute[eid].reset();

        const std::optional<Tuple> opp = e.switch_face(*this);
        if (!opp) {
            // Only one incident face: this is the domain boundary. Tagging it bbox is what
            // keeps the box from collapsing inward; there is no side to compare against, so it
            // can never be an offset boundary.
            m_edge_attribute[eid].m_is_bbox_fs = 0;
            continue;
        }

        // An edge of the input complex carries input geometry and is the class the envelope
        // (when there is one) applies to. It takes precedence: an input edge that also happens
        // to separate two labels is still input geometry.
        if (m_edge_extra[eid].label == 1) {
            m_edge_attribute[eid].m_is_surface_fs = true;
            m_edge_attribute[eid].m_surface_class = 0;
            continue;
        }

        // The offset boundary IS the face-label change across an edge. There is nothing else
        // that defines it -- it is not stored anywhere, it falls out of the labelling.
        //
        // A region-TAG change is tracked with it, and has to be. The shared 2D swap copies one
        // incident face's tags onto both faces it creates, so an unguarded flip across a tag
        // boundary silently relabels a face and moves the region -- which is what the output is
        // built from. Tagging the edge as tracked surface is what makes the swap refuse it.
        const size_t f0 = e.fid(*this), f1 = opp->fid(*this);
        const bool label_change = m_face_extra[f0].label != m_face_extra[f1].label;
        const bool tag_change = m_face_attribute[f0].tags != m_face_attribute[f1].tags;
        if (label_change || tag_change) {
            m_edge_attribute[eid].m_is_surface_fs = true;
            m_edge_attribute[eid].m_surface_class = OFFSET_SURFACE_CLASS;
        }
    }

    // Vertices inherit from their incident edges. m_is_on_surface is the union, which is what
    // the shared operations protect; the two extra flags say which surface.
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        m_vertex_attribute[vid].m_is_on_surface = false;
        m_vertex_attribute[vid].on_bbox_faces.clear();
        m_vertex_extra[vid].m_is_on_input = false;
        m_vertex_extra[vid].m_is_on_offset = false;

        for (const Tuple& e : get_one_ring_edges_for_vertex(v)) {
            const size_t eid = e.eid(*this);
            if (m_edge_attribute[eid].m_is_bbox_fs >= 0 &&
                m_vertex_attribute[vid].on_bbox_faces.empty()) {
                m_vertex_attribute[vid].on_bbox_faces.push_back(0);
            }
            if (!m_edge_attribute[eid].m_is_surface_fs) continue;
            m_vertex_attribute[vid].m_is_on_surface = true;
            if (m_edge_attribute[eid].m_surface_class == OFFSET_SURFACE_CLASS) {
                m_vertex_extra[vid].m_is_on_offset = true;
            } else {
                m_vertex_extra[vid].m_is_on_input = true;
            }
        }
    }

    size_t n_off = 0, n_in = 0, n_box = 0;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        n_off += edge_is_offset(eid);
        n_in += edge_is_input(eid);
        n_box += (m_edge_attribute[eid].m_is_bbox_fs >= 0);
    }
    logger()
        .info("\ttracked edges: {} offset boundary, {} input complex, {} bbox", n_off, n_in, n_box);
}

bool TopoOffsetTriMesh::swap_edge_before(const Tuple& t)
{
    if (!TriOptimizerMesh::swap_edge_before(t)) {
        return false;
    }

    // The flip replaces (a,b) with (c,d), the two apexes of the incident triangles.
    const std::optional<Tuple> opp = t.switch_face(*this);
    if (!opp) return false;
    const size_t c = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    const size_t d = opp->switch_edge(*this).switch_vertex(*this).vid(*this);
    if (c == d) return false;

    // Already joined? Then the flip would duplicate that edge.
    for (const Tuple& e : get_one_ring_edges_for_vertex(tuple_from_vertex(c))) {
        const size_t nb = (e.vid(*this) == c) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        if (nb == d) return false;
    }
    return true;
}

bool TopoOffsetTriMesh::collapse_edge_before(const Tuple& t)
{
    if (!TriOptimizerMesh::collapse_edge_before(t)) {
        return false;
    }
    // Unconditionally, not just when both endpoints are already on a tracked simplex -- see
    // the declaration. substructure_link_condition() evaluates the condition for the mesh and
    // for every substructure, which is what a topological offset needs preserved.
    return substructure_link_condition(t);
}

bool TopoOffsetTriMesh::collapse_before_vertex(const size_t v1_id, const size_t v2_id)
{
    // The base only knows that both endpoints are on SOME tracked surface. A vertex may not
    // leave the particular surface it belongs to: an input-complex vertex carries input
    // geometry, and an offset-boundary vertex carries the offset.
    if (m_vertex_extra[v1_id].m_is_on_input && !m_vertex_extra[v2_id].m_is_on_input) {
        return false;
    }
    if (m_vertex_extra[v1_id].m_is_on_offset && !m_vertex_extra[v2_id].m_is_on_offset) {
        return false;
    }
    return true;
}

void TopoOffsetTriMesh::collapse_after_vertex(const size_t v1_id, const size_t v2_id)
{
    // The base ORs its own m_is_on_surface, which is the union of the two; these say which.
    m_vertex_extra[v2_id].m_is_on_input =
        m_vertex_extra.at(v1_id).m_is_on_input || m_vertex_extra.at(v2_id).m_is_on_input;
    m_vertex_extra[v2_id].m_is_on_offset =
        m_vertex_extra.at(v1_id).m_is_on_offset || m_vertex_extra.at(v2_id).m_is_on_offset;
}

void TopoOffsetTriMesh::split_after_vertex(const size_t v_id)
{
    // The base has already set m_is_on_surface from the split edge's own attributes and copied
    // those attributes -- surface class included -- onto the two halves. All that is left is
    // to say which of the two surfaces the new vertex joined.
    const auto& e = split_cache.local().old_e_attrs;
    m_vertex_extra[v_id].m_is_on_input =
        e.m_is_surface_fs && e.m_surface_class != OFFSET_SURFACE_CLASS;
    m_vertex_extra[v_id].m_is_on_offset =
        e.m_is_surface_fs && e.m_surface_class == OFFSET_SURFACE_CLASS;
}

bool TopoOffsetTriMesh::smooth_before(const Tuple& t)
{
    // rounds the vertex and refuses the bounding box
    if (!TriOptimizerMesh::smooth_before(t)) {
        return false;
    }
    // the input complex must stay exactly where it is: it is the geometry the offset is
    // measured against, not something to be improved
    return !m_vertex_extra[t.vid(*this)].m_is_on_input;
}

bool TopoOffsetTriMesh::smooth_after(const Tuple& t)
{
    // An offset-boundary vertex is placed against the input complex's distance field, which is
    // what keeps the offset faithful. Every other vertex is an ordinary interior one and gets
    // the shared AMIPS smoother.
    if (m_vertex_extra[t.vid(*this)].m_is_on_offset) {
        return project_offset_vertex(t);
    }
    return TriOptimizerMesh::smooth_after(t);
}

bool TopoOffsetTriMesh::project_offset_vertex(const Tuple& t)
{
    const size_t vid = t.vid(*this);
    const Vector2d p0 = m_vertex_attribute[vid].m_posf;

    // Laplacian over the offset-boundary neighbours only. Pulling in input-complex or interior
    // neighbours would drag the boundary off its shape -- the 2D counterpart of restricting the
    // 3D Laplacian to offset-surface neighbours.
    Vector2d p_laplace = Vector2d::Zero();
    int n = 0;
    for (const Tuple& e : get_one_ring_edges_for_vertex(t)) {
        if (!edge_is_offset(e.eid(*this))) continue;
        const size_t nb = (e.vid(*this) == vid) ? e.switch_vertex(*this).vid(*this) : e.vid(*this);
        if (nb == vid) continue;
        p_laplace += m_vertex_attribute[nb].m_posf;
        ++n;
    }
    if (n == 0) return false;
    p_laplace /= n;

    // Projection onto the target distance: walk from the nearest point on the input complex
    // outward, in the direction the vertex already lies, by exactly target_distance.
    const Vector3d near3 = m_input_complex_bvh.nearest_point(VectorXd(p0));
    const Vector2d nearest(near3[0], near3[1]);
    const Vector2d diff = p0 - nearest;
    const double dist = diff.norm();
    if (dist < 1e-12) {
        return false; // sitting on the input complex; no direction to offset along
    }
    const Vector2d p_proj = nearest + m_offset_params.target_distance * (diff / dist);

    const double w = m_offset_params.smooth_quadrics_weight;
    const double u = m_offset_params.smooth_laplacian_weight;
    const Vector2d target = (1 - w - u) * p0 + w * p_proj + u * p_laplace;

    const std::vector<Tuple> ring = get_one_ring_tris_for_vertex(t);
    const auto any_inverted = [&]() {
        for (const Tuple& f : ring) {
            if (is_inverted(f)) return true;
        }
        return false;
    };

    set_vertex_position(vid, target);
    if (!any_inverted()) return true;

    // Binary search back toward the known-valid p0 for the furthest point that inverts nothing.
    double lo = 0., hi = 1.;
    for (int it = 0; it < 10; ++it) {
        const double mid = 0.5 * (lo + hi);
        set_vertex_position(vid, p0 + mid * (target - p0));
        if (any_inverted()) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    set_vertex_position(vid, p0 + lo * (target - p0));
    return !any_inverted();
}

size_t TopoOffsetTriMesh::update_sizing_field()
{
    // What matters on the offset boundary is how far it sits from the target distance. A
    // vertex whose error exceeds sizing_mrm_threshold of the target wants shorter edges around
    // it, so the next split pass can resolve that stretch; one that is already on target may
    // coarsen.
    std::vector<size_t> refined;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_extra[vid].m_is_on_offset) continue;

        const Vector2d p = m_vertex_attribute[vid].m_posf;
        const Vector3d near3 = m_input_complex_bvh.nearest_point(VectorXd(p));
        const double d = (p - Vector2d(near3[0], near3[1])).norm();
        const double err = std::abs(d - m_offset_params.target_distance) /
                           std::max(m_offset_params.target_distance, 1e-16);

        double& s = m_vertex_attribute[vid].m_sizing_scalar;
        const double old_s = s;
        s *= (err > m_offset_params.sizing_mrm_threshold) ? 0.5 : 1.5;
        s = std::clamp(s, m_offset_params.min_sizing_scalar, m_offset_params.max_sizing_scalar);
        if (s < old_s) refined.push_back(vid);
    }

    // Grade the refinement outward so neighbouring sizings never differ by more than
    // sizing_gradation. Monotone: it only ever lowers a neighbour's scalar.
    gradation_smooth_sizing(m_offset_params.sizing_gradation, refined);
    return refined.size();
}

void TopoOffsetTriMesh::optimize_offset(const std::filesystem::path& output_file)
{
    logger().info("Optimizing offset (2D)...");

    logger().info("\tLabelling tracked surfaces...");
    label_offset_boundary();

    // From here on every edge split is an optimization split, run by the shared engine. The
    // marching-triangles placement modes require one endpoint inside the offset and one
    // outside, which does not hold for an arbitrary long edge.
    m_edge_split_mode = EdgeSplitMode::Optimization;

    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    for (int i = 0; i < m_offset_params.optimization_iterations; ++i) {
        logger().info(
            "Optimization iteration {} / {}",
            i + 1,
            m_offset_params.optimization_iterations);

        // One split, one collapse, one swap and num_smoothing_passes smoothing passes, with
        // the sanity checks, debug output and per-pass energy reporting the shared driver does.
        // Not mesh_improvement(), for the same reason as in 3D: its stop condition needs a stop
        // metric the offset has not defined.
        local_operations({{1, 1, 1, m_offset_params.num_smoothing_passes}});

        logger().info("\tUpdating sizing field...");
        update_sizing_field();
    }

    // Deliberately NOT re-derived here. From the moment the tracked surfaces are tagged, it is
    // the shared operations that maintain those tags -- the face LABELS they were derived from
    // are construction data the optimization does not propagate, so re-deriving now would read
    // stale labels and mislabel the result.
}

} // namespace wmtk::components::topological_offset
