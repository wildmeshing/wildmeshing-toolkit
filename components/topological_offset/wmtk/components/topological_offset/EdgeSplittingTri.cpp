#include <set>
#include "TopoOffsetTriMesh.h"


namespace wmtk::components::topological_offset {

//// TriMesh splitting

bool TopoOffsetTriMesh::split_edge_before(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization && edge_is_offset_surface_live(t)) {
        ++iter_cnt_split_offset_before;
    }
    // Cleared for both modes: split_after_vertex() reads emptiness to tell which mode produced
    // the split, and the marching path sets its own labels. Entries left from a previous
    // optimization split would be stamped onto marching-created faces.
    m_opt_split_cache.local().face_label.clear();

    // The optimization phase runs wmtk::TriOptimizerMesh's split; everything below is the
    // marching-triangles machinery, which places the new vertex on the offset's distance field
    // and carries per-simplex labels the shared engine knows nothing about.
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        // No edge class is refused here, the domain wall included: a wall edge is a tracked
        // region boundary like any other, so the envelopes hold it. Do not re-add a wall
        // refusal; measured worse -- see git history of this file.

        // The shared split propagates FaceAttributes but not the construction label, and
        // offset_is_manifold() is built from that label: without this the new faces default to
        // label 0 and the offset region develops holes. Keyed by the apex -- the vertex opposite
        // the split edge -- which names the parent unambiguously from a child.
        auto& c = m_opt_split_cache.local();
        c.v1_id = t.vid(*this);
        c.v2_id = t.switch_vertex(*this).vid(*this);
        // Captured here, while both endpoints are in hand, and propagated as the endpoints' AND
        // -- never recomputed from the incident faces, whose tags execute_offset() replaces as
        // the band grows. Boundary membership is a property of the input partition, not of the
        // current tags. split_after_vertex() gates these bits on the edge's own persistent
        // class, so a chord's midpoint never picks them up.
        c.edge_bits =
            m_vertex_extra[c.v1_id].m_boundary_mask & m_vertex_extra[c.v2_id].m_boundary_mask;
        const simplex::Edge edge(c.v1_id, c.v2_id);
        // parent_q_max is diagnostic: split_after_vertex() uses it to say whether a needle child
        // came from a parent that was already unscoreable, or from a healthy one.
        c.parent_q_max = -1.;
        c.parent_flatness = 1.;
        const bool ends_on_input =
            m_vertex_extra[c.v1_id].m_is_on_input && m_vertex_extra[c.v2_id].m_is_on_input;
        // Is the edge itself a piece of the complex (a curve, which lies in no input triangle)?
        // Read from its construction label -- except in the refined march's remesh pass, which
        // splits and collapses along a curve many times over, and no operation writes the edge
        // label: a child of a split curve edge lands in a new slot holding whatever its last
        // occupant had, so its own split read label 0 and left the midpoint off the input.
        // Measured on presmooth2d/line_presmooth (the "left & right" curve): 3 such midpoints in
        // one pass, and the complex tube rebuilt after it held 2 of the curve's 6 segments; with
        // the rule below, 0 and all 6. The pass reads the edge's track instead, which every
        // operation maintains: a tracked,
        // non-front edge with both ends on the input is a piece of it -- the rule, and why it is
        // exact, are where remesh_refined_march() re-derives the labels. Outside the pass the
        // label is read as before, so every other run is unchanged.
        const size_t eid = t.eid(*this);
        const bool edge_is_piece =
            m_remesh_pass ? (m_edge_attribute[eid].m_is_surface_fs && !edge_is_offset(eid))
                          : m_edge_extra[eid].label == 1;
        c.edge_in_input = ends_on_input && edge_is_piece;
        for (const size_t fid : get_incident_fids_for_edge(t)) {
            const size_t apex = simplex_from_face(fid).opposite_vertex(edge).id();
            c.face_label[apex] = m_face_extra[fid].label;
            if (ends_on_input && m_face_extra[fid].label == 1) c.edge_in_input = true;
            c.parent_q_max = std::max(c.parent_q_max, get_quality(fid));
            c.parent_flatness = std::min(c.parent_flatness, face_flatness(fid));
        }
        return TriOptimizerMesh::split_edge_before(t);
    }
    return marching_split_edge_before(t);
}

bool TopoOffsetTriMesh::marching_split_edge_before(const Tuple& t)
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
    // Midpoint: no target_distance enters construction at all, and carrying the front out to
    // the level set is the optimization phase's job. SphereTrace (marching_tris under
    // sphere_trace_initialization): the vertex goes to the point of the edge where d(x) reaches
    // target_distance within the tolerance, and to the midpoint when the trace leaves the edge.
    if (m_edge_split_mode == EdgeSplitMode::Midpoint) {
        p_new = (p1 + p2) / 2.0;
    } else if (m_edge_split_mode == EdgeSplitMode::SphereTrace) {
        const bool in1 = m_vertex_extra[cache.v1_id].label != 0;
        const bool in2 = m_vertex_extra[cache.v2_id].label != 0;
        const double D = marching_target_distance();
        bool on_level = false;
        size_t steps = 0;
        // How many of those steps were bisections of the bracket rather than sphere-trace steps,
        // i.e. how much of the placement the sphere trace did not manage on its own. Reported by
        // marching_tris(), as a count of edges and a count of steps, zero or not. As in 3D.
        size_t bisections = 0;
        if (in1 != in2) {
            on_level = in1 ? edge_split_sphere_trace(p1, p2, D, p_new, steps, &bisections)
                           : edge_split_sphere_trace(p2, p1, D, p_new, steps, &bisections);
        }
        m_marching_trace_steps += steps;
        m_marching_trace_steps_max = std::max(m_marching_trace_steps_max, steps);
        if (bisections > 0) {
            ++m_marching_trace_bisected_edges;
            m_marching_trace_bisection_steps += bisections;
        }
        if (on_level) {
            ++m_marching_root_splits;
        } else {
            p_new = (p1 + p2) / 2.0;
            ++m_marching_midpoint_splits;
        }
    } else {
        log_and_throw_error("Invalid edge split mode.");
    }
    cache.new_v_pos = p_new;
    // Where that is on the edge. Only the refined march's rational fallback reads it, and it
    // needs a description of the point that cannot leave the edge; see marching_split_edge_after.
    {
        const Vector2d e12 = p2 - p1;
        const double len2 = e12.squaredNorm();
        cache.new_v_t = len2 > 0. ? std::clamp((p_new - p1).dot(e12) / len2, 0., 1.) : 0.5;
    }
    cache.new_v_extra = VertexExtra2d();
    cache.new_v_extra.label = m_edge_extra[e_id].label;
    // On the input complex exactly when the split edge is, by its label, as in 3D (see
    // EdgeSplittingTet.cpp): construction keeps the labels exact and the flag follows them.
    cache.new_v_extra.m_is_on_input = cache.new_v_extra.label == 1;
    // The flag is the edge's own class, not an AND of the endpoints: two vertices sharing a
    // region can be joined by a chord through the interior, and marching splits exactly such
    // chords. Behind that gate the bits are the endpoints' AND, propagated, never recomputed from
    // the incident faces, whose tags execute_offset() replaces as the band grows.
    cache.new_v_extra.m_is_on_region = edge_is_region(e_id);
    cache.new_v_extra.m_boundary_mask = cache.new_v_extra.m_is_on_region
                                            ? (m_vertex_extra[cache.v1_id].m_boundary_mask &
                                               m_vertex_extra[cache.v2_id].m_boundary_mask)
                                            : uint64_t(0);

    // split edge attribute
    cache.split_eattr = edge_snapshot(e_id);

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

        cache.existing_eattr[e1] = edge_snapshot(e1_id);
        cache.existing_eattr[e2] = edge_snapshot(e2_id);
        cache.opp_v_fattr[opp_v_id] = face_snapshot(f_id);
    }

    return true;
}

bool TopoOffsetTriMesh::edge_split_sphere_trace(
    const Vector2d& p_in,
    const Vector2d& p_out,
    const double D,
    Vector2d& p_new,
    size_t& steps,
    size_t* bisection_steps) const
{
    // Sphere tracing: d is 1-Lipschitz, so from a point at distance d the level set
    // d = D is at least D - d away in every direction, and stepping
    // exactly that far along the edge can never cross it. The step is positive while the trace
    // has not converged (D - d > tol), so t grows by more than tol each time and
    // the loop ends within L / tol steps, one way or the other. Same as 3D.
    //
    // The refined march drops that tolerance and always runs to the precision of double arithmetic
    // instead. It must: the refinement's own tolerance must stay above the roots' placement error,
    // and a trace stopped at a tolerance can leave that error above it -- measured on
    // presmooth2d/square at distance fraction 0.1 and tol_rel 0.002 with a trace tolerance of 0.01:
    // 853000 bisections in 5 hours with the worst certified residual frozen at 2.18e-4, the trace's
    // own error, against a bar of 4.56e-5. With no tolerance the roots are exact to the last bit
    // and every refinement tolerance terminates. INVARIANT: under the refined march the traced
    // points are the ROOTS -- the points of the marched edges where d = march_distance -- and the
    // roots are the CORNERS of the front pieces, so the trace's own placement error is a floor on
    // what the certified residual of a piece can ever reach: give the trace a tolerance of the size
    // of the refinement's tolerance and the loop bisects to chase the roots' own error.
    // Measured on presmooth3d/cylinder (distance_fraction 0.5, tol_rel 0.01, max_rounds 0): with
    // the default sphere_trace_target_rel_tol of 0.01 the loop needs 4129 bisections and makes
    // 24088 front pieces, against 2978 and 14058 with the exact trace -- 39% more bisections and
    // 71% more front pieces for the cheaper trace, which costs 3.3 steps per marched edge against
    // 15.3 (63 at most).
    //
    // WHERE IT STOPS, and why the rule is not "t is unchanged". In exact arithmetic the step
    // D - d is never negative (sphere tracing cannot cross the level set), so t only ever
    // increases; the trace has therefore hit the floor of double arithmetic at the first step
    // that fails to increase t. That is the rule, and it holds no constant. Stopping only on
    // t + (D - d) == t does NOT terminate: measured on presmooth2d/circle with the cap lifted to
    // 2 million steps, the average marched edge took 1.52 million steps and still never reached
    // that equality, because the iteration settles into a cycle over two or four ADJACENT doubles
    // -- |d - D| already down at 1 to 4 units in the last place of D (1.4e-17 to 5.6e-17 there),
    // t stepping one unit in the last place back and forth forever. The point is converged to the
    // last bit at that moment; it is the equality that is unreachable, not the root.
    //
    // WHERE IT GOES WHEN THE STEP STOPS WORKING, and why no iteration cap is needed. Sphere
    // tracing is the fast iteration, not the terminating one, so it is backed by a BRACKET. A
    // marched edge runs from an endpoint ON the input complex, where d = 0 < D, towards an endpoint
    // the marching classified as outside, so d - D changes sign along it and a point with d = D
    // lies between. Every forward step keeps that bracket valid: the step cannot cross the root, so
    // the point it lands on still has d < D and only moves the near end of the bracket up. The
    // first step that fails to move the point forward is one of two things, and they are told apart
    // by the sign of D - d, not by how far t moved:
    //   * d > D -- the trace crossed the root by a rounding error. That point closes the bracket
    //     from above and the loop BISECTS the bracket. Bisection terminates unconditionally: the
    //     width halves every step and cannot halve below the gap between two adjacent doubles, so
    //     the midpoint coincides with an end and the loop stops on that coincidence. The count is
    //     bounded: the root has t >= D (d is 1-Lipschitz and d = 0 at t = 0), so that gap is at
    //     least D x 2^-53, and a bracket no wider than the edge length L reaches it within about
    //     53 + log2(L / D) halvings. That is a property of double arithmetic, not a tuned
    //     constant, and it is what replaces the 200-step cap this function used to carry.
    //   * d < D -- the step is forward but smaller than the gap to the next double, so the root is
    //     inside the same double as t. There is no bracket left to subdivide and t is the answer.
    // `bisection_steps`, when the caller asks for it, returns how many midpoints this call had to
    // evaluate, so a march can report how much of its placement the sphere trace did not do by
    // itself. Under the exact trace the cycle described above stalls on the d > D side with the two
    // ends already adjacent doubles, so the usual answer is zero. Same as 3D.
    const bool exact_trace = m_offset_params.refined_marching;
    const double tol =
        exact_trace ? 0. : std::clamp(m_offset_params.sphere_trace_target_rel_tol, 0., 1.) * D;
    const Vector2d dir = p_out - p_in;
    const double L = dir.norm();
    steps = 0;
    if (bisection_steps) *bisection_steps = 0;
    if (!(L > 0.)) return false;
    const Vector2d u = dir / L;
    // The bracket, in the edge parameter t: d < D at t_lo, d > D at t_hi. t_lo starts at the
    // endpoint on the input complex, where d = 0 < D. t_hi is never read before the trace sets it
    // from a point it evaluated itself, so the far endpoint's distance is never assumed.
    double t_lo = 0.;
    double t_hi = L;
    double t = 0.;
    while (true) {
        const Vector2d p = p_in + t * u;
        const double d = m_input_complex_bvh->dist(p);
        ++steps;
        if (std::abs(d - D) <= tol) {
            p_new = p;
            return true;
        }
        const double t_next = t + (D - d);
        // Past (or at) the far endpoint: the level set is not on this edge. Behind the near
        // endpoint: d(p_in) is already beyond the target, the same conclusion.
        if (t_next >= L || t_next < 0.) return false;
        if (d < D) {
            if (t_next > t) {
                // A forward step: the near end of the bracket moves up to the point just tested.
                t_lo = t;
                t = t_next;
                continue;
            }
            // Forward, but the step vanished in double arithmetic: the root is inside the same
            // double as t, so t is the converged answer and no bracket is left to subdivide.
            p_new = p;
            return true;
        }
        // d > D (d == D returned above): the trace has crossed the root by a rounding error and
        // has no forward step left. This point closes the bracket from above. It is reached only
        // after at least one forward step, because at t = 0 either d < D, or d == D (returned
        // above), or d > D with t_next < 0 (returned above) -- so t_lo and t_hi are distinct.
        t_hi = t;
        break;
    }
    size_t bisections = 0;
    while (true) {
        const double t_mid = t_lo + 0.5 * (t_hi - t_lo);
        // The bracket can no longer be subdivided in double arithmetic: the midpoint is one of the
        // two ends. This is the only exit, and it is always reached (see above).
        if (!(t_mid > t_lo) || !(t_mid < t_hi)) break;
        const Vector2d p_mid = p_in + t_mid * u;
        const double d_mid = m_input_complex_bvh->dist(p_mid);
        ++steps;
        ++bisections;
        if (std::abs(d_mid - D) <= tol) {
            p_new = p_mid;
            if (bisection_steps) *bisection_steps = bisections;
            return true;
        }
        (d_mid < D ? t_lo : t_hi) = t_mid;
    }
    if (bisection_steps) *bisection_steps = bisections;
    // The OUTSIDE end of the bracket, t_hi, for two reasons. It is the side the stalled trace
    // itself stopped on, so a call whose bracket was already two adjacent doubles -- the common
    // case under the exact trace -- places the vertex exactly where this function placed it before
    // the bisection existed. And it is strictly inside the edge: t_hi only ever holds a parameter
    // that was checked against the far endpoint (t_next < L) or a midpoint of the bracket, while
    // t_lo is still 0 -- the endpoint on the input complex itself -- when the trace stalls on its
    // second evaluation.
    p_new = p_in + t_hi * u;
    return true;
}

bool TopoOffsetTriMesh::split_edge_after(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        if (!TriOptimizerMesh::split_edge_after(t)) {
            return false;
        }
        // The labels of the faces this split created were carried from their parents by
        // split_after_vertex(), which the base calls above. Never re-derive them from the tags:
        // a band filled with a tag used elsewhere would relabel that other region as offset.
        ++iter_cnt_split;
        if (m_vertex_extra[t.vid(*this)].m_is_on_offset) ++iter_cnt_split_offset;
        return true;
    }
    return marching_split_edge_after(t);
}

bool TopoOffsetTriMesh::marching_split_edge_after(const Tuple& t)
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
    // Which vertex this split created. refine_for_marching() re-marches its one-ring, and
    // marching_tris() collects them for validate_refined_marching(). As in 3D.
    m_marching_last_new_vid = v_id;
    m_vertex_extra[v_id] = cache.new_v_extra;
    set_vertex_position(v_id, cache.new_v_pos);

    /// check inversion
    //
    // Only under refined_marching, and there it is the position the vertex keeps that is tested.
    // The default path tests nothing here, which is what every existing run measured; 3D tests
    // the midpoint and refuses. On failure the vertex goes to the exact rational point of the
    // edge at the same parameter, the way the shared engine's split does
    // (TriOptimizerMesh::split_edge_after): a point of the edge cannot invert a triangle that was
    // valid before the split, so the bisection always succeeds and the refinement loop can keep
    // refining exactly where the mesh is worst.
    if (m_offset_params.refined_marching) {
        const std::vector<Tuple> locs = get_one_ring_tris_for_vertex(tuple_from_vertex(v_id));
        bool inverted = false;
        for (const Tuple& f : locs) {
            if (is_inverted(f)) {
                inverted = true;
                break;
            }
        }
        if (inverted) {
            ++m_marching_rational_fallbacks;
            const Vector2r& e1 = m_vertex_attribute[cache.v1_id].m_pos;
            const Vector2r& e2 = m_vertex_attribute[cache.v2_id].m_pos;
            // The double position of the vertex is off the edge by a rounding error, and that
            // error is what flattened the triangle. The exact point of the edge at the same
            // parameter is the same point to within that error and lies ON the edge, so it cannot
            // invert anything. The vertex stays un-rounded until a later round().
            m_vertex_attribute[v_id].m_pos = e1 + Rational(cache.new_v_t) * (e2 - e1);
            m_vertex_attribute[v_id].m_is_rounded = false;
            m_vertex_attribute[v_id].m_posf = to_double(m_vertex_attribute[v_id].m_pos);
            // Un-rounded now, so is_inverted takes its rational path: this re-check is exact. It
            // can only fail if an incident triangle was already inverted before the split.
            for (const Tuple& f : locs) {
                if (is_inverted(f)) {
                    log_and_throw_error(
                        "marching_split_edge_after: the exact point of edge ({}, {}) still "
                        "inverts triangle {} -- an incident triangle was already inverted before "
                        "the split",
                        cache.v1_id,
                        cache.v2_id,
                        f.fid(*this));
                }
            }
        }
    }

    // split edge attributes
    for (const auto& pair : cache.opp_v_fattr) {
        size_t opp_v_id = pair.first;

        Tuple ftup_1 = tuple_from_simplex(simplex::Face(cache.v1_id, v_id, opp_v_id));
        size_t f1_id = ftup_1.fid(*this);
        restore_edge(tuple_from_edge(cache.v1_id, v_id, f1_id).eid(*this), cache.split_eattr);

        Tuple ftup_2 = tuple_from_simplex(simplex::Face(cache.v2_id, v_id, opp_v_id));
        size_t f2_id = ftup_2.fid(*this);
        restore_edge(tuple_from_edge(cache.v2_id, v_id, f2_id).eid(*this), cache.split_eattr);

        break;
    }

    // per existing edge attributes
    for (const auto& pair : cache.existing_eattr) {
        size_t e_id = edge_id_from_simplex(pair.first);
        restore_edge(e_id, pair.second);
    }

    // new edges and faces
    for (const auto& pair : cache.opp_v_fattr) {
        size_t opp_v_id = pair.first;
        const FaceSnapshot2d& f_attr = pair.second;

        size_t f1_id = tuple_from_simplex(simplex::Face(cache.v1_id, v_id, opp_v_id)).fid(*this);
        restore_face(f1_id, f_attr);
        size_t f2_id = tuple_from_simplex(simplex::Face(cache.v2_id, v_id, opp_v_id)).fid(*this);
        restore_face(f2_id, f_attr);
        size_t new_e_id = edge_id_from_simplex(simplex::Edge(opp_v_id, v_id));
        // Brand-new edge on a possibly recycled slot: reset both records before writing what is
        // meant. A slot freed by a dead region edge otherwise keeps its m_is_surface_fs and
        // class, and the cross edge is born a phantom region boundary -- tracked, contained by
        // nothing, and poisoning m_is_on_region on every vertex a later split of it creates.
        m_edge_attribute[new_e_id].reset();
        m_edge_extra[new_e_id] = EdgeExtra2d();
        m_edge_extra[new_e_id].label = f_attr.extra.label;
    }

    return true;
}


bool TopoOffsetTriMesh::split_face_before(const Tuple& t)
{
    auto& cache = face_split_cache.local();
    cache.existing_eattr.clear();

    // face id, retain attribute
    size_t f_id = t.fid(*this);
    cache.split_fattr = face_snapshot(f_id);

    // vertices (new vertex attributes too)
    cache.v1_id = t.vid(*this);
    cache.v2_id = t.switch_vertex(*this).vid(*this);
    cache.v3_id = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    Vector2d p1 = m_vertex_attribute[cache.v1_id].m_posf;
    Vector2d p2 = m_vertex_attribute[cache.v2_id].m_posf;
    Vector2d p3 = m_vertex_attribute[cache.v3_id].m_posf;
    cache.new_v_pos = (p1 + p2 + p3) / 3;
    cache.new_v_extra = VertexExtra2d();
    cache.new_v_extra.label = cache.split_fattr.extra.label;
    // Inside the split triangle: on the input complex exactly when the triangle is (label 1), as
    // in 3D's tet split.
    cache.new_v_extra.m_is_on_input = cache.new_v_extra.label == 1;
    // A face's centroid is interior by construction, so it is on no region boundary and no
    // boundary tube: both defaults are the answer.

    // existing edges
    simplex::Edge e1(cache.v1_id, cache.v2_id);
    simplex::Edge e2(cache.v2_id, cache.v3_id);
    simplex::Edge e3(cache.v1_id, cache.v3_id);
    cache.existing_eattr[e1] = edge_snapshot(edge_id_from_simplex(e1));
    cache.existing_eattr[e2] = edge_snapshot(edge_id_from_simplex(e2));
    cache.existing_eattr[e3] = edge_snapshot(edge_id_from_simplex(e3));

    return true;
}


bool TopoOffsetTriMesh::split_face_after(const Tuple& t)
{
    if (!TriMesh::split_face_after(t)) {
        return false;
    }

    auto& cache = face_split_cache.local();
    size_t v_id = get_vertices().size() - 1;
    m_vertex_extra[v_id] = cache.new_v_extra;
    set_vertex_position(v_id, cache.new_v_pos);

    // existing edges
    for (const auto& pair : cache.existing_eattr) {
        size_t e_id = edge_id_from_simplex(pair.first);
        restore_edge(e_id, pair.second);
    }

    // all internal edges and faces
    std::array<size_t, 3> vs = {{cache.v1_id, cache.v2_id, cache.v3_id}};
    for (int i = 0; i < 3; i++) {
        // edge -- a brand-new spoke on a possibly recycled slot; reset both records before
        // writing the label, same phantom-region hazard as in marching_split_edge_after().
        size_t e_id = edge_id_from_simplex(simplex::Edge(vs[i], v_id));
        m_edge_attribute[e_id].reset();
        m_edge_extra[e_id] = EdgeExtra2d();
        m_edge_extra[e_id].label = cache.split_fattr.extra.label;

        // face
        size_t f_id = tuple_from_simplex(simplex::Face(vs[i], vs[(i + 1) % 3], v_id)).fid(*this);
        restore_face(f_id, cache.split_fattr);
    }

    return true;
}

} // namespace wmtk::components::topological_offset
