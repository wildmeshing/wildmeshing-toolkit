#include "TopoOffsetTetMesh.h"

#include <set>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

namespace wmtk::components::topological_offset {

//// TetMesh splitting

bool TopoOffsetTetMesh::split_edge_before(const Tuple& t)
{
    // The optimization phase runs wmtk::TetOptimizerMesh's split; everything else here is the
    // marching-tets machinery, which places the new vertex on the offset's distance field and
    // carries per-simplex labels the shared engine knows nothing about.
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        if (is_edge_on_offset(t)) ++iter_cnt_split_offset_before;
        // Nothing is frozen against splits: refining a surface is not moving it, so the midpoint
        // is checked against its tags' boundary envelopes like any other tracked geometry -- the
        // input complex and the domain wall alike, the wall being a region boundary like any
        // other.
        //
        // Only the Optimization mode is guarded at all: the marching path below is how the offset
        // is constructed, and it has to be able to cut through anything.
        return TetOptimizerMesh::split_edge_before(t);
    }
    return marching_split_edge_before(t);
}

bool TopoOffsetTetMesh::marching_split_edge_before(const Tuple& t)
{
    // load and reset cache
    auto& cache = edge_split_cache.local();
    cache.internal_e.clear();
    cache.external_e.clear();
    cache.link_e.clear();
    cache.split_f.clear();
    cache.internal_f.clear();
    cache.external_f.clear();
    cache.tets.clear();
    cache.changed_faces.clear();

    size_t e_id = t.eid(*this);

    const auto& VA = m_vertex_attribute;

    // vertices
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);

    cache.is_edge_on_region = is_edge_on_region(t);
    cache.is_edge_on_offset = is_edge_on_offset(t);
    // A marching edge has exactly one endpoint on the complex; that endpoint is the new vertex's
    // correspondence. Any other split through this hook records none.
    {
        const bool in1 = m_vertex_extra[cache.v1_id].label != 0;
        const bool in2 = m_vertex_extra[cache.v2_id].label != 0;
        cache.corr_input_vid = (in1 != in2) ? int64_t(in1 ? cache.v1_id : cache.v2_id) : -1;
    }

    Vector3d p1 = VA[cache.v1_id].m_posf;
    Vector3d p2 = VA[cache.v2_id].m_posf;
    Vector3d p_new;
    // Midpoint: no target_distance enters construction at all, and carrying the offset surface
    // out to the level set is the optimization phase's job. SphereTrace (marching_tets under
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
        // marching_tets(), as a count of edges and a count of steps, zero or not.
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
        const Vector3d e12 = p2 - p1;
        const double len2 = e12.squaredNorm();
        cache.new_v_t = len2 > 0. ? std::clamp((p_new - p1).dot(e12) / len2, 0., 1.) : 0.5;
    }
    cache.new_v_extra = VertexExtra();
    cache.new_v_extra.label = m_edge_attribute[e_id].label;
    // On the input complex exactly when the split edge is. Construction keeps the labels exact --
    // an edge of the complex has label 1 -- so the flag follows the label, the same rule
    // mark_input_complex_vertices() derives it by at load. Without this a vertex the refined
    // march bisected onto an input edge carried label 1 and no flag (1861 of them on
    // presmooth3d/cylinder), and the never-on-both-surfaces rule, which reads the flag, did not
    // know the vertex was on the input.
    cache.new_v_extra.m_is_on_input = cache.new_v_extra.label == 1;
    // On both boundaries only if the whole edge was: the midpoint's mask is the AND.
    cache.new_v_extra.m_boundary_mask =
        m_vertex_extra[cache.v1_id].m_boundary_mask & m_vertex_extra[cache.v2_id].m_boundary_mask;

    // split edge
    cache.split_e = m_edge_attribute[e_id];

    // link edge maps (and collect opp vert ids)
    std::set<size_t> opp_verts;
    auto tets = get_incident_tets_for_edge(t); // Tuples
    const simplex::Edge edge(cache.v1_id, cache.v2_id);
    for (const Tuple& t_inc : tets) {
        const simplex::Tet tet = simplex_from_tet(t_inc);
        const simplex::Edge opp = tet.opposite_edge(edge);
        opp_verts.insert(opp.vertices()[0]);
        opp_verts.insert(opp.vertices()[1]);

        // link edge attribute
        Tuple e1 = tuple_from_edge(opp.vertices());
        cache.link_e[opp] = m_edge_attribute[e1.eid(*this)];

        // face attributes
        FaceSnapshot new_f;
        new_f.extra.label = m_tet_attribute[t_inc.tid(*this)].label;
        cache.internal_f[opp] = new_f;

        auto [_1, global_fid1] =
            tuple_from_face({{opp.vertices()[0], opp.vertices()[1], cache.v1_id}});
        auto [_2, global_fid2] =
            tuple_from_face({{opp.vertices()[0], opp.vertices()[1], cache.v2_id}});
        cache.external_f[std::make_pair(opp, cache.v1_id)] = face_snapshot(global_fid1);
        cache.external_f[std::make_pair(opp, cache.v2_id)] = face_snapshot(global_fid2);

        // tet attribute
        cache.tets[opp] = m_tet_attribute[t_inc.tid(*this)];
    }

    // opp vertex maps
    for (const size_t opp_vid : opp_verts) {
        // edge maps
        auto [_1, global_fid1] = tuple_from_face({{opp_vid, cache.v1_id, cache.v2_id}});
        EdgeAttributes new_eattr;
        new_eattr.label = m_face_extra[global_fid1].label;
        cache.internal_e[opp_vid] = new_eattr;

        size_t glob_eid1 = tuple_from_edge({{cache.v1_id, opp_vid}}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({{cache.v2_id, opp_vid}}).eid(*this);
        simplex::Edge e1(cache.v1_id, opp_vid);
        simplex::Edge e2(cache.v2_id, opp_vid);
        cache.external_e[e1] = m_edge_attribute[glob_eid1];
        cache.external_e[e2] = m_edge_attribute[glob_eid2];

        // face maps
        cache.split_f[opp_vid] = face_snapshot(global_fid1);
        // if any incident face is on the surface, the new vertex is on the surface as well
        if (face_is_region(global_fid1)) {
            cache.new_v_extra.m_is_on_region = true;
        }
    }

    return true;
}

bool TopoOffsetTetMesh::edge_split_sphere_trace(
    const Vector3d& p_in,
    const Vector3d& p_out,
    const double D,
    Vector3d& p_new,
    size_t& steps,
    size_t* bisection_steps) const
{
    // Sphere tracing: d is 1-Lipschitz, so from a point at distance d the level set
    // d = D is at least D - d away in every direction, and stepping
    // exactly that far along the edge can never cross it. The step is positive while the trace
    // has not converged (D - d > tol), so t grows by more than tol each time and
    // the loop ends within L / tol steps, one way or the other.
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
    // ends already adjacent doubles, so the usual answer is zero.
    const bool exact_trace = m_offset_params.refined_marching;
    const double tol =
        exact_trace ? 0. : std::clamp(m_offset_params.sphere_trace_target_rel_tol, 0., 1.) * D;
    const Vector3d dir = p_out - p_in;
    const double L = dir.norm();
    steps = 0;
    if (bisection_steps) *bisection_steps = 0;
    if (!(L > 0.)) return false;
    const Vector3d u = dir / L;
    // The bracket, in the edge parameter t: d < D at t_lo, d > D at t_hi. t_lo starts at the
    // endpoint on the input complex, where d = 0 < D. t_hi is never read before the trace sets it
    // from a point it evaluated itself, so the far endpoint's distance is never assumed.
    double t_lo = 0.;
    double t_hi = L;
    double t = 0.;
    while (true) {
        const Vector3d p = p_in + t * u;
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
        const Vector3d p_mid = p_in + t_mid * u;
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

bool TopoOffsetTetMesh::split_edge_after(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        if (!TetOptimizerMesh::split_edge_after(t)) {
            return false;
        }
        ++iter_cnt_split;
        // Read from the result, not from a cached flag: the new vertex is on the offset iff
        // split_after_cells() derived it so from the endpoints.
        if (m_vertex_extra[t.vid(*this)].m_is_on_offset) ++iter_cnt_split_offset;
        return true;
    }
    return marching_split_edge_after(t);
}

bool TopoOffsetTetMesh::marching_split_edge_after(const Tuple& t)
{
    if (!TetMesh::split_edge_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = edge_split_cache.local();
    const size_t v_id = t.vid(*this); // new vertex
    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    // Which vertex this split created. refine_for_marching() re-marches its one-ring, and
    // marching_tets() collects them for validate_refined_marching().
    m_marching_last_new_vid = v_id;

    /// check inversion & rounding
    //
    // Which position is tested differs by path, deliberately. The default path tests the
    // MIDPOINT and refuses on failure -- the caller throws -- and the position the vertex
    // finally keeps, a traced root, is never tested; that is what every existing run measured.
    // refined_marching tests the position the vertex keeps, and on failure places it at the
    // exact rational point of the edge at the same parameter instead of refusing, the way the
    // shared engine's split does (TetOptimizerMesh::split_edge_after): a point of the edge
    // cannot invert a tet that was valid before the split, so the bisection always succeeds and
    // the refinement loop can keep refining exactly where the mesh is worst.
    const bool refined = m_offset_params.refined_marching;
    set_vertex_position(
        v_id,
        refined ? cache.new_v_pos
                : (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2);

    bool inverted = false;
    for (const Tuple& tt : locs) {
        if (is_inverted(tt)) {
            inverted = true;
            break;
        }
    }
    if (inverted && !refined) {
        return false;
    }

    // vertex attribute
    m_vertex_extra[v_id] = cache.new_v_extra;
    m_vertex_extra[v_id].m_corr_input_vid = cache.corr_input_vid;
    if (!refined) {
        set_vertex_position(v_id, cache.new_v_pos);
    } else if (inverted) {
        ++m_marching_rational_fallbacks;
        const Vector3r& e1 = m_vertex_attribute[v1_id].m_pos;
        const Vector3r& e2 = m_vertex_attribute[v2_id].m_pos;
        // The double position of the vertex is off the edge by a rounding error, and that error
        // is what flattened the tet. The exact point of the edge at the same parameter is the
        // same point to within that error and lies ON the edge, so it cannot invert anything.
        // The vertex stays un-rounded (m_pos exact, m_is_rounded false) until a later round().
        m_vertex_attribute[v_id].m_pos = e1 + Rational(cache.new_v_t) * (e2 - e1);
        m_vertex_attribute[v_id].m_is_rounded = false;
        m_vertex_attribute[v_id].m_posf = to_double(m_vertex_attribute[v_id].m_pos);
        // Un-rounded now, so is_inverted takes its rational path: this re-check is exact. It can
        // only fail if an incident tet was already inverted before the split.
        for (const Tuple& tt : locs) {
            if (is_inverted(tt)) {
                log_and_throw_error(
                    "marching_split_edge_after: the exact point of edge ({}, {}) still inverts "
                    "tet {} -- an incident tet was already inverted before the split",
                    v1_id,
                    v2_id,
                    tt.tid(*this));
            }
        }
    }
    m_vertex_extra[v_id].m_is_on_region = cache.is_edge_on_region;
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // on_bbox_faces is written after the base set m_is_on_surface, so a midpoint landing on the
    // domain wall can arrive here unflagged. Re-derive it once both halves of
    // vertex_is_on_region() are known: every containment check is gated on this flag.
    if (vertex_is_on_region(v_id)) {
        m_vertex_attribute[v_id].m_is_on_surface = true;
    }

    // split edges attribute
    size_t split_e1_id = tuple_from_edge({{v1_id, v_id}}).eid(*this);
    size_t split_e2_id = tuple_from_edge({{v2_id, v_id}}).eid(*this);
    m_edge_attribute[split_e1_id] = cache.split_e;
    m_edge_attribute[split_e2_id] = cache.split_e;

    // link edge maps
    for (const auto& pair : cache.link_e) { // for every link edge
        auto link_edge = pair.first;

        // tet attributes
        Tuple t1 = tuple_from_vids(link_edge.vertices()[0], link_edge.vertices()[1], v1_id, v_id);
        Tuple t2 = tuple_from_vids(link_edge.vertices()[0], link_edge.vertices()[1], v2_id, v_id);
        m_tet_attribute[t1.tid(*this)] = cache.tets[link_edge];
        m_tet_attribute[t2.tid(*this)] = cache.tets[link_edge];

        // face attributes
        auto [_1, glob_fid1] =
            tuple_from_face({{link_edge.vertices()[0], link_edge.vertices()[1], v_id}});
        restore_face(glob_fid1, cache.internal_f[link_edge]);

        auto [_2, glob_fid2] =
            tuple_from_face({{link_edge.vertices()[0], link_edge.vertices()[1], v1_id}});
        auto [_3, glob_fid3] =
            tuple_from_face({{link_edge.vertices()[0], link_edge.vertices()[1], v2_id}});
        restore_face(glob_fid2, cache.external_f[std::make_pair(link_edge, v1_id)]);
        restore_face(glob_fid3, cache.external_f[std::make_pair(link_edge, v2_id)]);

        // edge attributes
        size_t link_e_glob_id = tuple_from_edge(link_edge.vertices()).eid(*this);
        m_edge_attribute[link_e_glob_id] = pair.second;
    }

    // oppo vertex maps
    for (const auto& pair : cache.internal_e) { // for every oppo vertex
        size_t opp_vid = pair.first;

        // face attributes
        auto [_1, glob_fid1] = tuple_from_face({{opp_vid, v1_id, v_id}});
        auto [_2, glob_fid2] = tuple_from_face({{opp_vid, v2_id, v_id}});
        restore_face(glob_fid1, cache.split_f[opp_vid]);
        restore_face(glob_fid2, cache.split_f[opp_vid]);

        // edge attributes
        size_t glob_eid = tuple_from_edge({{v_id, opp_vid}}).eid(*this);
        size_t glob_eid1 = tuple_from_edge({{v1_id, opp_vid}}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({{v2_id, opp_vid}}).eid(*this);
        m_edge_attribute[glob_eid] = cache.internal_e[opp_vid];
        m_edge_attribute[glob_eid1] = cache.external_e[simplex::Edge(v1_id, opp_vid)];
        m_edge_attribute[glob_eid2] = cache.external_e[simplex::Edge(v2_id, opp_vid)];
    }

    return true;
}

bool TopoOffsetTetMesh::split_face_before(const Tuple& t)
{
    // load and reset cache
    auto& cache = face_split_cache.local();
    cache.existing_e.clear();
    cache.existing_f.clear();
    cache.tets.clear();

    // get split face tags (used a bunch later)
    size_t split_f_id = t.fid(*this);
    cache.splitf_label = m_face_extra[split_f_id].label;
    cache.splitf_on_offset = face_is_offset(split_f_id);

    // new vertex
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);
    cache.v3_id = switch_vertex(switch_edge(t)).vid(*this);

    // get 1 or 2 vertex(es) opposite to face
    std::vector<size_t> tet_ids;
    tet_ids.push_back(t.tid(*this));
    auto other_tet = switch_tetrahedron(t); // is std::optional<Tuple> object
    if (other_tet) {
        tet_ids.push_back(other_tet.value().tid(*this));
    }
    std::vector<size_t> oppo_vids;
    for (const size_t tet_id : tet_ids) {
        auto tet_vids = oriented_tet_vids(tet_id);
        for (const size_t tet_vid : tet_vids) {
            if ((tet_vid != cache.v1_id) && (tet_vid != cache.v2_id) && (tet_vid != cache.v3_id)) {
                oppo_vids.push_back(tet_vid);
            }
        }
    }
    assert(oppo_vids.size() == tet_ids.size());

    // cache tet attributes
    for (int i = 0; i < tet_ids.size(); i++) {
        cache.tets[oppo_vids[i]] = m_tet_attribute[tet_ids[i]];
    }

    // existing edge maps on split face
    size_t glob_eid1 = tuple_from_edge({{cache.v1_id, cache.v2_id}}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v1_id, cache.v2_id)] = m_edge_attribute[glob_eid1];
    size_t glob_eid2 = tuple_from_edge({{cache.v2_id, cache.v3_id}}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v2_id, cache.v3_id)] = m_edge_attribute[glob_eid2];
    size_t glob_eid3 = tuple_from_edge({{cache.v1_id, cache.v3_id}}).eid(*this);
    cache.existing_e[simplex::Edge(cache.v1_id, cache.v3_id)] = m_edge_attribute[glob_eid3];

    // existing (unmodified) edge and face maps per-tet
    for (const size_t oppo_vid : oppo_vids) {
        // edges
        size_t glob_opp_eid_1 = tuple_from_edge({{cache.v1_id, oppo_vid}}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v1_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_1];
        size_t glob_opp_eid_2 = tuple_from_edge({{cache.v2_id, oppo_vid}}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v2_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_2];
        size_t glob_opp_eid_3 = tuple_from_edge({{cache.v3_id, oppo_vid}}).eid(*this);
        cache.existing_e[simplex::Edge(cache.v3_id, oppo_vid)] = m_edge_attribute[glob_opp_eid_3];

        // faces
        auto [_1, glob_fid1] = tuple_from_face({{cache.v1_id, cache.v2_id, oppo_vid}});
        cache.existing_f[simplex::Face(cache.v1_id, cache.v2_id, oppo_vid)] =
            face_snapshot(glob_fid1);
        auto [_2, glob_fid2] = tuple_from_face({{cache.v2_id, cache.v3_id, oppo_vid}});
        cache.existing_f[simplex::Face(cache.v2_id, cache.v3_id, oppo_vid)] =
            face_snapshot(glob_fid2);
        auto [_3, glob_fid3] = tuple_from_face({{cache.v1_id, cache.v3_id, oppo_vid}});
        cache.existing_f[simplex::Face(cache.v1_id, cache.v3_id, oppo_vid)] =
            face_snapshot(glob_fid3);
    }

    return true;
}

bool TopoOffsetTetMesh::split_face_after(const Tuple& t)
{
    if (!TetMesh::split_face_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = face_split_cache.local();
    // size_t v_id = t.vid(*this);
    size_t v_id = vertex_size() - 1;
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;
    size_t v3_id = cache.v3_id;
    std::array<size_t, 3> splitf_vids = {{v1_id, v2_id, v3_id}};

    // new_vertex
    set_vertex_position(
        v_id,
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf +
         m_vertex_attribute[v3_id].m_posf) /
            3);
    m_vertex_extra[v_id].label = cache.splitf_label;
    // On the input complex exactly when the split face is, by its label, as in the edge split.
    // Assigned, not left alone -- the slot may be recycled and carry a dead vertex's flag.
    m_vertex_extra[v_id].m_is_on_input = cache.splitf_label == 1;
    // On the offset surface exactly when the split face is a front triangle, read in
    // split_face_before() -- the face's own class, not its corners, for the reason
    // split_after_cells() gives for edges: three front vertices can span a triangle that crosses
    // a region. It matters because split_face() is no longer construction-only: the cap phase
    // splits front faces at their centroids, and without this the new vertex is not a front
    // vertex, so nothing places it on the level set and nothing measures it (it sat at 0.6 of
    // target_distance while the criterion reported the front resolved).
    m_vertex_extra[v_id].m_is_on_offset = cache.splitf_on_offset;
    // Interior to the split face, so on exactly the boundaries the whole face is on: the AND of
    // its corners. Assigned, not OR'd -- the slot may be recycled.
    m_vertex_extra[v_id].m_boundary_mask = m_vertex_extra[v1_id].m_boundary_mask &
                                           m_vertex_extra[v2_id].m_boundary_mask &
                                           m_vertex_extra[v3_id].m_boundary_mask;

    // new edges/faces on split face
    EdgeAttributes splitf_eattr;
    splitf_eattr.label = cache.splitf_label;
    FaceSnapshot splitf_fattr;
    splitf_fattr.extra.label = cache.splitf_label;
    for (int i = 0; i < 3; i++) {
        size_t curr_v1_id = splitf_vids[i];
        size_t curr_v2_id = splitf_vids[(i + 1) % 3];

        size_t glob_eid = tuple_from_edge({{curr_v1_id, v_id}}).eid(*this); // new edge
        m_edge_attribute[glob_eid] = splitf_eattr;

        auto [_, glob_fid] = tuple_from_face({{curr_v1_id, curr_v2_id, v_id}}); // new face
        restore_face(glob_fid, splitf_fattr);
    }

    // existing edges on split face
    size_t glob_eid1 = tuple_from_edge({{v1_id, v2_id}}).eid(*this);
    size_t glob_eid2 = tuple_from_edge({{v2_id, v3_id}}).eid(*this);
    size_t glob_eid3 = tuple_from_edge({{v1_id, v3_id}}).eid(*this);
    m_edge_attribute[glob_eid1] = cache.existing_e[simplex::Edge(v1_id, v2_id)];
    m_edge_attribute[glob_eid2] = cache.existing_e[simplex::Edge(v2_id, v3_id)];
    m_edge_attribute[glob_eid3] = cache.existing_e[simplex::Edge(v1_id, v3_id)];

    // per oppo-vert
    for (const auto& pair : cache.tets) {
        size_t opp_vid = pair.first;

        // new edge
        size_t glob_newe_id = tuple_from_edge({{v_id, opp_vid}}).eid(*this);
        m_edge_attribute[glob_newe_id].label = pair.second.label;

        // every pair of existing split face verts (every split face edge)
        for (int i = 0; i < 3; i++) {
            size_t curr_v1_id = splitf_vids[i];
            size_t curr_v2_id = splitf_vids[(i + 1) % 3];

            // new tet
            Tuple tet = tuple_from_vids(v_id, opp_vid, curr_v1_id, curr_v2_id);
            m_tet_attribute[tet.tid(*this)] = pair.second;

            // new face
            auto [_, glob_fid] = tuple_from_face({{v_id, curr_v1_id, opp_vid}});
            m_face_extra[glob_fid].label = pair.second.label;

            // existing face
            auto [_2, glob_fid2] = tuple_from_face({{opp_vid, curr_v1_id, curr_v2_id}});
            restore_face(
                glob_fid2,
                cache.existing_f[simplex::Face(opp_vid, curr_v1_id, curr_v2_id)]);

            // existing edge
            size_t glob_eid = tuple_from_edge({{curr_v1_id, opp_vid}}).eid(*this);
            m_edge_attribute[glob_eid] = cache.existing_e[simplex::Edge(curr_v1_id, opp_vid)];
        }
    }

    return true;
}

bool TopoOffsetTetMesh::split_tet_before(const Tuple& t)
{
    auto& cache = tet_split_cache.local();
    cache.existing_e.clear();
    cache.existing_f.clear();

    // vertices
    auto vs = oriented_tet_vids(t);
    for (int i = 0; i < 4; i++) { // deep copy just to be safe
        cache.v_ids[i] = vs[i];
    }

    // cache retained edge attributes
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            size_t glob_eid = tuple_from_edge({{cache.v_ids[i], cache.v_ids[j]}}).eid(*this);
            cache.existing_e[simplex::Edge(cache.v_ids[i], cache.v_ids[j])] =
                m_edge_attribute[glob_eid];
        }
    }

    // cache retained face attributes
    for (int i = 0; i < 4; i++) {
        size_t v1 = cache.v_ids[i];
        size_t v2 = cache.v_ids[(i + 1) % 4];
        size_t v3 = cache.v_ids[(i + 2) % 4];
        auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
        cache.existing_f[simplex::Face(v1, v2, v3)] = face_snapshot(glob_fid);
    }

    // tet attribute
    cache.tet = m_tet_attribute[t.tid(*this)];

    return true;
}

bool TopoOffsetTetMesh::split_tet_after(const Tuple& t)
{
    if (!TetMesh::split_tet_after(t)) {
        return false;
    } // why do we need this?

    auto& cache = tet_split_cache.local();
    int tet_label = cache.tet.label;
    size_t v_id = vertex_size() - 1;

    // new vertex
    set_vertex_position(
        v_id,
        (m_vertex_attribute[cache.v_ids[0]].m_posf + m_vertex_attribute[cache.v_ids[1]].m_posf +
         m_vertex_attribute[cache.v_ids[2]].m_posf + m_vertex_attribute[cache.v_ids[3]].m_posf) /
            4);
    m_vertex_extra[v_id].label = tet_label;
    // Inside the split tet: on the input complex exactly when the tet is (label 1, a cell of a
    // solid complex), and never on the front. Both assigned -- the slot may be recycled and carry
    // a dead vertex's flags.
    m_vertex_extra[v_id].m_is_on_input = tet_label == 1;
    m_vertex_extra[v_id].m_is_on_offset = false;
    // Strictly interior to a tet: on no boundary at all. Assigned -- the slot may be recycled.
    // As in split_face_after(), split_tet() runs only during construction, so there are no
    // surface flags to derive; nothing in the optimization creates a tet-interior vertex.
    m_vertex_extra[v_id].m_boundary_mask = 0;

    // iterate over new tets (retained faces, new tets, new edge (opposite tet) )
    for (int i = 0; i < 4; i++) {
        size_t v1 = cache.v_ids[i];
        size_t v2 = cache.v_ids[(i + 1) % 4];
        size_t v3 = cache.v_ids[(i + 2) % 4];

        // new edge (doesn't matter which vertex, will iterate through all 4)
        size_t glob_new_eid = tuple_from_edge({{v_id, v1}}).eid(*this);
        m_edge_attribute[glob_new_eid].label = tet_label;

        // retained face
        auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
        restore_face(glob_fid, cache.existing_f[simplex::Face(v1, v2, v3)]);

        // new tet
        size_t t_id = tuple_from_vids(v1, v2, v3, v_id).tid(*this);
        m_tet_attribute[t_id] = cache.tet; // TO VERIFY: this creates deep copy?
    }

    // existing edges and new faces
    for (int i = 0; i < 3; i++) {
        for (int j = i + 1; j < 4; j++) {
            size_t v1 = cache.v_ids[i];
            size_t v2 = cache.v_ids[j];

            // existing edge
            size_t glob_eid = tuple_from_edge({{v1, v2}}).eid(*this);
            m_edge_attribute[glob_eid] = cache.existing_e[simplex::Edge(v1, v2)];

            // new face
            auto [_, glob_fid] = tuple_from_face({{v1, v2, v_id}});
            m_face_extra[glob_fid].label = tet_label;
        }
    }

    return true;
}

/**
 * The shared split places the new vertex, keeps quality and shared attributes, and checks envelope
 * containment. These three hooks add what only the offset knows: which region tag each child tet
 * inherits, and which tracked surfaces the new vertex joined.
 */
bool TopoOffsetTetMesh::split_before_cells(const Tuple& edge, const std::vector<Tuple>& parents)
{
    auto& cache = m_opt_split_cache.local();
    cache.tets.clear();
    cache.is_edge_on_region = is_edge_on_region(edge);
    cache.is_edge_on_offset = is_edge_on_offset(edge);
    cache.is_edge_in_input = false;
    {
        const size_t a = edge.vid(*this), b = edge.switch_vertex(*this).vid(*this);
        if (m_vertex_extra[a].m_is_on_input && m_vertex_extra[b].m_is_on_input) {
            for (const Tuple& tt : parents) {
                if (m_tet_attribute[tt.tid(*this)].label == 1) cache.is_edge_in_input = true;
                for (const size_t x : oriented_tet_vids(tt)) {
                    if (x == a || x == b) continue;
                    if (face_is_complex_boundary(std::get<0>(tuple_from_face({{a, b, x}})))) {
                        cache.is_edge_in_input = true;
                    }
                }
            }
        }
    }
    // parent_q_max is diagnostic: split_after_vertex() uses it to say whether a needle child
    // came from a parent that was already unscoreable, or from a healthy one.
    cache.parent_q_max = -1.;
    cache.parent_flatness = 1.;
    for (const Tuple& tt : parents) {
        cache.parent_q_max = std::max(cache.parent_q_max, tet_amips(tt.tid(*this)));
        cache.parent_flatness = std::min(cache.parent_flatness, tet_flatness(tt.tid(*this)));
    }

    // Key each parent by the edge OPPOSITE the one being split: that edge survives the split
    // and is shared by exactly the two children of this parent, so it names them afterwards.
    const simplex::Edge e(edge.vid(*this), edge.switch_vertex(*this).vid(*this));
    for (const Tuple& tt : parents) {
        cache.tets[simplex_from_tet(tt).opposite_edge(e)] = m_tet_attribute.at(tt.tid(*this));
    }
    return true;
}

bool TopoOffsetTetMesh::split_after_cells(
    const size_t v1_id,
    const size_t v2_id,
    const size_t v_id,
    const std::vector<Tuple>&)
{
    // The midpoint lies on a surface exactly when the split EDGE lies in it, which
    // split_before_cells() decided from the faces and cells around the edge before they were
    // replaced: on the front iff a front triangle contains the edge, on the input complex iff an
    // input cell or input face contains it. Not from the endpoints: two vertices of a surface can
    // be joined by an edge that crosses a region -- a chord across a bend of the surface -- and
    // its midpoint is then in that region, off the surface. The endpoint rule this replaces
    // labelled such midpoints as on the surface: measured, 70 front and 40 input mislabels on
    // presmooth3d/sheet at max_rounds 3 (0 on cube, prism, and the remesh pass on cylinder and
    // sphere). Both flags still require both endpoints on the surface (the cached tests check it),
    // so an edge from the complex to the front still yields a midpoint on neither, and the
    // never-both invariant survives the split.
    {
        const auto& c = m_opt_split_cache.local();
        const bool front_by_ends =
            m_vertex_extra[v1_id].m_is_on_offset && m_vertex_extra[v2_id].m_is_on_offset;
        const bool input_by_ends =
            m_vertex_extra[v1_id].m_is_on_input && m_vertex_extra[v2_id].m_is_on_input;
        m_vertex_extra[v_id].m_is_on_offset = c.is_edge_on_offset;
        m_vertex_extra[v_id].m_is_on_input = c.is_edge_in_input;
        // Forensics: the chords the endpoint rule would have mislabelled, see
        // iter_cnt_split_front_chord.
        if (front_by_ends && !c.is_edge_on_offset) ++iter_cnt_split_front_chord;
        if (input_by_ends && !c.is_edge_in_input) ++iter_cnt_split_input_chord;
    }
    // Churn instrumentation, read only by collapse_after_vertex(). Assigned, never OR'd: v_id may
    // be a recycled slot whose previous occupant was born long ago. See m_born_epoch.
    m_vertex_extra[v_id].m_born_epoch = m_op_epoch;
    if (m_op_epoch != 0) ++iter_cnt_split_born;
    // The boundary mask follows the same AND rule: the midpoint is on a tag boundary only if the
    // whole edge was. Assigned, not OR'd -- v_id may be a recycled slot carrying a dead vertex's
    // bits. Runs before the shared split's containment check, which reads the mask through
    // face_mask() on the two child triangles.
    m_vertex_extra[v_id].m_boundary_mask =
        m_vertex_extra[v1_id].m_boundary_mask & m_vertex_extra[v2_id].m_boundary_mask;

    const auto& cache = m_opt_split_cache.local();
    for (const size_t v_end : {v1_id, v2_id}) {
        const simplex::Edge half(v_end, v_id);
        for (const Tuple& tt : get_incident_tets_for_edge(v_end, v_id)) {
            const auto it = cache.tets.find(simplex_from_tet(tt).opposite_edge(half));
            if (it == cache.tets.end()) {
                return false; // no parent to inherit from; refuse rather than mislabel a tet
            }
            // The quality is written by the shared split just after this returns.
            m_tet_attribute[tt.tid(*this)] = it->second;
        }
    }
    return true;
}

} // namespace wmtk::components::topological_offset
