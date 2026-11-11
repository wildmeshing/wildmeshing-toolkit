#include "TopoOffsetTetMesh.h"

#include <algorithm>
#include <set>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

namespace wmtk::components::topological_offset {

//// TetMesh splitting

bool TopoOffsetTetMesh::split_edge_before(const Tuple& t)
{
    // A correspondence carried from an earlier offset is the edge from its offset vertex to the
    // input vertex it names; cutting that edge loses it, whichever step cuts (the simplicial
    // embedding, the march). The march cuts no edge of a protected cell (marching_tets()), so an
    // earlier band that is protected keeps every one. Until 2026-11-06 the run instead compared
    // each carried vertex's star before and after construction, which also refused cuts beside
    // the earlier band: feature demo 4, the wire's band protected, 258 of its 1886 stars.
    {
        const size_t a = t.vid(*this);
        const size_t b = switch_vertex(t).vid(*this);
        const auto names = [&](const size_t o, const size_t i) {
            return m_vertex_extra[o].m_corr_carried &&
                   m_vertex_extra[o].m_corr_input_vid == int64_t(i);
        };
        if (names(a, b) || names(b, a)) {
            log_and_throw_error(
                "the offset cuts the edge from an earlier offset's vertex to its input vertex "
                "(corr_input_vid), vertices {} and {}: protect the earlier band (protected_tags)",
                a,
                b);
        }
    }
    // The optimization phase runs wmtk::TetOptimizerMesh's split; everything else here is the
    // marching-tets machinery, which places the new vertex on the offset's distance field and
    // carries per-simplex labels the shared engine knows nothing about.
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        // repulsion_rounds, before the march: a marched edge is never split -- its midpoint would
        // be a new outer end nearer the input than the one the rounds push out.
        if (m_repulsion_potential && is_marched_edge(t.vid(*this), switch_vertex(t).vid(*this))) {
            return false;
        }
        if (is_edge_on_offset(t) && !dry_run()) ++iter_cnt_split_offset_before;
        // Longest-edge order: a split waits while one of the tets incident to its edge has a
        // strictly longer edge that is itself over the split gate. Bisecting a tet on a shorter
        // edge than its longest flattens it; its edges stay over the gate and it is bisected
        // again, compounding. The serial pass never does this, because its max-heap on length
        // pops every longer edge first. The parallel pass does: an operation that cannot lock
        // its ring is deferred, long edges sit in busy neighbourhoods and lose that race, and
        // shorter edges of the same tets are split first. On the cube at 1e-2 / 1e-4, 10 threads,
        // 570 of turn 1's first 810 splits were off a parent's longest edge (serial: 0 of 789)
        // and the split passes' max energy reached 1e8..1e11.
        //
        // A waiting split is not lost. When the longer edge is split, the renewal re-queues every
        // edge of the new tets, this one among them, and run_localized_to_convergence retries a
        // failure whose endpoints a success touched.
        //
        // A pocket split (front_pocket_split()) does not wait: it is one serial call outside the
        // pass, nothing retries it, and a refused one leaves its pinned cell to the smoother.
        // MEASURED with the wait (cube_in, target 0.3, 7 turns): 580 pocket splits refused by it
        // against 248 committed, and 46 of 55 pinned cells left before turn 1's collapse group's
        // first smoothing pass. It is still counted as off the longest edge.
        const double l2 = get_length2(t);
        bool off_longest = false;
        for (const Tuple& tet : get_incident_tets_for_edge(t)) {
            for (int j = 0; j < 6; ++j) {
                const Tuple f = tuple_from_edge(tet.tid(*this), j);
                if (!(get_length2(f) > l2)) continue;
                if (!m_pocket_splitting && split_edge_is_due(f)) {
                    ++m_split_order_waits;
                    return false;
                }
                off_longest = true; // longer, but under the gate
            }
        }
        split_off_longest() = off_longest;
        // Nothing is frozen against splits: refining a surface is not moving it, so the midpoint
        // is checked against its tags' boundary envelopes like any other tracked geometry -- the
        // input complex and every other region boundary alike. A midpoint on the domain wall
        // stays on the planes both ends share (the base's split_edge_after()), the box rule.
        //
        // Only the Optimization mode is guarded at all: the marching path below is how the offset
        // is constructed, and it has to be able to cut through anything.
        return TetOptimizerMesh::split_edge_before(t);
    }
    return marching_split_edge_before(t);
}

bool TopoOffsetTetMesh::split_edge_is_due(const Tuple& e) const
{
    // Must stay the shared pass's own test, or the order above waits on edges the pass will never
    // split: an edge is due when length^2 >= splitting_l2 * s^2, s the mean of its endpoints'
    // sizing scalars, and a force-split edge always is. Both the mean and the length are
    // symmetric in IEEE arithmetic, so the endpoint order of `e` does not matter.
    const size_t v1 = e.vid(*this);
    const size_t v2 = e.switch_vertex(*this).vid(*this);
    // repulsion_rounds: split_edge_before() refuses a marched edge, so it is never due, and no
    // shorter edge waits on it.
    if (m_repulsion_potential && is_marched_edge(v1, v2)) return false;
    if (is_force_split_edge(v1, v2)) return true;
    const double s =
        (m_vertex_attribute[v1].m_sizing_scalar + m_vertex_attribute[v2].m_sizing_scalar) / 2;
    return !(get_length2(e) < m_params.splitting_l2 * s * s);
}

void TopoOffsetTetMesh::op_event(const OpKind k, const OpEvent e) const
{
    TetOptimizerMesh::op_event(k, e);
    if (k != OpKind::split) return;
    // Counted here because `committed` is the one place a split is known to have survived both
    // hooks and invariants(); the flag is cleared at `attempt`, so a marching split never counts.
    if (e == OpEvent::attempt) {
        split_off_longest() = false;
    } else if (e == OpEvent::committed && split_off_longest()) {
        m_split_off_longest.fetch_add(1, std::memory_order_relaxed);
    }
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

    // The feature and order-2 edge tags, cached as TetOptimizerMesh::split_edge_before() caches
    // them: they are stored by edge slot, and the march runs after hold_curves_and_points() tagged
    // the order-2 edges, so a split that does not rewrite them leaves each on whatever edge takes
    // over its slot. MEASURED (2026-11-05, an open sheet offset with the optimization): at the
    // loop's start none of the sheet's 16 open edges was tagged and 16 tagged edges lay outside
    // their tube; the construction pass then cut the sheet's boundary, 11% of its area.
    if (m_track_feature_edges) {
        feature_edges_cache(get_incident_tids_for_edge(t), cache.feature_edges);
        cache.split_edge_tags = m_feature_edge_attribute[e_id];
    }

    const auto& VA = m_vertex_attribute;

    // vertices
    cache.v1_id = t.vid(*this);
    cache.v2_id = switch_vertex(t).vid(*this);

    cache.is_edge_on_region = is_edge_on_region(t);
    cache.is_edge_on_offset = is_edge_on_offset(t);

    Vector3d p1 = VA[cache.v1_id].m_posf;
    Vector3d p2 = VA[cache.v2_id].m_posf;
    Vector3d p_new;
    // Midpoint: the plain edge midpoint. SphereTrace (marching_tets(), see construction_mode):
    // the vertex goes to the point of the edge where d(x) reaches m_construction_distance within
    // the tolerance, and to the midpoint when the trace leaves the edge.
    if (m_edge_split_mode == EdgeSplitMode::Midpoint) {
        p_new = (p1 + p2) / 2.0;
    } else if (m_edge_split_mode == EdgeSplitMode::SphereTrace) {
        const bool in1 = m_vertex_extra[cache.v1_id].label != 0;
        const bool in2 = m_vertex_extra[cache.v2_id].label != 0;
        bool on_level = false;
        size_t steps = 0;
        // The entry of offsets the inner end is on: its distance, its target, its mode.
        const size_t k = offset_of(in1 ? cache.v1_id : cache.v2_id);
        if (in1 != in2 && k < m_construction_traced.size() && m_construction_traced[k]) {
            on_level = in1 ? edge_split_sphere_trace(k, p1, p2, p_new, steps)
                           : edge_split_sphere_trace(k, p2, p1, p_new, steps);
        }
        m_marching_trace_steps += steps;
        m_marching_trace_steps_max = std::max(m_marching_trace_steps_max, steps);
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
    cache.new_v_extra = VertexExtra();
    cache.new_v_extra.label = m_edge_attribute[e_id].label;
    // The march splits exactly the edges with one endpoint labelled 0 (marching_tets()), and the
    // other endpoint is on the input complex: that endpoint is the new vertex's correspondence.
    // The simplicial embedding's edge splits pass through here too; their edges have both
    // endpoints on the complex and record none. As in 2D.
    {
        const bool in1 = m_vertex_extra[cache.v1_id].label != 0;
        const bool in2 = m_vertex_extra[cache.v2_id].label != 0;
        cache.new_v_extra.m_corr_input_vid =
            (in1 != in2) ? int64_t(in1 ? cache.v1_id : cache.v2_id) : int64_t(-1);
    }

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
    const size_t k,
    const Vector3d& p_in,
    const Vector3d& p_out,
    Vector3d& p_new,
    size_t& steps) const
{
    // Sphere tracing to D = m_construction_distances[k] (entry k's target_distance, or the
    // construction_mode fallback; see the marching), on d the distance to entry k's complex
    // (offset_distance()): d is 1-Lipschitz, so from a point at distance d the level set
    // d = D is at least D - d away in every direction, and stepping exactly that far along the
    // edge can never cross it. The step is positive while the trace has not converged
    // (D - d > tol), so t grows by more than tol each time and the loop ends within L / tol
    // steps, one way or the other.
    const double D = m_construction_distances[k];
    const double tol = std::clamp(m_offset_params.sphere_trace_target_rel_tol, 0., 1.) * D;
    const Vector3d dir = p_out - p_in;
    const double L = dir.norm();
    steps = 0;
    if (!(L > 0.)) return false;
    const Vector3d u = dir / L;
    double t = 0.;
    while (true) {
        const Vector3d p = p_in + t * u;
        const double d = offset_distance(k, p);
        ++steps;
        if (std::abs(d - D) <= tol) {
            p_new = p;
            return true;
        }
        t += D - d;
        // Past (or at) the far endpoint: the level set is not on this edge. Behind the near
        // endpoint: d(p_in) is already beyond the target, the same conclusion.
        if (t >= L || t < 0.) return false;
    }
}

bool TopoOffsetTetMesh::split_edge_after(const Tuple& t)
{
    if (m_edge_split_mode == EdgeSplitMode::Optimization) {
        if (!TetOptimizerMesh::split_edge_after(t)) return false;
        // EXPERIMENTAL_split_energy_gate: the children's tet_energy() -- every cell at the new
        // vertex, labels inherited -- must sum strictly below the parents' (a NaN refuses). The
        // engine rolls a refused split back. The loop's gate: TetWild's rules (m_op_rules, every
        // TetWild pass) have no split rule.
        // "exact_swept": the AMIPS terms alone, a split leaving B and its band term unchanged
        if (m_offset_params.split_energy_gate && !tetwild_rules() &&
            !(tet_energy_sum(get_one_ring_tids_for_vertex(t.vid(*this)), !swept_rule()) <
              m_split_energy_before.local())) {
            ++m_volume_reject_split;
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

    /// check inversion & rounding
    set_vertex_position(
        v_id,
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2);

    for (const Tuple& tt : locs) {
        if (is_inverted(tt)) {
            return false;
        }
    }

    // vertex attribute
    m_vertex_extra[v_id] = cache.new_v_extra;
    set_vertex_position(v_id, cache.new_v_pos);
    m_vertex_extra[v_id].m_is_on_region = cache.is_edge_on_region;
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // The sizing field's value where the new vertex is, linear along the edge: the mean of the
    // ends' scalars at the midpoint, as the engine's split gives it, and the traced point's own
    // interpolate under SphereTrace. Without it the vertex kept its slot's scalar, so the march
    // dropped any field set before it at every front vertex. With every scalar 1 it is 1.
    {
        const double s1 = m_vertex_attribute[v1_id].m_sizing_scalar;
        const double s2 = m_vertex_attribute[v2_id].m_sizing_scalar;
        const Vector3d d = m_vertex_attribute[v2_id].m_posf - m_vertex_attribute[v1_id].m_posf;
        const double dd = d.squaredNorm();
        double& s = m_vertex_attribute[v_id].m_sizing_scalar;
        if (m_edge_split_mode == EdgeSplitMode::SphereTrace && dd > 0.) {
            const double u = std::clamp(
                (cache.new_v_pos - m_vertex_attribute[v1_id].m_posf).dot(d) / dd,
                0.,
                1.);
            s = s1 + u * (s2 - s1);
        } else {
            s = (s1 + s2) / 2;
        }
    }
    // The base's union flag, which every containment check is gated on: raised for a midpoint on
    // a region boundary. Not for one on the domain wall alone, which is no tracked face (the box
    // rule holds it, through on_bbox_faces), as no wall vertex carries it at load. Until 2026-11
    // the wall was tracked, and this raised it for the wall too (vertex_is_on_region()).
    if (m_vertex_extra[v_id].m_is_on_region) {
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

        // face attributes: the two halves of the split face (v1, v2, opp) lie on its surfaces,
        // each with its orientation converted as the engine's split converts it
        // (TetOptimizerMeshSplit.cpp): (v1, v2, opp) -> (v1, v, opp) + (v, v2, opp), each half
        // replacing one endpoint by v in place, so along those orders it carries the face's
        // orientation along (v1, v2, opp). The orientation is stored against the ascending order
        // of a face's own ids; until 2026-11 the snapshot was restored as is, right on a half only
        // when v sorted into the replaced corner's place, so the output's face-tag triangles
        // (io::collect_input_set_tags()) could come out turned over.
        const FaceSnapshot& split_face = cache.split_f[opp_vid];
        const std::array<size_t, 3> parent{{v1_id, v2_id, opp_vid}};
        FaceSnapshot half1 = split_face, half2 = split_face;
        half1.tags.set_orientation_from(
            split_face.tags,
            parent,
            std::array<size_t, 3>{{v1_id, v_id, opp_vid}});
        half2.tags.set_orientation_from(
            split_face.tags,
            parent,
            std::array<size_t, 3>{{v_id, v2_id, opp_vid}});
        auto [_1, glob_fid1] = tuple_from_face({{opp_vid, v1_id, v_id}});
        auto [_2, glob_fid2] = tuple_from_face({{opp_vid, v2_id, v_id}});
        restore_face(glob_fid1, half1);
        restore_face(glob_fid2, half2);

        // edge attributes
        size_t glob_eid = tuple_from_edge({{v_id, opp_vid}}).eid(*this);
        size_t glob_eid1 = tuple_from_edge({{v1_id, opp_vid}}).eid(*this);
        size_t glob_eid2 = tuple_from_edge({{v2_id, opp_vid}}).eid(*this);
        m_edge_attribute[glob_eid] = cache.internal_e[opp_vid];
        m_edge_attribute[glob_eid1] = cache.external_e[simplex::Edge(v1_id, opp_vid)];
        m_edge_attribute[glob_eid2] = cache.external_e[simplex::Edge(v2_id, opp_vid)];
    }

    // The edge tags, as TetOptimizerMesh::split_edge_after() restores them: every edge of the new
    // cells from the cache, the split edge's two halves inheriting its tags.
    if (m_track_feature_edges) {
        feature_edges_restore_region(locs, cache.feature_edges);
        if (cache.split_edge_tags.is_tagged()) {
            m_feature_edge_attribute[tuple_from_edge({{v1_id, v_id}}).eid(*this)] =
                cache.split_edge_tags;
            m_feature_edge_attribute[tuple_from_edge({{v2_id, v_id}}).eid(*this)] =
                cache.split_edge_tags;
        }
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
    cache.split_f = face_snapshot(split_f_id);
    cache.splitf_on_region = face_is_region(split_f_id);

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
    // The returned tuple sits on the edge from an original vertex to the new one. Not
    // vertex_size() - 1: that counts the unremoved vertices, which undercounts once a collapse
    // has run without a consolidate in between, and a slot is only the last one until another
    // thread splits.
    const size_t v_id = t.switch_vertex(*this).vid(*this);
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
    // The sizing field at the centroid, as marching_split_edge_after() carries it along an edge.
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar +
         m_vertex_attribute[v3_id].m_sizing_scalar) /
        3;
    // On the split face's surfaces, as the edge split's new vertex is on its edge's
    // (marching_split_edge_after()): a region boundary's, the walls all three corners share, and
    // then, for a region boundary, the base's union flag, which every containment check is gated
    // on (not for the wall, which is no tracked face).
    m_vertex_extra[v_id].m_is_on_region = cache.splitf_on_region;
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        wmtk::set_intersection(
            m_vertex_attribute[v1_id].on_bbox_faces,
            m_vertex_attribute[v2_id].on_bbox_faces),
        m_vertex_attribute[v3_id].on_bbox_faces);
    if (m_vertex_extra[v_id].m_is_on_region) {
        m_vertex_attribute[v_id].m_is_on_surface = true;
    }

    // new edges/faces on split face
    EdgeAttributes splitf_eattr;
    splitf_eattr.label = cache.splitf_label;
    // THE SPLIT RULE, as the edge split, the tet split and the 2D edge split keep it: a piece of a
    // split simplex lies on every surface the simplex was on; a simplex new inside one cell lies
    // on none. So each piece takes the split face's snapshot -- tracking, wall side, class, input
    // set, sheet, label -- with its orientation, which is stored against the ascending order of
    // a face's own vertices, converted: the piece (v_i, v_i+1, v) replaces v_i+2 by v in place,
    // so along that order it carries the face's orientation along (v_i, v_i+1, v_i+2), a cyclic
    // turn of (v1, v2, v3). Until 2026-11 the pieces took a fresh snapshot with the label alone,
    // so a tracked face split here lost its tracking: its surface got a hole -- its topology no
    // longer kept, its face tags missing on the pieces, nothing holding them.
    for (int i = 0; i < 3; i++) {
        size_t curr_v1_id = splitf_vids[i];
        size_t curr_v2_id = splitf_vids[(i + 1) % 3];

        size_t glob_eid = tuple_from_edge({{curr_v1_id, v_id}}).eid(*this); // new edge
        m_edge_attribute[glob_eid] = splitf_eattr;

        FaceSnapshot piece = cache.split_f;
        piece.tags.set_orientation_from(
            cache.split_f.tags,
            splitf_vids,
            std::array<size_t, 3>{{curr_v1_id, curr_v2_id, v_id}});
        auto [_, glob_fid] = tuple_from_face({{curr_v1_id, curr_v2_id, v_id}}); // new face
        restore_face(glob_fid, piece);
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

            // new face: a fresh snapshot carrying the tet's label alone, as split_edge_after()
            // writes its interior faces and the 2D face split resets its new edges
            // (EdgeSplittingTri.cpp). Writing only the label left whatever surface tags the
            // reused slot held. Measured on the Geneva base (simplicial embedding): an input
            // face's tracked tag landed on a new interior face whose new vertex lies 0.48 delta
            // off the input, and that face held the march's vertices on it.
            FaceSnapshot new_f;
            new_f.extra.label = pair.second.label;
            auto [_, glob_fid] = tuple_from_face({{v_id, curr_v1_id, opp_vid}});
            restore_face(glob_fid, new_f);

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
    // As in split_face_after().
    const size_t v_id = t.switch_vertex(*this).vid(*this);

    // new vertex
    set_vertex_position(
        v_id,
        (m_vertex_attribute[cache.v_ids[0]].m_posf + m_vertex_attribute[cache.v_ids[1]].m_posf +
         m_vertex_attribute[cache.v_ids[2]].m_posf + m_vertex_attribute[cache.v_ids[3]].m_posf) /
            4);
    m_vertex_extra[v_id].label = tet_label;
    // The sizing field at the centroid, as marching_split_edge_after() carries it along an edge.
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[cache.v_ids[0]].m_sizing_scalar +
         m_vertex_attribute[cache.v_ids[1]].m_sizing_scalar +
         m_vertex_attribute[cache.v_ids[2]].m_sizing_scalar +
         m_vertex_attribute[cache.v_ids[3]].m_sizing_scalar) /
        4;

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

            // new face: a fresh snapshot carrying the tet's label alone (see split_face_after())
            FaceSnapshot new_f;
            new_f.extra.label = tet_label;
            auto [_, glob_fid] = tuple_from_face({{v1, v2, v_id}});
            restore_face(glob_fid, new_f);
        }
    }

    return true;
}

void TopoOffsetTetMesh::face_extra_capture(const std::vector<size_t>& tids, FaceExtraMap& out) const
{
    // See the declaration: the base's face_attribute_tracker() (TetOptimizerMeshSwaps.cpp).
    out.clear();
    for (const size_t tid : tids) {
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            auto vids = get_face_vids(f);
            std::sort(vids.begin(), vids.end());
            out.try_emplace(vids, m_face_extra[f.fid(*this)]);
        }
    }
}

void TopoOffsetTetMesh::face_extra_assign(
    const std::vector<size_t>& tids,
    const FaceExtraMap& captured)
{
    // See the declaration: the base's tracker_assign_after() (TetOptimizerMeshSwaps.cpp).
    for (const size_t tid : tids) {
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            auto vids = get_face_vids(f);
            std::sort(vids.begin(), vids.end());
            const auto it = captured.find(vids);
            m_face_extra[f.fid(*this)] = it != captured.end() ? it->second : FaceExtra{};
        }
    }
}

void TopoOffsetTetMesh::edge_label_capture(
    const std::vector<size_t>& tids,
    EdgeLabelMap& out,
    const size_t from,
    const size_t to) const
{
    out.clear();
    for (const size_t tid : tids) {
        for (int j = 0; j < 6; ++j) {
            const Tuple e = tuple_from_edge(tid, j);
            std::array<size_t, 2> key{{e.vid(*this), switch_vertex(e).vid(*this)}};
            for (size_t& u : key) {
                if (u == from) u = to;
            }
            if (key[0] == key[1]) continue; // the collapsed edge itself
            std::sort(key.begin(), key.end());
            const int l = m_edge_attribute[e.eid(*this)].label;
            const auto [it, fresh] = out.try_emplace(key, l);
            if (!fresh) it->second = closed_label(it->second, l);
        }
    }
}

void TopoOffsetTetMesh::edge_label_assign(
    const std::vector<size_t>& tids,
    const EdgeLabelMap& captured)
{
    for (const size_t tid : tids) {
        for (int j = 0; j < 6; ++j) {
            const Tuple e = tuple_from_edge(tid, j);
            std::array<size_t, 2> key{{e.vid(*this), switch_vertex(e).vid(*this)}};
            std::sort(key.begin(), key.end());
            const auto it = captured.find(key);
            m_edge_attribute[e.eid(*this)].label = it != captured.end() ? it->second : 0;
        }
    }
}

void TopoOffsetTetMesh::close_labels(const std::vector<size_t>& tids)
{
    for (const size_t tid : tids) {
        const int lt = m_tet_attribute[tid].label;
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            int& lf = m_face_extra[f.fid(*this)].label;
            lf = closed_label(lf, lt);
            // The face's three edges, as label_input_complex() walks them.
            for (const Tuple& e :
                 {f, f.switch_edge(*this), f.switch_vertex(*this).switch_edge(*this)}) {
                int& le = m_edge_attribute[e.eid(*this)].label;
                le = closed_label(le, lf);
                for (const size_t v : {e.vid(*this), e.switch_vertex(*this).vid(*this)}) {
                    int& lv = m_vertex_extra[v].label;
                    lv = closed_label(lv, le);
                }
            }
        }
    }
    for (const size_t tid : tids) {
        for (const size_t v : oriented_tet_vids(tid)) {
            m_vertex_extra[v].m_is_on_input = m_vertex_extra[v].label == 1;
        }
    }
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
    // THE LIVE predicate, not is_edge_on_offset(): that one short-circuits on m_is_on_offset of
    // both endpoints, and this value is what writes that flag on the child. Deriving the flag
    // from itself could never correct a wrong one. edge_is_offset_surface_live() reads cell
    // labels only.
    cache.is_edge_on_offset =
        edge_is_offset_surface_live(edge.vid(*this), edge.switch_vertex(*this).vid(*this));
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
    // The parents' faces and edges, for m_face_extra and the edge label in split_after_cells().
    {
        std::vector<size_t> tids;
        for (const Tuple& tt : parents) tids.push_back(tt.tid(*this));
        face_extra_capture(tids, m_face_extra_split.local());
        edge_label_capture(tids, m_edge_label_split.local());
    }
    // EXPERIMENTAL_split_energy_gate: the parents' sum of tet_energy(), which split_edge_after()
    // compares the children's against.
    if (m_offset_params.split_energy_gate) {
        std::vector<size_t> tids;
        for (const Tuple& tt : parents) tids.push_back(tt.tid(*this));
        m_split_energy_before.local() = tet_energy_sum(tids, !swept_rule());
    }
    return true;
}

bool TopoOffsetTetMesh::split_after_cells(
    const size_t v1_id,
    const size_t v2_id,
    const size_t v_id,
    const std::vector<Tuple>&)
{
    // Every field of the new vertex's VertexExtra is written here, as the base writes every field
    // of its VertexAttributes (TetOptimizerMesh::split_edge_after()): v_id may be a slot past a
    // consolidation's end, which still holds a dead vertex's values.
    VertexExtra ve;
    // The label (and m_is_on_input with it) is left 0 here: close_labels() at the end gives the
    // midpoint the split edge's, which the two halves carry.
    // The new vertex is on the offset surface exactly when the edge it was placed on was: a
    // split subdivides the surface's faces and moves none of them, so the child inherits the
    // EDGE's membership. split_before_cells() measured that live, before the split. (It used to
    // be the AND of the endpoints' flags, which over-approximates: two surface vertices joined by
    // an edge through the band interior are not a surface edge.)
    ve.m_is_on_offset = m_opt_split_cache.local().is_edge_on_offset;
    // The other tag regions, from the edge (split_adjust_position() wrote the same before).
    ve.m_is_on_region = m_opt_split_cache.local().is_edge_on_region;
    // Read by the needle diagnostics.
    ve.m_born_epoch = m_op_epoch;
    // m_corr_input_vid -1 (the march did not make it), no front solve yet: the defaults.
    m_vertex_extra[v_id] = ve;

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
    // m_face_extra, as the base's split writes m_face_attribute: a child face without the
    // midpoint is a parent face and keeps its value; a half (v_end, v, c) of a parent face
    // (v1, v2, c) takes that face's; a face (a, b, v) with a, b in the edge's link is new, reset
    // (and then closed below).
    const std::vector<size_t> children = get_one_ring_tids_for_vertex(v_id);
    const FaceExtraMap& faces = m_face_extra_split.local();
    for (const size_t tid : children) {
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            auto vids = get_face_vids(f);
            std::array<size_t, 3> key = vids;
            const auto at_v = std::find(vids.begin(), vids.end(), v_id);
            if (at_v != vids.end()) {
                bool half = false;
                for (const size_t u : vids) {
                    if (u == v_id) continue;
                    if (u == v1_id || u == v2_id) half = true;
                }
                if (!half) {
                    m_face_extra[f.fid(*this)] = FaceExtra{};
                    continue;
                }
                // The parent: the midpoint replaced by whichever endpoint the face lacks.
                const bool has_v1 = std::find(vids.begin(), vids.end(), v1_id) != vids.end();
                key[size_t(at_v - vids.begin())] = has_v1 ? v2_id : v1_id;
            }
            std::sort(key.begin(), key.end());
            const auto it = faces.find(key);
            if (it != faces.end()) m_face_extra[f.fid(*this)] = it->second;
        }
    }
    // The edge label the same way: an edge without the midpoint keeps its value, the halves
    // (v1, v) and (v2, v) take the split edge's, an edge (v, l) to the link is new, 0.
    const EdgeLabelMap& edges = m_edge_label_split.local();
    const std::array<size_t, 2> split_key{{std::min(v1_id, v2_id), std::max(v1_id, v2_id)}};
    for (const size_t tid : children) {
        for (int j = 0; j < 6; ++j) {
            const Tuple e = tuple_from_edge(tid, j);
            const size_t a = e.vid(*this), b = switch_vertex(e).vid(*this);
            std::array<size_t, 2> key{{std::min(a, b), std::max(a, b)}};
            if (a == v_id || b == v_id) {
                const size_t u = a == v_id ? b : a;
                if (u != v1_id && u != v2_id) {
                    m_edge_attribute[e.eid(*this)].label = 0;
                    continue;
                }
                key = split_key;
            }
            const auto it = edges.find(key);
            m_edge_attribute[e.eid(*this)].label = it != edges.end() ? it->second : 0;
        }
    }
    // The children's labels onto their faces, edges and corners; the midpoint's comes from here.
    close_labels(children);
    return true;
}

} // namespace wmtk::components::topological_offset
