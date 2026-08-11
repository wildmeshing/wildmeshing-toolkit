#include "TriWildMesh.h"

#include <igl/Timer.h>
#include <atomic>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

namespace wmtk::components::triwild {

void TriWildMesh::split_all_edges()
{
    igl::Timer timer;
    double time;
    m_force_split_count = 0;

    // Reset the high-valence claims for this pass. Sized to the preallocated capacity, not
    // vert_capacity(): vert_capacity() returns the live vertex count, so every vertex a split
    // creates during this pass would get an id at or above it, trip the guard in
    // split_edge_before, and be exempted from the gate entirely -- and those are exactly the
    // ones that run away. The attribute collections are resized to the reservation, so their
    // size is the capacity to use. make_unique value-initialises, so every slot starts at 0.
    if (m_tri_params.split_high_valence_threshold > 0) {
        m_high_valence_claim_size = std::max(vert_capacity(), m_vertex_attribute.size());
        m_high_valence_claim = std::make_unique<std::atomic<int>[]>(m_high_valence_claim_size);
    }
    m_high_valence_rejects = 0;

    timer.start();
    auto collect_all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TriWildMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_split", e); });
    time = timer.getElapsedTime();
    logger().info("edge split prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples =
            [](const TriWildMesh& m, std::string op, const auto& newts) {
                std::vector<std::pair<std::string, TriMesh::Tuple>> op_tups;
                for (const auto& t : newts) {
                    op_tups.emplace_back(op, t);
                    op_tups.emplace_back(op, t.switch_edge(m));
                    op_tups.emplace_back(op, t.switch_vertex(m).switch_edge(m));
                }
                return op_tups;
            };

        executor.priority = [&](const TriWildMesh& m, std::string op, const Tuple& t) {
            return m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const TriWildMesh& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) {
                return false;
            }
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            // Force-split: a worst triangle's longest edge (queued by
            // refine_sizing_around_worst when the max energy stalls) is split once
            // regardless of the length gate, to unstick a sliver without changing the
            // sizing field. The new midpoint is not in m_force_split_edges, so the two
            // halves are NOT force-split again -- exactly one split per edge.
            if (is_force_split_edge(v1_id, v2_id)) {
                return true;
            }
            const auto& VA = m_vertex_attribute;
            double sizing_ratio = 0.5 * (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar);
            if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                return false;
            }
            return true;
        };
        // Retry a failed split only where the mesh actually changed this round
        // (dirty-epoch localized retry), instead of re-testing every failure every pass.
        wmtk::run_localized_to_convergence(*this, executor, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = ExecutePass<TriWildMesh>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = ExecutePass<TriWildMesh>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time serial: {:.4}s", time);
    }
    if (m_force_split_count > 0) {
        wmtk::logger().info(
            "[force-split] {} worst-triangle longest edges force-split",
            m_force_split_count);
    }
    if (const size_t n = m_high_valence_rejects.load(); n > 0) {
        wmtk::logger().info(
            "[high-valence] {} splits refused to avoid growing a vertex past {} incident "
            "triangles",
            n,
            m_tri_params.split_high_valence_threshold);
    }
    // Consumed: the queued force-split edges no longer exist after this pass.
    m_force_split_edges.clear();
}

bool TriWildMesh::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_edges.clear();
    cache.faces.clear();

    cache.v1_id = loc0.vid(*this);
    cache.v2_id = loc0.switch_vertex(*this).vid(*this);

    cache.max_quality_before = 0.;
    for (const size_t fid : get_incident_fids_for_edge(loc0)) {
        cache.max_quality_before =
            std::max(cache.max_quality_before, m_face_attribute[fid].m_quality);
    }

    cache.old_e_attrs = m_edge_attribute[loc0.eid(*this)];

    const simplex::Edge edge(cache.v1_id, cache.v2_id);

    const auto faces = get_incident_fids_for_edge(loc0);

    // High-valence gate. Splitting (v1,v2) leaves the endpoints' own incident-triangle counts
    // unchanged -- each keeps one child of every triangle it was in -- and adds one to every
    // vertex in the edge's link, since those sit in both children. In 2D the link is the
    // vertex opposite the edge in each incident face: one on a boundary edge, two inside.
    //
    // A vertex past the threshold accepts one such split per pass and refuses the rest, which
    // spreads the refinement instead of letting it pile onto the same vertex. Done here,
    // before the edge caching below, so a refusal is cheap.
    if (m_tri_params.split_high_valence_threshold > 0 && m_high_valence_claim) {
        const size_t threshold = static_cast<size_t>(m_tri_params.split_high_valence_threshold);
        std::vector<size_t> to_claim;
        for (const size_t fid : faces) {
            const size_t vid = simplex_from_face(fid).opposite_vertex(edge).id();
            if (vid >= m_high_valence_claim_size) continue;
            if (vertex_valence(vid) <= threshold) continue;
            if (m_high_valence_claim[vid].load(std::memory_order_relaxed) != 0) {
                // Already spent this pass. Refuse rather than grow it further.
                m_high_valence_rejects.fetch_add(1, std::memory_order_relaxed);
                return false;
            }
            to_claim.push_back(vid);
        }
        // Claim only once the whole link is known to be free, so a refusal late in the loop
        // does not burn the budget of a vertex found earlier.
        for (const size_t vid : to_claim) {
            m_high_valence_claim[vid].store(1, std::memory_order_relaxed);
        }
    }

    for (const size_t fid : faces) {
        auto vs = oriented_tri_vids(fid);
        for (int j = 0; j < 3; j++) {
            const simplex::Edge e(vs[j], vs[(j + 1) % 3]);
            if (e == edge) {
                continue;
            }
            if (cache.changed_edges.count(e) != 0) {
                continue;
            }
            auto [_, eid] = tuple_from_edge(e.vertices());
            cache.changed_edges[e] = m_edge_attribute[eid];
        }
    }

    // store tet attributes
    for (const size_t fid : faces) {
        const simplex::Face face = simplex_from_face(fid);
        const size_t opp = face.opposite_vertex(edge).id();
        cache.faces[opp] = m_face_attribute.at(fid);
    }

    return true;
}

bool TriWildMesh::split_edge_after(const Tuple& loc)
{
    if (!TriMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    const std::vector<Tuple> locs = get_one_ring_tris_for_vertex(loc.switch_vertex(*this));
    const size_t v_id = loc.switch_vertex(*this).vid(*this);

    auto& cache = split_cache.local();

    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    auto& p = m_vertex_attribute[v_id].m_posf;
    p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    m_vertex_attribute[v_id].m_is_rounded = true;

    // this has to be done before the inversion check
    m_vertex_attribute[v_id].m_pos = to_rational(p);

    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[v_id].m_is_rounded = false;
            break;
        }
    }
    if (is_force_split_edge(v1_id, v2_id)) {
        std::atomic_ref<size_t>(m_force_split_count).fetch_add(1, std::memory_order_relaxed);
    }
    if (!m_vertex_attribute[v_id].m_is_rounded) {
        // The rounded (double) midpoint inverts an incident triangle, so place the new vertex
        // at the EXACT rational midpoint of the two endpoints instead. That midpoint lies on
        // the shared edge, so it can never invert a previously-valid incident triangle: the
        // split always succeeds and a stuck region can keep being refined. The vertex stays
        // un-rounded (m_pos exact, m_is_rounded = false) until a later round() reclaims it.
        //
        // This used to apply only when an endpoint was already rational, to stop a split
        // between two rounded endpoints from reintroducing exact coordinates into a
        // fully-rounded mesh. That restriction is what stalled the optimizer: it rejected the
        // split outright exactly where the mesh is degenerate, which is where refinement is
        // needed, and the stuck-refine machinery then hammered the region from outside,
        // driving the max energy up by orders of magnitude per pass. On Thingi10K 509315 the
        // restriction cost 5 of 8 runs, which diverged to 1e16..1e20 and hit the sweep's
        // one-hour timeout; without it 8 of 8 converge, in 2-5 iterations.
        //
        // Exact coordinates reaching the output is prevented by the iteration, not here: a
        // split is the only operation that can un-round a vertex (collapse, swap and
        // smoothing never do), the post-optimization pass is collapse-only, and
        // mesh_improvement does not stop until every vertex is rounded as well as the energy
        // target being met.
        m_vertex_attribute[v_id].m_pos =
            (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
        // Unlike tetwild, keep m_posf in step with the exact position: when an endpoint is
        // itself un-rounded, the rounded midpoint of the two *approximations* is a worse
        // approximation of the exact midpoint than rounding the exact midpoint once.
        p = to_double(m_vertex_attribute[v_id].m_pos);
        // Guard against a pre-existing inverted incident triangle: re-check in exact
        // arithmetic (un-rounded v_id => is_inverted uses the rational path). This check
        // was missing, so a split could leave an inverted triangle behind.
        for (const Tuple& loc : locs) {
            if (is_inverted(loc)) {
                return false;
            }
        }
        // This split keeps an un-rounded vertex, so the sweep must not skip the next pass.
        // Set after the rollback checks above, which leave the mesh unchanged.
        m_all_rounded.store(false, std::memory_order_relaxed);
    }

    // update face attributes
    {
        // v1 - v_new
        const auto faces1 = get_incident_fids_for_edge(v1_id, v_id);
        const simplex::Edge edge1(v1_id, v_id);
        for (const size_t fid : faces1) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge1).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        // v2 - v_new
        const auto faces2 = get_incident_fids_for_edge(v2_id, v_id);
        const simplex::Edge edge2(v2_id, v_id);
        for (const size_t fid : faces2) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge2).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        assert(faces1.size() + faces2.size() == locs.size());

        const auto [_1, eid1] = tuple_from_edge(edge1.vertices());
        const auto [_2, eid2] = tuple_from_edge(edge2.vertices());

        m_edge_attribute[eid1] = cache.old_e_attrs;
        m_edge_attribute[eid2] = cache.old_e_attrs;
        for (const auto& [vid, _] : cache.faces) {
            const auto [_tup, eid] = tuple_from_edge({{v_id, vid}});
            m_edge_attribute[eid].reset();
        }
    }

    /// update quality
    //
    // A split is otherwise unconditional: it checks orientation and the envelope but never
    // quality. That is right for a length-driven split of a long, well-behaved edge and
    // wrong for the force-split of a stalled sliver's longest edge, where the midpoint can
    // land essentially on the opposite edge and leave a correctly-oriented element whose
    // area/volume is too small for AMIPS -- get_quality then returns MAX_ENERGY, and every
    // control decision that divides by the max energy is meaningless from then on. Refuse
    // to be the operation that creates one; subdividing an already-degenerate region is
    // still allowed, so a stuck region can keep being refined. See tetwild's EdgeSplitting
    // for the measurement this comes from.
    double max_quality_after = 0.;
    for (const Tuple& loc : locs) {
        max_quality_after = std::max(max_quality_after, get_quality(loc));
    }
    if (max_quality_after >= MAX_ENERGY && cache.max_quality_before < MAX_ENERGY) {
        return false;
    }
    for (const Tuple& loc : locs) {
        m_face_attribute[loc.fid(*this)].m_quality = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.old_e_attrs.m_is_surface_fs;

    /// update edge attribute
    for (const auto& [e, e_attr] : cache.changed_edges) {
        auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attr;
    }

    // A split midpoint stands for no input feature: the endpoints keep theirs, and the new
    // vertex is interior to the curve by construction. Set explicitly because attribute slots
    // are recycled and could carry a stale id.
    m_vertex_extra[v_id].m_feature_id = NO_FEATURE;

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    return true;
}

} // namespace wmtk::components::triwild