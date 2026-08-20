#include <wmtk/TetOptimizerMesh.h>
#include <limits>

#include <igl/Timer.h>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>

namespace wmtk {

void TetOptimizerMesh::split_all_edges()
{
    igl::Timer timer;
    double time;
    m_force_split_count = 0;

    // Reset the high-valence claims for this pass. Sized at vert_capacity() now: storage
    // is preallocated per phase, so any vertex a split creates during this pass already
    // has an id below it (same invariant run_localized_to_convergence relies on for
    // vertex_epoch). make_unique value-initialises, so every slot starts at 0.
    if (m_params.split_high_valence_threshold > 0) {
        // Size to the preallocated capacity, not vert_capacity(). vert_capacity() returns
        // current_vert_size -- the live vertex count -- so every vertex a split creates during
        // this pass gets an id at or above it, trips the `vid >= m_high_valence_claim_size`
        // guard below, and is exempted from the gate entirely. Those exempt vertices are
        // exactly the ones that run away: on Thingi10K 509315 v15601 absorbed 1792
        // valence-increasing splits in a single pass, where the gate allows one, reaching
        // valence ~700 while the max energy climbed with it. The attribute collections are
        // resized to the reservation, so their size is the capacity to use.
        m_high_valence_claim_size = std::max(vert_capacity(), m_vertex_attribute.size());
        m_high_valence_claim = std::make_unique<std::atomic<int>[]>(m_high_valence_claim_size);
    }
    m_high_valence_rejects = 0;
    reset_slot_exhausted();

    timer.start();
    auto collect_all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TetOptimizerMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_split", e); });
    time = timer.getElapsedTime();
    wmtk::logger().info("edge split prepare time: {:.4}s", time);
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeRing,
        "edge split operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_simple;

            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
                auto [weight, op, tup] = ele;
                auto length = m.get_length2(tup);
                if (length != weight) return false;
                //
                size_t v1_id = tup.vid(*this);
                size_t v2_id = tup.switch_vertex(*this).vid(*this);
                // Force-split: a worst tet's longest edge (queued by
                // refine_sizing_around_worst when the max energy stalls) is split once
                // regardless of the length gate, to unstick a sliver without changing the
                // sizing field. The new midpoint is not in m_force_split_edges, so the
                // two halves are NOT force-split again -- exactly one split per edge.
                if (is_force_split_edge(v1_id, v2_id)) {
                    return true;
                }
                double sizing_ratio = (m_vertex_attribute[v1_id].m_sizing_scalar +
                                       m_vertex_attribute[v2_id].m_sizing_scalar) /
                                      2;
                if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                    return false;
                }
                return true;
            };
            wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (m_force_split_count > 0) {
        wmtk::logger().info(
            "[force-split] {} worst-tet longest edges force-split",
            m_force_split_count);
    }
    if (const size_t n = m_high_valence_rejects.load(); n > 0) {
        wmtk::logger().info(
            "[high-valence] {} splits refused to avoid growing a vertex past {} incident tets",
            n,
            m_params.split_high_valence_threshold);
    }
    // A split that ran out of preallocated slots did not happen and left no other trace: the
    // abort is inside the connectivity update, before split_edge_after(), so no application hook
    // sees it and no rejection is counted anywhere. Say so, or a pass that delivered a fraction
    // of the refinement it was asked for is indistinguishable from one that had nothing to do.
    //
    // mesh_improvement() consolidates every iteration and the queue is re-collected, so there
    // the work is merely deferred. A caller that runs split_all_edges() once -- as the offset
    // optimization does -- loses it. Measured on specific_models/prism: 31445 splits per
    // iteration, against 2638 that succeeded.
    if (const size_t n = slot_exhausted(); n > 0) {
        wmtk::logger().warn(
            "[slots] {} operations aborted with the preallocated slot pool exhausted (capacity is "
            "{}x the live count at the last consolidate). They are NOT refusals: the work was "
            "dropped. Consolidate and repeat the pass, or raise the preallocation factor.",
            n,
            preallocation_factor());
    }
    // Consumed: the queued force-split edges no longer exist after this pass.
    m_force_split_edges.clear();
}

bool TetOptimizerMesh::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_faces.clear();

    cache.v1_id = loc0.vid(*this);
    auto loc1 = loc0.switch_vertex(*this);
    cache.v2_id = loc1.vid(*this);
    //
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    cache.is_edge_on_surface = is_edge_on_surface(loc0);

    if (cache.is_edge_on_surface) {
        cache.edge_order = get_order_of_edge({{cache.v1_id, cache.v2_id}});
    } else {
        cache.edge_order = 0;
    }

    // todo: can be optimized
    cache.is_edge_open_boundary = cache.is_edge_on_surface && is_open_boundary_edge(loc0);

    /// save face track info
    auto comp = [](const std::pair<FaceAttributes, std::array<size_t, 3>>& v1,
                   const std::pair<FaceAttributes, std::array<size_t, 3>>& v2) {
        return v1.second < v2.second;
    };
    auto is_equal = [](const std::pair<FaceAttributes, std::array<size_t, 3>>& v1,
                       const std::pair<FaceAttributes, std::array<size_t, 3>>& v2) {
        return v1.second == v2.second;
    };

    auto tets = get_incident_tets_for_edge(loc0);

    // High-valence gate. Splitting (v1,v2) leaves the endpoints' own incident-tet counts
    // unchanged -- each keeps one child of every tet it was in -- and adds one to every
    // vertex in the edge's link, since those sit in both children. So the gate applies to
    // the link.
    //
    // A vertex past the threshold accepts one such split per pass and refuses the rest,
    // which spreads the refinement instead of letting it pile onto the same vertex. Done
    // here, before the face caching below, so a refusal is cheap.
    if (m_params.split_high_valence_threshold > 0 && m_high_valence_claim) {
        const size_t threshold = static_cast<size_t>(m_params.split_high_valence_threshold);
        std::vector<size_t> to_claim;
        for (const Tuple& t : tets) {
            const auto vs = oriented_tet_vertices(t);
            for (int j = 0; j < 4; j++) {
                const size_t vid = vs[j].vid(*this);
                if (vid == v1_id || vid == v2_id) continue;
                if (vid >= m_high_valence_claim_size) continue;
                if (vertex_valence(vid) <= threshold) continue;
                if (m_high_valence_claim[vid].load(std::memory_order_relaxed) != 0) {
                    // Already spent this pass. Refuse rather than grow it further.
                    m_high_valence_rejects.fetch_add(1, std::memory_order_relaxed);
                    return false;
                }
                to_claim.push_back(vid);
            }
        }
        // Claim only once the whole link is known to be free, so a refusal late in the
        // loop does not burn the budget of a vertex found earlier.
        for (const size_t vid : to_claim) {
            m_high_valence_claim[vid].store(1, std::memory_order_relaxed);
        }
    }

    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            std::array<size_t, 3> f_vids = {{
                vs[(j + 1) % 4].vid(*this),
                vs[(j + 2) % 4].vid(*this),
                vs[(j + 3) % 4].vid(*this),
            }}; // todo: speedup
            std::sort(f_vids.begin(), f_vids.end());
            auto [_, global_fid] = tuple_from_face(f_vids);
            split_cache.local().changed_faces.push_back(
                std::make_pair(m_face_attribute[global_fid], f_vids));
        }
    }
    wmtk::vector_unique(cache.changed_faces, comp, is_equal);

    return split_before_cells(loc0, tets);
}

bool TetOptimizerMesh::split_edge_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TetMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    auto& cache = split_cache.local();

    std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);
    size_t v_id = loc.vid(*this);

    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    m_vertex_attribute[v_id].m_posf =
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    m_vertex_attribute[v_id].m_is_rounded = true;

    // this has to be done before the inversion check
    m_vertex_attribute[v_id].m_pos = to_rational(m_vertex_attribute[v_id].m_posf);

    for (const Tuple& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[v_id].m_is_rounded = false;
            break;
        }
    }
    if (is_force_split_edge(v1_id, v2_id)) {
        std::atomic_ref<size_t>(m_force_split_count).fetch_add(1, std::memory_order_relaxed);
    }
    if (!m_vertex_attribute[v_id].m_is_rounded) {
        // The rounded (double) midpoint inverts an incident tet, so place the new vertex at
        // the EXACT rational midpoint of the two endpoints instead. That midpoint lies on the
        // shared edge, so it can never invert a previously-valid incident tet: the split
        // always succeeds and a stuck region can keep being refined. The vertex stays
        // un-rounded (m_pos exact, m_is_rounded = false) until a later round() reclaims it.
        //
        // This used to apply only when an endpoint was already rational, to stop a split
        // between two rounded endpoints from reintroducing exact coordinates into a
        // fully-rounded mesh. That restriction is what stalled the optimizer: it rejected the
        // split outright exactly where the mesh is degenerate, which is where refinement is
        // needed, and the stuck-refine machinery then hammered the region from outside,
        // driving the max energy up by orders of magnitude per pass. On Thingi10K 509315 the
        // restriction cost 5 of 8 runs, which diverged to 1e16..1e20 and hit the sweep's
        // one-hour timeout; without it 8 of 8 converge, in 2-5 iterations, and healthy models
        // are unchanged in both time and quality.
        //
        // Exact coordinates reaching the output is prevented by the iteration, not here: a
        // split is the only operation that can un-round a vertex (collapse, all four swaps
        // and smoothing never do), the post-optimization pass is collapse-only, and the loop
        // does not stop until every vertex is rounded as well as the energy target being met.
        m_vertex_attribute[v_id].m_pos =
            (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
        // Keep m_posf in step with the exact position. It was left holding the double
        // midpoint, which is a different point -- and specifically the one just found to
        // invert an incident tet. Every un-guarded m_posf read (edge length, the envelope
        // tests, the smoothing seed) then works from a position the vertex does not have.
        // When an endpoint is itself un-rounded the gap is not a rounding step but the whole
        // distance between that endpoint's exact and approximate positions.
        m_vertex_attribute[v_id].m_posf = to_double(m_vertex_attribute[v_id].m_pos);
        // Guard against a pre-existing inverted incident tet: re-check in exact
        // arithmetic (un-rounded v_id => is_inverted uses the rational path).
        for (const Tuple& loc : locs) {
            if (is_inverted(loc)) {
                return false;
            }
        }
        // This split keeps an un-rounded vertex, so the sweep must not skip the next
        // pass. Set after the rollback checks above, which leave the mesh unchanged.
        m_all_rounded.store(false, std::memory_order_relaxed);
    }

    if (!split_adjust_position(v_id, locs)) return false;
    if (!split_after_cells(v1_id, v2_id, v_id, locs)) return false;

    /// update quality
    //
    // A split is never refused on quality. It checks orientation, rounding and (above) the
    // envelope, and that is all: subdividing is the operation the optimizer reaches for when a
    // region is stuck, so it has to be allowed to run even where the result scores badly.
    for (const Tuple& loc : locs) {
        set_cell_quality(loc.tid(*this), get_quality(loc));
    }

    /// containment: the new surface triangles must stay inside the envelope
    //
    // Every other operation that moves or creates surface geometry tests this --- collapse
    // in collapse_edge_before/after, all four swaps in their *_after --- and split did not.
    // The comment above claiming it "checks orientation and the envelope" was wrong: this
    // file contained no envelope query at all.
    //
    // It matters because split is the operation that CREATES surface vertices, and it puts
    // the new one at the chord midpoint. On a curved region the midpoint of a chord lies off
    // the surface by about L^2/8R, so a long enough edge over a tight enough curve places it
    // outside the envelope the moment it is created, and nothing afterwards is obliged to
    // bring it back. Smoothing is the only operation that could, and with
    // quality_veto_on_surface it refuses whenever the pull-back worsens an incident element.
    //
    // Measured on Thingi10K 243014 (exact envelope, serial): 19 of 24702 surface vertices
    // outside the envelope by iteration 12 --- a silent violation of the containment
    // invariant, invisible in the sweep because DEBUG_hausdorff is off by default.
    //
    // The face split is (v1,v2,other) -> (v1,v_id,other) + (v2,v_id,other), which is the same
    // pair of new triangles the attribute update below writes.
    if (cache.is_edge_on_surface) {
        for (const auto& info : cache.changed_faces) {
            if (!info.first.m_is_surface_fs) continue;
            const auto& old_vids = info.second;
            size_t other = std::numeric_limits<size_t>::max();
            int n_shared = 0;
            for (int j = 0; j < 3; j++) {
                if (old_vids[j] == v1_id || old_vids[j] == v2_id) {
                    ++n_shared;
                } else {
                    other = old_vids[j];
                }
            }
            if (n_shared != 2) continue; // face does not contain the split edge
            if (surface_triangle_is_outside(v1_id, v_id, other)) {
                return false;
            }
            if (surface_triangle_is_outside(v2_id, v_id, other)) {
                return false;
            }
        }
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.is_edge_on_surface;
    m_vertex_attribute[v_id].m_order = cache.edge_order;

    split_after_vertex(v_id, cache.is_edge_open_boundary);

    /// update face attribute
    // add new and erase old
    for (auto& info : cache.changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        std::vector<int> j_vn;
        for (int j = 0; j < 3; j++) {
            if (old_vids[j] != v1_id && old_vids[j] != v2_id) {
                j_vn.push_back(j);
            }
        }
        if (j_vn.size() == 1) {
            auto [_1, global_fid1] = tuple_from_face({{v1_id, v_id, old_vids[j_vn[0]]}});
            m_face_attribute[global_fid1] = f_attr;
            auto [_2, global_fid2] = tuple_from_face({{v2_id, v_id, old_vids[j_vn[0]]}});
            m_face_attribute[global_fid2] = f_attr;
        } else { // j_vn.size() == 2
            auto [_, global_fid] = tuple_from_face(old_vids);
            m_face_attribute[global_fid] = f_attr;
            //
            auto [_2, global_fid2] = tuple_from_face(
                {{old_vids[j_vn[0]], old_vids[j_vn[1]], v_id}}); // todo: avoid dup comp
            m_face_attribute[global_fid2].reset();
        }
    }

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    // if (m_vertex_attribute[v_id].m_is_on_surface) {
    //     if (!check_vertex_param_type()) {
    //         std::cout << v1_id << " " << v2_id << std::endl;
    //         for (auto vp : m_vertex_attribute[v1_id].face_nearly_param_type_with_ineffective)
    //             std::cout << vp << " ";
    //         std::cout << std::endl;
    //         for (auto vp : m_vertex_attribute[v2_id].face_nearly_param_type_with_ineffective)
    //             std::cout << vp << " ";
    //         std::cout << std::endl;
    //         output_faces("bug_surface_miss_param_after_split.obj", [](auto& f) {
    //             return f.m_is_surface_fs;
    //         });
    //         // exit(0);
    //     }
    // }

    return true;
}

} // namespace wmtk
