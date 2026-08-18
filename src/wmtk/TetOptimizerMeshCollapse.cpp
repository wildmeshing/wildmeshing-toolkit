#include <wmtk/threading/enumerable_thread_specific.hpp>
#include "TetOptimizerMesh.h"
#include "wmtk/TetMesh.h"

#include <igl/Timer.h>
#include <algorithm>
#include <atomic>
#include <mutex>
#include <unordered_set>
#include <wmtk/threading/collector.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>

namespace wmtk {

void TetOptimizerMesh::collapse_all_edges(bool is_limit_length)
{
    collapse_all_edges_impl(is_limit_length, wmtk::default_ring(wmtk::PassLock::EdgeRing));
}

size_t
TetOptimizerMesh::collapse_all_edges_impl(bool is_limit_length, int lock_ring, size_t max_passes)
{
    m_collapse_limit_length = is_limit_length;
    igl::Timer timer;
    double time;
    timer.start();

    // Build the collapse op list in parallel (both edge directions). Filtering of
    // too-long edges still happens in is_weight_up_to_date.
    auto collect_all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TetOptimizerMesh& m, const Tuple& e, auto& out) {
            out.emplace_back("edge_collapse", e);
            out.emplace_back("edge_collapse", e.switch_vertex(m));
        });
    logger().info("#edges = {}", collect_all_ops.size() / 2);
    time = timer.getElapsedTime();
    logger().info("edge collapse prepare time: {:.4}s", time);
    size_t accepted = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeRing,
        lock_ring,
        "edge collapse operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = [](const auto& m, auto op, const auto& newts) {
                std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
                for (const Tuple& t : newts) {
                    op_tups.emplace_back(op, t);
                    op_tups.emplace_back(op, t.switch_vertex(m));
                }
                return op_tups;
            };
            executor.priority = [&](auto& m, auto op, auto& t) { return -m.get_length2(t); };
            executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
                auto& [weight, op, tup] = ele;
                auto length = m.get_length2(tup);
                if (length != -weight) return false;
                // Deliberately NOT filtered on length here. An over-length edge stays a
                // candidate and collapse_edge_before decides it on quality instead: it is kept
                // only if it STRICTLY improves the worst element of the ring. See there.
                return true;
            };
            // Retry a failed collapse only where the mesh actually changed this round
            // (dirty-epoch localized retry), instead of re-testing every failure every pass.
            accepted =
                wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops, max_passes);
        });
    return accepted;
}

bool TetOptimizerMesh::collapse_edge_before(const Tuple& loc) // input is an edge
{
    const auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();

    cache.changed_faces.clear();
    cache.changed_tids.clear();
    cache.changed_energies.clear();
    cache.surface_faces.clear();
    cache.boundary_edges.clear();

    size_t v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    size_t v2_id = loc1.vid(*this);

    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    cache.edge_length =
        (VA[v1_id].m_posf - VA[v2_id].m_posf).norm(); // todo: duplicated computation

    // The coarsening pass stops at the target edge length unless told otherwise. Collapsing
    // an edge that is already at target only makes its neighbours longer, so this is where
    // "as few elements as hold the max energy" has to be traded against "elements the size
    // the user asked for". Deliberately NOT scaled by the sizing field: see
    // OptimizerParameters::coarsen_unbounded. Cheap, so it goes before any ring work.
    if (m_coarsen_mode && !m_params.coarsen_unbounded) {
        if (cache.edge_length * cache.edge_length > m_params.collapsing_l2) {
            return false;
        }
    }

    ///check if on bbox/surface/boundary
    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) return false;
        for (int on_bbox : VA[v1_id].on_bbox_faces)
            if (std::find(
                    VA[v2_id].on_bbox_faces.begin(),
                    VA[v2_id].on_bbox_faces.end(),
                    on_bbox) == VA[v2_id].on_bbox_faces.end()) {
                return false;
            }
    }

    // surface
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_surface) {
        // Moving a surface vertex onto a non-surface one is only safe if the destination is
        // inside the envelope. With no envelope to ask, that cannot be certified, so refuse --
        // which is also what keeps this from dereferencing a null envelope. Both applications
        // that reach here always have one, so the guard never fires for them.
        if (!VA[v2_id].m_is_on_surface &&
            (!m_envelope || m_envelope->is_outside(VA[v2_id].m_posf))) {
            return false;
        }
    }

    if (!collapse_before_vertex(v1_id, v2_id, cache.edge_length)) {
        return false;
    }


    const auto n1_locs = get_one_ring_tids_for_vertex(loc);

    cache.changed_tids.reserve(n1_locs.size());
    cache.max_energy = 0;
    for (const size_t& tid : n1_locs) {
        const double q = cell_quality(tid);
        cache.max_energy = std::max(cache.max_energy, q);
        const auto vs = oriented_tet_vids(tid);
        if (vs[0] != v2_id && vs[1] != v2_id && vs[2] != v2_id && vs[3] != v2_id) {
            cache.changed_tids.emplace_back(tid);
        }
    }

    // pre-compute after-collapse energies
    cache.changed_energies.reserve(cache.changed_tids.size());
    for (const size_t tid : cache.changed_tids) {
        std::array<size_t, 4> vs = oriented_tet_vids(tid);
        for (size_t i = 0; i < 4; ++i) {
            if (vs[i] == v1_id) {
                vs[i] = v2_id;
                break;
            }
        }

        if (is_inverted(vs)) {
            return false;
        }
        double q = get_quality(vs);
        // The coarsening pass deliberately skips the quality gate and decides on the region
        // AFTER re-smoothing instead -- that is the whole point of it. The inversion check
        // above still applies: an inverted cell is not something smoothing can repair, since
        // smooth_vertex_3d refuses to start from one.
        if (!m_coarsen_mode && !collapse_quality_allowed(v1_id, q, cache.max_energy)) {
            return false;
        }
        cache.changed_energies.emplace_back(q);
    }
    assert(cache.changed_energies.size() == cache.changed_tids.size());

    // Length gate, applied here rather than when the candidate list is built.
    //
    // Coarsening a well-shaped mesh is what the length limit exists to prevent, so a short
    // edge keeps the old behaviour. But applying it to the CANDIDATE LIST made any element
    // whose edges are all longer than 0.8 * l invisible to this pass: it was never offered,
    // so it produced no rejection record anywhere and looked untouched rather than refused.
    //
    // Measured in triwild on triwild20k 189017 at eps_rel 1e-4, where a collinear triangle
    // with edges 55 / 165 / 220 against a gate of 38.7 survived every pass while stuck-refine
    // split it -- halving its short edge and DOUBLING its energy each round, 6.7e16 -> 1.5e17
    // -> 3.1e17 -> inverted -- and the mesh grew from 17k to 6.6M elements in ten iterations.
    // Refinement cannot repair a shape defect (AMIPS is scale-invariant, so splitting a
    // collinear element leaves it collinear); collapse is the only operation that can remove
    // one, so it has to be allowed to see it. With this, that model converges in 21
    // iterations, and the eps_rel 1e-3 run it already handled is unchanged at 12.
    //
    // The condition is strict improvement of the ring's worst element, which needs no
    // threshold and is self-limiting: in a healthy mesh almost no long-edge collapse strictly
    // improves anything. Note changed_tids excludes the tets the collapse deletes, so when
    // the ring's worst is one of those the test passes by construction -- the rule admits
    // precisely the collapses that remove a bad element.
    if (m_collapse_limit_length && VA[v1_id].m_is_rounded) {
        const double sizing_ratio = (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2;
        const double len2 = cache.edge_length * cache.edge_length;
        // UNCONDITIONAL, where this used to fire only for an edge longer than the collapse
        // target. A collapse that makes the worst element in the ring worse is a bad collapse
        // whatever the edge's length is; exempting short edges meant the overwhelming majority
        // of collapses -- the ones a length-driven pass actually performs -- had no quality gate
        // at all, and could freely degrade the mesh.
        {
            double max_after = 0.;
            for (const double q : cache.changed_energies) {
                max_after = std::max(max_after, q);
            }
            if (max_after >= cache.max_energy) {
                return false;
            }
        }
    }


    //
    const auto n12_locs = get_incident_tids_for_edge(loc); // todo: duplicated computation
    for (const size_t& tid : n12_locs) {
        auto vs = oriented_tet_vids(tid);
        std::array<size_t, 3> f_vids = {{v1_id, 0, 0}};
        int cnt = 1;
        // get the two vertices that are not v1/v2, i.e., the edge-link vertices.
        for (int j = 0; j < 4; j++) {
            if (vs[j] != v1_id && vs[j] != v2_id) {
                f_vids[cnt] = vs[j];
                cnt++;
            }
        }
        auto [_1, global_fid1] = tuple_from_face(f_vids);
        auto [_2, global_fid2] = tuple_from_face({{v2_id, f_vids[1], f_vids[2]}});
        auto f_attr = m_face_attribute.at(global_fid1);
        f_attr.merge(m_face_attribute.at(global_fid2));
        cache.changed_faces.push_back(std::make_pair(f_attr, f_vids));
    }

    if (VA[v1_id].m_is_on_surface) {
        // this code must check if a face is tagged as surface face
        // only checking the vertices is not enough
        std::vector<std::array<size_t, 3>> fs;
        for (const size_t& tid : n1_locs) {
            const auto vs = oriented_tet_vids(tid);

            int j_v1 = -1;
            auto skip = [&]() {
                for (int j = 0; j < 4; j++) {
                    const size_t vid = vs[j];
                    if (vid == v2_id) {
                        // ignore tets incident to the edge (v1,v2)
                        return true; // v1-v2 definitely not on surface.
                    }
                    if (vid == v1_id) j_v1 = j;
                }
                return false;
            };
            if (skip()) continue;

            for (int k = 0; k < 3; k++) {
                auto va = vs[(j_v1 + 1 + k) % 4];
                auto vb = vs[(j_v1 + 1 + (k + 1) % 3) % 4];
                if ((VA[va].m_is_on_surface && VA[vb].m_is_on_surface)) {
                    std::array<size_t, 3> f = {{v1_id, va, vb}};
                    const auto [f_tuple, fid] = tuple_from_face(f);
                    if (!m_face_attribute.at(fid).m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    std::sort(f.begin(), f.end());
                    fs.push_back(f);
                }
            }
        }
        wmtk::vector_unique(fs);

        cache.surface_faces.reserve(fs.size());
        for (auto& f : fs) {
            std::replace(f.begin(), f.end(), v1_id, v2_id);
            cache.surface_faces.push_back(f);
        }

        std::vector<std::array<size_t, 2>> bs;
        // iterate through all faces inicdent to v1
        for (const size_t& tid : n1_locs) {
            const auto vs = oriented_tet_vids(tid);

            int j_v1 = -1;
            for (int j = 0; j < 4; j++) {
                const size_t vid = vs[j];
                if (vid == v1_id) {
                    j_v1 = j;
                }
            }

            for (int k = 0; k < 3; k++) {
                auto va = vs[(j_v1 + 1 + k) % 4];
                auto vb = vs[(j_v1 + 1 + (k + 1) % 3) % 4];
                if ((VA[va].m_is_on_surface && VA[vb].m_is_on_surface)) {
                    std::array<size_t, 3> f = {{v1_id, va, vb}};
                    const auto [f_tuple, fid] = tuple_from_face(f);
                    if (!m_face_attribute.at(fid).m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    if (va != v2_id) { // ignore collapsing edge (v1,v2)
                        std::array<size_t, 2> ba = {{v1_id, va}};
                        if (collapse_is_order_2_edge(ba)) {
                            ba[0] = v2_id; // replace v1 with v2 for check in `after` function
                            std::sort(ba.begin(), ba.end());
                            bs.push_back(ba);
                        }
                    }
                    if (vb != v2_id) { // ignore collapsing edge (v1,v2)
                        std::array<size_t, 2> bb = {{v1_id, vb}};
                        if (collapse_is_order_2_edge(bb)) {
                            bb[0] = v2_id; // replace v1 with v2 for check in `after` function
                            std::sort(bb.begin(), bb.end());
                            bs.push_back(bb);
                        }
                    }
                }
            }
        }
        wmtk::vector_unique(bs);
        cache.boundary_edges = bs;
    }

    if (m_params.preserve_topology && VA[v1_id].m_is_on_surface && VA[v2_id].m_is_on_surface) {
        if (!substructure_link_condition(loc)) {
            return false;
        }
    }

    // Last, so a candidate rejected above never pays for the ball walk.
    if (m_coarsen_mode) {
        const size_t seeds[2] = {v1_id, v2_id};
        auto& scr = coarsen_scratch.local();
        cache.region_max_rel_before = region_max_quality_rel(
            collect_vertex_ball(seeds, 2, m_params.coarsen_smooth_ring, scr));
    }

    return true;
}

bool TetOptimizerMesh::collapse_edge_after(const Tuple& loc)
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    if (!TetMesh::collapse_edge_after(loc)) {
        // debug code
        // wmtk::logger().info("edge {} not pass connectivity after check", loc.fid(*this));
        // if (debug_flag) std::cout << "connectivity reject" << std::endl;

        return false;
    }

    if (!collapse_after_connectivity(v1_id, v2_id, cache.boundary_edges)) {
        return false;
    }
    // auto& VA = m_vertex_attribute;
    // auto& cache = collapse_cache.local();
    // size_t v1_id = cache.v1_id;
    // size_t v2_id = cache.v2_id;
    // size_t v3_id = loc.switch_vertex(*this).vid(*this);
    // if (m_vertex_attribute[v2_id].is_freezed && m_vertex_attribute[v3_id].is_freezed) return
    // false;

    // surface
    // and open boundary
    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_faces) {
            // surface envelope
            if (surface_triangle_is_outside(vids[0], vids[1], vids[2])) {
                return false;
            }

            // // open boundary envelope
            // // by checking each edge on cached surface
            // if (VA[vids[0]].m_is_on_open_boundary && VA[vids[1]].m_is_on_open_boundary) {
            //     if (m_order2_envelope->is_outside(
            //             {{VA[vids[0]].m_posf, VA[vids[1]].m_posf, VA[vids[0]].m_posf}}))
            //         return false;
            // }
            // if (VA[vids[1]].m_is_on_open_boundary && VA[vids[2]].m_is_on_open_boundary) {
            //     if (m_order2_envelope->is_outside(
            //             {{VA[vids[1]].m_posf, VA[vids[2]].m_posf, VA[vids[1]].m_posf}}))
            //         return false;
            // }
            // if (VA[vids[2]].m_is_on_open_boundary && VA[vids[0]].m_is_on_open_boundary) {
            //     if (m_order2_envelope->is_outside(
            //             {{VA[vids[2]].m_posf, VA[vids[0]].m_posf, VA[vids[2]].m_posf}}))
            //         return false;
            // }
        }
        // for (const auto& vids : cache.boundary_edges) {
        //     if (!is_open_boundary_edge(vids)) {
        //        // edge was an open boundary before (that is why it got cached) but is not anymore
        //        // after collapse
        //        return false;
        //    }
        //}
    }

    // Must run HERE, before the attribute updates below -- not at the end of the function.
    // tetwild's override asks is_vertex_on_boundary(v2), which reads BOTH
    // m_vertex_attribute[..].m_is_on_surface and m_face_attribute[..].m_is_surface_fs
    // (TetWildMesh.cpp), and the two blocks below overwrite exactly those: the vertex flag is
    // OR-ed from v1 just after round(), and the cached face attributes are written onto the
    // post-collapse faces. Asking afterwards is asking a different question, and it changes
    // which vertices keep their open-boundary flag -- and therefore which later collapses are
    // allowed.
    collapse_after_vertex(v1_id, v2_id);

    //// update attrs
    // tet attr
    for (int i = 0; i < cache.changed_tids.size(); i++) {
        set_cell_quality(cache.changed_tids[i], cache.changed_energies[i]);
    }
    // vertex attr
    round(loc);
    VA[v2_id].m_is_on_surface = VA.at(v1_id).m_is_on_surface || VA.at(v2_id).m_is_on_surface;
    VA[v2_id].m_sizing_scalar =
        collapse_merged_sizing(VA.at(v1_id).m_sizing_scalar, VA.at(v2_id).m_sizing_scalar);
    VA[v2_id].m_order = std::max(VA.at(v1_id).m_order, VA.at(v2_id).m_order);

    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        const auto found = try_tuple_from_face({{v2_id, old_vids[1], old_vids[2]}});
        if (!found.has_value()) {
            return false; // the collapse removed the face this attribute was to land on
        }
        m_face_attribute[std::get<1>(found.value())] = f_attr;
    }

    if (!m_coarsen_mode) {
        return true;
    }

    // ---- The coarsening composite ---------------------------------------------------
    //
    // Everything from here on happens inside TetMesh::collapse_edge's protected region, so a
    // `false` return undoes the smoothing below AND the collapse above in one shot:
    // collapse_edge_rollback restores the connectivity and the old tet hashes, and
    // rollback_protected_attributes replays the attribute journal. The journal records each
    // index on FIRST write, so however many times the loop below rewrites a position, the
    // recorded value is still the pre-collapse one.
    auto& scr = coarsen_scratch.local();
    const size_t seed = v2_id; // v1 was merged into v2
    collect_vertex_ball(&seed, 1, m_params.coarsen_smooth_ring, scr);

    // smooth_vertex_reversible touches only scr's saved_* members, so iterating scr.ring
    // while handing it the same scratch is safe.
    for (int pass = 0; pass < m_params.coarsen_local_smoothing_passes; ++pass) {
        for (size_t i = 0; i < scr.ring.size(); ++i) {
            smooth_vertex_reversible(scr.ring[i], scr);
        }
    }

    // Keep it only if the worst element in the region this operation could have disturbed is
    // no worse than before. Nothing outside that region was touched, so this local test is
    // exactly the global one: max energy cannot have risen. Measured relative to each cell's
    // own target -- see quality_rel for why a raw comparison would not hold when an
    // application gives different regions different targets.
    return region_max_quality_rel(scr.ring) <= cache.region_max_rel_before;
}

const std::vector<size_t>& TetOptimizerMesh::collect_vertex_ball(
    const size_t* seeds,
    size_t n_seeds,
    int n,
    CoarsenScratch& scr) const
{
    const size_t cap = vert_capacity();
    if (scr.stamp.size() < cap) {
        scr.stamp.resize(cap, 0);
    }
    if (++scr.epoch == 0) { // wrapped: every stale stamp would read as current
        std::fill(scr.stamp.begin(), scr.stamp.end(), 0);
        scr.epoch = 1;
    }
    const uint32_t epoch = scr.epoch;

    // BFS order, nearest first, which is also the order the smoothing wants: relax the merged
    // vertex before the ring around it. Deterministic because m_conn_tets is kept sorted.
    scr.ring.clear();
    scr.frontier.clear();
    // An empty one-ring is how a removed or isolated vertex presents itself; such a vertex has
    // no quality to measure and nothing to smooth, so it never enters the ball. v1 lands here
    // on every call from collapse_edge_after, having just been merged away.
    const auto skip = [this](size_t v) { return get_one_ring_tids_for_vertex(v).empty(); };

    for (size_t i = 0; i < n_seeds; ++i) {
        const size_t v = seeds[i];
        if (scr.stamp[v] == epoch || skip(v)) {
            continue;
        }
        scr.stamp[v] = epoch;
        scr.ring.push_back(v);
        scr.frontier.push_back(v);
    }

    for (int depth = 0; depth < n; ++depth) {
        scr.next.clear();
        for (const size_t v : scr.frontier) {
            scr.one_ring.clear();
            for (const size_t tid : get_one_ring_tids_for_vertex(v)) {
                for (const size_t w : oriented_tet_vids(tid)) {
                    scr.one_ring.push_back(w);
                }
            }
            for (const size_t w : scr.one_ring) {
                if (scr.stamp[w] == epoch || skip(w)) {
                    continue;
                }
                scr.stamp[w] = epoch;
                scr.ring.push_back(w);
                scr.next.push_back(w);
            }
        }
        if (scr.next.empty()) {
            break;
        }
        scr.frontier.swap(scr.next);
    }
    return scr.ring;
}

double TetOptimizerMesh::region_max_quality_rel(const std::vector<size_t>& vids) const
{
    double worst = 0.;
    for (const size_t vid : vids) {
        for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
            worst = std::max(worst, quality_rel(tid));
        }
    }
    return worst;
}

bool TetOptimizerMesh::smooth_vertex_reversible(const size_t vid, CoarsenScratch& scr)
{
    const Tuple t = tuple_from_vertex(vid);
    if (!t.is_valid(*this)) {
        return false;
    }

    // smooth_vertex_3d writes the position and the one-ring qualities and only THEN decides
    // whether to keep them -- its contract puts the undo on the caller, because its usual
    // caller (TetMesh::smooth_vertex) has a protected region of its own. Here the protected
    // region belongs to the collapse and is all-or-nothing, so one rejected vertex would
    // poison the whole composite; hence this narrower save/restore.
    scr.saved_vertex = m_vertex_attribute.at(vid);
    scr.saved_qualities.clear();
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        scr.saved_qualities.emplace_back(tid, cell_quality(tid));
    }

    if (smooth_before(t) && smooth_after(t)) {
        return true;
    }

    m_vertex_attribute[vid] = scr.saved_vertex;
    for (const auto& [tid, quality] : scr.saved_qualities) {
        set_cell_quality(tid, quality);
    }
    return false;
}

bool TetOptimizerMesh::coarsen_collapse_edge(const Tuple& e, std::vector<Tuple>& new_tets)
{
    const bool saved = m_coarsen_mode;
    const bool saved_limit = m_collapse_limit_length;
    m_coarsen_mode = true;
    m_collapse_limit_length = false;
    const bool accepted = collapse_edge(e, new_tets);
    m_coarsen_mode = saved;
    m_collapse_limit_length = saved_limit;
    return accepted;
}

size_t TetOptimizerMesh::coarsen_mesh()
{
    if (!m_params.coarsen_pass) {
        return 0;
    }

    logger().info("========coarsening========");
    igl::Timer timer;
    timer.start();

    const size_t cells_before = get_tets().size();
    const double max_before = std::get<0>(optimization_quality_stats());

    size_t total = 0;
    for (int round = 0; round < m_params.coarsen_max_rounds; ++round) {
        m_coarsen_mode = true;
        // The lock claims one ring more than coarsen_smooth_ring. With smoothing on that is
        // what the writes reach (smoothing a vertex at distance r reads its one-ring and
        // writes the quality of its incident cells); with coarsen_local_smoothing_passes at 0
        // it is what the accept test READS, since a cell incident to a distance-r vertex has
        // vertices at r+1. Either way the +1 is required.
        const size_t accepted = collapse_all_edges_impl(
            false,
            m_params.coarsen_smooth_ring + 1,
            size_t(m_params.coarsen_max_inner_passes));
        m_coarsen_mode = false;
        total += accepted;

        const double e_after_collapse = std::get<0>(optimization_quality_stats());

        // THE PASS'S CONTRACT APPLIES TO ITS OWN SMOOTHING TOO.
        //
        // Every collapse above is kept only if the region it disturbed came out no worse
        // (collapse_edge_after, coarsen branch). This smoothing has no such test, and with
        // smooth_quality_veto off -- which is topological_offset's default -- smooth_vertex_3d
        // accepts any move that neither inverts a cell nor leaves the envelope, however much it
        // degrades one. So the pass could hand back a mesh worse than it was given, which is the
        // one thing it promises not to do.
        //
        // Measured on specific_models/prism, per round, before this:
        //   round 0  start 9.92987 -> collapses 9.86786 (-0.062) -> smooth_all 10.9986 (+1.131)
        //   round 1  start 10.9986 -> collapses 9.89344 (-1.105) -> smooth_all 11.5366 (+1.643)
        // The collapses IMPROVE the mesh on both rounds; this smoothing is the entire regression,
        // and it crossed stop_energy 10, which made Phase A report a failure it had not had.
        // round_all_vertices contributes exactly 0 -- round() sets the RATIONAL from the float,
        // and quality is computed from the float, so rounding cannot move it.
        //
        // This matters MORE since coarsen_local_smoothing_passes defaulted to 0: the local
        // composite that was covered by the accept test is off, so this global pass is now the
        // only relaxation the coarsening does, and it was the only one with no test on it.
        //
        // The veto is off for a real reason and that reason is Phase B's: the offset boundary is
        // placed by minimising a term whose minimum can be most of target_distance away, so
        // nearly every solved position worsens some incident face and vetoing freezes the
        // boundary. Coarsening is not placing anything -- it is banking element count against a
        // result the loop already reached -- so the reason does not reach here. Forced on for
        // this call only; smooth_after() rebuilds its options per vertex from this field, and no
        // other pass is running.
        const bool saved_veto = m_params.smooth_quality_veto;
        m_params.smooth_quality_veto = true;
        smooth_all_vertices(size_t(m_params.coarsen_global_smoothing_passes));
        m_params.smooth_quality_veto = saved_veto;
        const double e_after_smooth = std::get<0>(optimization_quality_stats());

        double e_after_round = e_after_smooth;
        if (m_params.coarsen_global_smoothing_passes > 0) {
            round_all_vertices();
            e_after_round = std::get<0>(optimization_quality_stats());
        }

        // DIAGNOSTIC: the round's own line below reports only the total, and three different
        // things move it. This says which, so the contract above is observable rather than
        // asserted -- it is what localised the regression to smooth_all in the first place.
        logger().warn(
            "[coarsen substeps round {}] collapses {:.6g} -> smooth_all {:.6g} ({:+.6g}) -> "
            "round_all {:.6g} ({:+.6g})",
            round,
            e_after_collapse,
            e_after_smooth,
            e_after_smooth - e_after_collapse,
            e_after_round,
            e_after_round - e_after_smooth);

        const auto [max_metric, avg_metric] = optimization_quality_stats();
        logger().info(
            "coarsen round {}: accepted {} | #V = {}, #T = {} | max energy = {:.6} avg = {:.6}",
            round,
            accepted,
            get_vertices().size(),
            get_tets().size(),
            max_metric,
            avg_metric);

        if (accepted == 0) {
            break;
        }
    }

    m_coarsen_stats.cells_before = cells_before;
    m_coarsen_stats.cells_after = get_tets().size();
    m_coarsen_stats.accepted = total;
    m_coarsen_stats.max_energy_before = max_before;
    m_coarsen_stats.max_energy_after = std::get<0>(optimization_quality_stats());
    logger().info(
        "coarsening: accepted {} | #T {} -> {} | max energy {:.6} -> {:.6} | time = {:.4}s",
        total,
        m_coarsen_stats.cells_before,
        m_coarsen_stats.cells_after,
        m_coarsen_stats.max_energy_before,
        m_coarsen_stats.max_energy_after,
        timer.getElapsedTimeInSec());
    return total;
}

} // namespace wmtk
