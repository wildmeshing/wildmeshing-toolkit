#include "TriOptimizerMesh.h"

#include <cstdlib>
#include "wmtk/TriMesh.h"

#include <igl/Timer.h>
#include <algorithm>
#include <atomic>
#include <unordered_set>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>

namespace wmtk {

void TriOptimizerMesh::collapse_all_edges(bool is_limit_length)
{
    collapse_all_edges_impl(is_limit_length, wmtk::default_ring(wmtk::PassLock::EdgeRing));
}

size_t TriOptimizerMesh::collapse_all_edges_impl(bool is_limit_length, int lock_ring)
{
    m_collapse_limit_length = is_limit_length;
    collapse_pass_begin();
    igl::Timer timer;
    timer.start();

    // Build the collapse op list in parallel (both edge directions). Filtering of
    // too-long edges still happens in is_weight_up_to_date.
    auto all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TriOptimizerMesh& m, const Tuple& e, auto& out) {
            out.emplace_back("edge_collapse", e);
            out.emplace_back("edge_collapse", e.switch_vertex(m));
        });
    logger().info("#E = {}", all_ops.size() / 2);
    logger().info("edge collapse prepare time: {:.4}s", timer.getElapsedTimeInSec());

    size_t accepted = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeRing,
        lock_ring,
        "edge collapse",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples =
                [](const wmtk::TriOptimizerMesh& m, Op op, const auto& newts) {
                    std::vector<std::pair<std::string, TriMesh::Tuple>> op_tups;
                    op_tups.reserve(2 * newts.size());
                    for (const Tuple& t : newts) {
                        op_tups.emplace_back(op, t);
                        op_tups.emplace_back(op, t.switch_vertex(m));
                    }
                    return op_tups;
                };
            executor.priority = [](const wmtk::TriOptimizerMesh& m, Op op, const Tuple& t) {
                return -m.get_length2(t);
            };
            executor.is_weight_up_to_date = [&](const wmtk::TriOptimizerMesh& m,
                                                const std::tuple<double, Op, Tuple>& ele) {
                const auto& VA = m_vertex_attribute;
                auto& [weight, op, tup] = ele;
                const double length = m.get_length2(tup);
                if (length != -weight) {
                    return false;
                }
                // Deliberately NOT filtered on length here. An over-length edge stays a
                // candidate and collapse_edge_before decides it on quality instead: it is kept
                // only if it STRICTLY improves the worst element of the ring. See there.
                return true;
            };

            // Retry a failed collapse only where the mesh actually changed this round
            // (dirty-epoch localized retry). This replaces the loop that rebuilt the whole op
            // list from get_edges() after every pass and re-ran the expensive geometric
            // pre-checks on every failure, even where nothing could have changed.
            const size_t total_success =
                wmtk::run_localized_to_convergence(mesh, executor, all_ops);
            logger().info("collapse success: {}", total_success);
            collapse_pass_end(total_success);
            accepted = total_success;
        });
    return accepted;
}

bool TriOptimizerMesh::collapse_edge_before(const Tuple& loc) // input is an edge
{
    const auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();

    cache.changed_edges.clear();
    cache.changed_fids.clear();
    cache.changed_energies.clear();
    cache.surface_edges.clear();

    size_t v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    size_t v2_id = loc1.vid(*this);

    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    cache.edge_length = (VA[v1_id].m_posf - VA[v2_id].m_posf).norm();

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
    if (!collapse_before_vertex(v1_id, v2_id)) {
        return false;
    }

    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) {
            return false;
        }
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
        if (VA[v1_id].m_is_rounded && !VA[v2_id].m_is_on_surface) {
            return false; // do not collapse away from surface
        }

        if (VA[v1_id].m_is_rounded && get_order_of_vertex(v1_id) > 1 &&
            get_order_of_vertex(v2_id) <= 1) {
            /**
             * In general, we don't want to collapse away from feature vertices. It is always fine
             * to collapse into a feature vertex, though. However, we allow to feature vertices to
             * be collapsed.
             */
            return false;
        }

        // if both vertices are on the surface, the collapsing edge should be inside the envelope
        const size_t eid = loc.eid(*this);
        if (VA[v2_id].m_is_on_surface && !m_edge_attribute.at(eid).m_is_surface_fs) {
            if (surface_segment_is_outside(v1_id, v2_id)) {
                return false;
            }
        }
    }

    const auto& n1_locs = get_one_ring_fids_for_vertex(loc);

    cache.changed_fids.reserve(n1_locs.size());
    cache.max_energy = 0;
    for (const size_t& tid : n1_locs) {
        const double q = m_face_attribute.at(tid).m_quality;
        cache.max_energy = std::max(cache.max_energy, q);
        const auto vs = oriented_tri_vids(tid);
        if (vs[0] != v2_id && vs[1] != v2_id && vs[2] != v2_id) {
            cache.changed_fids.emplace_back(tid);
        }
    }

    // pre-compute after-collapse energies
    cache.changed_energies.reserve(cache.changed_fids.size());
    for (const size_t tid : cache.changed_fids) {
        std::array<size_t, 3> vs = oriented_tri_vids(tid);
        for (size_t i = 0; i < 3; ++i) {
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
        // above still applies: an inverted face is not something smoothing can repair, since
        // smooth_vertex_2d refuses to start from one.
        if (!m_coarsen_mode && !collapse_quality_allowed(v1_id, tid, q, cache.max_energy)) {
            return false;
        }
        cache.changed_energies.emplace_back(q);
    }
    assert(cache.changed_energies.size() == cache.changed_fids.size());

    // Length gate, applied here rather than when the candidate list is built.
    //
    // Coarsening a well-shaped mesh is what the length limit exists to prevent, so a short
    // edge keeps the old behaviour. But applying it to the CANDIDATE LIST made any element
    // whose edges are all longer than 0.8 * l invisible to this pass: it was never offered,
    // so it produced no rejection record anywhere and looked untouched rather than refused.
    // A collinear triangle is exactly that -- measured on triwild20k 189017 at eps_rel 1e-4,
    // one with edges 55 / 165 / 220 against a gate of 38.7 survived every pass while
    // stuck-refine split it, halving its short edge and DOUBLING its energy each round
    // (6.7e16 -> 1.5e17 -> 3.1e17 -> inverted) and growing the mesh from 17k to 6.6M faces.
    //
    // Refinement cannot repair a shape defect -- AMIPS is scale-invariant, so splitting a
    // collinear triangle leaves it collinear -- and collapse is the only operation that can
    // remove one. So an over-length edge is allowed, on the condition that it STRICTLY
    // improves the worst element of the ring. That needs no threshold and is self-limiting:
    // in a healthy mesh almost no long-edge collapse strictly improves anything.
    //
    // Note changed_fids excludes the faces the collapse deletes, so when the worst element
    // in the ring is one of those, the test passes by construction -- the rule admits
    // precisely the collapses that remove a bad element.
    if (m_collapse_limit_length && VA[v1_id].m_is_rounded) {
        const double sizing_ratio = (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2;
        const double len2 = cache.edge_length * cache.edge_length;
        if (len2 > m_params.collapsing_l2 * sizing_ratio * sizing_ratio) {
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
    const auto& n12_locs = get_incident_fids_for_edge(loc);
    for (const size_t& tid : n12_locs) {
        auto vs = oriented_tri_vids(tid);
        std::array<size_t, 2> e_vids = {{v1_id, 0}};
        int cnt = 1;
        // get the vertex that is not v1/v2, i.e., the edge-link vertices.
        for (int j = 0; j < 3; j++) {
            if (vs[j] != v1_id && vs[j] != v2_id) {
                e_vids[cnt] = vs[j];
                cnt++;
            }
        }
        auto [_1, global_eid1] = tuple_from_edge(e_vids);
        auto [_2, global_eid2] = tuple_from_edge({{v2_id, e_vids[1]}});
        auto e_attr = m_edge_attribute.at(global_eid1);
        e_attr.merge(m_edge_attribute.at(global_eid2));
        cache.changed_edges.push_back(std::make_pair(e_attr, e_vids));
    }

    if (VA[v1_id].m_is_on_surface) {
        // this code must check if a face is tagged as surface face
        // only checking the vertices is not enough
        std::vector<std::array<size_t, 2>> fs;
        for (const size_t& tid : n1_locs) {
            const auto vs = oriented_tri_vids(tid);

            int j_v1 = -1;
            auto skip = [&]() {
                for (int j = 0; j < 3; j++) {
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

            for (int k = 0; k < 2; k++) {
                auto va = vs[(j_v1 + 1 + k) % 3];
                if (VA[va].m_is_on_surface) {
                    std::array<size_t, 2> f = {{v1_id, va}};
                    const auto [f_tuple, fid] = tuple_from_edge(f);
                    if (!m_edge_attribute.at(fid).m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    std::sort(f.begin(), f.end());
                    fs.push_back(f);
                }
            }
        }
        wmtk::vector_unique(fs);

        cache.surface_edges.reserve(fs.size());
        for (auto& f : fs) {
            std::replace(f.begin(), f.end(), v1_id, v2_id);
            cache.surface_edges.push_back(f);
        }
    }

    if (m_params.preserve_topology) {
        const bool v1_surf = VA[v1_id].m_is_on_surface;
        const bool v2_surf = VA[v2_id].m_is_on_surface;
        const bool v1_bbox = !VA[v1_id].on_bbox_faces.empty();
        const bool v2_bbox = !VA[v2_id].on_bbox_faces.empty();

        if ((v1_surf || v1_bbox) && (v2_surf || v2_bbox)) {
            if (!substructure_link_condition(loc)) {
                return false;
            }
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

bool TriOptimizerMesh::collapse_edge_after(const Tuple& loc)
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    if (!TriMesh::collapse_edge_after(loc)) {
        return false;
    }

    // surface
    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_edges) {
            // surface envelope
            if (surface_segment_is_outside(vids[0], vids[1])) {
                return false;
            }
        }
    }

    //// update attrs
    // tet attr
    for (int i = 0; i < cache.changed_fids.size(); i++) {
        m_face_attribute[cache.changed_fids[i]].m_quality = cache.changed_energies[i];
    }
    // vertex attr
    VA[v2_id].m_is_on_surface = VA.at(v1_id).m_is_on_surface || VA.at(v2_id).m_is_on_surface;
    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_edges) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        const auto found = try_tuple_from_edge({{v2_id, old_vids[1]}});
        if (!found.has_value()) {
            return false; // the collapse removed the edge this attribute was to land on
        }
        m_edge_attribute[std::get<1>(found.value())] = f_attr;
    }

    collapse_after_vertex(v1_id, v2_id);

    if (!m_coarsen_mode) {
        return true;
    }

    // ---- The coarsening composite ---------------------------------------------------
    //
    // Everything from here on happens inside TriMesh::collapse_edge's protected region, so a
    // `false` return undoes the smoothing below AND the collapse above in one shot:
    // collapse_edge_rollback restores the connectivity and the old face/vertex hashes, and
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
    // exactly the global one: max energy cannot have risen. Measured relative to each face's
    // own target -- see quality_rel for why a raw comparison would not hold when an
    // application gives different regions different targets.
    return region_max_quality_rel(scr.ring) <= cache.region_max_rel_before;
}

const std::vector<size_t>& TriOptimizerMesh::collect_vertex_ball(
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
    // vertex before the ring around it. Deterministic because m_conn_tris is kept sorted.
    scr.ring.clear();
    scr.frontier.clear();
    // An empty one-ring is how a removed or isolated vertex presents itself; such a vertex has
    // no quality to measure and nothing to smooth, so it never enters the ball. v1 lands here
    // on every call from collapse_edge_after, having just been merged away.
    const auto skip = [this](size_t v) { return get_one_ring_fids_for_vertex(v).empty(); };

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
            get_one_ring_vids_for_vertex_duplicate(v, scr.one_ring);
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

double TriOptimizerMesh::region_max_quality_rel(const std::vector<size_t>& vids) const
{
    double worst = 0.;
    for (const size_t vid : vids) {
        for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
            worst = std::max(worst, quality_rel(fid));
        }
    }
    return worst;
}

bool TriOptimizerMesh::smooth_vertex_reversible(const size_t vid, CoarsenScratch& scr)
{
    const Tuple t = tuple_from_vertex(vid);
    if (!t.is_valid(*this)) {
        return false;
    }

    // smooth_vertex_2d writes the position and the one-ring qualities and only THEN decides
    // whether to keep them -- its contract puts the undo on the caller, because its usual
    // caller (TriMesh::smooth_vertex) has a protected region of its own. Here the protected
    // region belongs to the collapse and is all-or-nothing, so one rejected vertex would
    // poison the whole composite; hence this narrower save/restore.
    scr.saved_vertex = m_vertex_attribute.at(vid);
    scr.saved_qualities.clear();
    for (const size_t fid : get_one_ring_fids_for_vertex(vid)) {
        scr.saved_qualities.emplace_back(fid, m_face_attribute.at(fid).m_quality);
    }

    if (smooth_before(t) && smooth_after(t)) {
        return true;
    }

    m_vertex_attribute[vid] = scr.saved_vertex;
    for (const auto& [fid, quality] : scr.saved_qualities) {
        m_face_attribute[fid].m_quality = quality;
    }
    return false;
}

bool TriOptimizerMesh::coarsen_collapse_edge(const Tuple& e, std::vector<Tuple>& new_tris)
{
    const bool saved = m_coarsen_mode;
    const bool saved_limit = m_collapse_limit_length;
    m_coarsen_mode = true;
    m_collapse_limit_length = false;
    const bool accepted = collapse_edge(e, new_tris);
    m_coarsen_mode = saved;
    m_collapse_limit_length = saved_limit;
    return accepted;
}

size_t TriOptimizerMesh::coarsen_mesh()
{
    if (!m_params.coarsen_pass) {
        return 0;
    }

    logger().info("========coarsening========");
    igl::Timer timer;
    timer.start();

    const size_t faces_before = get_faces().size();
    const double max_before = std::get<0>(optimization_quality_stats());

    size_t total = 0;
    for (int round = 0; round < m_params.coarsen_max_rounds; ++round) {
        m_coarsen_mode = true;
        // The lock claims one ring more than the smoothing reaches: smoothing a vertex at
        // distance r reads its one-ring and writes the quality of its incident faces.
        const size_t accepted = collapse_all_edges_impl(false, m_params.coarsen_smooth_ring + 1);
        m_coarsen_mode = false;
        total += accepted;

        smooth_all_vertices(size_t(m_params.coarsen_global_smoothing_passes));
        if (m_params.coarsen_global_smoothing_passes > 0) {
            round_all_vertices();
        }

        const auto [max_metric, avg_metric] = optimization_quality_stats();
        logger().info(
            "coarsen round {}: accepted {} | #V = {}, #F = {} | max energy = {:.6} avg = {:.6}",
            round,
            accepted,
            get_vertices().size(),
            get_faces().size(),
            max_metric,
            avg_metric);

        if (accepted == 0) {
            break;
        }
    }

    m_coarsen_stats.cells_before = faces_before;
    m_coarsen_stats.cells_after = get_faces().size();
    m_coarsen_stats.accepted = total;
    m_coarsen_stats.max_energy_before = max_before;
    m_coarsen_stats.max_energy_after = std::get<0>(optimization_quality_stats());
    logger().info(
        "coarsening: accepted {} | #F {} -> {} | max energy {:.6} -> {:.6} | time = {:.4}s",
        total,
        m_coarsen_stats.cells_before,
        m_coarsen_stats.cells_after,
        m_coarsen_stats.max_energy_before,
        m_coarsen_stats.max_energy_after,
        timer.getElapsedTimeInSec());
    return total;
}

} // namespace wmtk
