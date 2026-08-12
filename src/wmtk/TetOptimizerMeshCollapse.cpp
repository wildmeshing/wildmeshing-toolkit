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
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
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
            wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
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
        if (!VA[v2_id].m_is_on_surface && m_envelope->is_outside(VA[v2_id].m_posf)) {
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
        if (!collapse_quality_allowed(v1_id, q, cache.max_energy)) {
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
            bool is_out = m_envelope->is_outside(
                {{VA.at(vids[0]).m_posf, VA.at(vids[1]).m_posf, VA.at(vids[2]).m_posf}});
            if (is_out) {
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
    VA[v2_id].m_order = std::max(VA.at(v1_id).m_order, VA.at(v2_id).m_order);

    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        auto [_, global_fid] = tuple_from_face({{v2_id, old_vids[1], old_vids[2]}});
        if (global_fid == -1) {
            return false;
        }
        m_face_attribute[global_fid] = f_attr;
    }

    return true;
}

} // namespace wmtk
