#include "TetRemeshingMesh.h"
#include "oneapi/tbb/concurrent_vector.h"
#include "wmtk/TetMesh.h"

#include <igl/Timer.h>
#include <algorithm>
#include <atomic>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::tet_remeshing {

void TetRemeshingMesh::collapse_all_edges(bool is_limit_length)
{
    igl::Timer timer;
    double time;
    timer.start();

    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (const Tuple& loc : get_edges()) {
        // collect all edges. Filtering too long edges happens in `is_weight_up_to_date`
        collect_all_ops.emplace_back("edge_collapse", loc);
        collect_all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
    }
    auto collect_failure_ops = tbb::concurrent_vector<std::pair<std::string, Tuple>>();
    std::atomic_int count_success = 0;
    time = timer.getElapsedTime();
    wmtk::logger().info("edge collapse prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples =
            [&count_success](const TetRemeshingMesh& m, Op op, const std::vector<Tuple>& newts) {
                count_success++;
                std::vector<std::pair<std::string, Tuple>> op_tups;
                for (const auto& t : newts) {
                    op_tups.emplace_back(op, t);
                    op_tups.emplace_back(op, t.switch_vertex(m));
                }
                return op_tups;
            };
        executor.priority = [](const TetRemeshingMesh& m, Op op, const Tuple& t) {
            return -m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const TetRemeshingMesh& m,
                                            const std::tuple<double, Op, Tuple>& ele) {
            auto& VA = m_vertex_attribute;
            auto& [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != -weight) return false;
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            double sizing_ratio = (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2;
            if (is_limit_length && length > m_params.collapsing_l2 * sizing_ratio * sizing_ratio)
                return false;
            return true;
        };

        executor.on_fail =
            [&collect_failure_ops](const TetRemeshingMesh& m, Op op, const Tuple& t) {
                collect_failure_ops.emplace_back(op, t);
            };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            wmtk::logger().info("Prepare to collapse {}", collect_all_ops.size());
            executor(*this, collect_all_ops);
            wmtk::logger().info(
                "Collapsed {}, retrying failed {}",
                (int)count_success,
                collect_failure_ops.size());
            collect_all_ops.clear();
            for (auto& item : collect_failure_ops) collect_all_ops.emplace_back(item);
            collect_failure_ops.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = ExecutePass<TetRemeshingMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](TetRemeshingMesh& m, const Tuple& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = ExecutePass<TetRemeshingMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time serial: {}s", time);
    }
}

bool TetRemeshingMesh::collapse_edge_before(const Tuple& loc) // input is an edge
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();

    cache.changed_faces.clear();
    cache.changed_tids.clear();
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

    // open boundary
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_open_boundary) {
        if (!VA[v2_id].m_is_on_open_boundary &&
            m_open_boundary_envelope.is_outside(VA[v2_id].m_posf)) {
            return false;
        }
    }


    auto n1_locs = get_one_ring_tets_for_vertex(loc);
    auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

    std::map<size_t, double> qs;
    cache.max_energy = 0;
    for (const Tuple& l : n1_locs) {
        qs[l.tid(*this)] = m_tet_attribute[l.tid(*this)].m_quality; // get_quality(l);
        cache.max_energy = std::max(cache.max_energy, qs[l.tid(*this)]);
    }
    for (const Tuple& l : n12_locs) {
        qs.erase(l.tid(*this));
    }

    for (auto& q : qs) {
        cache.changed_tids.push_back(q.first);
    }

    //
    std::set<int> unique_fid;
    for (auto& t : n12_locs) {
        auto vs = oriented_tet_vids(t);
        std::array<size_t, 3> f_vids = {{v1_id, 0, 0}};
        int cnt = 1;
        for (int j = 0; j < 4; j++) {
            if (vs[j] != v1_id && vs[j] != v2_id) {
                f_vids[cnt] = vs[j];
                cnt++;
            }
        }
        auto [_1, global_fid1] = tuple_from_face(f_vids);
        auto [it, suc] = unique_fid.insert(global_fid1);
        if (!suc) continue;

        auto [_2, global_fid2] = tuple_from_face({{v2_id, f_vids[1], f_vids[2]}});
        auto f_attr = m_face_attribute[global_fid1];
        f_attr.merge(m_face_attribute[global_fid2]);
        cache.changed_faces.push_back(std::make_pair(f_attr, f_vids));
    }

    if (VA[v1_id].m_is_on_surface) {
        // this code must check if a face is tagged as boundary
        // only checking the vertices is not enough
        std::vector<std::array<size_t, 3>> fs;
        for (const Tuple& t : n1_locs) {
            const auto vs = oriented_tet_vids(t);

            int j_v1 = -1;
            auto skip = [&]() {
                for (int j = 0; j < 4; j++) {
                    const size_t vid = vs[j];
                    if (vid == v2_id) {
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
                    if (!m_face_attribute[fid].m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    std::sort(f.begin(), f.end());
                    fs.push_back(f);
                }
            }
        }
        wmtk::vector_unique(fs);

        for (auto& f : fs) {
            auto [_1, global_fid1] = tuple_from_face(f);
            if (m_face_attribute[global_fid1].m_is_surface_fs) {
                std::replace(f.begin(), f.end(), v1_id, v2_id);
                cache.surface_faces.push_back(f);
            }
        }

        std::vector<std::array<size_t, 2>> bs;
        for (const Tuple& t : n1_locs) {
            const auto vs = oriented_tet_vids(t);

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
                    if (!m_face_attribute[fid].m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    if (va != v2_id) {
                        std::array<size_t, 2> ba = {{v1_id, va}};
                        if (is_open_boundary_edge(ba)) {
                            ba[0] = v2_id; // replace v1 with v2 for check in `after` function
                            std::sort(ba.begin(), ba.end());
                            bs.push_back(ba);
                        }
                    }
                    if (vb != v2_id) {
                        std::array<size_t, 2> bb = {{v1_id, vb}};
                        if (is_open_boundary_edge(bb)) {
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

    return true;
}

bool TetRemeshingMesh::collapse_edge_after(const Tuple& loc)
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    // bool debug_flag = (v1_id == 1060 && v2_id == 174);


    if (!TetMesh::collapse_edge_after(loc)) {
        return false;
    }

    // check quality
    std::vector<double> qs;
    for (size_t tid : cache.changed_tids) {
        auto tet = tuple_from_tet(tid);
        auto tvs = oriented_tet_vertices(tet);

        if (is_inverted(tet)) {
            return false;
        }
        double q = get_quality(tet);
        // only check quality if v1 is rounded
        if (q > cache.max_energy) {
            // if (debug_flag)
            //     std::cout << "energy reject " << q << " " << cache.max_energy << std::endl;

            return false;
        }
        qs.push_back(q);
    }

    // wmtk::logger().info("changed qualities: {}", qs);


    // open boundary - must be set before checking for open boundary
    VA[v2_id].m_is_on_open_boundary =
        VA[v1_id].m_is_on_open_boundary || VA[v2_id].m_is_on_open_boundary;

    // surface
    // and open boundary

    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_faces) {
            // surface envelope
            bool is_out = m_envelope->is_outside(
                {{VA[vids[0]].m_posf, VA[vids[1]].m_posf, VA[vids[2]].m_posf}});
            if (is_out) {
                return false;
            }

            // // open boundary envelope
            // // by checking each edge on cached surface
            // if (VA[vids[0]].m_is_on_open_boundary && VA[vids[1]].m_is_on_open_boundary) {
            //     if (m_open_boundary_envelope.is_outside(
            //             {{VA[vids[0]].m_posf, VA[vids[1]].m_posf, VA[vids[0]].m_posf}}))
            //         return false;
            // }
            // if (VA[vids[1]].m_is_on_open_boundary && VA[vids[2]].m_is_on_open_boundary) {
            //     if (m_open_boundary_envelope.is_outside(
            //             {{VA[vids[1]].m_posf, VA[vids[2]].m_posf, VA[vids[1]].m_posf}}))
            //         return false;
            // }
            // if (VA[vids[2]].m_is_on_open_boundary && VA[vids[0]].m_is_on_open_boundary) {
            //     if (m_open_boundary_envelope.is_outside(
            //             {{VA[vids[2]].m_posf, VA[vids[0]].m_posf, VA[vids[2]].m_posf}}))
            //         return false;
            // }
        }
        for (const auto& vids : cache.boundary_edges) {
            if (!is_open_boundary_edge(vids)) {
                // edge was an open boundary before (that is why it got cached) but is not anymore
                // after collapse
                return false;
            }
        }
    }

    //// update attrs
    // tet attr
    for (int i = 0; i < cache.changed_tids.size(); i++) {
        m_tet_attribute[cache.changed_tids[i]].m_quality = qs[i];
    }
    // vertex attr
    VA[v2_id].m_is_on_surface = VA[v1_id].m_is_on_surface || VA[v2_id].m_is_on_surface;
    // open boundary
    VA[v2_id].m_is_on_open_boundary =
        VA[v1_id].m_is_on_open_boundary || VA[v2_id].m_is_on_open_boundary;

    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        auto [_, global_fid] = tuple_from_face({{v2_id, old_vids[1], old_vids[2]}});
        if (global_fid == -1) {
            // if (debug_flag) std::cout << "whatever reject" << std::endl;

            return false;
        }
        m_face_attribute[global_fid] = f_attr;
    }


    // global topology check
    {
        std::map<std::pair<size_t, size_t>, int>
            after_edge_link; // edge represented by sorted vids (could use simplex::Edge here)
        std::map<size_t, int> after_vertex_link;

        auto v_on_surf = [this](const Tuple& t) {
            return m_vertex_attribute[t.vid(*this)].m_is_on_surface;
        };

        auto one_ring_tets = get_one_ring_tets_for_vertex(loc);
        std::vector<Tuple> surface_fs;
        for (const Tuple& t : one_ring_tets) {
            const Tuple f1 = t;
            const Tuple f2 = t.switch_face(*this);
            const Tuple f3 = t.switch_edge(*this).switch_face(*this);
            const Tuple f4 = t.switch_vertex(*this).switch_edge(*this).switch_face(*this);

            const auto f1vs = get_face_vertices(f1);
            if (v_on_surf(f1vs[0]) && v_on_surf(f1vs[1]) && v_on_surf(f1vs[2])) {
                if (m_face_attribute[f1.fid(*this)].m_is_surface_fs) surface_fs.push_back(f1);
            }
            const auto f2vs = get_face_vertices(f2);
            if (v_on_surf(f2vs[0]) && v_on_surf(f2vs[1]) && v_on_surf(f2vs[2])) {
                if (m_face_attribute[f2.fid(*this)].m_is_surface_fs) surface_fs.push_back(f2);
            }
            const auto f3vs = get_face_vertices(f3);
            if (v_on_surf(f3vs[0]) && v_on_surf(f3vs[1]) && v_on_surf(f3vs[2])) {
                if (m_face_attribute[f3.fid(*this)].m_is_surface_fs) surface_fs.push_back(f3);
            }
            const auto f4vs = get_face_vertices(f4);
            if (v_on_surf(f4vs[0]) && v_on_surf(f4vs[1]) && v_on_surf(f4vs[2])) {
                if (m_face_attribute[f4.fid(*this)].m_is_surface_fs) surface_fs.push_back(f4);
            }
        }
        for (const Tuple& f : surface_fs) {
            // e1 = v1v2 e2 = v1v3 e3 = v2v3
            const Tuple v1 = f;
            const Tuple v2 = v1.switch_vertex(*this);
            const Tuple v3 = v1.switch_edge(*this).switch_vertex(*this);
            const Tuple e1 = v1;
            const Tuple e2 = v1.switch_edge(*this);
            const Tuple e3 = v1.switch_vertex(*this).switch_edge(*this);

            const size_t vid1 = v1.vid(*this);
            const size_t vid2 = v2.vid(*this);
            const size_t vid3 = v3.vid(*this);


            if (after_vertex_link.find(vid1) == after_vertex_link.end()) {
                after_vertex_link[vid1] = count_vertex_links(v1);
            }
            if (after_vertex_link.find(vid2) == after_vertex_link.end()) {
                after_vertex_link[vid2] = count_vertex_links(v2);
            }
            if (after_vertex_link.find(vid3) == after_vertex_link.end()) {
                after_vertex_link[vid3] = count_vertex_links(v3);
            }

            if (vid1 < vid2) {
                if (after_edge_link.find(std::make_pair(vid1, vid2)) == after_edge_link.end()) {
                    after_edge_link[std::make_pair(vid1, vid2)] = count_edge_links(e1);
                }
            } else {
                if (after_edge_link.find(std::make_pair(vid2, vid1)) == after_edge_link.end()) {
                    after_edge_link[std::make_pair(vid2, vid1)] = count_edge_links(e1);
                }
            }

            if (vid1 < vid3) {
                if (after_edge_link.find(std::make_pair(vid1, vid3)) == after_edge_link.end()) {
                    after_edge_link[std::make_pair(vid1, vid3)] = count_edge_links(e2);
                }
            } else {
                if (after_edge_link.find(std::make_pair(vid3, vid1)) == after_edge_link.end()) {
                    after_edge_link[std::make_pair(vid3, vid1)] = count_edge_links(e2);
                }
            }

            if (vid2 < vid3) {
                if (after_edge_link.find(std::make_pair(vid2, vid3)) == after_edge_link.end()) {
                    after_edge_link[std::make_pair(vid2, vid3)] = count_edge_links(e3);
                }
            } else {
                if (after_edge_link.find(std::make_pair(vid3, vid2)) == after_edge_link.end()) {
                    after_edge_link[std::make_pair(vid3, vid2)] = count_edge_links(e3);
                }
            }
        }

        // check if #links remain the same
        for (const auto& [key, val] : cache.vertex_link) {
            if (key == v1_id) {
                // the collpased vertex
                if (val != after_vertex_link[v2_id]) {
                    return false;
                }
            } else {
                if (val != after_vertex_link[key]) {
                    return false;
                }
            }
        }

        for (const auto& [key, val] : cache.edge_link) {
            if (key.first == v1_id && key.second == v2_id) {
                continue;
            }
            if (key.first == v2_id && key.second == v1_id) {
                continue;
            }
            if (key.first == v1_id) {
                if (v2_id < key.second) {
                    if (val != after_edge_link[std::make_pair(v2_id, key.second)]) {
                        return false;
                    }
                } else {
                    if (val != after_edge_link[std::make_pair(key.second, v2_id)]) {
                        return false;
                    }
                }
            } else if (key.second == v1_id) {
                if (v2_id < key.first) {
                    if (val != after_edge_link[std::make_pair(v2_id, key.first)]) {
                        return false;
                    }
                } else {
                    if (val != after_edge_link[std::make_pair(key.first, v2_id)]) {
                        return false;
                    }
                }
            } else {
                if (val != after_edge_link[key]) {
                    return false;
                }
            }
        }
    }

    return true;
}

} // namespace wmtk::components::tet_remeshing