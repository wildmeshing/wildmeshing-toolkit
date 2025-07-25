#include "IncrementalTetWild.h"
#include "oneapi/tbb/concurrent_vector.h"
#include "wmtk/TetMesh.h"

#include <igl/Timer.h>
#include <algorithm>
#include <atomic>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>


void tetwild::TetWild::collapse_all_edges(bool is_limit_length)
{
    igl::Timer timer;
    double time;
    timer.start();

    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_collapse", loc);
        collect_all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
    }
    auto collect_failure_ops = tbb::concurrent_vector<std::pair<std::string, Tuple>>();
    std::atomic_int count_success = 0;
    time = timer.getElapsedTime();
    wmtk::logger().info("edge collapse prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [&](const auto& m, auto op, const auto& newts) {
            count_success++;
            std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
            for (auto t : newts) {
                op_tups.emplace_back(op, t);
                op_tups.emplace_back(op, t.switch_vertex(m));
            }
            return op_tups;
        };
        executor.priority = [&](auto& m, auto op, auto& t) { return -m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
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

        executor.on_fail = [&](auto& m, auto op, auto& t) {
            collect_failure_ops.emplace_back(op, t);
        };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            wmtk::logger().info("Prepare to collapse {}", collect_all_ops.size());
            executor(*this, collect_all_ops);
            wmtk::logger().info(
                "Collapsed {}, retrying failed {}",
                count_success,
                collect_failure_ops.size());
            collect_all_ops.clear();
            for (auto& item : collect_failure_ops) collect_all_ops.emplace_back(item);
            collect_failure_ops.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time serial: {}s", time);
    }
}

bool tetwild::TetWild::collapse_edge_before(const Tuple& loc) // input is an edge
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();

    cache.changed_faces.clear();
    cache.changed_tids.clear();
    cache.surface_faces.clear();

    size_t v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    size_t v2_id = loc1.vid(*this);

    // bool debug_flag = (v1_id == 1060 && v2_id == 174);
    // if (debug_flag) {
    //     std::cout << m_vertex_attribute[v1_id].m_posf.transpose() << std::endl;
    //     std::cout << m_vertex_attribute[v2_id].m_posf.transpose() << std::endl;
    // }
    //
    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    cache.edge_length =
        (VA[v1_id].m_posf - VA[v2_id].m_posf).norm(); // todo: duplicated computation

    // debug code
    // wmtk::logger().info(
    //     "try to collapse eid:{}, vid1: {}, vid2: {}, length: {}",
    //     loc.fid(*this),
    //     v1_id,
    //     v2_id,
    //     cache.edge_length);


    ///check if on bbox/surface/boundary
    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) return false;
        for (int on_bbox : VA[v1_id].on_bbox_faces)
            if (std::find(
                    VA[v2_id].on_bbox_faces.begin(),
                    VA[v2_id].on_bbox_faces.end(),
                    on_bbox) == VA[v2_id].on_bbox_faces.end()) {
                // debug code
                // wmtk::logger().info("edge {} not passing bbox before check!", loc.fid(*this));
                // if (debug_flag) std::cout << "box reject" << std::endl;
                return false;
            }
    }

    // surface
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_surface) {
        if (!VA[v2_id].m_is_on_surface && m_envelope.is_outside(VA[v2_id].m_posf)) {
            // debug code
            // wmtk::logger().info("edge {} not passing surface before check!", loc.fid(*this));
            // if (debug_flag) std::cout << "surface reject" << std::endl;

            return false;
        }
    }

    // open boundary
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_open_boundary) {
        if (!VA[v2_id].m_is_on_open_boundary &&
            m_open_boundary_envelope.is_outside(VA[v2_id].m_posf)) {
            // if (debug_flag) std::cout << "open boundary reject" << std::endl;

            return false;
        }
    }


    auto n1_locs = get_one_ring_tets_for_vertex(loc);
    auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

    std::map<size_t, double> qs;
    cache.max_energy = 0;
    for (auto& l : n1_locs) {
        qs[l.tid(*this)] = m_tet_attribute[l.tid(*this)].m_quality; // get_quality(l);
        cache.max_energy = std::max(cache.max_energy, qs[l.tid(*this)]);
    }
    for (auto& l : n12_locs) {
        qs.erase(l.tid(*this));
    }


    for (auto& q : qs) {
        //
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
        std::vector<std::array<size_t, 3>> fs;
        for (auto& t : n1_locs) {
            auto vs = oriented_tet_vids(t);

            int j_v1 = -1;
            auto skip = [&]() {
                for (auto j = 0; j < 4; j++) {
                    auto vid = vs[j];
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
    }

    // for (size_t tid : cache.changed_tids) { // fortest
    //     assert(!is_inverted(tuple_from_tet(tid)));
    // }

    // debug code
    // wmtk::logger().info("edge {} pass collapse before check", loc.fid(*this));

    // test code
    // check global number nonmanifold vertex

    // cache.global_nonmani_ver_cnt = 0;
    // for (auto v : get_vertices()) {
    //     if (m_vertex_attribute[v.vid(*this)].m_is_on_surface) {
    //         if (count_vertex_links(v) > 1) {
    //             cache.global_nonmani_ver_cnt++;
    //         }
    //     }
    // }

    if (m_params.preserve_geometry) {
        // if (m_vertex_attribute[v1_id].m_is_on_surface !=
        // m_vertex_attribute[v2_id].m_is_on_surface)
        //     return false;
        if (m_vertex_attribute[v1_id].m_is_on_surface &&
            !m_vertex_attribute[v2_id].m_is_on_surface) {
            // if (debug_flag) std::cout << "param surface reject" << std::endl;
            return false;
        }

        if (m_vertex_attribute[v1_id].is_freezed) {
            // if (debug_flag) std::cout << "param freeze reject" << std::endl;
            return false;
        }


        // if (m_vertex_attribute[v1_id].face_nearly_param_type.size() > 1) return false;

        // only collapse when param_type v1 is included in param_type v2
        if (m_vertex_attribute[v1_id].face_nearly_param_type.size() >
            m_vertex_attribute[v2_id].face_nearly_param_type.size()) {
            // if (debug_flag) std::cout << "face param set larger reject" << std::endl;

            return false;
        }

        auto s = wmtk::set_intersection(
            m_vertex_attribute[v1_id].face_nearly_param_type,
            m_vertex_attribute[v2_id].face_nearly_param_type);
        if (s.size() != m_vertex_attribute[v1_id].face_nearly_param_type.size()) {
            // if (debug_flag) std::cout << "face param not subset reject" << std::endl;

            return false;
        }

        if (m_vertex_attribute[v1_id].in_edge_param.size() >
            m_vertex_attribute[v2_id].in_edge_param.size()) {
            // if (debug_flag) std::cout << "edge param set larger reject" << std::endl;
            return false;
        }

        auto s2 = wmtk::set_intersection(
            m_vertex_attribute[v1_id].in_edge_param,
            m_vertex_attribute[v2_id].in_edge_param);
        if (s2.size() != m_vertex_attribute[v1_id].in_edge_param.size()) {
            // if (debug_flag) std::cout << "fedge param not subset reject" << std::endl;

            return false;
        }
    }

    return true;
}

bool tetwild::TetWild::collapse_edge_after(const Tuple& loc)
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    // bool debug_flag = (v1_id == 1060 && v2_id == 174);


    if (!TetMesh::collapse_edge_after(loc)) {
        // debug code
        // wmtk::logger().info("edge {} not pass connectivity after check", loc.fid(*this));
        // if (debug_flag) std::cout << "connectivity reject" << std::endl;

        return false;
    }
    // auto& VA = m_vertex_attribute;
    // auto& cache = collapse_cache.local();
    // size_t v1_id = cache.v1_id;
    // size_t v2_id = cache.v2_id;
    // size_t v3_id = loc.switch_vertex(*this).vid(*this);
    // if (m_vertex_attribute[v2_id].is_freezed && m_vertex_attribute[v3_id].is_freezed) return
    // false;


    // wmtk::logger().info("changed tids: {}", cache.changed_tids);

    // check quality
    std::vector<double> qs;
    for (size_t tid : cache.changed_tids) {
        auto tet = tuple_from_tet(tid);
        auto tvs = oriented_tet_vertices(tet);

        // coplanar check
        if (m_params.preserve_geometry) {
            auto tmp1 = wmtk::set_intersection(
                m_vertex_attribute[tvs[0].vid(*this)].face_nearly_param_type,
                m_vertex_attribute[tvs[1].vid(*this)].face_nearly_param_type);

            auto tmp2 = wmtk::set_intersection(
                m_vertex_attribute[tvs[2].vid(*this)].face_nearly_param_type,
                m_vertex_attribute[tvs[3].vid(*this)].face_nearly_param_type);

            auto tmp = wmtk::set_intersection(tmp1, tmp2);
            if (tmp.size() > 0) return false;
        }


        if (is_inverted(tet)) {
            // if (debug_flag) std::cout << "tet inverted reject" << std::endl;

            return false;
        }
        double q = get_quality(tet);
        if (q > cache.max_energy) {
            // if (debug_flag)
            //     std::cout << "energy reject " << q << " " << cache.max_energy << std::endl;

            return false;
        }
        qs.push_back(q);
    }

    // wmtk::logger().info("changed qualities: {}", qs);

    // surface
    // and open boundary

    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_faces) {
            // surface envelope
            bool is_out = m_envelope.is_outside(
                {{VA[vids[0]].m_posf, VA[vids[1]].m_posf, VA[vids[2]].m_posf}});
            if (is_out) {
                // if (debug_flag) std::cout << "surface enve reject" << std::endl;

                return false;
            }

            // open boundary envelope
            // by checking each edge on cached surface
            if (VA[vids[0]].m_is_on_open_boundary && VA[vids[1]].m_is_on_open_boundary) {
                if (m_open_boundary_envelope.is_outside(
                        {{VA[vids[0]].m_posf, VA[vids[1]].m_posf, VA[vids[0]].m_posf}}))
                    return false;
            }
            if (VA[vids[1]].m_is_on_open_boundary && VA[vids[2]].m_is_on_open_boundary) {
                if (m_open_boundary_envelope.is_outside(
                        {{VA[vids[1]].m_posf, VA[vids[2]].m_posf, VA[vids[1]].m_posf}}))
                    return false;
            }
            if (VA[vids[2]].m_is_on_open_boundary && VA[vids[0]].m_is_on_open_boundary) {
                if (m_open_boundary_envelope.is_outside(
                        {{VA[vids[2]].m_posf, VA[vids[0]].m_posf, VA[vids[2]].m_posf}}))
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
    round(loc);
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

    // test code
    // check global number nonmanifold vertex

    // size_t nonmani_ver_cnt = 0;
    // for (auto v : get_vertices()) {
    //     if (m_vertex_attribute[v.vid(*this)].m_is_on_surface) {
    //         if (count_vertex_links(v) > 1) {
    //             nonmani_ver_cnt++;
    //         }
    //     }
    // }
    // if (nonmani_ver_cnt != cache.global_nonmani_ver_cnt) {
    //     wmtk::logger().info(
    //         "COLLAPSE EDGE CAUSE NONMANIFOLDNESS CHANGE ON VERTICE. BEFORE COUNT: {} AFTER COUNT:
    //         "
    //         "{}",
    //         cache.global_nonmani_ver_cnt,
    //         nonmani_ver_cnt);
    // }

    // geometry preservation
    // if (m_params.preserve_geometry) {
    //     std::vector<size_t> after_edge_incident_param_type = wmtk::set_intersection(
    //         m_vertex_attribute[loc.vid(*this)].face_param_type,
    //         m_vertex_attribute[loc.switch_vertex(*this).vid(*this)].face_param_type,
    //     );


    // }

    // //// update attrs
    // // tet attr
    // for (int i = 0; i < cache.changed_tids.size(); i++) {
    //     m_tet_attribute[cache.changed_tids[i]].m_quality = qs[i];
    // }
    // // vertex attr
    // round(loc);
    // VA[v2_id].m_is_on_surface = VA[v1_id].m_is_on_surface || VA[v2_id].m_is_on_surface;
    // // open boundary
    // VA[v2_id].m_is_on_open_boundary =
    //     VA[v1_id].m_is_on_open_boundary || VA[v2_id].m_is_on_open_boundary;

    // // no need to update on_bbox_faces
    // // face attr
    // for (auto& info : cache.changed_faces) {
    //     auto& f_attr = info.first;
    //     auto& old_vids = info.second;
    //     //
    //     auto [_, global_fid] = tuple_from_face({{v2_id, old_vids[1], old_vids[2]}});
    //     if (global_fid == -1) {
    //         // if (debug_flag) std::cout << "whatever reject" << std::endl;

    //         return false;
    //     }
    //     m_face_attribute[global_fid] = f_attr;
    // }

    return true;
}
