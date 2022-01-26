#include "TetWild.h"
#include "wmtk/TetMesh.h"

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>


void tetwild::TetWild::collapse_all_edges(bool is_limit_length)
{
    collapse_cache.local().is_limit_length = is_limit_length;
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_collapse", loc);
        collect_all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
    }
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [](const auto& m, auto op, const auto& newts) {
            std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
            for (auto t : newts) {
                op_tups.emplace_back(op, t);
                op_tups.emplace_back(op, t.switch_vertex(m));
            }
            return op_tups;
        };
        executor.priority = [&](auto& m, auto op, auto& t) { return -m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.should_process = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != -weight) return false;
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            if (is_limit_length && length > m_params.collapsing_l2 *
                                                (m_vertex_attribute[v1_id].m_sizing_scalar +
                                                 m_vertex_attribute[v2_id].m_sizing_scalar) /
                                                2)
                return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}

bool tetwild::TetWild::collapse_before(const Tuple& loc) // input is an edge
{
    collapse_cache.local().changed_faces.clear();
    collapse_cache.local().changed_tids.clear();
    collapse_cache.local().surface_faces.clear();

    size_t v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    size_t v2_id = loc1.vid(*this);
    //
    collapse_cache.local().v1_id = v1_id;
    collapse_cache.local().v2_id = v2_id;

    collapse_cache.local().edge_length =
        (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf)
            .norm(); // todo: duplicated computation

    ///check if on bbox/surface/boundary
    // bbox
    if (!m_vertex_attribute[v1_id].on_bbox_faces.empty()) {
        if (m_vertex_attribute[v2_id].on_bbox_faces.size() <
            m_vertex_attribute[v1_id].on_bbox_faces.size())
            return false;
        for (int on_bbox : m_vertex_attribute[v1_id].on_bbox_faces)
            if (std::find(
                    m_vertex_attribute[v2_id].on_bbox_faces.begin(),
                    m_vertex_attribute[v2_id].on_bbox_faces.end(),
                    on_bbox) == m_vertex_attribute[v2_id].on_bbox_faces.end())
                return false;
    }
    // surface
    if (collapse_cache.local().edge_length > 0 && m_vertex_attribute[v1_id].m_is_on_surface) {
        if (!m_vertex_attribute[v2_id].m_is_on_surface &&
            m_envelope.is_outside(m_vertex_attribute[v2_id].m_posf))
            return false;
    }
    // remove isolated vertex
    if (m_vertex_attribute[v1_id].m_is_on_surface) {
        auto vids = get_one_ring_vids_for_vertex(v1_id);
        bool is_isolated = true;
        for (size_t vid : vids) {
            if (m_vertex_attribute[vid].m_is_on_surface) {
                is_isolated = false;
                break;
            }
        }
        if (is_isolated) m_vertex_attribute[v1_id].m_is_on_surface = false;
    }

    // todo: store surface info into cache

    auto n1_locs = get_one_ring_tets_for_vertex(loc);
    auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

    std::map<size_t, double> qs;
    for (auto& l : n1_locs) {
        qs[l.tid(*this)] = m_tet_attribute[l.tid(*this)].m_quality; // get_quality(l);
    }
    for (auto& l : n12_locs) {
        qs.erase(l.tid(*this));
    }

    collapse_cache.local().max_energy = 0;
    for (auto& q : qs) {
        if (q.second > collapse_cache.local().max_energy)
            collapse_cache.local().max_energy = q.second;
        //
        collapse_cache.local().changed_tids.push_back(q.first);
    }

    /// record faces
    auto comp = [](const std::pair<size_t, std::array<size_t, 3>>& v1,
                   const std::pair<size_t, std::array<size_t, 3>>& v2) {
        return v1.first < v2.first;
    };
    auto is_equal = [](const std::pair<size_t, std::array<size_t, 3>>& v1,
                       const std::pair<size_t, std::array<size_t, 3>>& v2) {
        return v1.first == v2.first;
    };
    //
    for (auto& t : n12_locs) {
        auto vs = oriented_tet_vertices(t);
        std::array<size_t, 3> f_vids = {{v1_id, 0, 0}};
        int cnt = 1;
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id) {
                f_vids[cnt] = vs[j].vid(*this);
                cnt++;
            }
        }
        auto [_1, global_fid1] = tuple_from_face(f_vids);
        auto [_2, global_fid2] = tuple_from_face({{v2_id, f_vids[1], f_vids[2]}});
        FaceAttributes f_attr = m_face_attribute[global_fid1];
        f_attr.merge(m_face_attribute[global_fid2]);
        collapse_cache.local().changed_faces.push_back(std::make_pair(f_attr, f_vids));

        //        wmtk::vector_unique(collapse_cache.local().changed_faces, comp, is_equal);
    }

    std::vector<std::array<size_t, 3>> fs;
    for (auto& t : n1_locs) {
        auto vs = oriented_tet_vertices(t);
        bool find_v2 = false;
        int j_v1 = -1;
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) == v2_id) {
                find_v2 = true;
                break;
            }
            if (vs[j].vid(*this) == v1_id) j_v1 = j;
        }
        if (find_v2) continue;

        for (int k = 0; k < 3; k++) {
            std::array<size_t, 3> f = {
                {v1_id,
                 vs[(j_v1 + 1 + k) % 4].vid(*this),
                 vs[(j_v1 + 1 + (k + 1) % 3) % 4].vid(*this)}};
            std::sort(f.begin(), f.end());
            fs.push_back(f);
        }
    }
    wmtk::vector_unique(fs);

    for (auto& f : fs) {
        auto [_1, global_fid1] = tuple_from_face(f);
        if (m_face_attribute[global_fid1].m_is_surface_fs) {
            int j = std::find(f.begin(), f.end(), v1_id) - f.begin();
            f[j] = v2_id;
            collapse_cache.local().surface_faces.push_back(f);
        }
    }

    for (size_t tid : collapse_cache.local().changed_tids) { // fortest
        assert(!is_inverted(tuple_from_tet(tid)));
    }

    return true;
}

bool tetwild::TetWild::collapse_after(const Tuple& loc)
{
    if (!TetMesh::collapse_after(loc)) return false;

    size_t v1_id = collapse_cache.local().v1_id;
    size_t v2_id = collapse_cache.local().v2_id;

    // check quality
    std::vector<double> qs;
    for (size_t tid : collapse_cache.local().changed_tids) {
        double q = get_quality(tuple_from_tet(tid));
        if (q > collapse_cache.local().max_energy) {
            return false;
        }
        qs.push_back(q);
    }

    // surface
    if (collapse_cache.local().edge_length > 0) {
        for (auto& vids : collapse_cache.local().surface_faces) {
            bool is_out = m_envelope.is_outside(
                {{m_vertex_attribute[vids[0]].m_posf,
                  m_vertex_attribute[vids[1]].m_posf,
                  m_vertex_attribute[vids[2]].m_posf}});
            if (is_out) {
                //                cout<<"Env"<<endl;
                return false;
            }
        }
    }

    //// update attrs
    // tet attr
    for (int i = 0; i < collapse_cache.local().changed_tids.size(); i++) {
        m_tet_attribute[collapse_cache.local().changed_tids[i]].m_quality = qs[i];
    }
    // vertex attr
    m_vertex_attribute[v2_id].m_is_on_surface =
        m_vertex_attribute[v1_id].m_is_on_surface || m_vertex_attribute[v2_id].m_is_on_surface;
    // no need to update on_bbox_faces
    // face attr
    for (auto& info : collapse_cache.local().changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        auto [_, global_fid] = tuple_from_face({{v2_id, old_vids[1], old_vids[2]}});
        m_face_attribute[global_fid] = f_attr;
    }

    cnt_collapse++;

    return true;
}
