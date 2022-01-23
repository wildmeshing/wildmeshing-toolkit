#include "TetWild.h"

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>

void tetwild::TetWild::split_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_split", loc);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_simple;
        
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.should_process = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) return false;
            if (length < m_params.splitting_l2) return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [&](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
            auto stack = std::vector<size_t>();
            if (!m.try_set_edge_mutex_two_ring(e, stack)) return {};
            return stack;
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}

bool tetwild::TetWild::split_before(const Tuple& loc0)
{
    split_cache.local().v1_id = loc0.vid(*this);
    auto loc1 = loc0.switch_vertex(*this);
    split_cache.local().v2_id = loc1.vid(*this);
    //
    size_t v1_id = split_cache.local().v1_id;
    size_t v2_id = split_cache.local().v2_id;

    split_cache.local().is_edge_on_surface = is_edge_on_surface(loc0);

    /// save face track info
    auto comp = [](const std::pair<size_t, std::array<size_t, 3>>& v1,
                   const std::pair<size_t, std::array<size_t, 3>>& v2) {
        return v1.second < v2.second;
    };
    auto is_equal = [](const std::pair<size_t, std::array<size_t, 3>>& v1,
                       const std::pair<size_t, std::array<size_t, 3>>& v2) {
        return v1.second == v2.second;
    };
    //
    auto tets = get_one_ring_tets_for_vertex(loc0);
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        bool find_v1 = false;
        bool find_v2 = false;
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) == v1_id) find_v1 = true;
            if (vs[j].vid(*this) == v2_id) find_v2 = true;
            if (find_v1 && find_v2) break;
        }
        if (!find_v1 || !find_v2) continue;
        //
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id) {
                std::array<size_t, 3> f_vids = {{v1_id, v2_id, vs[j].vid(*this)}};
                auto [f, global_fid] = tuple_from_face(f_vids);
                if (m_face_attribute[global_fid].m_is_surface_fs > 0) {
                    split_cache.local().changed_faces.push_back(std::make_pair(global_fid, f_vids));
                }
            }
        }
        wmtk::vector_unique(split_cache.local().changed_faces, comp, is_equal);
    }

    return true;
}

bool tetwild::TetWild::split_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TetMesh::split_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);
    size_t v_id = loc.vid(*this);

    size_t v1_id = split_cache.local().v1_id;
    size_t v2_id = split_cache.local().v2_id;

    /// check inversion & rounding
    m_vertex_attribute[v_id].m_posf =
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    m_vertex_attribute[v_id].m_is_rounded = true;
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[v_id].m_is_rounded = false;
            break;
        }
    }
    if (!m_vertex_attribute[v_id].m_is_rounded) {
        m_vertex_attribute[v_id].m_pos =
            (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
        m_vertex_attribute[v_id].m_posf = to_double(m_vertex_attribute[v_id].m_pos);
    } else
        m_vertex_attribute[v_id].m_pos = to_rational(m_vertex_attribute[v_id].m_posf);

    /// update quality
    for (auto& loc : locs) {
        m_tet_attribute[loc.tid(*this)].m_qualities = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    //surface
    m_vertex_attribute[v_id].m_is_on_surface = split_cache.local().is_edge_on_surface;

    /// update face attribute
    // add new and erase old
    for(auto& info: split_cache.local().changed_faces) {
        size_t old_fid = info.first;
        auto& old_vids = info.second;
        //
        auto [f1, global_fid1] = tuple_from_face({{old_vids[0], old_vids[2], v_id}});
        m_face_attribute[global_fid1] = m_face_attribute[old_fid];
        auto [f2, global_fid2] = tuple_from_face({{old_vids[1], old_vids[2], v_id}});
        m_face_attribute[global_fid2] = m_face_attribute[old_fid];
        //
        m_face_attribute[old_fid].reset();
    }

    cnt_split++;

    return true;
}
