#include "TetRemeshingMesh.h"

#include <igl/Timer.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::tet_remeshing {

void TetRemeshingMesh::split_all_edges()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_split", loc);
    time = timer.getElapsedTime();
    wmtk::logger().info("edge split prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_simple;

        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) return false;
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            double sizing_ratio = (m_vertex_attribute[v1_id].m_sizing_scalar +
                                   m_vertex_attribute[v2_id].m_sizing_scalar) /
                                  2;
            if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetRemeshingMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetRemeshingMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time serial: {:.4}s", time);
    }
}

bool TetRemeshingMesh::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_faces.clear();

    cache.v1_id = loc0.vid(*this);
    auto loc1 = loc0.switch_vertex(*this);
    cache.v2_id = loc1.vid(*this);

    cache.is_edge_on_surface = is_edge_on_surface(loc0);

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
            cache.changed_faces.push_back(std::make_pair(m_face_attribute[global_fid], f_vids));
        }
    }
    wmtk::vector_unique(cache.changed_faces, comp, is_equal);

    // store tet attributes
    const simplex::Edge edge(cache.v1_id, cache.v2_id);
    for (const Tuple& t : tets) {
        const simplex::Tet tet = simplex_from_tet(t);
        const simplex::Edge opp = tet.opposite_edge(edge);
        if (m_tet_attribute.at(t.tid(*this)).tags.empty()) {
            log_and_throw_error("No tags in tet {}", t.tid(*this)); // for debugging
        }
        cache.tets[opp] = m_tet_attribute.at(t.tid(*this));
    }

    return true;
}

bool TetRemeshingMesh::split_edge_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TetMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);
    const size_t v_id = loc.vid(*this);

    auto& cache = split_cache.local();

    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    m_vertex_attribute[v_id].m_posf =
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;

    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            return false;
        }
    }

    // update tet attributes
    {
        // v1 - v_new
        const auto tets1 = get_incident_tets_for_edge(v1_id, v_id);
        const simplex::Edge edge1(v1_id, v_id);
        for (const Tuple& t : tets1) {
            const simplex::Tet tet = simplex_from_tet(t);
            const simplex::Edge opp = tet.opposite_edge(edge1);
            m_tet_attribute[t.tid(*this)] = cache.tets[opp];
        }
        // v2 - v_new
        const auto tets2 = get_incident_tets_for_edge(v2_id, v_id);
        const simplex::Edge edge2(v2_id, v_id);
        for (const Tuple& t : tets2) {
            const simplex::Tet tet = simplex_from_tet(t);
            const simplex::Edge opp = tet.opposite_edge(edge2);
            m_tet_attribute[t.tid(*this)] = cache.tets[opp];
        }
        assert(tets1.size() + tets2.size() == locs.size());
    }

    /// update quality
    for (const Tuple& loc : locs) {
        m_tet_attribute[loc.tid(*this)].m_quality = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.is_edge_on_surface;

    // open boundary
    m_vertex_attribute[v_id].m_is_on_open_boundary = cache.is_edge_open_boundary;

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

} // namespace wmtk::components::tet_remeshing