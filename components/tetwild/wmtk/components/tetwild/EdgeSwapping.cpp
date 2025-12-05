#include "TetWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include "spdlog/spdlog.h"
#include "wmtk/utils/TupleUtils.hpp"

#include <cassert>

namespace wmtk::components::tetwild {

bool face_attribute_tracker(
    const wmtk::TetMesh& m,
    const std::vector<wmtk::TetMesh::Tuple>& incident_tets,
    const TetWildMesh::FaceAttCol& m_face_attribute,
    std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces)
{
    changed_faces.clear();
    std::set<int> middle_face;
    for (const auto& t : incident_tets) {
        for (auto j = 0; j < 4; j++) {
            auto f_t = m.tuple_from_face(t.tid(m), j);
            auto global_fid = f_t.fid(m);
            auto vs = m.get_face_vertices(f_t);
            auto vids = std::array<size_t, 3>{{vs[0].vid(m), vs[1].vid(m), vs[2].vid(m)}};
            std::sort(vids.begin(), vids.end());
            auto [it, suc] = changed_faces.emplace(vids, m_face_attribute[global_fid]);
            if (!suc) {
                changed_faces.erase(it); // erase if already there.
                middle_face.insert(global_fid);
            }
        }
    }

    for (const size_t f : middle_face) {
        if (m_face_attribute[f].m_is_surface_fs || m_face_attribute[f].m_is_bbox_fs >= 0) {
            wmtk::logger().debug("Attempting to Swap a boundary/bbox face, reject.");
            return false;
        }
    }
    return true;
}

void tracker_assign_after(
    const wmtk::TetMesh& m,
    const std::vector<size_t>& incident_tids,
    const std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces,
    TetWildMesh::FaceAttCol& m_face_attribute)
{
    auto middle_face = std::vector<size_t>();
    auto new_faces = std::set<std::array<size_t, 3>>();

    for (const auto& t : incident_tids) {
        for (auto j = 0; j < 4; j++) {
            auto f_t = m.tuple_from_face(t, j);
            auto global_fid = f_t.fid(m);
            auto vs = m.get_face_vertices(f_t);
            auto vids = std::array<size_t, 3>{{vs[0].vid(m), vs[1].vid(m), vs[2].vid(m)}};
            std::sort(vids.begin(), vids.end());
            auto it = (changed_faces.find(vids));
            if (it == changed_faces.end()) {
                middle_face.push_back(global_fid);
                continue;
            }


            m_face_attribute[global_fid] = it->second; // m_face_attribute[it->second];
        }
    }
    for (const size_t f : middle_face) {
        m_face_attribute[f].reset();
    }
}

void tracker_assign_after(
    const wmtk::TetMesh& m,
    const std::vector<wmtk::TetMesh::Tuple>& incident_tets,
    const std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces,
    TetWildMesh::FaceAttCol& m_face_attribute)
{
    std::vector<size_t> incident_tids;
    incident_tids.reserve(incident_tets.size());
    for (const wmtk::TetMesh::Tuple& t : incident_tets) {
        incident_tids.emplace_back(t.tid(m));
    }

    tracker_assign_after(m, incident_tids, changed_faces, m_face_attribute);
}


void TetWildMesh::swap_all_edges()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time serial: {}s", time);
    }
}

bool TetWildMesh::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) {
        return false;
    }

    if (is_edge_on_surface(t) || is_edge_on_bbox(t)) {
        return false;
    }
    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (const Tuple& l : incident_tets) {
        max_energy = std::max(m_tet_attribute[l.tid(*this)].m_quality, max_energy);
    }
    swap_cache.local().max_energy = max_energy;

    if (!face_attribute_tracker(
            *this,
            incident_tets,
            m_face_attribute,
            swap_cache.local().changed_faces))
        return false;

    return true;
}

bool TetWildMesh::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};
    auto max_energy = -1.0;
    for (auto& l : twotets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);
    }
    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(*this, twotets, swap_cache.local().changed_faces, m_face_attribute);
    cnt_swap++;

    return true;
}


void TetWildMesh::swap_all_faces()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("face_swap", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("face swap prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
        executor.priority = [](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_face_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("face swap operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("face swap operation time serial: {}s", time);
    }
}

bool TetWildMesh::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;
    // if (m_params.preserve_global_topology) return false;

    auto fid = t.fid(*this);
    if (m_face_attribute[fid].m_is_surface_fs || m_face_attribute[fid].m_is_bbox_fs >= 0) {
        return false;
    }
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    swap_cache.local().max_energy = std::max(
        m_tet_attribute[t.tid(*this)].m_quality,
        m_tet_attribute[oppo_tet->tid(*this)].m_quality);

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};

    if (!face_attribute_tracker(*this, twotets, m_face_attribute, swap_cache.local().changed_faces))
        return false;
    return true;
}

bool TetWildMesh::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);
    }
    wmtk::logger().trace("quality {} from {}", max_energy, swap_cache.local().max_energy);

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}


void TetWildMesh::swap_all_edges_44()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap_44", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap 44 prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 44 operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 44 operation time serial: {}s", time);
    }
}

bool TetWildMesh::swap_edge_44_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_before(t)) return false;
    // if (m_params.preserve_global_topology) return false;

    if (is_edge_on_surface(t) || is_edge_on_bbox(t)) return false;
    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(m_tet_attribute[l.tid(*this)].m_quality, max_energy);
    }
    swap_cache.local().max_energy = max_energy;

    if (!face_attribute_tracker(
            *this,
            incident_tets,
            m_face_attribute,
            swap_cache.local().changed_faces))
        return false;

    return true;
}

bool TetWildMesh::swap_edge_44_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);
    }

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

void TetWildMesh::swap_all_edges_56()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap_56", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap 56 prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 56 operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<TetWildMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 56 operation time serial: {}s", time);
    }
}

bool TetWildMesh::swap_edge_56_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_before(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tets_for_edge(t);
    if (incident_tets.size() != 5) {
        return false;
    }
    if (is_edge_on_surface(t) || is_edge_on_bbox(t)) {
        return false;
    }

    double max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(m_tet_attribute[l.tid(*this)].m_quality, max_energy);
    }
    swap_cache.local().max_energy = max_energy;

    if (!face_attribute_tracker(
            *this,
            incident_tets,
            m_face_attribute,
            swap_cache.local().changed_faces)) {
        return false;
    }

    return true;
}

bool TetWildMesh::swap_edge_56_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_after(t)) {
        return false;
    }

    const auto e1 = get_incident_tids_for_edge(t);
    const auto e2 = get_incident_tids_for_edge(t.switch_edge(*this));
    const auto tids = wmtk::set_union(e1, e2);

    double max_energy = -1.0;
    for (const size_t tid : tids) {
        const Tuple tet = tuple_from_tet(tid);
        if (is_inverted_f(tet)) {
            return false;
        }

        auto q = get_quality(tet);
        m_tet_attribute[tid].m_quality = q;
        max_energy = std::max(q, max_energy);
    }

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(*this, tids, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

} // namespace wmtk::components::tetwild