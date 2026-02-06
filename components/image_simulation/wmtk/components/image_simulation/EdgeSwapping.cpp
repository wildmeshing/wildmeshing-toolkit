#include "ImageSimulationMesh.h"

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include "spdlog/spdlog.h"
#include "wmtk/utils/TupleUtils.hpp"

#include <cassert>

namespace wmtk::components::image_simulation {

void face_attribute_tracker(
    const TetMesh& m,
    const std::vector<size_t>& incident_tets,
    const ImageSimulationMesh::FaceAttCol& m_face_attribute,
    std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces)
{
    changed_faces.clear();
    for (const auto& t : incident_tets) {
        for (int j = 0; j < 4; j++) {
            const TetMesh::Tuple f_t = m.tuple_from_face(t, j);
            auto vids = m.get_face_vids(f_t);
            std::sort(vids.begin(), vids.end());
            changed_faces.try_emplace(vids, m_face_attribute[f_t.fid(m)]);
        }
    }
}

void tracker_assign_after(
    const wmtk::TetMesh& m,
    const std::vector<size_t>& incident_tids,
    const std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces,
    ImageSimulationMesh::FaceAttCol& m_face_attribute)
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
    ImageSimulationMesh::FaceAttCol& m_face_attribute)
{
    std::vector<size_t> incident_tids;
    incident_tids.reserve(incident_tets.size());
    for (const wmtk::TetMesh::Tuple& t : incident_tets) {
        incident_tids.emplace_back(t.tid(m));
    }

    tracker_assign_after(m, incident_tids, changed_faces, m_face_attribute);
}


size_t ImageSimulationMesh::swap_all_edges_32()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time parallel: {:.4}s", time);
        return executor.get_cnt_success();
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time serial: {:.4}s", time);
        return executor.get_cnt_success();
    }
}

bool ImageSimulationMesh::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) {
        return false;
    }

    auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 3) {
        return false;
    }

    if (is_edge_on_surface(t) || is_edge_on_bbox(t)) {
        return false;
    }

    const auto& TA = m_tet_attribute;
    auto& cache = swap_cache.local();

    cache.tet_tags = TA[incident_tets[0]].tags;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
        if (TA[l].tags != cache.tet_tags) {
            log_and_throw_error("not all tets have the same tag"); // for debugging
        }
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(
        *this,
        incident_tets,
        m_face_attribute,
        swap_cache.local().changed_faces);

    return true;
}

bool ImageSimulationMesh::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const auto& cache = swap_cache.local();

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};
    auto max_energy = -1.0;
    for (auto& l : twotets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[l.tid(*this)].tags = cache.tet_tags;
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    tracker_assign_after(*this, twotets, cache.changed_faces, m_face_attribute);
    cnt_swap++;

    return true;
}

size_t ImageSimulationMesh::swap_all_faces()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_faces()) {
        collect_all_ops.emplace_back("face_swap", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("face swap prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
        executor.priority = [](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_face_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("face swap operation time parallel: {:.4}s", time);
        return executor.get_cnt_success();
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("face swap operation time serial: {:.4}s", time);
        return executor.get_cnt_success();
    }
}

bool ImageSimulationMesh::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) {
        return false;
    }
    // if (m_params.preserve_global_topology) return false;

    auto& cache = swap_cache.local();

    const SmartTuple tt(*this, t);

    auto fid = tt.fid();
    if (m_face_attribute[fid].m_is_surface_fs || m_face_attribute[fid].m_is_bbox_fs >= 0) {
        return false;
    }
    auto oppo_tet = tt.switch_tetrahedron();
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const size_t t0 = tt.tid();
    const size_t t1 = oppo_tet.value().tid();

    const auto& TA = m_tet_attribute;

    const double max_energy = std::max(TA[t0].m_quality, TA[t1].m_quality);

    // pre-compute energy
    {
        const auto t1_vids = oriented_tet_vids(t1);

        const size_t v0 = tt.vid();
        const size_t v1 = tt.switch_vertex().vid();
        const size_t v2 = tt.switch_edge().switch_vertex().vid();
        const size_t v3 = tt.switch_face().switch_edge().switch_vertex().vid();

        std::array<size_t, 3> tri{v0, v1, v2};

        for (auto i = 0; i < 3; i++) {
            std::array<size_t, 4> new_tet = t1_vids;
            wmtk::array_replace_inline(new_tet, tri[i], v3);
            if (is_inverted(new_tet)) {
                return false;
            }
            const double q = get_quality(new_tet);
            if (q >= max_energy) {
                return false;
            }
        }
    }

    cache.tet_tags = TA[tt.tid()].tags;
    if (TA[oppo_tet.value().tid()].tags != cache.tet_tags) {
        log_and_throw_error("not all tets have the same tag"); // for debugging
    }

    std::vector<size_t> twotets{t0, t1};

    face_attribute_tracker(*this, twotets, m_face_attribute, cache.changed_faces);
    return true;
}

bool ImageSimulationMesh::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    for (auto& l : incident_tets) {
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        m_tet_attribute[l.tid(*this)].tags = swap_cache.local().tet_tags;
    }

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

size_t ImageSimulationMesh::swap_all_edges_all()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap", loc);
        collect_all_ops.emplace_back("edge_swap_44", loc);
        collect_all_ops.emplace_back("edge_swap_56", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        // executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.renew_neighbor_tuples =
            [](const TetMesh& m, const std::string& op, const std::vector<Tuple>& newt) {
                std::vector<std::pair<std::string, TetMesh::Tuple>> op_tups;
                std::vector<TetMesh::Tuple> new_edges;
                for (const TetMesh::Tuple& ti : newt) {
                    for (auto j = 0; j < 6; j++) {
                        new_edges.push_back(m.tuple_from_edge(ti.tid(m), j));
                    }
                };
                wmtk::unique_edge_tuples(m, new_edges);
                op_tups.reserve(new_edges.size() * 3);
                for (const Tuple& loc : new_edges) {
                    op_tups.emplace_back("edge_swap", loc);
                    op_tups.emplace_back("edge_swap_44", loc);
                    op_tups.emplace_back("edge_swap_56", loc);
                }
                return op_tups;
            };
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time parallel: {:.4}s", time);
        return executor.get_cnt_success();
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time serial: {:.4}s", time);
        return executor.get_cnt_success();
    }
}


size_t ImageSimulationMesh::swap_all_edges_44()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap_44", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap 44 prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 44 operation time parallel: {:.4}s", time);
        return executor.get_cnt_success();
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 44 operation time serial: {:.4}s", time);
        return executor.get_cnt_success();
    }
}

bool ImageSimulationMesh::swap_edge_44_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_before(t)) {
        return false;
    }
    // if (m_params.preserve_global_topology) return false;

    auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 4) {
        return false;
    }

    if (is_edge_on_surface(t) || is_edge_on_bbox(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    const auto& TA = m_tet_attribute;

    cache.tet_tags = TA[incident_tets[0]].tags;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
        if (TA[l].tags != cache.tet_tags) {
            log_and_throw_error(
                "not all tets have the same tags. {} != {}",
                cache.tet_tags,
                TA[l].tags); // for debugging
        }
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

double ImageSimulationMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

bool ImageSimulationMesh::swap_edge_44_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_after(t)) return false;

    const auto incident_tets = get_incident_tets_for_edge(t);

    double max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[l.tid(*this)].tags = swap_cache.local().tet_tags;
    }

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

size_t ImageSimulationMesh::swap_all_edges_56()
{
    igl::Timer timer;
    double time;
    timer.start();
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_swap_56", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap 56 prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 56 operation time parallel: {:.4}s", time);
        return executor.get_cnt_success();
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<ImageSimulationMesh, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 56 operation time serial: {:.4}s", time);
        return executor.get_cnt_success();
    }
}

bool ImageSimulationMesh::swap_edge_56_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_before(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 5) {
        return false;
    }
    if (is_edge_on_surface(t) || is_edge_on_bbox(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    const auto& TA = m_tet_attribute;

    cache.tet_tags = TA[incident_tets[0]].tags;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
        if (TA[l].tags != cache.tet_tags) {
            log_and_throw_error(
                "not all tets have the same tags. {} != {}",
                cache.tet_tags,
                TA[l].tags); // for debugging
        }
    }

    face_attribute_tracker(
        *this,
        incident_tets,
        m_face_attribute,
        swap_cache.local().changed_faces);

    return true;
}

double ImageSimulationMesh::swap_edge_56_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

bool ImageSimulationMesh::swap_edge_56_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_after(t)) {
        return false;
    }

    /**
     * There is no need to check for inversion or energy here. The operation would have been
     * rejected already due to `swap_edge_56_energy()`.
     */

    const auto e1 = get_incident_tids_for_edge(t);
    const auto e2 = get_incident_tids_for_edge(t.switch_edge(*this));
    const auto tids = wmtk::set_union(e1, e2);

    double max_energy = -1.0;
    for (const size_t tid : tids) {
        const Tuple tet = tuple_from_tet(tid);
        if (is_inverted(tet)) return false;
        auto q = get_quality(tet);
        m_tet_attribute[tid].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[tid].tags = swap_cache.local().tet_tags;
    }

    tracker_assign_after(*this, tids, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

} // namespace wmtk::components::image_simulation