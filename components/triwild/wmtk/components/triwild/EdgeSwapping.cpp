#include "TriWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/TriMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include "wmtk/utils/TupleUtils.hpp"

#include <cassert>

namespace wmtk::components::triwild {

auto renew = [](const TriWildMesh& m, auto op, auto& tris) {
    using Tuple = TriMesh::Tuple;
    std::vector<Tuple> edges;
    for (const auto& t : tris) {
        for (auto j = 0; j < 3; j++) {
            edges.push_back(m.tuple_from_edge(t.fid(m), j));
        }
    }
    wmtk::unique_edge_tuples(m, edges);

    std::vector<std::pair<std::string, Tuple>> optup;
    optup.reserve(edges.size());
    for (const Tuple& e : edges) {
        optup.emplace_back(op, e);
    }
    return optup;
};

auto edge_locker = [](auto& m, const auto& e, int task_id) {
    // TODO: this should not be here
    return m.try_set_edge_mutex_two_ring(e, task_id);
};

size_t TriWildMesh::swap_all_edges()
{
    std::vector<std::pair<std::string, Tuple>> collect_all_ops;
    {
        igl::Timer timer;
        timer.start();
        const auto edges = get_edges();
        collect_all_ops.reserve(edges.size());
        for (const Tuple& t : edges) {
            collect_all_ops.emplace_back("edge_swap", t);
        }
        timer.stop();
        logger().info("edge collapse prepare time: {:.4}s", timer.getElapsedTimeInSec());
    }
    logger().info("#E = {}", collect_all_ops.size());

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = renew;
        executor.num_threads = NUM_THREADS;
        executor.priority = [](const TriWildMesh& m, std::string op, const Tuple& e) {
            return m.swap_weight(e);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](const TriWildMesh& m, auto& ele) {
            auto& [val, _, e] = ele;
            const double w = m.swap_weight(e);
            return (w > 1e-5) && ((w - val) * (w - val) < 1e-8);
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = ExecutePass<TriWildMesh>(ExecutionPolicy::kPartition);
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = ExecutePass<TriWildMesh>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }

    return true;
}

double TriWildMesh::swap_weight(const Tuple& t) const
{
    const SmartTuple tt(*this, t);
    const auto t_opp = tt.switch_face();
    if (!t_opp) {
        return std::numeric_limits<double>::lowest();
    }

    if (is_edge_on_surface(t)) {
        return std::numeric_limits<double>::lowest();
    }

    const size_t v0 = tt.vid();
    const size_t v1 = tt.switch_vertex().vid();
    const size_t v2 = tt.switch_edge().switch_vertex().vid();
    const size_t v3 = t_opp.value().switch_edge().switch_vertex().vid();

    // before swap
    const double q012 = get_quality({{v0, v1, v2}});
    const double q031 = get_quality({{v0, v3, v1}});
    // after swap
    const double q032 = get_quality({{v0, v3, v2}});
    const double q231 = get_quality({{v2, v3, v1}});

    const double q_before = std::max(q012, q031);
    const double q_after = std::max(q032, q231);

    return q_before - q_after;
}

bool TriWildMesh::swap_edge_before(const Tuple& t)
{
    if (is_edge_on_surface(t)) {
        return false;
    }

    const auto& FA = m_face_attribute;
    auto& cache = swap_cache.local();
    cache.changed_edges.clear();

    const auto incident_faces = get_incident_fids_for_edge(t);

    cache.face_tags = FA[incident_faces[0]].tags;

    double max_energy = -1.0;
    for (const size_t fid : incident_faces) {
        max_energy = std::max(FA[fid].m_quality, max_energy);
    }
    cache.max_energy = max_energy;

    // cache edges
    simplex::Edge edge = simplex_from_edge(t);
    for (const size_t fid : incident_faces) {
        for (int j = 0; j < 3; j++) {
            const Tuple tup = tuple_from_edge(fid, j);
            simplex::Edge e = simplex_from_edge(tup);
            if (e == edge) {
                continue;
            }
            cache.changed_edges.try_emplace(e, m_edge_attribute[tup.eid(*this)]);
        }
    }

    return true;
}

bool TriWildMesh::swap_edge_after(const Tuple& t)
{
    auto& cache = swap_cache.local();
    const auto incident_faces = get_incident_fids_for_edge(t);

    auto& FA = m_face_attribute;

    double max_energy = -1.0;
    for (const size_t fid : incident_faces) {
        if (is_inverted(fid)) {
            return false;
        }
        double q = get_quality(fid);
        FA[fid].m_quality = q;
        max_energy = std::max(q, max_energy);

        FA[fid].tags = cache.face_tags;
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    // cached edges
    for (const auto& [e, e_attrs] : cache.changed_edges) {
        const auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attrs;
    }
    m_edge_attribute[t.eid(*this)].reset();

    return true;
}


} // namespace wmtk::components::triwild