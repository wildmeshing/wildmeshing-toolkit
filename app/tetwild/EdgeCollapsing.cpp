#include "TetWild.h"
#include "wmtk/TetMesh.h"

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>


double tetwild::TetWild::get_length2(const wmtk::TetMesh::Tuple& l) const {
    auto &m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
};

void tetwild::TetWild::collapse_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_simple;
        executor.priority = [&](auto& m, auto op, auto& t) { return -m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.should_process = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != - weight) return false;
            if (length > m_params.collapsing_l2) return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
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

bool tetwild::TetWild::collapse_before(const Tuple& loc) // input is an edge
{
    //check if on bbox/surface/boundary
    // todo: store surface info into cache

    int v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    int v2_id = loc1.vid(*this);
    collapse_cache.local().edge_length =
        (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf)
            .norm(); // todo: duplicated computation

    auto n1_locs = get_one_ring_tets_for_vertex(loc);
    auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

    std::map<int, double> qs;
    for (auto& l : n1_locs) {
        qs[l.tid(*this)] = get_quality(l);
    }
    for (auto& l : n12_locs) {
        auto it = qs.find(l.tid(*this));
        if (it != qs.end()) qs.erase(it);
    }

    collapse_cache.local().max_energy = 0;
    for (auto& q : qs) {
        if (q.second > collapse_cache.local().max_energy) collapse_cache.local().max_energy = q.second;
    }

    return true;
}

bool tetwild::TetWild::collapse_after(const Tuple& loc)
{
    if (!TetMesh::collapse_after(loc)) return false;

    auto locs = get_one_ring_tets_for_vertex(loc);

    ////check first
    // check inversion
    for (auto& l : locs) {
        if (is_inverted(l)) {
            return false;
        }
    }

    // check quality
    std::vector<double> qs;
    for (auto& l : locs) {
        double q = get_quality(l);
        if (q > collapse_cache.local().max_energy) {
            // spdlog::critical("After Collapse {} from ({})", q, collapse_cache.local().max_energy);
            return false;
        }
        qs.push_back(q);
    }

    ////then update
    if (collapse_cache.local().edge_length > 0) {
        // todo: surface check
    } else {
        for (int i = 0; i < locs.size(); i++) {
            m_tet_attribute[locs[i].tid(*this)].m_qualities = qs[i];
        }
    }
    cnt_collapse ++;

    return true;
}
