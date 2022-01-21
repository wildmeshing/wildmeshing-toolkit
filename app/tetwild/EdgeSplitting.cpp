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
        executor.lock_vertices = [](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
            auto stack = std::vector<size_t>();
            if (!m.try_set_edge_mutex_two_ring(e, stack)) return {};
            return stack;
        };
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.should_process = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) return false;
            if (length < m_params.splitting_l2) return false;
            return true;
        };
    };
    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}

bool tetwild::TetWild::split_before(const Tuple& loc0)
{
    auto loc1 = loc0;
    int v1_id = loc1.vid(*this);
    auto loc2 = loc1.switch_vertex(*this);
    int v2_id = loc2.vid(*this);

    //	double length = (m_vertex_attribute[v1_id].m_posf -
    // m_vertex_attribute[v2_id].m_posf).norm();
    //	if (length < m_params.l * 4 / 3)
    //		return false;

    split_cache.local().vertex_info.m_posf =
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;

    return true;
}

bool tetwild::TetWild::split_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TetMesh::split_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);

    int v_id = loc.vid(*this);
    auto old_pos = m_vertex_attribute[v_id].m_posf;
    m_vertex_attribute[v_id].m_posf = split_cache.local().vertex_info.m_posf;

    // check inversion
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[v_id].m_posf = old_pos;
            return false;
        }
    }

    // update quality
    for (auto& loc : locs) {
        m_tet_attribute[loc.tid(*this)].m_qualities = get_quality(loc);
    }

    cnt_split ++;

    return true;
}
