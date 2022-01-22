#include "TetWild.h"

#include <wmtk/TetMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/Logger.hpp>
#include "wmtk/utils/TupleUtils.hpp"
#include <wmtk/utils/ExecutorUtils.hpp>

#include <cassert>

void tetwild::TetWild::swap_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) {
            return m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
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

void tetwild::TetWild::swap_all_faces()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("face_swap", loc);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
        executor.lock_vertices = [](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
            auto stack = std::vector<size_t>();
            if (!m.try_set_face_mutex_two_ring(e, stack)) return {};
            return stack;
        };
        executor.priority = [](auto& m, auto op, auto& t) {
            return m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
    };
    if (NUM_THREADS > 1) {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TetWild, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}


bool tetwild::TetWild::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    edgeswap_cache.local().max_energy = max_energy;
    return true;
}

bool tetwild::TetWild::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    auto max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    std::vector<Tuple> locs{{t, *oppo_tet}};
    if (!tetrahedron_invariant(locs)) return false;
    
    if (max_energy > edgeswap_cache.local().max_energy) return false;
    cnt_swap ++;
    return true;
}

bool tetwild::TetWild::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.local().max_energy = std::max(get_quality(t), get_quality(*oppo_tet));
    return true;
}

bool tetwild::TetWild::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    if (!tetrahedron_invariant(incident_tets)) return false;
    
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    wmtk::logger().trace("quality {} from {}", max_energy, edgeswap_cache.local().max_energy);

    if (max_energy > edgeswap_cache.local().max_energy) return false;

    cnt_swap ++;
    return true;
}
