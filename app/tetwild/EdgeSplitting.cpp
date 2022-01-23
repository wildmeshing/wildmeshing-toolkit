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

    split_cache.local().is_edge_on_surface = is_edge_on_surface(loc0);

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
    vertex_attrs->m_attributes[v_id].m_posf =
        (vertex_attrs->m_attributes[v1_id].m_posf + vertex_attrs->m_attributes[v2_id].m_posf) / 2;
    vertex_attrs->m_attributes[v_id].m_is_rounded = true;

    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            vertex_attrs->m_attributes[v_id].m_is_rounded = false;
            break;
        }
    }
    if (!vertex_attrs->m_attributes[v_id].m_is_rounded) {
        vertex_attrs->m_attributes[v_id].m_pos =
            (vertex_attrs->m_attributes[v1_id].m_pos + vertex_attrs->m_attributes[v2_id].m_pos) / 2;
        vertex_attrs->m_attributes[v_id].m_posf = to_double(vertex_attrs->m_attributes[v_id].m_pos);
    } else
        vertex_attrs->m_attributes[v_id].m_pos = to_rational(vertex_attrs->m_attributes[v_id].m_posf);

    if (!tetrahedron_invariant(locs)) return false;
  
    /// update quality
    for (auto& loc : locs) {
        tet_attrs->m_attributes[loc.tid(*this)].m_qualities = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    vertex_attrs->m_attributes[v_id].on_bbox_faces = wmtk::set_intersection(
        vertex_attrs->m_attributes[v1_id].on_bbox_faces,
        vertex_attrs->m_attributes[v2_id].on_bbox_faces);
    //surface
    vertex_attrs->m_attributes[v_id].m_is_on_surface = split_cache.local().is_edge_on_surface;

    /// update face attribute
    // todo

    cnt_split++;

    return true;
}
