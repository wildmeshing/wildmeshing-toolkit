#include "TetWild.h"

#include <wmtk/TetMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/Logger.hpp>
#include "wmtk/utils/TupleUtils.hpp"
#include <wmtk/utils/ExecutorUtils.hpp>

#include <cassert>

auto face_attribute_tracker =
    [](auto& changed_faces, const auto& incident_tets, auto& m, auto& m_face_attribute) {
        changed_faces.clear();
        auto middle_face = std::set<int>();
        for (auto t : incident_tets) {
            for (auto j = 0; j < 4; j++) {
                auto f_t = m.tuple_from_face(t.tid(m), j);
                auto global_fid = f_t.fid(m);
                auto vs = m.get_face_vertices(f_t);
                auto vids = std::array<size_t, 3>{{vs[0].vid(m), vs[1].vid(m), vs[2].vid(m)}};
                std::sort(vids.begin(), vids.end());
                auto [it, suc] = changed_faces.emplace(vids, global_fid);
                if (!suc) {
                    changed_faces.erase(it); // erase if already there.
                    middle_face.insert(global_fid);
                }
            }
        }

        for (auto f : middle_face) {
            if (m_face_attribute[f].m_is_surface_fs || m_face_attribute[f].m_is_bbox_fs >= 0) {
                wmtk::logger().debug("Attempting to Swap a boundary/bbox face, reject.");
                return false;
            }
        }
        return true;
    };

auto tracker_assign_after =
    [](const auto& changed_faces, const auto& incident_tets, auto& m, auto& m_face_attribute) {
        auto middle_face = std::vector<size_t>();
        auto new_faces = std::set<std::array<size_t, 3>>();
        auto assign_attrs = [](auto& attr, auto from, auto to) { // TODO: support rollback later.
            if (from == to) return;
            attr[to] = attr[from];
        };
        for (auto t : incident_tets) {
            for (auto j = 0; j < 4; j++) {
                auto f_t = m.tuple_from_face(t.tid(m), j);
                auto global_fid = f_t.fid(m);
                auto vs = m.get_face_vertices(f_t);
                auto vids = std::array<size_t, 3>{{vs[0].vid(m), vs[1].vid(m), vs[2].vid(m)}};
                std::sort(vids.begin(), vids.end());
                auto it = (changed_faces.find(vids));
                if (it == changed_faces.end()) continue;

                // if found, then assign
                assign_attrs(m_face_attribute, it->second, global_fid);
            }
        }
    };

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

    if (!face_attribute_tracker(
            edgeswap_cache.local().changed_faces,
            incident_tets,
            *this,
            m_face_attribute))
        return false;

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
    
    if (max_energy > edgeswap_cache.local().max_energy) return false;

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};
    tracker_assign_after(faceswap_cache.local().changed_faces, twotets, *this, m_face_attribute);
    cnt_swap ++;
    return true;
}

bool tetwild::TetWild::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.local().max_energy = std::max(get_quality(t), get_quality(*oppo_tet));

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};

    if (!face_attribute_tracker(
            faceswap_cache.local().changed_faces,
            twotets,
            *this,
            m_face_attribute))
        return false;
    return true;
}

bool tetwild::TetWild::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(get_quality(l), max_energy);
    }
    wmtk::logger().trace("quality {} from {}", max_energy, faceswap_cache.local().max_energy);

    if (max_energy > faceswap_cache.local().max_energy) return false;

    tracker_assign_after(
        faceswap_cache.local().changed_faces,
        incident_tets,
        *this,
        m_face_attribute);

    cnt_swap ++;
    return true;
}
