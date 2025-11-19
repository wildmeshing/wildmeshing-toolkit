#include "HarmonicTet.hpp"

#include <wmtk/TetMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/EnergyHarmonicTet.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>

#include <igl/predicates/predicates.h>

#include <limits>
#include <queue>

namespace harmonic_tet {

double HarmonicTet::get_quality(const Tuple& loc) const
{
    std::array<double, 12> T;
    auto tups = oriented_tet_vids(loc);
    return get_quality(tups);
}

double HarmonicTet::get_quality(const std::array<size_t, 4>& vids) const
{
    std::array<double, 12> T;
    for (auto j = 0; j < 4; j++) {
        auto& p = vertex_attrs[vids[j]].pos;
        for (auto k = 0; k < 3; k++) {
            T[j * 3 + k] = p[k];
        }
    }
    return wmtk::harmonic_tet_energy(T);
}


bool HarmonicTet::is_inverted(const Tuple& loc)
{
    std::array<Eigen::Vector3d, 4> ps;
    auto tups = oriented_tet_vids(loc);
    for (auto j = 0; j < 4; j++) {
        ps[j] = vertex_attrs[tups[j]].pos;
    }

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
    if (res == igl::predicates::Orientation::NEGATIVE) return false; // extremely annoying.
    return true;
}


bool harmonic_tet::HarmonicTet::smooth_after(const Tuple& t)
{
    wmtk::logger().trace("Gradient Descent iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tets_for_vertex(t);
    assert(locs.size() > 0);
    std::vector<std::array<double, 12>> assembles(locs.size());
    auto loc_id = 0;

    for (auto& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid(*this);

        assert(!is_inverted(loc));
        auto local_verts = oriented_tet_vids(loc);

        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (auto i = 0; i < 4; i++) {
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = vertex_attrs[local_verts[i]].pos[j];
            }
        }
        loc_id++;
    }

    auto old_pos = vertex_attrs[vid].pos;
    vertex_attrs[vid].pos = wmtk::gradient_descent_from_stack(
        assembles,
        wmtk::harmonic_tet_energy,
        wmtk::harmonic_tet_jacobian);
    if (vertex_attrs[vid].pos == old_pos) return false;
    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        vertex_attrs[vid].pos.transpose());
    // note: duplicate code snippets.
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            vertex_attrs[vid].pos = old_pos;
            return false;
        }
    }

    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
        tet_attrs[t_id].quality = get_quality(loc);
    }
    return true;
}

bool HarmonicTet::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    auto total_energy = 0.;
    for (auto& l : incident_tets) {
        total_energy += tet_attrs[l.tid(*this)].quality;
    }
    edgeswap_cache.local().total_energy = total_energy;
    return true;
}

bool HarmonicTet::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) return false;

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    auto q0 = get_quality(t);
    auto q1 = get_quality(*oppo_tet);
    auto total_energy = q0 + q1;
    wmtk::logger().trace("energy {} {}", edgeswap_cache.local().total_energy, total_energy);

    if (total_energy > edgeswap_cache.local().total_energy) return false;

    tet_attrs[t.tid(*this)].quality = q0;
    tet_attrs[oppo_tet->tid(*this)].quality = q1;
    return true;
}

bool HarmonicTet::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    auto total_energy = 0.;
    for (auto& l : incident_tets) {
        auto q = get_quality(l);
        tet_attrs[l.tid(*this)].quality = q;
        total_energy += q;
    }
    wmtk::logger().trace("quality {} from {}", total_energy, faceswap_cache.local().total_energy);

    if (total_energy > faceswap_cache.local().total_energy) return false;
    return true;
}


void harmonic_tet::HarmonicTet::smooth_all_vertices(bool interior_only)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        if (interior_only && !vertex_adjacent_boundary_faces(loc).empty()) continue;
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    wmtk::logger().debug("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    } else {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kSeq>();
        executor(*this, collect_all_ops);
    }
}

bool HarmonicTet::invariants(const std::vector<Tuple>& tets)
{
    for (auto& t : tets) {
        if (is_inverted(t)) return false;
    }
    return true;
}

void HarmonicTet::swap_all_faces(bool parallel)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_faces()) collect_all_ops.emplace_back("face_swap", loc);
    if (parallel) {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kPartition>();
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_face_mutex_two_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    } else {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kSeq>();
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
        executor.num_threads = 1;
        executor(*this, collect_all_ops);
    }
}

bool HarmonicTet::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.local().total_energy =
        tet_attrs[t.tid(*this)].quality + tet_attrs[oppo_tet->tid(*this)].quality;
    return true;
}

void HarmonicTet::output_mesh(std::string file) const
{
    // warning: duplicate code.
    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return vertex_attrs[i].pos;
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) { return i; });
    msh.add_tet_attribute<1>("t index", [&](size_t i) { return i; });

    msh.save(file, true);
}

auto renewal_all = [](const auto& m, auto op, const auto& newt) {
    auto optup1 = wmtk::renewal_edges(m, op, newt);
    auto optup2 = wmtk::renewal_faces(m, op, newt);
    for (auto& o : optup2) optup1.emplace_back(o);
    return optup1;
};


auto replace = [](auto& arr, auto i, auto j) {
    for (auto k = 0; k < arr.size(); k++) {
        if (arr[k] == i) {
            arr[k] = j;
            break;
        }
    }
};

constexpr auto swap_3_2 = [](const std::vector<std::array<size_t, 4>>& tets, auto v0, auto v1) {
    auto n0 = -1, n1 = -1, n2 = -1;
    // T0 : n1, n2, v0, *v1*->n0
    // T1: n0, n1, *v0*,v1 -> n2
    for (auto j = 0; j < 4; j++) {
        auto v = tets[0][j];
        if (v != v0 && v != v1) {
            if (n2 == -1)
                n2 = v;
            else if (n1 == -1)
                n1 = v;
        }
    }
    auto flag = false;
    for (auto j = 0; j < 4; j++) {
        auto v = tets[1][j];
        if (v != v0 && v != v1 && v != n1 && v != n2) n0 = v;
        if (v == n1) flag = true;
    }
    assert(n0 != n1 && n1 != n2);

    auto newtet = std::vector<std::array<size_t, 4>>{tets[0], tets[flag ? 1 : 2]};
    replace(newtet[0], v1, n0);
    replace(newtet[1], v0, n2);
    return newtet;
};
constexpr auto swap_2_3 = [](const std::vector<std::array<size_t, 4>>& tets,
                             std::array<size_t, 3> n) {
    auto u = std::array<int, 2>{{-1, -1}};
    // T0 : *n0*,n1,n2 v1-> v2
    // T0 : *n1*, v1-> v2
    // T0 : *n2*, v1-> v2
    for (auto i = 0; i < 2; i++) {
        for (auto j = 0; j < 4; j++) {
            auto v = tets[i][j];
            for (auto k = 0; k < 3; k++)
                if (v == n[k]) continue;
            u[i] = v;
            break;
        }
    }

    auto newtet = std::vector<std::array<size_t, 4>>{tets[0], tets[0], tets[0]};
    for (auto i = 0; i < 3; i++) replace(newtet[i], n[i], u[1]);
    return newtet;
};


auto compute_operation_gain =
    [](auto& m, auto op, const wmtk::TetMesh::Tuple& t) -> double { // using delta trace
    if (op == "edge_swap") {
        auto tets = m.get_incident_tets_for_edge(t);
        if (tets.size() != 3) return -1.;
        auto before = 0.;
        for (auto e : tets) before += m.tet_attrs[e.tid(m)].quality;
        auto after = 0.;

        std::vector<std::array<size_t, 4>> data(3);
        for (auto i = 0; i < 3; i++) {
            auto vs = m.oriented_tet_vids(tets[i]);
            for (int j = 0; j < 4; j++) {
                data[i][j] = vs[j];
            }
        }
        auto v0 = t.vid(m), v1 = t.switch_vertex(m).vid(m);
        auto new_conn = swap_3_2(data, v0, v1);
        for (auto tc : new_conn) {
            after += m.get_quality(tc);
        }
        return before - after; // decrease amount.
    } else if (op == "face_swap") {
        auto f1 = t.switch_tetrahedron(m);
        if (!f1) return -1.; // boundary
        auto tets = std::vector<wmtk::TetMesh::Tuple>{{t, f1.value()}};

        std::vector<std::array<size_t, 4>> data(2);
        for (auto i = 0; i < 2; i++) {
            auto vs = m.oriented_tet_vertices(tets[i]);
            for (int j = 0; j < 4; j++) {
                data[i][j] = vs[j].vid(m);
            }
        }
        auto v0 = t.vid(m), v1 = t.switch_vertex(m).vid(m);
        auto v2 = t.switch_edge(m).switch_vertex(m).vid(m);
        auto new_conn = swap_2_3(data, {{v0, v1, v2}});

        auto decrease = 0.;
        for (auto e : data) decrease += m.get_quality(e);
        for (auto tc : new_conn) decrease -= m.get_quality(tc);
        return decrease; // decrease amount.
    }
    return -1.;
};

int HarmonicTet::swap_all()
{
    auto suc = std::atomic<int>(0);
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    collect_all_ops.reserve(tet_capacity() * 4);
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);
    for (auto& loc : get_faces()) collect_all_ops.emplace_back("face_swap", loc);


    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [&](auto& m, auto op, auto& t) {
            suc++;
            return renewal_all(m, op, t);
        };
        executor.priority = compute_operation_gain;
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kSeq>();
        executor.num_threads = 1;
        setup_and_execute(executor);
    }
    return suc;
}


void HarmonicTet::swap_all_edges(bool parallel)
{
    if (NUM_THREADS == 0) parallel = false;
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();

    auto collect_tuples = tbb::concurrent_vector<Tuple>();
    for_each_edge([&](auto& tup) {
        if (compute_operation_gain(*this, std::string("edge_swap"), tup) > 0)
            collect_tuples.emplace_back(tup);
    });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [](const auto& m, auto op, const auto& newt) {
            std::set<size_t> eids;
            std::vector<size_t> twice;
            for (auto ti : newt) {
                for (auto j = 0; j < 6; j++) {
                    auto e_tup = m.tuple_from_edge(ti.tid(m), j);
                    auto eid = e_tup.eid(m);
                    auto [it, suc] = eids.insert(eid);
                    if (!suc) twice.push_back(eid);
                }
            }
            std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
            for (auto eid : twice) op_tups.emplace_back(op, m.tuple_from_edge(eid / 6, eid % 6));
            return op_tups;
        };
        executor(*this, collect_all_ops);
    };

    if (parallel) {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kPartition>();
        executor.num_threads = NUM_THREADS;
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        executor.priority = compute_operation_gain;
        executor.should_renew = [](auto val) { return val > 0; };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kSeq>();
        executor.num_threads = 1;
        setup_and_execute(executor);
    }
}


} // namespace harmonic_tet
