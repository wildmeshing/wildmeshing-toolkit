#include "HarmonicTet.hpp"

#include <igl/predicates/predicates.h>
#include <limits>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>

#include <queue>
#include <wmtk/ExecutionScheduler.hpp>
#include "wmtk/TetMesh.h"
#include "wmtk/utils/EnergyHarmonicTet.hpp"

namespace harmonic_tet {

double HarmonicTet::get_quality(const Tuple& loc) const
{
    Eigen::MatrixXd ps(4, 3);
    auto tups = oriented_tet_vertices(loc);
    for (auto j = 0; j < 4; j++) {
        ps.row(j) = m_vertex_attribute[tups[j].vid(*this)];
    }
    return wmtk::harmonic_energy(ps);
}

double HarmonicTet::get_quality(const std::array<size_t, 4>& vids) const
{
    Eigen::MatrixXd ps(4, 3);
    for (auto j = 0; j < 4; j++) {
        ps.row(j) = m_vertex_attribute[vids[j]];
    }
    return wmtk::harmonic_energy(ps);
}


bool HarmonicTet::is_inverted(const Tuple& loc)
{
    std::array<Eigen::Vector3d, 4> ps;
    auto tups = oriented_tet_vertices(loc);
    for (auto j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[tups[j].vid(*this)];
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
        auto local_tuples = oriented_tet_vertices(loc);
        std::array<size_t, 4> local_verts;
        for (auto i = 0; i < 4; i++) {
            local_verts[i] = local_tuples[i].vid(*this);
        }

        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (auto i = 0; i < 4; i++) {
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]][j];
            }
        }
        loc_id++;
    }

    auto old_pos = m_vertex_attribute[vid];
    m_vertex_attribute[vid] = wmtk::gradient_descent_from_stack(
        assembles,
        wmtk::harmonic_tet_energy,
        wmtk::harmonic_tet_jacobian);
    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].transpose());
    // note: duplicate code snippets.
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[vid] = old_pos;
            return false;
        }
    }

    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
    }
    return true;
}

bool HarmonicTet::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    auto total_energy = 0.;
    for (auto& l : incident_tets) {
        total_energy += (get_quality(l));
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
    auto total_energy = get_quality(t) + get_quality(*oppo_tet);
    wmtk::logger().debug("energy {} {}", edgeswap_cache.local().total_energy, total_energy);
    if (is_inverted(t) || is_inverted(*oppo_tet)) {
        wmtk::logger().debug(
            "invert w/ energy {} {}",
            edgeswap_cache.local().total_energy,
            total_energy);
        return false;
    }
    if (total_energy > edgeswap_cache.local().total_energy) return false;
    return true;
}

bool HarmonicTet::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);
    for (auto& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
    }
    auto total_energy = 0.;
    for (auto& l : incident_tets) {
        total_energy += get_quality(l);
    }
    wmtk::logger().trace("quality {} from {}", total_energy, faceswap_cache.local().total_energy);

    if (total_energy > faceswap_cache.local().total_energy) return false;
    return true;
}

auto renewal_edges = [](const auto& m, auto op, const auto& newt) {
    auto new_edges = std::vector<wmtk::TetMesh::Tuple>();
    assert(newt.switch_tetrahedron(m));
    for (auto ti : {newt.tid(m), newt.switch_tetrahedron(m)->tid(m)}) {
        for (auto j = 0; j < 6; j++) new_edges.push_back(m.tuple_from_edge(ti, j));
    };
    wmtk::unique_edge_tuples(m, new_edges);
    std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
    for (auto f : new_edges) op_tups.emplace_back("edge_swap", f);
    return op_tups;
};

void HarmonicTet::swap_all_edges(bool parallel)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);
    if (parallel) {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kPartition>();
        executor.renew_neighbor_tuples = renewal_edges;
        executor.lock_vertices = [](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
            auto stack = std::vector<size_t>();
            if (!m.try_set_edge_mutex_two_ring(e, stack)) return {};
            return stack;
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    } else {
        auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kSeq>();
        executor.renew_neighbor_tuples = renewal_edges;
        executor.num_threads = 1;
        executor(*this, collect_all_ops);
    }
}

void harmonic_tet::HarmonicTet::smooth_all_vertices()
{
    auto executor = wmtk::ExecutePass<HarmonicTet>();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) collect_all_ops.emplace_back("vertex_smooth", loc);
    executor(*this, collect_all_ops);
}

auto renewal_faces = [](const auto& m, auto op, const auto& t) {
    auto newt = t;
    auto new_tets = std::vector<size_t>(1, newt.tid(m));
    for (auto k = 0; k < 2; k++) {
        newt = newt.switch_face(m);
        newt = newt.switch_tetrahedron(m).value();
        new_tets.push_back(newt.tid(m));
    }

    auto new_faces = std::vector<wmtk::TetMesh::Tuple>();
    for (auto ti : new_tets) {
        for (auto j = 0; j < 4; j++) new_faces.push_back(m.tuple_from_face(ti, j));
    }
    wmtk::unique_face_tuples(m, new_faces);
    std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
    for (auto f : new_faces) op_tups.emplace_back("face_swap", f);
    return op_tups;
};

void HarmonicTet::swap_all_faces()
{
    auto executor = wmtk::ExecutePass<HarmonicTet>();
    executor.renew_neighbor_tuples = renewal_faces;
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_faces()) collect_all_ops.emplace_back("face_swap", loc);
    executor(*this, collect_all_ops);
};

bool HarmonicTet::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) return false;

    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    faceswap_cache.local().total_energy = (get_quality(t) + get_quality(*oppo_tet));
    return true;
}

void HarmonicTet::output_mesh(std::string file) const
{
    // warning: duplicate code.
    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i];
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
    auto new_edges = std::vector<wmtk::TetMesh::Tuple>();
    for (auto ti : {newt.tid(m), newt.switch_tetrahedron(m)->tid(m)}) {
        for (auto j = 0; j < 6; j++) new_edges.push_back(m.tuple_from_edge(ti, j));
    };
    wmtk::unique_edge_tuples(m, new_edges);
    std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
    for (auto f : new_edges) op_tups.emplace_back("edge_swap", f);
    return op_tups;
};

void HarmonicTet::swap_all()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);

    auto executor = wmtk::ExecutePass<HarmonicTet, wmtk::ExecutionPolicy::kPartition>();
    executor.renew_neighbor_tuples = renewal_all;
    executor.priority =
        [](auto& m, auto op, const wmtk::TetMesh::Tuple& t) -> double { // using delta trace
        if (op == "edge_swap") {
            auto tets = m.get_incident_tets_for_edge(t);
            if (tets.size() != 3) return -1.;
            auto before = 0.;
            for (auto e : tets) before += m.get_quality(e);
            auto after = 0.;

            std::vector<std::array<size_t, 4>> data(3);
            for (auto i = 0; i < 3; i++) {
                auto vs = m.oriented_tet_vertices(tets[i]);
                for (int j = 0; j < 4; j++) {
                    data[i][j] = vs[j].vid(m);
                }
            }
        } else if (op == "face_swap") {
            auto f1 = t.switch_tetrahedron(m);
            if (!f1) return -1.;
            auto before = m.get_quality(t) + m.get_quality(f1.value());
        }
        return 0.;


    };
    executor.lock_vertices = [](auto& m, const auto& e) -> std::optional<std::vector<size_t>> {
        auto stack = std::vector<size_t>();
        if (!m.try_set_edge_mutex_two_ring(e, stack)) return {};
        return stack;
    };
    executor.num_threads = NUM_THREADS;
    executor(*this, collect_all_ops);
};


} // namespace harmonic_tet