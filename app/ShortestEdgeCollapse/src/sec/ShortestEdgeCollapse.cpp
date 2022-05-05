#include "ShortestEdgeCollapse.h"
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace wmtk;
using namespace sec;


bool sec::ShortestEdgeCollapse::invariants(const std::vector<Tuple>& new_tris)
{
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = t.oriented_tri_vertices(*this);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            bool outside = m_envelope.is_outside(tris);
            if (outside) return false;
        }
    }
    return true;
}

bool sec::ShortestEdgeCollapse::write_triangle_mesh(std::string path)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = vs[j].vid(*this);
        }
    }

    return igl::write_triangle_mesh(path, V, F);
}

bool sec::ShortestEdgeCollapse::collapse_edge_before(const Tuple& t)
{
    if (!ConcurrentTriMesh::collapse_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].freeze || vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
        return false;
    position_cache.local().v1p = vertex_attrs[t.vid(*this)].pos;
    position_cache.local().v2p = vertex_attrs[t.switch_vertex(*this).vid(*this)].pos;
    return true;
}


bool sec::ShortestEdgeCollapse::collapse_edge_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = p;

    return true;
}


std::vector<TriMesh::Tuple> sec::ShortestEdgeCollapse::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool sec::ShortestEdgeCollapse::collapse_shortest(int target_vert_number)
{
    size_t initial_size = get_vertices().size();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    int starting_num = get_vertices().size();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

    auto renew = [](auto& m, auto op, auto& tris) {
        auto edges = m.new_edges_after(tris);
        auto optup = std::vector<std::pair<std::string, Tuple>>();
        for (auto& e : edges) optup.emplace_back("edge_collapse", e);
        return optup;
    };
    auto measure_len2 = [](auto& m, auto op, const Tuple& new_e) {
        auto len2 =
            (m.vertex_attrs[new_e.vid(m)].pos - m.vertex_attrs[new_e.switch_vertex(m).vid(m)].pos)
                .squaredNorm();
        return -len2;
    };
    auto setup_and_execute = [&](auto executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.stopping_criterion_checking_frequency =
            target_vert_number > 0 ? (initial_size - target_vert_number - 1)
                                   : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
    return true;
}