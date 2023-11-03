#include "ShortestEdgeCollapse.h"
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/operations/TriMeshEdgeCollapseOperation.h>
#include <wmtk/operations/TriMeshOperationShim.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace wmtk;
using namespace app::sec;

namespace {
class ShortestEdgeCollapseOperation : public wmtk::TriMeshOperationShim<
                                          ShortestEdgeCollapse,
                                          ShortestEdgeCollapseOperation,
                                          wmtk::TriMeshEdgeCollapseOperation>
{
public:
    bool execute(ShortestEdgeCollapse& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeCollapseOperation::execute(m, t);
    }
    bool before(ShortestEdgeCollapse& m, const Tuple& t)
    {
        return wmtk::TriMeshEdgeCollapseOperation::before(m, t) && m.collapse_edge_before(t);
    }
    bool after(ShortestEdgeCollapse& m)
    {
        return wmtk::TriMeshEdgeCollapseOperation::after(m) &&
               m.collapse_edge_after(get_return_tuple_opt().value());
    }
};
} // namespace


std::map<std::string, std::shared_ptr<wmtk::TriMeshOperation>> ShortestEdgeCollapse::get_operations() const
{
    std::map<std::string, std::shared_ptr<wmtk::TriMeshOperation>> r;
    auto add_operation = [&](auto&& op) { r[op->name()] = op; };
    add_operation(std::make_shared<ShortestEdgeCollapseOperation>());
    return r;
}
ShortestEdgeCollapse::ShortestEdgeCollapse(
    std::vector<Eigen::Vector3d> _m_vertex_positions,
    int num_threads,
    bool use_exact_envelope)
{
    NUM_THREADS = (num_threads);
    m_envelope.use_exact = use_exact_envelope;
    p_vertex_attrs = &vertex_attrs;

    vertex_attrs.grow_to_at_least(_m_vertex_positions.size());

    for (auto i = 0; i < _m_vertex_positions.size(); i++)
        vertex_attrs[i] = {_m_vertex_positions[i], 0, false};
}

void ShortestEdgeCollapse::set_freeze(TriMesh::Tuple& v)
{
    for (auto e : get_one_ring_edges_for_vertex(v)) {
        if (is_boundary_edge(e)) {
            vertex_attrs[v.vid(*this)].freeze = true;
            continue;
        }
    }
}

void ShortestEdgeCollapse::create_mesh_nofreeze(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris)
{
    wmtk::TriMesh::create_mesh(n_vertices, tris);
}

void ShortestEdgeCollapse::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<size_t>& frozen_verts,
    double eps)
{
    wmtk::TriMesh::create_mesh(n_vertices, tris);

    if (eps > 0) {
        std::vector<Eigen::Vector3d> V(n_vertices);
        std::vector<Eigen::Vector3i> F(tris.size());
        for (auto i = 0; i < V.size(); i++) {
            V[i] = vertex_attrs[i].pos;
        }
        for (int i = 0; i < F.size(); ++i) F[i] << tris[i][0], tris[i][1], tris[i][2];
        m_envelope.init(V, F, eps);
        m_has_envelope = true;
    }
    partition_mesh();
    for (auto v : frozen_verts) vertex_attrs[v].freeze = true;
    for (auto v : get_vertices()) { // the better way is to iterate through edges.
        set_freeze(v);
    }
}

void ShortestEdgeCollapse::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        vertex_attrs[i].partition_id = m_vertex_partition_id[i];
}


bool ShortestEdgeCollapse::invariants(const wmtk::TriMeshOperation& op)
{
    if (m_has_envelope) {
        for (auto& t : op.modified_triangles(*this)) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            bool outside = m_envelope.is_outside(tris);
            if (outside) return false;
        }
    }
    return true;
}

bool ShortestEdgeCollapse::write_triangle_mesh(std::string path)
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

bool ShortestEdgeCollapse::collapse_edge_before(const Tuple& t)
{
    if (vertex_attrs[t.vid(*this)].freeze || vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
        return false;
    position_cache.local().v1p = vertex_attrs[t.vid(*this)].pos;
    position_cache.local().v2p = vertex_attrs[t.switch_vertex(*this).vid(*this)].pos;
    return true;
}


bool ShortestEdgeCollapse::collapse_edge_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = p;

    return true;
}


std::vector<TriMesh::Tuple> ShortestEdgeCollapse::new_edges_after(
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

bool ShortestEdgeCollapse::collapse_shortest(int target_vert_number)
{
    size_t initial_size = get_vertices().size();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
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
    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        spdlog::info("Num threads: {}", executor.num_threads);
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.stopping_criterion_checking_frequency =
            target_vert_number > 0 ? (initial_size - target_vert_number - 1)
                                   : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
        spdlog::info("Calling consolidate");
        executor(*this, {{"consolidate", {}}});
    };

    if (NUM_THREADS > 0) {
        wmtk::ExecutePass<ShortestEdgeCollapse, ExecutionPolicy::kPartition> executor(static_cast<const ShortestEdgeCollapse&>(*this));

        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        wmtk::ExecutePass<ShortestEdgeCollapse, ExecutionPolicy::kSeq> executor(static_cast<const ShortestEdgeCollapse&>(*this));
        setup_and_execute(executor);
    }
    return true;
}
