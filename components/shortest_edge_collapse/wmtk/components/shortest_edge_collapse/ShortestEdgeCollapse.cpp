#include "ShortestEdgeCollapse.h"
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <paraviewo/VTUWriter.hpp>

namespace wmtk::components::shortest_edge_collapse {

ShortestEdgeCollapse::ShortestEdgeCollapse(
    std::vector<Eigen::Vector3d> _m_vertex_positions,
    int num_threads,
    bool use_exact_envelope)
{
    NUM_THREADS = (num_threads);
    m_envelope.use_exact = use_exact_envelope;
    p_vertex_attrs = &vertex_attrs;

    vertex_attrs.resize(_m_vertex_positions.size());

    for (auto i = 0; i < _m_vertex_positions.size(); i++)
        vertex_attrs[i] = {_m_vertex_positions[i], 0, false};
}

void ShortestEdgeCollapse::freeze_boundary()
{
    for (const Tuple& e : get_edges()) {
        if (is_boundary_edge(e)) {
            vertex_attrs[e.vid(*this)].freeze = true;
            vertex_attrs[e.switch_vertex(*this).vid(*this)].freeze = true;
        }
    }
}

void ShortestEdgeCollapse::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<size_t>& frozen_verts,
    double eps,
    bool freeze_bnd)
{
    wmtk::TriMesh::init(n_vertices, tris);

    if (eps > 0) {
        std::vector<Eigen::Vector3d> V(n_vertices);
        std::vector<Eigen::Vector3i> F(tris.size());
        for (size_t i = 0; i < V.size(); i++) {
            V[i] = vertex_attrs[i].pos;
        }
        for (size_t i = 0; i < F.size(); ++i) {
            F[i] << (int)tris[i][0], (int)tris[i][1], (int)tris[i][2];
        }
        m_envelope.init(V, F, eps);
        m_has_envelope = true;

        // Only needed when the boundary is allowed to move; a frozen boundary cannot
        // leave its own envelope, so building the BVH for it would be wasted work.
        if (!freeze_bnd) {
            std::vector<Eigen::Vector2i> boundary_edges;
            for (const Tuple& e : get_edges()) {
                if (is_boundary_edge(e)) {
                    boundary_edges.emplace_back(
                        (int)e.vid(*this),
                        (int)e.switch_vertex(*this).vid(*this));
                }
            }
            if (!boundary_edges.empty()) {
                m_boundary_envelope.init(V, boundary_edges, eps);
                m_has_boundary_envelope = true;

                // Pin the ends and junctions of the boundary curves. The envelope keeps the
                // boundary from leaving the input outline but not from sliding along it, so
                // an open boundary chain would otherwise be free to retract from its ends.
                // Boundary valence != 2 is the same endpoint/junction test simplify_segments
                // uses in 2D.
                //
                // On a manifold surface the boundary is a union of closed loops and every
                // boundary vertex has valence exactly 2, so this freezes nothing; a closed
                // loop cannot retract along itself in any case, only shrink, which the
                // envelope already catches. It matters for input that is not manifold along
                // its boundary.
                std::vector<int> bnd_valence(n_vertices, 0);
                for (const Eigen::Vector2i& be : boundary_edges) {
                    bnd_valence[be[0]]++;
                    bnd_valence[be[1]]++;
                }
                size_t n_pinned = 0;
                for (size_t v = 0; v < n_vertices; v++) {
                    if (bnd_valence[v] != 0 && bnd_valence[v] != 2) {
                        vertex_attrs[v].freeze = true;
                        n_pinned++;
                    }
                }
                logger().info(
                    "boundary envelope: {} edges, {} boundary ends/junctions pinned",
                    boundary_edges.size(),
                    n_pinned);
            }
        }
    }
    partition_mesh();
    for (size_t v : frozen_verts) {
        vertex_attrs[v].freeze = true;
    }
    if (freeze_bnd) {
        freeze_boundary();
    }
}

void ShortestEdgeCollapse::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        vertex_attrs[i].partition_id = m_vertex_partition_id[i];
}


bool ShortestEdgeCollapse::invariants(const std::vector<Tuple>& new_tris)
{
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            bool outside = m_envelope.is_outside(tris);
            if (outside) return false;
        }
    }
    // The surface envelope contains the boundary but says nothing about where along the
    // surface it sits, so an unfrozen boundary needs its own check against the input
    // outline -- otherwise it is free to retract inwards without ever leaving the surface
    // envelope. This mirrors is_open_boundary_edge in tetwild.
    if (m_has_boundary_envelope) {
        for (auto& t : new_tris) {
            for (int j = 0; j < 3; j++) {
                const Tuple e = tuple_from_edge(t.fid(*this), j);
                if (!is_boundary_edge(e)) continue;
                const size_t v1 = e.vid(*this);
                const size_t v2 = e.switch_vertex(*this).vid(*this);
                if (m_boundary_envelope.is_outside(
                        std::array<Eigen::Vector3d, 2>{
                            {vertex_attrs[v1].pos, vertex_attrs[v2].pos}})) {
                    return false;
                }
            }
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
            F(i, j) = (int)vs[j].vid(*this);
        }
    }

    logger().info("Write {}", path);
    return igl::write_triangle_mesh(path, V, F);
}

void ShortestEdgeCollapse::write_vtu(const std::string& path)
{
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);

    MatrixXd V = MatrixXd::Zero(vert_capacity(), 3);
    MatrixXi F = MatrixXi::Zero(tri_capacity(), 3);

    VectorXd freeze(vertex_attrs.size());
    freeze.setZero();

    for (Tuple& t : get_vertices()) {
        const size_t i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
        freeze(i) = vertex_attrs[i].freeze ? 1 : 0;
    }

    for (Tuple& t : get_faces()) {
        const size_t i = t.fid(*this);
        const auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = (int)vs[j].vid(*this);
        }
    }

    paraviewo::VTUWriter writer;
    writer.add_field("freeze", freeze);
    writer.write_mesh(out_path, V, F, paraviewo::CellType::Triangle);
}

bool ShortestEdgeCollapse::collapse_edge_before(const Tuple& t)
{
    // TriMesh::collapse_edge_before is the classical link condition, and it is applied
    // unconditionally -- it is not gated on preserve_topology.
    //
    // It does stop the simplification from changing the surface topology, so on the face of
    // it it belongs under that flag. But it is not only a topology guard: it is the only
    // check standing between collapse_edge_conn and a connectivity that wmtk::TriMesh
    // cannot represent, since nothing downstream re-checks it (invariants() only tests the
    // envelope). Relaxing it does not produce a valid mesh with different topology, it
    // produces a corrupt one.
    //
    // TODO: the simplification therefore still preserves topology even when
    // preserve_topology is off, because the toolkit does not support non-manifold meshes.
    // Lifting this needs a mesh data structure that can represent them.
    if (!TriMesh::collapse_edge_before(t)) return false;

    // v1 is removed by the collapse, v2 survives (TriMesh::collapse_edge_conn keeps vid2).
    const size_t v1 = t.vid(*this);
    const size_t v2 = t.switch_vertex(*this).vid(*this);
    auto& cache = position_cache.local();
    cache.v1_frozen = vertex_attrs[v1].freeze;
    cache.v2_frozen = vertex_attrs[v2].freeze;

    // Two frozen endpoints cannot be merged without moving one of them. In particular this
    // is what keeps the outline of an open surface from retracting: the envelope is a
    // containment test, so it would not notice a boundary sliding inwards along itself.
    if (cache.v1_frozen && cache.v2_frozen) {
        return false;
    }

    cache.v1p = vertex_attrs[v1].pos;
    cache.v2p = vertex_attrs[v2].pos;
    return true;
}


bool ShortestEdgeCollapse::collapse_edge_after(const TriMesh::Tuple& t)
{
    const auto& cache = position_cache.local();
    // Collapse onto the frozen endpoint when exactly one is frozen, so its position is
    // preserved exactly and the collapse is still allowed; onto the midpoint otherwise.
    // Rejecting these outright, as this used to, froze not just the boundary but every
    // vertex adjacent to it.
    const Eigen::Vector3d p = cache.v1_frozen   ? cache.v1p
                              : cache.v2_frozen ? cache.v2p
                                                : (cache.v1p + cache.v2p) / 2.0;
    const size_t vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    // The survivor now stands exactly where the frozen vertex stood, so it takes over its
    // frozen role -- otherwise a later collapse could move that position after all.
    vertex_attrs[vid].freeze = cache.v1_frozen || cache.v2_frozen;

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
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.stopping_criterion_checking_frequency =
            target_vert_number > 0 ? (initial_size - target_vert_number - 1)
                                   : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }
    return true;
}

} // namespace wmtk::components::shortest_edge_collapse