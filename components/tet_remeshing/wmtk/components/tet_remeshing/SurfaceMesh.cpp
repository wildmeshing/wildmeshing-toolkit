#include "SurfaceMesh.hpp"

#include <igl/remove_unreferenced.h>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/ExecutionScheduler.hpp>

namespace wmtk::components::tet_remeshing::surf {

SurfaceMesh::SurfaceMesh(const MatrixXd& V, const MatrixXi& F, int num_threads)
{
    NUM_THREADS = num_threads;
    p_vertex_attrs = &vertex_attrs;
    p_edge_attrs = &edge_attrs;

    TriMesh::init(F);

    vertex_attrs.resize(V.rows());

    for (size_t i = 0; i < vertex_attrs.size(); ++i) {
        vertex_attrs[i].m_posf = V.row(i);
    }


    // find non-manifold edges and frozen vertices
    for (const Tuple& t : get_edges()) {
        if (t.switch_faces(*this).size() == 1) {
            continue;
        }

        edge_attrs[t.eid(*this)].m_is_on_edge = true;

        size_t v0 = t.vid(*this);
        size_t v1 = t.switch_vertex(*this).vid(*this);
        vertex_attrs[v0].m_is_on_edge = true;
        vertex_attrs[v1].m_is_on_edge = true;
    }

    for (const Tuple& t : get_vertices()) {
        if (!vertex_attrs[t.vid(*this)].m_is_on_edge) {
            continue;
        }
        auto edges = get_one_ring_edges_for_vertex(t);
        size_t counter = 0;
        for (const Tuple& e : edges) {
            if (edge_attrs[e.eid(*this)].m_is_on_edge) {
                ++counter;
            }
        }
        if (counter != 2) {
            vertex_attrs[t.vid(*this)].m_is_freeze = true;
        }
    }
}

void SurfaceMesh::get_surface(MatrixXd& V, MatrixXi& F) const
{
    const auto verts = get_vertices();
    const auto faces = get_faces();

    V.resize(verts.size(), 3);
    for (size_t i = 0; i < verts.size(); ++i) {
        size_t vid = verts[i].vid(*this);
        V.row(i) = vertex_attrs[vid].m_posf;
    }

    F.resize(faces.size(), 3);
    int index = 0;
    for (const Tuple& t : faces) {
        const auto& vs = oriented_tri_vids(t);
        for (int j = 0; j < 3; j++) {
            F(index, j) = vs[j];
        }
        ++index;
    }

    MatrixXd NV;
    MatrixXi NF;
    MatrixXi I;
    igl::remove_unreferenced(V, F, NV, NF, I);
    V = NV;
    F = NF;
}

void SurfaceMesh::get_edge_mesh(MatrixXd& V, MatrixXi& E) const
{
    const auto& vs = get_vertices();

    std::vector<std::array<size_t, 2>> edges;
    for (const Tuple& e : get_edges()) {
        auto eid = e.eid(*this);
        if (edge_attrs[eid].m_is_on_edge) {
            size_t v0 = e.vid(*this);
            size_t v1 = e.switch_vertex(*this).vid(*this);
            edges.emplace_back(std::array<size_t, 2>{{v0, v1}});
        }
    }

    V.resize(vert_capacity(), 3);
    E.resize(edges.size(), 2);

    V.setZero();
    E.setZero();

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = edges[i][j];
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = vertex_attrs[vid].m_posf;
    }

    MatrixXd NV;
    MatrixXi NF;
    MatrixXi I;
    igl::remove_unreferenced(V, E, NV, NF, I);
    V = NV;
    E = NF;
}

void SurfaceMesh::smooth_surface()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (const Tuple& t : get_vertices()) {
        if (!vertex_attrs[t.vid(*this)].m_is_on_edge) {
            collect_all_ops.emplace_back("vertex_smooth", t);
        }
    }

    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<SurfaceMesh, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<SurfaceMesh, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}

void SurfaceMesh::smooth_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (const Tuple& t : get_vertices()) {
        if (vertex_attrs[t.vid(*this)].m_is_on_edge && !vertex_attrs[t.vid(*this)].m_is_freeze) {
            collect_all_ops.emplace_back("vertex_smooth", t);
        }
    }

    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<SurfaceMesh, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<SurfaceMesh, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}

void SurfaceMesh::smooth_all(const size_t num_iterations)
{
    for (size_t i = 0; i < num_iterations; ++i) {
        smooth_edges();
        smooth_surface();
    }
}

bool SurfaceMesh::smooth_before(const Tuple& t)
{
    if (vertex_attrs[t.vid(*this)].m_is_freeze) {
        return false;
    }

    return true;
}

bool SurfaceMesh::smooth_after(const TriMesh::Tuple& t)
{
    const size_t vid = t.vid(*this);
    std::vector<size_t> neighs;

    bool edge_smoothing = vertex_attrs.at(vid).m_is_on_edge;
    auto edges = get_one_ring_edges_for_vertex(t);
    for (const Tuple& e : edges) {
        if (!edge_smoothing ^ edge_attrs[e.eid(*this)].m_is_on_edge) {
            neighs.push_back(e.vid(*this));
        }
    }

    Vector3d p = Vector3d::Zero();
    for (const size_t n : neighs) {
        p += vertex_attrs.at(n).m_posf;
    }
    p /= neighs.size();

    vertex_attrs[vid].m_posf = 0.5 * (vertex_attrs.at(vid).m_posf + p);

    return true;
}

void SurfaceMesh::write_vtu(const std::string& path)
{
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& faces = get_faces();

    std::vector<std::array<size_t, 2>> edges;
    for (const Tuple& e : get_edges()) {
        auto eid = e.eid(*this);
        if (edge_attrs[eid].m_is_on_edge) {
            size_t v0 = e.vid(*this);
            size_t v1 = e.switch_vertex(*this).vid(*this);
            edges.emplace_back(std::array<size_t, 2>{{v0, v1}});
        }
    }

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi F(tri_capacity(), 3);
    Eigen::MatrixXi E(edges.size(), 2);

    V.setZero();
    F.setZero();
    E.setZero();

    Eigen::VectorXd v_is_frozen(vert_capacity());
    Eigen::VectorXd v_is_on_edge(vert_capacity());

    int index = 0;
    for (const Tuple& t : faces) {
        const auto& vs = oriented_tri_vids(t);
        for (int j = 0; j < 3; j++) {
            F(index, j) = vs[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = edges[i][j];
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = vertex_attrs[vid].m_posf;
        v_is_frozen(vid) = vertex_attrs[vid].m_is_freeze;
        v_is_on_edge(vid) = vertex_attrs[vid].m_is_on_edge;
    }

    paraviewo::VTUWriter writer;
    writer.add_field("on_edge", v_is_on_edge);
    writer.add_field("is_frozen", v_is_frozen);
    writer.write_mesh(path + ".vtu", V, F);

    // feature edges
    {
        const auto edge_out_path = path + "_edges.vtu";
        paraviewo::VTUWriter edge_writer;
        edge_writer.add_field("on_edge", v_is_on_edge);
        edge_writer.add_field("is_frozen", v_is_frozen);

        logger().info("Write {}", edge_out_path);
        edge_writer.write_mesh(edge_out_path, V, E);
    }
}

} // namespace wmtk::components::tet_remeshing::surf