#pragma once
#include <igl/per_face_normals.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>
// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/task_group.h>
#include <fastenvelope/FastEnvelope.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <atomic>
#include <memory>
#include <queue>

namespace qslim {

struct Quadrics
{
    Eigen::MatrixXd A;
    Eigen::Vector3d b;
    double c;
};

struct VertexAttributes
{
    Eigen::Vector3d pos;
    size_t partition_id = 0;
    bool freeze = false;
    Quadrics Q;
};

struct FaceAttributes
{
    Quadrics Q;
    Eigen::Vector3d n = Eigen::Vector3d::Zero(); // for quadrics computation
};

struct EdgeAttributes
{
    Eigen::Vector3d vbar; // for quadrics computation
};

class QSLIM : public wmtk::ConcurrentTriMesh
{
public:
    fastEnvelope::FastEnvelope m_envelope;
    bool m_has_envelope = false;
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    wmtk::AttributeCollection<FaceAttributes> face_attrs;
    wmtk::AttributeCollection<EdgeAttributes> edge_attrs;

    int NUM_THREADS = 1;
    int retry_limit = 10;
    QSLIM(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1)
        : NUM_THREADS(num_threads)
    {
        p_vertex_attrs = &vertex_attrs;
        p_face_attrs = &face_attrs;
        p_edge_attrs = &edge_attrs;

        vertex_attrs.resize(_m_vertex_positions.size());

        for (auto i = 0; i < _m_vertex_positions.size(); i++) {
            vertex_attrs[i] = {_m_vertex_positions[i], 0, false};
        }
    }

    void set_freeze(TriMesh::Tuple& v)
    {
        for (auto e : get_one_ring_edges_for_vertex(v)) {
            if (is_boundary_edge(e)) {
                vertex_attrs[v.vid(*this)].freeze = true;
                continue;
            }
        }
    }

    void create_mesh(
        size_t n_vertices,
        const std::vector<std::array<size_t, 3>>& tris,
        const std::vector<size_t>& frozen_verts = std::vector<size_t>(),
        double eps = 0)
    {
        wmtk::ConcurrentTriMesh::create_mesh(n_vertices, tris);
        std::vector<Eigen::Vector3d> V(n_vertices);
        Eigen::MatrixXd Vm(n_vertices, 3);
        std::vector<Eigen::Vector3i> F(tris.size());
        Eigen::MatrixXi Fm(tris.size(), 3);
        for (auto i = 0; i < V.size(); i++) {
            V[i] = vertex_attrs[i].pos;
            Vm.row(i) = V[i];
        }
        for (int i = 0; i < F.size(); ++i) {
            F[i] << tris[i][0], tris[i][1], tris[i][2];
            Fm.row(i) = F[i];
        }
        if (eps > 0) {
            m_envelope.init(V, F, eps);
            m_has_envelope = true;
        } else
            m_envelope.init(V, F, 0.0);

        face_attrs.resize(tri_capacity());
        edge_attrs.resize(tri_capacity() * 3);

        Eigen::MatrixXd N = Eigen::MatrixXd::Zero(F.size(), 3);
        igl::per_face_normals(Vm, Fm, N);
        for (int i = 0; i < N.rows(); i++) face_attrs[i].n = N.row(i);

        initiate_quadrics_for_face();

        initiate_quadrics_for_vertices();


        partition_mesh();
        for (auto v : frozen_verts) vertex_attrs[v].freeze = true;
        for (auto v : get_vertices()) { // the better way is to iterate through edges.
            set_freeze(v);
        }
    }

    void initiate_quadrics_for_face()
    {
        // initaite A, b, c for each traingle
        auto faces = get_faces();

        for (int i = 0; i < faces.size(); i++) {
            // get A,b,c of one face
            Quadrics Qf = compute_quadric_for_face(faces[i]);
            // update the face_attr
            face_attrs[i].Q = Qf;
        }
    }
    void initiate_quadrics_for_vertices()
    { // get one circle of faces and linearly add A,b, c
        auto verts = get_vertices();
        Eigen::MatrixXd A;
        Eigen::Vector3d b(0.0, 0.0, 0.0);
        double c = 0.0;
        double w = 1e-10;
        for (int i = 0; i < verts.size(); i++) {
            A = w * Eigen::MatrixXd::Identity(3, 3);
            b = -w * vertex_attrs[i].pos;
            c = w * vertex_attrs[i].pos.dot(vertex_attrs[i].pos);
            auto one_ring_faces = get_one_ring_tris_for_vertex(verts[i]);
            for (auto tri : one_ring_faces) {
                A += face_attrs[tri.fid(*this)].Q.A;
                b += face_attrs[tri.fid(*this)].Q.b;
                c += face_attrs[tri.fid(*this)].Q.c;
            }
            vertex_attrs[i].Q = {A, b, c};
        }
    }

    ~QSLIM() {}

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

public:
    bool collapse_before(const Tuple& t) override;
    bool collapse_after(const Tuple& t) override;
    bool collapse_qslim(int target_vertex_count);
    bool write_triangle_mesh(std::string path);
    bool invariants(const std::vector<Tuple>& new_tris) override;
    double compute_cost_for_e(const TriMesh::Tuple& v_tuple);
    Quadrics compute_quadric_for_face(const TriMesh::Tuple& f_tuple);
    void update_quadrics(const TriMesh::Tuple& v_tuple);

private:
    struct InfoCache
    {
        Eigen::Vector3d v1p;
        Eigen::Vector3d v2p;
        Eigen::Vector3d vbar;
        Quadrics Q1;
        Quadrics Q2;
    };
    tbb::enumerable_thread_specific<InfoCache> cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};

} // namespace qslim
