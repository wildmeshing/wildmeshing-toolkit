#pragma once
#include <igl/per_face_normals.h>
#include <wmtk/ConcurrentTriMesh.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/AttributeCollection.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/write_triangle_mesh.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <tbb/task_arena.h>
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
    Eigen::Matrix3d A;
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

    int retry_limit = 10;
    QSLIM(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads = 1)
    {
        p_vertex_attrs = &vertex_attrs;
        p_face_attrs = &face_attrs;
        p_edge_attrs = &edge_attrs;
        NUM_THREADS = (num_threads);

        vertex_attrs.resize(_m_vertex_positions.size());

        for (auto i = 0; i < _m_vertex_positions.size(); i++) {
            vertex_attrs[i] = {_m_vertex_positions[i], 0, false, {}};
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
        wmtk::logger().info("----start create mesh-------");
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
        } 
        face_attrs.resize(tri_capacity());
        edge_attrs.resize(tri_capacity() * 3);

        Eigen::MatrixXd N = Eigen::MatrixXd::Zero(F.size(), 3);
        wmtk::logger().info("----normal-------");
        igl::per_face_normals(Vm, Fm, N);
        for (int i = 0; i < N.rows(); i++) face_attrs[i].n = N.row(i);

        wmtk::logger().info("----start init qua for face-------");
        initiate_quadrics_for_face();
        wmtk::logger().info("----start init qua for vertices-------");
        initiate_quadrics_for_vertices();


        partition_mesh_morton();
        // for (auto v : frozen_verts) vertex_attrs[v].freeze = true;
        // for (auto v : get_vertices()) { // the better way is to iterate through edges.
        //     set_freeze(v);
        // }
    }

    void initiate_quadrics_for_face()
    {
        // initaite A, b, c for each traingle
        // auto faces = get_faces();
        for_each_face([&](auto& tup) { 
            // get A,b,c of one face
            int i = tup.fid(*this);
            Quadrics Qf = compute_quadric_for_face(tup);
            // update the face_attr
            face_attrs[i].Q = Qf;
        });
    }
    void initiate_quadrics_for_vertices()
    { // get one circle of faces and linearly add A,b, c

        for_each_vertex([&](auto& tup) { 
            Eigen::MatrixXd A;
            Eigen::Vector3d b(0.0, 0.0, 0.0);
            double c = 0.0;
            double w = 1e-10;

            int i = tup.vid(*this);
            A = w * Eigen::MatrixXd::Identity(3, 3);
            b = -w * vertex_attrs[i].pos;
            c = w * vertex_attrs[i].pos.dot(vertex_attrs[i].pos);
            auto one_ring_faces = get_one_ring_tris_for_vertex(tup);
            for (auto tri : one_ring_faces) {
                A += face_attrs[tri.fid(*this)].Q.A;
                b += face_attrs[tri.fid(*this)].Q.b;
                c += face_attrs[tri.fid(*this)].Q.c;
            }
            vertex_attrs[i].Q = {A, b, c};
            
        });   

        

    }

    ~QSLIM() {}

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

    void partition_mesh_morton()
    {
        if (NUM_THREADS == 0) return;
        wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

        tbb::task_arena arena(NUM_THREADS);

        arena.execute([&] {
            std::vector<Eigen::Vector3d> V_v(vert_capacity());

            tbb::parallel_for(
                tbb::blocked_range<int>(0, V_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        V_v[i] = vertex_attrs[i].pos;
                    }
                });

            struct sortstruct
            {
                int order;
                Resorting::MortonCode64 morton;
            };

            std::vector<sortstruct> list_v;
            list_v.resize(V_v.size());
            const int multi = 1000;
            // since the morton code requires a correct scale of input vertices,
            //  we need to scale the vertices if their coordinates are out of range
            std::vector<Eigen::Vector3d> V = V_v; // this is for rescaling vertices
            Eigen::Vector3d vmin, vmax;
            vmin = V.front();
            vmax = V.front();

            for (size_t j = 0; j < V.size(); j++) {
                for (int i = 0; i < 3; i++) {
                    vmin(i) = std::min(vmin(i), V[j](i));
                    vmax(i) = std::max(vmax(i), V[j](i));
                }
            }

            Eigen::Vector3d center = (vmin + vmax) / 2;

            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] - center;
                }
            });

            Eigen::Vector3d scale_point =
                vmax - center; // after placing box at origin, vmax and vmin are symetric.

            double xscale, yscale, zscale;
            xscale = fabs(scale_point[0]);
            yscale = fabs(scale_point[1]);
            zscale = fabs(scale_point[2]);
            double scale = std::max(std::max(xscale, yscale), zscale);
            if (scale > 300) {
                tbb::parallel_for(
                    tbb::blocked_range<int>(0, V.size()),
                    [&](tbb::blocked_range<int> r) {
                        for (int i = r.begin(); i < r.end(); i++) {
                            V[i] = V[i] / scale;
                        }
                    });
            }

            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    list_v[i].morton = Resorting::MortonCode64(
                        int(V[i][0] * multi),
                        int(V[i][1] * multi),
                        int(V[i][2] * multi));
                    list_v[i].order = i;
                }
            });

            const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
                return (a.morton < b.morton);
            };

            tbb::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

            int interval = list_v.size() / NUM_THREADS + 1;

            tbb::parallel_for(
                tbb::blocked_range<int>(0, list_v.size()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        vertex_attrs[list_v[i].order].partition_id = i / interval;
                    }
                });
        });
    }

public:
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
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
        int partition_id;
    };
    tbb::enumerable_thread_specific<InfoCache> cache;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& t) const;
};

} // namespace qslim
