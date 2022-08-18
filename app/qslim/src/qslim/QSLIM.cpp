#include "QSLIM.h"

#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;
using namespace app::qslim;


QSLIM::QSLIM(std::vector<Eigen::Vector3d> _m_vertex_positions, int num_threads)
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

void QSLIM::set_freeze(TriMesh::Tuple& v)
{
    for (auto e : get_one_ring_edges_for_vertex(v)) {
        if (is_boundary_edge(e)) {
            vertex_attrs[v.vid(*this)].freeze = true;
            continue;
        }
    }
}

void QSLIM::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<size_t>& frozen_verts,
    double eps)
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

void QSLIM::initiate_quadrics_for_face()
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

void QSLIM::initiate_quadrics_for_vertices()
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

void QSLIM::partition_mesh_morton()
{
    if (NUM_THREADS == 0) return;
    wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&] {
        std::vector<Eigen::Vector3d> V_v(vert_capacity());

        tbb::parallel_for(tbb::blocked_range<int>(0, V_v.size()), [&](tbb::blocked_range<int> r) {
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
            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
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

 
// called in collapse_edge_after. update the quadric for v
// adding the A, b , c for two vertices of the old edge and store at the vert_attr of new vertex
void QSLIM::update_quadrics(const Tuple& new_v)
{
    vertex_attrs[new_v.vid(*this)].Q = {
        cache.local().Q1.A + cache.local().Q2.A,
        cache.local().Q1.b + cache.local().Q2.b,
        cache.local().Q1.c + cache.local().Q2.c};
}

// want to store the A, b, c for each face as face attribute
Quadrics QSLIM::compute_quadric_for_face(const TriMesh::Tuple& f_tuple)
{
    Quadrics Q;
    Eigen::Vector3d n = face_attrs[f_tuple.fid(*this)].n;
    Q.A = n * n.transpose();
    double d = -n.dot(vertex_attrs[oriented_tri_vertices(f_tuple)[0].vid(*this)].pos);
    Q.b = d * n;
    Q.c = d * d;
    return Q;
}
double QSLIM::compute_cost_for_e(const TriMesh::Tuple& v_tuple)
{
    // first get Q
    Quadrics& Q1 = vertex_attrs[v_tuple.vid(*this)].Q;
    Quadrics& Q2 = vertex_attrs[v_tuple.switch_vertex(*this).vid(*this)].Q;
    Quadrics Q = {Q1.A + Q2.A, Q1.b + Q2.b, Q1.c + Q2.c};
    double cost = 0.0;
    Eigen::Vector3d vbar(0.0, 0.0, 0.0);
    Eigen::Vector3d v1 = vertex_attrs[v_tuple.vid(*this)].pos;
    Eigen::Vector3d v2 = vertex_attrs[v_tuple.switch_vertex(*this).vid(*this)].pos;

    // test if A is invertible
    // not invertible vbar is smallest of two vertices
    // if A is invertible, compute vbar using A.inv@b
    auto determinant = Q.A.determinant();
    if (determinant < 1e-10 && determinant > -1e-10) {
        // not invertible
        if ((v1.transpose() * Q1.A * v1 + 2 * Q1.b.dot(v1) + Q1.c) <
            (v2.transpose() * Q2.A * v2 + 2 * Q2.b.dot(v2) + Q2.c))
            vbar = v1;
        else
            vbar = v2;

    } else {
        vbar = -Q.A.ldlt().solve(Q.b);
        //
        // vbar = -Q.A.inverse() * Q.b;
    }
    cost = (vbar.dot(Q.A * vbar) + 2 * Q.b.dot(vbar) + Q.c);
    // the cost has to greater than 0
    if (cost < 0) cost = 0; // floating point error
    // wmtk::logger().info("cost is smaller than zero");
    edge_attrs[v_tuple.eid(*this)].vbar = vbar;
    return cost;
}

bool QSLIM::invariants(const std::vector<Tuple>& new_tris)
{
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = t.oriented_tri_vertices(*this);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            if (m_envelope.is_outside(tris)) return false;
        }
    }
    return true;
}

bool QSLIM::write_triangle_mesh(std::string path)
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

bool QSLIM::collapse_edge_before(const Tuple& t)
{
    if (!ConcurrentTriMesh::collapse_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].freeze || vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
        return false;
    cache.local().v1p = vertex_attrs[t.vid(*this)].pos;
    cache.local().v2p = vertex_attrs[t.switch_vertex(*this).vid(*this)].pos;
    cache.local().Q1 = vertex_attrs[t.vid(*this)].Q;
    cache.local().Q2 = vertex_attrs[t.switch_vertex(*this).vid(*this)].Q;
    cache.local().vbar = edge_attrs[t.eid(*this)].vbar;
    cache.local().partition_id = vertex_attrs[t.vid(*this)].partition_id;
    return true;
}


bool QSLIM::collapse_edge_after(const TriMesh::Tuple& t)
{
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = cache.local().vbar;
    vertex_attrs[vid].partition_id = cache.local().partition_id;
    // update the quadrics
    update_quadrics(t);
    return true;
}


std::vector<TriMesh::Tuple> QSLIM::new_edges_after(const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<TriMesh::Tuple> one_ring_verts;
    for (auto t : tris) {
        auto incident_verts = t.oriented_tri_vertices(*this);
        for (auto j = 0; j < 3; j++) one_ring_verts.push_back(incident_verts[j]);
    }
    for (auto v : one_ring_verts) {
        auto incident_edges = get_one_ring_edges_for_vertex(v);
        for (auto e : incident_edges) new_edges.push_back(e);
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool QSLIM::collapse_qslim(int target_vert_number)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    int starting_num = vert_capacity();

    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_collapse", t);

    auto renew = [](auto& m, auto op, auto& tris) {
        auto edges = m.new_edges_after(tris);
        auto optup = std::vector<std::pair<std::string, Tuple>>();
        for (auto& e : edges) optup.emplace_back("edge_collapse", e);
        return optup;
    };
    auto measure_priority = [this](auto& m, auto op, const Tuple& new_e) {
        return -compute_cost_for_e(new_e);
    };
    auto setup_and_execute = [&](auto executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_priority;
        executor.stopping_criterion_checking_frequency = std::numeric_limits<int>::max();
        executor.stopping_criterion_checking_frequency = target_vert_number > 0
                                                             ? starting_num - target_vert_number - 1
                                                             : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor.is_weight_up_to_date = [&collect_all_ops, this](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            //if (val > 0) return false; // priority is negated.
            double pri = -compute_cost_for_e(e);
            if (((val - pri) * (val - pri)) > 1e-8) {
                wmtk::logger().info("the priority is different val is {}, pri is {}", val, pri);
                return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<QSLIM, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<QSLIM, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
    return true;
}