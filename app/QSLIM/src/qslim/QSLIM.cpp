#include "QSLIM.h"


#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;
using namespace qslim;

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
    Quadrics Q1 = vertex_attrs[v_tuple.vid(*this)].Q;
    Quadrics Q2 = vertex_attrs[v_tuple.switch_vertex(*this).vid(*this)].Q;
    Quadrics Q = {Q1.A + Q2.A, Q1.b + Q2.b, Q1.c + Q2.c};
    double cost = 0.0;
    Eigen::Vector3d vbar(0.0, 0.0, 0.0);
    Eigen::Vector3d v1 = vertex_attrs[v_tuple.vid(*this)].pos;
    Eigen::Vector3d v2 = vertex_attrs[v_tuple.switch_vertex(*this).vid(*this)].pos;

    // test if A is invertible
    // not invertible vbar is smallest of two vertices
    // if A is invertible, compute vbar using A.inv@b
    if (Q.A.determinant() < 1e-10 && Q.A.determinant() > -1e-10) {
        // not invertible
        if ((v1.transpose() * Q1.A * v1 + 2 * Q1.b.dot(v1) + Q1.c) <
            (v2.transpose() * Q2.A * v2 + 2 * Q2.b.dot(v2) + Q2.c))
            vbar = v1;
        else
            vbar = v2;

    } else {
        vbar = -Q.A.inverse() * Q.b;
    }
    cost = (vbar.dot(Q.A * vbar) + 2 * Q.b.dot(vbar) + Q.c);
    // the cost has to greater than 0
    if (cost < 0 && cost > -1e-15)
        cost = 0; // floating point error
    else if (cost < 0)
        wmtk::logger().info(
            "cost is {} smaller than zero \n Q.A is \n{} \nQ.b \n{}\n Q.c \n {} \nvbar is\n{} "
            " ",
            cost,
            Q.A,
            Q.b,
            Q.c,
            vbar);
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
    return true;
}


bool QSLIM::collapse_edge_after(const TriMesh::Tuple& t)
{
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = cache.local().vbar;
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
    int starting_num = get_vertices().size();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

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