#include "UniformRemeshing.h"

#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

using namespace remeshing;
using namespace wmtk;

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};


auto edge_locker = [](auto& m, const auto& e, int task_id) {
    return m.try_set_edge_mutex_two_ring(e, task_id);
};

std::vector<TriMesh::Tuple> UniformRemeshing::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}
bool UniformRemeshing::swap_after(const TriMesh::Tuple& t)
{
    std::vector<TriMesh::Tuple> tris;
    tris.push_back(t);
    tris.push_back(t.switch_edge(*this));
    return true;
}

bool UniformRemeshing::collapse_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.vid();
    vertex_attrs[vid].pos = p;

    return true;
}

bool UniformRemeshing::split_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.vid();
    vertex_attrs[vid].pos = p;

    return true;
}

double UniformRemeshing::compute_edge_cost_collapse(const TriMesh::Tuple& t, double L) const
{
    double l = (vertex_attrs[t.vid()].pos - vertex_attrs[t.switch_vertex(*this).vid()].pos).norm();
    if (l < (4. / 5.) * L) return ((4. / 5.) * L - l);
    return -1;
}
double UniformRemeshing::compute_edge_cost_split(const TriMesh::Tuple& t, double L) const
{
    double l = (vertex_attrs[t.vid()].pos - vertex_attrs[t.switch_vertex(*this).vid()].pos).norm();
    if (l > (4. / 3.) * L) return (l - (4. / 3.) * L);
    return -1;
}

double UniformRemeshing::compute_vertex_valence(const TriMesh::Tuple& t) const
{
    std::vector<std::pair<TriMesh::Tuple, int>> valences(3);
    valences[0] = std::make_pair(t, get_one_ring_tris_for_vertex(t).size());
    auto t2 = t.switch_vertex(*this);
    valences[1] = std::make_pair(t2, get_one_ring_tris_for_vertex(t2).size());
    auto t3 = (t.switch_edge(*this)).switch_vertex(*this);
    valences[2] = std::make_pair(t3, get_one_ring_tris_for_vertex(t3).size());

    if ((t.switch_face(*this)).has_value()) {
        auto t4 = (((t.switch_face(*this)).value()).switch_edge(*this)).switch_vertex(*this);
        valences.emplace_back(t4, get_one_ring_tris_for_vertex(t4).size());
    }
    double cost_before_swap = 0.0;
    double cost_after_swap = 0.0;

    // check if it's internal vertex or bondary vertex
    // navigating starting one edge and getting back to the start

    for (int i = 0; i < valences.size(); i++) {
        TriMesh::Tuple vert = valences[i].first;
        int val = 6;
        auto one_ring_edges = get_one_ring_edges_for_vertex(vert);
        for (auto edge : one_ring_edges) {
            if (is_boundary_edge(edge)) {
                val = 4;
                break;
            }
        }
        cost_before_swap += (double)(valences[i].second - val) * (valences[i].second - val);
        cost_after_swap +=
            (i < 2) ? (double)(valences[i].second - 1 - val) * (valences[i].second - 1 - val)
                    : (double)(valences[i].second + 1 - val) * (valences[i].second + 1 - val);
    }
    return (cost_before_swap - cost_after_swap);
}

std::vector<double> UniformRemeshing::average_len_valen()
{
    double average_len = 0.0;
    double average_valen = 0.0;
    auto edges = get_edges();
    auto verts = get_vertices();
    double maxlen = std::numeric_limits<double>::min();
    double maxval = std::numeric_limits<double>::min();
    double minlen = std::numeric_limits<double>::max();
    double minval = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        double currentlen =
            (vertex_attrs[loc.vid()].pos - vertex_attrs[loc.switch_vertex(*this).vid()].pos).norm();
        average_len += currentlen;
        if (maxlen < currentlen) maxlen = currentlen;
        if (minlen > currentlen) minlen = currentlen;
    }
    average_len /= edges.size();
    for (auto& loc : verts) {
        double currentval = get_one_ring_edges_for_vertex(loc).size();
        average_valen += currentval;
        if (maxval < currentval) maxval = currentval;
        if (minval > currentval) minval = currentval;
    }
    average_valen /= verts.size();
    int cnt = 0;
    std::vector<double> rtn{average_len, maxlen, minlen, average_valen, maxval, minval};
    return rtn;
}

std::vector<TriMesh::Tuple> UniformRemeshing::new_edges_after_swap(const TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    new_edges.push_back(t.switch_edge(*this));
    new_edges.push_back((t.switch_face(*this).value()).switch_edge(*this));
    new_edges.push_back((t.switch_vertex(*this)).switch_edge(*this));
    new_edges.push_back(((t.switch_vertex(*this)).switch_face(*this).value()).switch_edge(*this));
    return new_edges;
}

bool UniformRemeshing::collapse_remeshing(double L)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto _, auto& e) {
            return -m.compute_edge_cost_collapse(e, L);
        };
        executor.lock_vertices = edge_locker;
        executor.num_threads = NUM_THREADS;

        executor.should_process = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val > 0) return false; // priority is negated.
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}
bool UniformRemeshing::split_remeshing(double L)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();

    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_split", loc);
    wmtk::logger().info("size for edges to be split is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.num_threads = NUM_THREADS;

        executor.lock_vertices = edge_locker;

        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto _, auto& e) {
            return m.compute_edge_cost_split(e, L);
        };
        executor.should_process = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val < 0) return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}


bool UniformRemeshing::swap_remeshing()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);

    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.num_threads = NUM_THREADS;
        executor.priority = [](auto& m, auto op, const Tuple& e) {
            return m.compute_vertex_valence(e);
        };
        executor.lock_vertices = edge_locker;
        executor.should_process = [](auto& m, auto& ele) {
            auto& [val, _, e] = ele;
            auto val_energy = (m.compute_vertex_valence(e));
            return (val_energy > 1e-5);
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}
double area(UniformRemeshing& m, std::array<TriMesh::Tuple, 3>& verts)
{
    return ((m.vertex_attrs[verts[0].vid()].pos - m.vertex_attrs[verts[2].vid()].pos)
                .cross(m.vertex_attrs[verts[1].vid()].pos - m.vertex_attrs[verts[2].vid()].pos))
               .norm() /
           2.0;
}

Eigen::Vector3d normal(UniformRemeshing& m, std::array<TriMesh::Tuple, 3>& verts)
{
    return ((m.vertex_attrs[verts[0].vid()].pos - m.vertex_attrs[verts[2].vid()].pos)
                .cross(m.vertex_attrs[verts[1].vid()].pos - m.vertex_attrs[verts[2].vid()].pos))
        .normalized();
}

Eigen::Vector3d UniformRemeshing::smooth(const TriMesh::Tuple& t)
{
    auto one_ring_edges = get_one_ring_edges_for_vertex(t);
    if (one_ring_edges.size() < 3) return vertex_attrs[t.vid()].pos;
    Eigen::Vector3d after_smooth(0, 0, 0);
    Eigen::Vector3d after_smooth_boundary(0, 0, 0);
    int boundary = 0;
    for (auto e : one_ring_edges) {
        if (is_boundary_edge(e)) {
            after_smooth_boundary += vertex_attrs[e.vid()].pos;
            boundary++;
            continue;
        }
        after_smooth += vertex_attrs[e.vid()].pos;
    }

    if (boundary)
        after_smooth = after_smooth_boundary / boundary;
    else
        after_smooth /= one_ring_edges.size();
    return after_smooth;
}

Eigen::Vector3d UniformRemeshing::tangential_smooth(const Tuple& t)
{
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    if (one_ring_tris.size() < 2) return vertex_attrs[t.vid()].pos;
    Eigen::Vector3d after_smooth = smooth(t);
    // get normal and area of each face
    auto area = [](auto& m, auto& verts) {
        return ((m.vertex_attrs[verts[0].vid()].pos - m.vertex_attrs[verts[2].vid()].pos)
                    .cross(m.vertex_attrs[verts[1].vid()].pos - m.vertex_attrs[verts[2].vid()].pos))
                   .norm() /
               2.0;
    };
    auto normal = [](auto& m, auto& verts) {
        return ((m.vertex_attrs[verts[0].vid()].pos - m.vertex_attrs[verts[2].vid()].pos)
                    .cross(m.vertex_attrs[verts[1].vid()].pos - m.vertex_attrs[verts[2].vid()].pos))
            .normalized();
    };
    auto w0 = 0.0;
    Eigen::Vector3d n0(0.0, 0.0, 0.0);
    for (auto& e : one_ring_tris) {
        auto verts = oriented_tri_vertices(e);
        w0 += area(*this, verts);
        n0 += area(*this, verts) * normal(*this, verts);
    }
    n0 /= w0;
    after_smooth += n0 * n0.transpose() * (vertex_attrs[t.vid()].pos - after_smooth);
    assert(check_mesh_connectivity_validity());
    return after_smooth;
}


bool UniformRemeshing::uniform_remeshing(double L, int iterations)
{
    std::vector<double> avg_lens, max_lens, min_lens;
    std::vector<double> avg_valens, max_vals, min_vals;
    int cnt = 0;
    auto properties = average_len_valen();
    while ((properties[0] - L) * (properties[0] - L) > 1e-8 && cnt < iterations) {
        cnt++;
        avg_lens.push_back(properties[0]);
        avg_valens.push_back(properties[3]);
        max_lens.push_back(properties[1]);
        max_vals.push_back(properties[4]);
        min_lens.push_back(properties[2]);
        min_vals.push_back(properties[5]);

        write_triangle_mesh("remesh_itr" + std::to_string(cnt) + ".obj");
        // split
        wmtk::logger().info("starting split");
        split_remeshing(L);
        wmtk::logger().info("finished split");
        // collpase
        collapse_remeshing(L);
        wmtk::logger().info("finished collapse");
        write_triangle_mesh("itr" + std::to_string(cnt)+ "_collapse_0224.obj");
        // swap edges
        swap_remeshing();
        wmtk::logger().info("finished swap");
        write_triangle_mesh("itr" + std::to_string(cnt)+ "_swap_0224.obj");
        // smoothing
        auto vertices = get_vertices();
        for (auto& loc : vertices) vertex_attrs[loc.vid(*this)].pos = tangential_smooth(loc);
        wmtk::logger().info("finished smooth");
        write_triangle_mesh("itr" + std::to_string(cnt)+ "_smooth_0224.obj");

        for (auto& loc : vertices) vertex_attrs[loc.vid()].pos = tangential_smooth(loc);
        wmtk::logger().info("finished smooth");
        assert(check_mesh_connectivity_validity());
        consolidate_mesh();
        write_triangle_mesh("remesh_consolidate_itr" + std::to_string(cnt+1) + "_0224.obj");
        properties = average_len_valen();
    }
    wmtk::logger().info("avg edge len after each remesh is: ");
    wmtk::vector_print(avg_lens);
    // wmtk::logger().info("max edge len after each remesh is: ");
    // wmtk::vector_print(max_lens);
    // wmtk::logger().info("min edge len after each remesh is: ");
    // wmtk::vector_print(min_lens);


    wmtk::logger().info("avg valence after each remesh is: ");
    wmtk::vector_print(avg_valens);
    // wmtk::logger().info("max valence after each remesh is: ");
    // wmtk::vector_print(max_vals);
    // wmtk::logger().info("min valence after each remesh is: ");
    // wmtk::vector_print(min_vals);
    return true;
}
// write the collapsed mesh into a obj
bool UniformRemeshing::write_triangle_mesh(std::string path)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vertex_attrs.size(), 3);
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
    igl::write_triangle_mesh(path, V, F);
    assert(igl::is_edge_manifold(F));
    return igl::is_edge_manifold(F);
}