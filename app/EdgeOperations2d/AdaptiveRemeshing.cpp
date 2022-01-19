#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <wmtk/ExecutionScheduler.hpp>
#include "EdgeOperations2d.h"

using namespace Edge2d;
using namespace wmtk;

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

double EdgeOperations2d::compute_edge_cost_collapse_ar(const TriMesh::Tuple& t, double L) const
{
    double l =
        (m_vertex_positions[t.vid()] - m_vertex_positions[t.switch_vertex(*this).vid()]).norm();
    if (l < (4. / 5.) * L) return ((4. / 5.) * L - l);
    return -1;
}
double EdgeOperations2d::compute_edge_cost_split_ar(const TriMesh::Tuple& t, double L) const
{
    double l =
        (m_vertex_positions[t.vid()] - m_vertex_positions[t.switch_vertex(*this).vid()]).norm();
    if (l > (4. / 3.) * L) return (l - (4. / 3.) * L);
    return -1;
}

double EdgeOperations2d::compute_vertex_valence_ar(const TriMesh::Tuple& t) const
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

std::pair<double, double> EdgeOperations2d::average_len_valen()
{
    double average_len = 0.0;
    double average_valen = 0.0;
    auto edges = get_edges();
    auto verts = get_vertices();
    for (auto& loc : edges) {
        average_len +=
            (m_vertex_positions[loc.vid()] - m_vertex_positions[loc.switch_vertex(*this).vid()])
                .norm();
    }
    average_len /= edges.size();
    for (auto& loc : verts) {
        average_valen += get_one_ring_edges_for_vertex(loc).size();
    }
    average_valen /= verts.size();
    int cnt = 0;
    return std::make_pair(average_len, average_valen);
}

std::vector<TriMesh::Tuple> Edge2d::EdgeOperations2d::new_edges_after_swap(
    const TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    new_edges.push_back(t.switch_edge(*this));
    new_edges.push_back((t.switch_face(*this).value()).switch_edge(*this));
    new_edges.push_back((t.switch_vertex(*this)).switch_edge(*this));
    new_edges.push_back(((t.switch_vertex(*this)).switch_face(*this).value()).switch_edge(*this));
    return new_edges;
}

bool EdgeOperations2d::collapse_remeshing(double L)
{
     auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

    auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kSeq>();
    executor.renew_neighbor_tuples = renew;
    executor.priority = [&](auto& m, auto _, auto& e) {
        return -m.compute_edge_cost_collapse_ar(e, L);
    };
    executor.should_process =[](auto& m, auto&ele){
        auto& [val, op, e] = ele;
        if (val > 0) return false; // priority is negated.
        return true;
    };

    executor(*this, collect_all_ops);
    return true;
}
bool EdgeOperations2d::split_remeshing(double L)
{

    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_split", loc);

    auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kSeq>();
    executor.renew_neighbor_tuples = renew;
    executor.priority = [&](auto& m, auto _, auto& e) {
        return m.compute_edge_cost_split_ar(e, L);
    };
     executor.should_process =[](auto& m, auto&ele){
        auto& [val, op, e] = ele;
        if (val < 0) return false;
        return true;
    };

    executor(*this, collect_all_ops);
    return true;
}


bool EdgeOperations2d::swap_remeshing()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);

    auto executor = wmtk::ExecutePass<EdgeOperations2d, ExecutionPolicy::kSeq>();
    executor.renew_neighbor_tuples = renew;
    executor.priority = [](auto& m, auto op, const Tuple& e) {
        return m.compute_vertex_valence_ar(e);
    };
    executor.should_process = [](auto& m, auto& ele) {
        auto& [val, _, e] = ele;
        auto val_energy = (m.compute_vertex_valence_ar(e));
        return (val_energy > 1e-5);
    };

    executor(*this, collect_all_ops);
    return true;
}

bool EdgeOperations2d::adaptive_remeshing(double L, int iterations)
{
    std::vector<double> avg_lens;
    std::vector<double> avg_valens;
    int cnt = 0;
    auto average = average_len_valen();
    while ((average.first - L) * (average.first - L) > 1e-8 && cnt < iterations) {
        wmtk::logger().set_level(spdlog::level::trace);
        wmtk::logger().debug(" on iteration {}", cnt);
        cnt++;
        wmtk::logger().debug(" average length is {} {}", average.first, average.second);
        avg_lens.push_back(average.first);
        avg_valens.push_back(average.second);

        // split
        split_remeshing(L);

        // collpase
        collapse_remeshing(L);

        // swap edges
        swap_remeshing();

        // smoothing
        auto vertices = get_vertices();
        for (auto& loc : vertices) smooth(loc);

        assert(check_mesh_connectivity_validity());
        consolidate_mesh();
        average = average_len_valen();
    }
    wmtk::vector_print(avg_lens);

    return true;
}