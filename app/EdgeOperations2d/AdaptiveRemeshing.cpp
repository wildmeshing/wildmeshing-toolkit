#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeOperations2d.h"

using namespace Edge2d;
using namespace wmtk;
double EdgeOperations2d::compute_edge_cost_ar(const TriMesh::Tuple& t, double L)
{
    double l =
        (m_vertex_positions[t.vid()] - m_vertex_positions[t.switch_vertex(*this).vid()]).norm();
    if (l < (4. / 5.) * L) return ((4. / 5.) * L - l);
    if (l > (4. / 3.) * L) return (l - (4. / 3.) * L);
    return -1;
}

double EdgeOperations2d::compute_vertex_valence_ar(const TriMesh::Tuple& t)
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

bool EdgeOperations2d::adaptive_remeshing(double L, int iterations)
{
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> e_length_queue;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> e_valence_queue;
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
        // collapse and split edge
        assert(check_mesh_connectivity_validity());
        for (auto& loc : get_edges()) {
            double l_weight = compute_edge_cost_ar(loc, L);
            e_length_queue.push(ElementInQueue(loc, l_weight));
        }
        // int cntin = 0;
        while (!e_length_queue.empty()) {
            auto loc = e_length_queue.top().edge;
            auto l_weight = e_length_queue.top().weight;
            e_length_queue.pop();
            if (!loc.is_valid(*this) || l_weight < 0.01 * L) {
                continue;
            }

            TriMesh::Tuple new_vert;

            assert(check_mesh_connectivity_validity());
            double length =
                (m_vertex_positions[loc.vid()] - m_vertex_positions[loc.switch_vertex(*this).vid()])
                    .norm();
            if (length > ((4. / 3.) * L)) {
                if (!TriMesh::split_edge(loc, new_vert)) continue;
                auto new_edges = new_edges_after_collapse_split(new_vert);
                for (auto new_e : new_edges)
                    e_length_queue.push(ElementInQueue(new_e, compute_edge_cost_ar(new_e, L)));
            } else if (length < ((4 / 5.) * L)) {
                if (!TriMesh::collapse_edge(loc, new_vert)) continue;
                auto new_edges = new_edges_after_collapse_split(new_vert);
                for (auto new_e : new_edges)
                    e_length_queue.push(ElementInQueue(new_e, compute_edge_cost_ar(new_e, L)));
            } else
                continue;
            // cntin++;
            // write_triangle_mesh(
            //     "split_collapse_test" + std::to_string(cntin) + "_" + std::to_string(cnt) +
            //     ".obj");
        }
        write_triangle_mesh("split_collapse_test" + std::to_string(cnt) + ".obj");

        assert(check_mesh_connectivity_validity());

        // swap edges
        for (auto& loc : get_edges()) {
            double valence = compute_vertex_valence_ar(loc);
            e_valence_queue.push(ElementInQueue(loc, valence));
        }
        while (!e_valence_queue.empty()) {
            auto [loc, valence] = e_valence_queue.top();
            e_valence_queue.pop();

            if (!loc.is_valid(*this)) {
                continue;
            }
            valence = compute_vertex_valence_ar(loc);
            if (valence < 1e-5) continue;
            TriMesh::Tuple new_vert;

            assert(check_mesh_connectivity_validity());

            if (valence > 0) {
                if (!swap_edge(loc, new_vert)) continue;

                auto new_edges = new_edges_after_swap(new_vert);
                for (auto new_e : new_edges)
                    e_valence_queue.push(ElementInQueue(new_e, compute_vertex_valence_ar(new_e)));
            } else
                continue;
        }
        assert(check_mesh_connectivity_validity());
        write_triangle_mesh("swaped_" + std::to_string(cnt) + ".obj");
        // smoothing
        // auto vertices = get_vertices();
        // for (auto& loc : vertices) smooth(loc);

        assert(check_mesh_connectivity_validity());
        consolidate_mesh();
        average = average_len_valen();
        write_triangle_mesh("smoothed_" + std::to_string(cnt) + ".obj");
    }
    wmtk::vector_print(avg_lens);

    return true;
}