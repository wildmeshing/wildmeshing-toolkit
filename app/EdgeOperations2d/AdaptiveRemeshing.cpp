#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "EdgeOperations2d.h"

using namespace Edge2d;

double Edge2d::EdgeOperations2d::compute_edge_cost_ar(const TriMesh::Tuple& t, double L)
{
    double l =
        (m_vertex_positions[t.vid()] - m_vertex_positions[t.switch_vertex(*this).vid()]).norm();
    if (l < (4 / 5) * L) return ((4 / 5) * L - l);
    if (l > (4 / 3) * L) return (l - (4 / 3) * L);
    return 0.0;
}

double Edge2d::EdgeOperations2d::compute_vertex_valence_ar(const TriMesh::Tuple& t)
{
    std::vector<std::pair<TriMesh::Tuple, int>> valences(3);
    valences[0] = std::make_pair(t, get_one_ring_edges_for_vertex(t).size());
    auto t2 = t.switch_vertex(*this);
    valences[1] = std::make_pair(t2, get_one_ring_edges_for_vertex(t2).size());
    auto t3 = (t.switch_edge(*this)).switch_vertex(*this);
    valences[2] = std::make_pair(t3, get_one_ring_edges_for_vertex(t3).size());


    if ((t.switch_face(*this)).has_value()) {
        auto t4 = (((t.switch_face(*this)).value()).switch_edge(*this)).switch_vertex(*this);
        valences.emplace_back(t4, get_one_ring_edges_for_vertex(t4).size());
    }
    double cost_before_swap = 0.0;
    double cost_after_swap = 0.0;

    // check if it's internal vertex or bondary vertex
    // navigating starting one edge and getting back to the start

    for (int i = 0; i < valences.size(); i++) {
        TriMesh::Tuple vert = valences[i].first;
        size_t start = vert.switch_vertex(*this).vid();
        int cnt = valences[i].second;
        int val = -1;
        while ((vert.switch_edge(*this)).switch_face(*this).has_value() && cnt > 0) {
            vert = ((vert.switch_edge(*this)).switch_face(*this)).value();
            if (vert.vid() == start) {
                val = 6;
                break;
            }
        }
        if (!(vert.switch_edge(*this)).switch_face(*this).has_value())
            val = 4;
        else
            throw "error in navigation";
        cost_before_swap += (double)(valences[i].second - val) * (valences[i].second - val);
        cost_after_swap +=
            (i < 2) ? (double)(valences[i].second - 1 - val) * (valences[i].second - 1 - val)
                    : (double)(valences[i].second + 1 - val) * (valences[i].second + 1 - val);
    }
    return (cost_before_swap - cost_after_swap);
}

bool Edge2d::EdgeOperations2d::adaptive_remeshing(double L)
{
    std::vector<TriMesh::Tuple> edges = get_edges();
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> e_length_queue;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> e_valence_queue;

    for (auto& loc : edges) {
        double weight = compute_edge_cost_ar(loc, L);
        e_length_queue.push(ElementInQueue(loc, weight));
    }
    int cnt = 10;
    while (cnt > 0) {
        cnt--;
        // collapse and split edge
        while (!e_length_queue.empty()) {
            auto loc = e_length_queue.top().edge;
            if (!loc.is_valid(*this)) {
                e_length_queue.pop();
                continue;
            }
            auto weight = e_length_queue.top().weight;
            e_length_queue.pop();

            TriMesh::Tuple new_vert;

            assert(check_mesh_connectivity_validity());
            double length =
                (m_vertex_positions[loc.vid()] - m_vertex_positions[loc.switch_vertex(*this).vid()])
                    .norm();
            if (length > (5 / 4) * L)
                TriMesh::split_edge(loc, new_vert);
            else
                TriMesh::collapse_edge(loc, new_vert);
        }
        e_length_queue = std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l>();

        // swap edges
        edges = get_edges();
        for (auto& loc : edges) {
            double valence = compute_vertex_valence_ar(loc);
            e_valence_queue.push(ElementInQueue(loc, valence));
        }
        while (!e_valence_queue.empty()) {
            auto loc = e_valence_queue.top().edge;
            if (!loc.is_valid(*this)) {
                e_valence_queue.pop();
                continue;
            }
            auto valence = e_valence_queue.top().weight;
            e_valence_queue.pop();

            TriMesh::Tuple new_vert;

            assert(check_mesh_connectivity_validity());

            if (valence > 0)
                TriMesh::swap_edge(loc, new_vert);
            else
                continue;
        }
        e_valence_queue = std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l>();

        // smoothing
        auto vertices = get_vertices();
        for (auto& loc : vertices) smooth(loc);

        assert(check_mesh_connectivity_validity());
        consolidate_mesh();
        assert(check_mesh_connectivity_validity());

        // update the pq for collpase and split
        edges = get_edges();
        for (auto& loc : edges) {
            double weight = compute_edge_cost_ar(loc, L);
            e_length_queue.push(ElementInQueue(loc, weight));
        }
    }
    return true;
}