//
// Created by Yixin Hu on 12/7/21.
//

#include "Logger.hpp"
#include "TetWild.h"


void tetwild::TetWild::collapse_all_edges()
{
    //    compact();

    std::vector<Tuple> edges = get_edges();

    apps::logger().debug("edges.size() = {}", edges.size());

    int cnt_suc = 0;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue;
    for (auto& loc : edges) {
        Tuple& v1 = loc;
        Tuple v2 = loc.switch_vertex(*this);
        double length = (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf)
                            .squaredNorm();
        if (length > m_params.collapsing_l2) continue;
        ec_queue.push(ElementInQueue(loc, length));
    }

    while (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();

        // check timestamp
        if (!loc.is_valid(*this)) continue;
        { // check weight
            Tuple& v1 = loc;
            Tuple v2 = loc.switch_vertex(*this);
            double length =
                (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf)
                    .squaredNorm();
            if (length != weight) continue;
        }

        std::vector<Tuple> new_edges;
        if (collapse_edge(loc, new_edges)) {
            cnt_suc++;
            for (auto& new_loc : new_edges) {
                Tuple& v1 = new_loc;
                Tuple v2 = new_loc.switch_vertex(*this);
                double length =
                    (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf)
                        .squaredNorm();
                if (length < m_params.collapsing_l2) continue;
                ec_queue.push(ElementInQueue(new_loc, length));
            }
            //            std::cout<<"success"<<std::endl;
        }
    }
}

bool tetwild::TetWild::collapse_before(const Tuple& loc) // input is an edge
{
    //check if on bbox/surface/boundary
    // todo: store surface info into cache

    int v1_id = loc.vid();
    auto loc1 = switch_vertex(loc);
    int v2_id = loc1.vid();
    collapse_cache.edge_length =
        (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf)
            .norm(); // todo: duplicated computation

    auto n1_locs = get_one_ring_tets_for_vertex(loc);
    auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

    std::map<int, double> qs;
    for (auto& loc : n1_locs) {
        qs[loc.tid()] = get_quality(loc);
    }
    for (auto& loc : n12_locs) {
        auto it = qs.find(loc.tid());
        if (it != qs.end()) qs.erase(it);
    }

    collapse_cache.max_energy = 0;
    for (auto& q : qs) {
        if (q.second > collapse_cache.max_energy) collapse_cache.max_energy = q.second;
    }

    return true;
}

bool tetwild::TetWild::collapse_after(const Tuple& loc)
{
    if (!TetMesh::collapse_after(loc)) return false;

    auto locs = get_one_ring_tets_for_vertex(loc);

    ////check first
    // check inversion
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            return false;
        }
    }

    // check quality
    std::vector<double> qs;
    for (auto& loc : locs) {
        double q = get_quality(loc);
        if (q > collapse_cache.max_energy) {
            return false;
        }
        qs.push_back(q);
    }

    ////then update
    if (collapse_cache.edge_length > 0) {
        // todo: surface check
    } else {
        for (int i = 0; i < locs.size(); i++) {
            m_tet_attribute[locs[i].tid()].m_qualities = qs[i];
        }
    }

    return true;
}