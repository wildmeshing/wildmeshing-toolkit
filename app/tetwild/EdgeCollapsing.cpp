#include "TetWild.h"

#include <wmtk/utils/Logger.hpp>


void tetwild::TetWild::collapse_all_edges()
{
    std::vector<Tuple> edges = get_edges();

    wmtk::logger().debug("edges.size() = {}", edges.size());

    int cnt_suc = 0;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue(cmp_s(*this));
    for (auto& loc : edges) {
        Tuple& v1 = loc;
        Tuple v2 = loc.switch_vertex(*this);
        double length =
            (m_vertex_attribute[v1.vid(*this)].m_posf - m_vertex_attribute[v2.vid(*this)].m_posf)
                .squaredNorm();
        if (length > m_params.collapsing_l2) continue;
        ec_queue.push(ElementInQueue(loc, length));
    }

    while (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();

        // check hash
        if (!loc.is_valid(*this)) continue;
        { // check weight
            Tuple& v1 = loc;
            Tuple v2 = loc.switch_vertex(*this);
            double length = (m_vertex_attribute[v1.vid(*this)].m_posf -
                             m_vertex_attribute[v2.vid(*this)].m_posf)
                                .squaredNorm();
            if (length != weight) continue;
        }

        std::vector<Tuple> new_edges;
        if (collapse_edge(loc, new_edges)) {
            cnt_suc++;
            for (auto& new_loc : new_edges) {
                Tuple& v1 = new_loc;
                Tuple v2 = new_loc.switch_vertex(*this);
                double length = (m_vertex_attribute[v1.vid(*this)].m_posf -
                                 m_vertex_attribute[v2.vid(*this)].m_posf)
                                    .squaredNorm();
                if (length < m_params.collapsing_l2) continue;
                ec_queue.push(ElementInQueue(new_loc, length));
            }
        }
    }
}

bool tetwild::TetWild::collapse_before(const Tuple& loc) // input is an edge
{
    //check if on bbox/surface/boundary
    // todo: store surface info into cache

    int v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    int v2_id = loc1.vid(*this);
    collapse_cache.edge_length =
        (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf)
            .norm(); // todo: duplicated computation

    auto n1_locs = get_one_ring_tets_for_vertex(loc);
    auto n12_locs = get_incident_tets_for_edge(loc); // todo: duplicated computation

    std::map<int, double> qs;
    for (auto& l : n1_locs) {
        qs[l.tid(*this)] = get_quality(l);
    }
    for (auto& l : n12_locs) {
        auto it = qs.find(l.tid(*this));
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
    for (auto& l : locs) {
        if (is_inverted(l)) {
            return false;
        }
    }

    // check quality
    std::vector<double> qs;
    for (auto& l : locs) {
        double q = get_quality(l);
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
            m_tet_attribute[locs[i].tid(*this)].m_qualities = qs[i];
        }
    }

    return true;
}
