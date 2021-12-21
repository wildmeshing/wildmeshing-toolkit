//
// Created by Yixin Hu on 12/7/21.
//

#include "TetWild.h"
#include "Logger.hpp"

void tetwild::TetWild::collapse_all_edges()
{
    reset_timestamp();

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
        if (!loc.is_version_number_valid(*this)) continue;
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
        }
    }
}

bool tetwild::TetWild::collapse_before(const Tuple& t)
{
    //check if on bbox/surface/boundary

    return true;
}

bool tetwild::TetWild::collapse_after(const std::vector<Tuple>& locs)
{
    return true;
}