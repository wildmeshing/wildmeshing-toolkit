//
// Created by Yixin Hu on 12/7/21.
//

#include "Logger.hpp"
#include "TetWild.h"

void tetwild::TetWild::split_all_edges()
{
    //    compact();

    std::vector<Tuple> edges = get_edges();

    apps::logger().debug("edges.size() = {}", edges.size());

    int cnt_suc = 0;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> es_queue;
    for (auto& loc : edges) {
        Tuple& v1 = loc;
        Tuple v2 = loc.switch_vertex(*this);
        double length = (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf)
                            .squaredNorm();
        if (length < m_params.splitting_l2) continue;
        es_queue.push(ElementInQueue(loc, length));
    }

    bool is_failed = false;
    while (!es_queue.empty()) {
        auto loc = es_queue.top().edge;
        //        double weight = es_queue.top().weight;
        es_queue.pop();

        // check timestamp
        if (!loc.is_valid(*this)) continue;

        std::vector<Tuple> new_edges;
        if (split_edge(loc, new_edges)) {
            cnt_suc++;
            if (!is_failed) {
                for (auto& new_loc : new_edges) {
                    Tuple& v1 = new_loc;
                    Tuple v2 = new_loc.switch_vertex(*this);
                    double length =
                        (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf)
                            .squaredNorm();
                    if (length < m_params.splitting_l2) continue;
                    es_queue.push(ElementInQueue(new_loc, length));
                }
            }
        } else
            is_failed = true;
    }
}

bool tetwild::TetWild::split_before(const Tuple& loc0)
{
    auto loc1 = loc0;
    int v1_id = loc1.vid();
    auto loc2 = loc1.switch_vertex(*this);
    int v2_id = loc2.vid();

    //	double length = (m_vertex_attribute[v1_id].m_posf -
    // m_vertex_attribute[v2_id].m_posf).norm();
    //	if (length < m_params.l * 4 / 3)
    //		return false;

    split_cache.vertex_info.m_posf =
        (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;

    return true;
}

bool tetwild::TetWild::split_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TetMesh::split_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);

    int v_id = loc.vid();
    auto old_pos = m_vertex_attribute[v_id].m_posf;
    m_vertex_attribute[v_id].m_posf = split_cache.vertex_info.m_posf;

    // check inversion
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[v_id].m_posf = old_pos;
            return false;
        }
    }

    // update quality
    for (auto& loc : locs) {
        m_tet_attribute[loc.tid()].m_qualities = get_quality(loc);
    }

    return true;
}
