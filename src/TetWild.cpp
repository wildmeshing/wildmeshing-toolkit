//
// Created by Yixin Hu on 11/3/21.
//

#include "TetWild.h"

bool wmtk::TetWild::is_inverted(size_t t_id){
    //todo

    return true;
}

double wmtk::TetWild::get_quality(size_t t_id){
    //todo

    return 0;
}

bool wmtk::TetWild::split_before(const Tuple &loc0) {
    auto loc1 = loc0;
    int v1_id = loc1.get_vid();
    auto loc2 = loc1.switch_vertex(*this);
    int v2_id = loc2.get_vid();

    double length = (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf).norm();
    if (length < m_params.l * 4 / 3)
        return false;

    return true;
}

bool wmtk::TetWild::split_after(const Tuple &loc0){
    //todo: compute pos for v_id

    //check inversion
    int v_id = loc0.get_vid();
    auto t_ids = loc0.get_conn_tets(*this);
    for(int t_id: t_ids){
        if(is_inverted(t_id))
            return false;
    }

    //update quality
    for(int t_id: t_ids){
        m_tet_attribute[t_id].m_qualities = get_quality(t_id);
    }

    return true;
}

