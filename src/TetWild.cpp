//
// Created by Yixin Hu on 11/3/21.
//

#include "TetWild.h"
#include "AMIPS.h"

#include <igl/predicates/predicates.h>
bool wmtk::TetWild::is_inverted(size_t t_id) {
    auto &p1 = m_vertex_attribute[m_tet_connectivity[t_id][0]].m_posf;
    auto &p2 = m_vertex_attribute[m_tet_connectivity[t_id][1]].m_posf;
    auto &p3 = m_vertex_attribute[m_tet_connectivity[t_id][2]].m_posf;
    auto &p4 = m_vertex_attribute[m_tet_connectivity[t_id][3]].m_posf;
    //
    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(p1, p2, p3, p4);
    Scalar result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result <= 0)
        return true;
    return false;
}

double wmtk::TetWild::get_quality(size_t t_id) {
    std::array<double, 12> T;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 3; j++)
            T[i * 4 + j] = m_vertex_attribute[m_tet_connectivity[t_id][i]].m_posf[j];
    }
    double energy = AMIPS_energy(T);
    if (std::isinf(energy) || std::isnan(energy) || energy < 3 - 1e-3)
        return MAX_ENERGY;
    return energy;
}

bool wmtk::TetWild::split_before(const Tuple &loc0, std::shared_ptr<InfoCache> info0) {
    auto loc1 = loc0;
    int v1_id = loc1.get_vid();
    auto loc2 = loc1.switch_vertex(*this);
    int v2_id = loc2.get_vid();

    double length = (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf).norm();
    if (length < m_params.l * 4 / 3)
        return false;

    std::shared_ptr<InfoCacheSplit> info = std::make_shared<InfoCacheSplit>();;
    info->mid_p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    info0 = info;

    return true;
}

bool wmtk::TetWild::split_after(const Tuple &loc0, std::shared_ptr<InfoCache> info0){
    std::shared_ptr<InfoCacheSplit> info = std::dynamic_pointer_cast<InfoCacheSplit> (info0);
    int v_id = loc0.get_vid();
    auto old_pos = m_vertex_attribute[v_id].m_posf;
    m_vertex_attribute[v_id].m_posf = info->mid_p;

    //check inversion
    auto locs = loc0.get_conn_tets(*this);
    for(auto& loc: locs){
        int t_id = loc.get_tid();
        if(is_inverted(t_id)) {
            m_vertex_attribute[v_id].m_posf = old_pos;
            return false;
        }
    }

    //update quality
    for(auto& loc: locs){
        int t_id = loc.get_tid();
        m_tet_attribute[t_id].m_qualities = get_quality(t_id);
    }

    return true;
}

