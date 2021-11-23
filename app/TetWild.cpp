//
// Created by Yixin Hu on 11/3/21.
//

#include "TetWild.h"
#include "external/MshSaver.h"

#include <wmtk/AMIPS.h>

#include <igl/predicates/predicates.h>

bool tetwild::TetWild::is_inverted(size_t t_id)
{
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

double tetwild::TetWild::get_quality(size_t t_id)
{
	std::array<double, 12> T;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 3; j++) {
            T[i * 3 + j] = m_vertex_attribute[m_tet_connectivity[t_id][i]].m_posf[j];
        }
	}
	double energy = wmtk::AMIPS_energy(T);
	if (std::isinf(energy) || std::isnan(energy) || energy < 3 - 1e-3)
		return MAX_ENERGY;
	return energy;
}

void tetwild::TetWild::split_all_edges() {
    reset_timestamp();

    std::vector <Tuple> edges;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        for (int j = 0; j < 6; j++) {
            Tuple loc;
            int vid = m_tet_connectivity[i][l_edges[j][0]];
            int eid = j;
            int fid = map_edge2face[eid];
            int tid = i;
            edges.push_back(Tuple(vid, eid, fid, tid));
        }
    }
    std::sort(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
        return a.compare_edges(*this, b) < 0;
    });
    edges.erase(std::unique(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
        return a.compare_edges(*this, b) == 0;
    }), edges.end());

    cout << "edges.size() = " << edges.size() << endl;

    int cnt_suc = 0;
    std::priority_queue<wmtk::ElementInQueue, std::vector<wmtk::ElementInQueue>, wmtk::cmp_l> es_queue;
    for (auto& loc: edges) {
        int v1_id = m_tet_connectivity[loc.get_tid()][l_edges[loc.get_eid()][0]];
        int v2_id = m_tet_connectivity[loc.get_tid()][l_edges[loc.get_eid()][1]];
        double length = (m_vertex_attribute[v1_id].m_posf-m_vertex_attribute[v2_id].m_posf).squaredNorm();
        if (length < m_params.splitting_l2)
            continue;
        es_queue.push(wmtk::ElementInQueue(loc, length));
    }

    while(!es_queue.empty()){
        auto &loc = es_queue.top().edge;
//        double weight = es_queue.top().weight;
        es_queue.pop();

        //check timestamp
        if (loc.timestamp != m_tet_connectivity[loc.get_tid()].timestamp)
            continue;

        std::vector <Tuple> new_edges;
        if (split_edge(loc, new_edges)) {
            cnt_suc++;
            cout << "ok" << endl;
            cout << this->m_tet_connectivity.size() << endl;

            for(auto& new_loc:new_edges) {
                int v1_id = m_tet_connectivity[new_loc.get_tid()][l_edges[new_loc.get_eid()][0]];
                int v2_id = m_tet_connectivity[new_loc.get_tid()][l_edges[new_loc.get_eid()][1]];
                double length = (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf).squaredNorm();
                if (length < m_params.splitting_l2)
                    continue;
                es_queue.push(wmtk::ElementInQueue(new_loc, length));
            }
        }
    }
}

bool tetwild::TetWild::split_before(const Tuple &loc0){
	auto loc1 = loc0;
	int v1_id = loc1.get_vid();
	auto loc2 = loc1.switch_vertex(*this);
	int v2_id = loc2.get_vid();//todo: still need get id

//	double length = (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf).norm();
//	if (length < m_params.l * 4 / 3)
//		return false;

    split_cache.vertex_info.m_posf = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;

	return true;
}

bool tetwild::TetWild::split_after(const std::vector<Tuple> &locs){//input: locs pointing to a list of tets and v_id
    int v_id = locs[0].get_vid();
    auto old_pos = m_vertex_attribute[v_id].m_posf;
    m_vertex_attribute[v_id].m_posf = split_cache.vertex_info.m_posf;

    // check inversion
    for (auto &loc: locs) {
        size_t t_id = loc.get_tid();
        if (is_inverted(t_id)) {
            m_vertex_attribute[v_id].m_posf = old_pos;
            return false;
        }
    }

    // update quality
    for (auto &loc: locs) {
        size_t t_id = loc.get_tid();
        m_tet_attribute[t_id].m_qualities = get_quality(t_id);
    }

    return true;
}

void tetwild::TetWild::output_mesh(std::string file) {
    PyMesh::MshSaver mSaver(file, true);

    Eigen::VectorXd V_flat(3 * m_vertex_attribute.size());
    for (int i = 0; i < m_vertex_attribute.size(); i++) {
        for (int j = 0; j < 3; j++)
            V_flat(3 * i + j) = m_vertex_attribute[i].m_posf[j];
    }
    Eigen::VectorXi T_flat(4 * m_tet_connectivity.size());
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        for (int j = 0; j < 4; j++)
            T_flat(4 * i + j) = m_tet_connectivity[i][j];
    }

    mSaver.save_mesh(V_flat, T_flat, 3, mSaver.TET);
}
