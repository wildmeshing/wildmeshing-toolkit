//
// Created by Yixin Hu on 11/3/21.
//

#include "TetWild.h"
#include "external/MshSaver.h"

#include <wmtk/AMIPS.h>
#include "Logger.hpp"

#include <igl/predicates/predicates.h>

bool tetwild::TetWild::is_inverted(const Tuple &loc)
{
//	size_t t_id = loc.tid(); // todo: remove

	// auto &p1 = m_vertex_attribute[m_tet_connectivity[t_id][0]].m_posf;
	// auto &p2 = m_vertex_attribute[m_tet_connectivity[t_id][1]].m_posf;
	// auto &p3 = m_vertex_attribute[m_tet_connectivity[t_id][2]].m_posf;
	// auto &p4 = m_vertex_attribute[m_tet_connectivity[t_id][3]].m_posf;
	// NO!!!!

//	auto &p1 = m_vertex_attribute[loc.vid()].m_posf;
//	auto &p2 = m_vertex_attribute[switch_vertex(loc).vid()].m_posf;
//	auto &p3 = m_vertex_attribute[switch_vertex(switch_edge(loc)).vid()].m_posf;
//	auto &p4 = m_vertex_attribute[switch_vertex(switch_edge(switch_face(loc))).vid()].m_posf;//todo: wrong orientation

    std::array<Vector3f, 4> ps;
    auto its = loc.iterate_tet_vertices(*this);
    for(int j=0;j<4;j++){
        ps[j] = m_vertex_attribute[its[j].vid()].m_posf;
    }

	//
	igl::predicates::exactinit();
	auto res = igl::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
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

double tetwild::TetWild::get_quality(const Tuple &loc)
{
//	size_t t_id = loc.tid(); // todo: remove

	std::array<double, 12> T;
//	auto &p1 = m_vertex_attribute[loc.vid()].m_posf;
//	auto &p2 = m_vertex_attribute[switch_vertex(loc).vid()].m_posf;
//	auto &p3 = m_vertex_attribute[switch_vertex(switch_edge(loc)).vid()].m_posf;
//	auto &p4 = m_vertex_attribute[switch_vertex(switch_edge(switch_face(loc))).vid()].m_posf;

    std::array<Vector3f, 4> ps;
    auto its = loc.iterate_tet_vertices(*this);
    for(int j=0;j<4;j++){
        ps[j] = m_vertex_attribute[its[j].vid()].m_posf;
    }

    for (int j = 0; j < 3; j++)
	{
        T[0 * 3 + j] = ps[0][j];
		T[1 * 3 + j] = ps[1][j];
		T[2 * 3 + j] = ps[2][j];
		T[3 * 3 + j] = ps[3][j];
	}

	double energy = wmtk::AMIPS_energy(T);
	if (std::isinf(energy) || std::isnan(energy) || energy < 3 - 1e-3)
		return MAX_ENERGY;
	return energy;
}

void tetwild::TetWild::split_all_edges()
{
	reset_timestamp();

	std::vector<Tuple> edges = get_edges();

	logger().debug("edges.size() = {}", edges.size());

	int cnt_suc = 0;
	std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> es_queue;
	for (auto &loc : edges)
	{
		Tuple &v1 = loc;
		Tuple v2 = loc.switch_vertex(*this);
		double length = (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
		if (length < m_params.splitting_l2)
			continue;
		es_queue.push(ElementInQueue(loc, length));
	}

    bool is_failed = false;
	while (!es_queue.empty())
	{
		auto loc = es_queue.top().edge;
		//        double weight = es_queue.top().weight;
		es_queue.pop();

		// check timestamp
		if (!loc.is_version_number_valid(*this))
			continue;

		std::vector<Tuple> new_edges;
		if (split_edge(loc, new_edges))
		{
			cnt_suc++;
            if(!is_failed) {
                for (auto &new_loc: new_edges) {
                    Tuple &v1 = new_loc;
                    Tuple v2 = new_loc.switch_vertex(*this);
                    double length = (m_vertex_attribute[v1.vid()].m_posf -
                                     m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
                    if (length < m_params.splitting_l2)
                        continue;
                    es_queue.push(ElementInQueue(new_loc, length));
                }
            }
		} else
            is_failed = true;

	}
}

bool tetwild::TetWild::split_before(const Tuple &loc0)
{
	auto loc1 = loc0;
	int v1_id = loc1.vid();
	auto loc2 = loc1.switch_vertex(*this);
	int v2_id = loc2.vid();

	//	double length = (m_vertex_attribute[v1_id].m_posf - m_vertex_attribute[v2_id].m_posf).norm();
	//	if (length < m_params.l * 4 / 3)
	//		return false;

	split_cache.vertex_info.m_posf = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;

	return true;
}

bool tetwild::TetWild::split_after(const std::vector<Tuple> &locs)
{ // input: locs pointing to a list of tets and v_id
	int v_id = locs[0].vid();
	auto old_pos = m_vertex_attribute[v_id].m_posf;
	m_vertex_attribute[v_id].m_posf = split_cache.vertex_info.m_posf;

	// check inversion
	for (auto &loc : locs)
	{
		if (is_inverted(loc))
		{
			m_vertex_attribute[v_id].m_posf = old_pos;
			return false;
		}
	}

	// update quality
	for (auto &loc : locs)
	{
		m_tet_attribute[loc.tid()].m_qualities = get_quality(loc);
	}

	return true;
}

void tetwild::TetWild::collapse_all_edges() {
    reset_timestamp();

    std::vector<Tuple> edges = get_edges();

    logger().debug("edges.size() = {}", edges.size());

    int cnt_suc = 0;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_s> ec_queue;
    for (auto &loc: edges) {
        Tuple &v1 = loc;
        Tuple v2 = loc.switch_vertex(*this);
        double length = (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
        if (length > m_params.collapsing_l2)
            continue;
        ec_queue.push(ElementInQueue(loc, length));
    }

    while (!ec_queue.empty()) {
        auto loc = ec_queue.top().edge;
        double weight = ec_queue.top().weight;
        ec_queue.pop();

        // check timestamp
        if (!loc.is_version_number_valid(*this))
            continue;
        if (!loc.is_valid(*this))
            continue;
        {//check weight
            Tuple &v1 = loc;
            Tuple v2 = loc.switch_vertex(*this);
            double length = (m_vertex_attribute[v1.vid()].m_posf - m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
            if (length != weight)
                continue;
        }

        std::vector<Tuple> new_edges;
        if (collapse_edge(loc, new_edges)) {
            cnt_suc++;
            for (auto &new_loc: new_edges) {
                Tuple &v1 = new_loc;
                Tuple v2 = new_loc.switch_vertex(*this);
                double length = (m_vertex_attribute[v1.vid()].m_posf -
                                 m_vertex_attribute[v2.vid()].m_posf).squaredNorm();
                if (length < m_params.collapsing_l2)
                    continue;
                ec_queue.push(ElementInQueue(new_loc, length));
            }
        }
    }
}

bool tetwild::TetWild::collapse_before(const Tuple &t) {
    //check if on bbox/surface/boundary
    
    return true;
}

bool tetwild::TetWild::collapse_after(const std::vector<Tuple> &locs) {
    return true;
}

void tetwild::TetWild::output_mesh(std::string file)
{
	PyMesh::MshSaver mSaver(file, true);

	Eigen::VectorXd V_flat(3 * m_vertex_attribute.size());
	for (int i = 0; i < m_vertex_attribute.size(); i++)
	{
		for (int j = 0; j < 3; j++)
			V_flat(3 * i + j) = m_vertex_attribute[i].m_posf[j];
	}

	Eigen::VectorXi T_flat(4 * n_tets());
	for (int i = 0; i < n_tets(); i++)
	{
		for (int j = 0; j < 4; j++)
			T_flat(4 * i + j) = v_id(i, j);
	}

	mSaver.save_mesh(V_flat, T_flat, 3, mSaver.TET);
}
