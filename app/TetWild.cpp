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
            edges.push_back(Tuple::get_edge_tuple(*this, i, j));

//            Tuple loc;
//            int vid = m_tet_connectivity[i][local_edges[j][0]];
//            int eid = j;
//            int fid = map_edge2face[eid];
//            int tid = i;
//            edges.push_back(Tuple(vid, eid, fid, tid));
        }
    }
    Tuple::unique_edge_tuples(*this, edges);
//    std::sort(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
//        return a.compare_edges(*this, b) < 0;
//    });
//    edges.erase(std::unique(edges.begin(), edges.end(), [&](const Tuple &a, const Tuple &b) {
//        return a.compare_edges(*this, b) == 0;
//    }), edges.end());

    cout << "edges.size() = " << edges.size() << endl;

    int cnt_suc = 0;
    std::priority_queue<ElementInQueue, std::vector<ElementInQueue>, cmp_l> es_queue;
    for (auto& loc: edges) {
        Tuple& v1 = loc;
        Tuple v2 = loc.switch_vertex(*this);
        double length = (m_vertex_attribute[v1.get_vid()].m_posf-m_vertex_attribute[v2.get_vid()].m_posf).squaredNorm();
        if (length < m_params.splitting_l2)
            continue;
        es_queue.push(ElementInQueue(loc, length));
    }

    while(!es_queue.empty()){
        auto &loc = es_queue.top().edge;
//        double weight = es_queue.top().weight;
        es_queue.pop();

        //check timestamp
        if (!loc.is_version_number_valid(*this))
            continue;

        std::vector <Tuple> new_edges;
        if (split_edge(loc, new_edges)) {
            cnt_suc++;
            for(auto& new_loc:new_edges) {
                Tuple& v1 = new_loc;
                Tuple v2 = new_loc.switch_vertex(*this);
                double length = (m_vertex_attribute[v1.get_vid()].m_posf-m_vertex_attribute[v2.get_vid()].m_posf).squaredNorm();
                if (length < m_params.splitting_l2)
                    continue;
                es_queue.push(ElementInQueue(new_loc, length));
            }
        }
    }
}

bool tetwild::TetWild::split_before(const Tuple &loc0){
	auto loc1 = loc0;
	int v1_id = loc1.get_vid();
	auto loc2 = loc1.switch_vertex(*this);
	int v2_id = loc2.get_vid();

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

bool tetwild::TetWild::smooth_before(const Tuple &t) {
	return true;
}

bool tetwild::TetWild::smooth_after(const std::vector<Tuple> &locs)
{
	// Newton iterations are encapsulated here.
	// TODO: tags. envelope check.
	using vec = Vector3f;
	auto v_id = locs.front().get_vid();

	std::vector<std::array<double, 12>> assembles(locs.size());
    auto loc_id = 0;
	for (auto &loc : locs)
	{
		auto &T = assembles[loc_id];
		auto t_id = loc.get_tid();
		assert(loc.get_vid() == v_id);
		// if local traversal is required, v0 (EV) v1 (EV) v2 (FEV) v3
		auto vl_id = m_tet_connectivity[t_id].find(v_id);

		for (auto i = 0; i < 4; i++) // assuming cyclic orientation preservation.
		{
			for (auto j = 0; j < 3; i++)
			{
				T[i * 3 + j] = m_vertex_attribute[m_tet_connectivity[t_id][(vl_id + i) % 4]].m_posf[j];
			}
		}
        loc_id ++;
	}

	// Compute New Coordinate.
	auto newton_direction = [&assembles](auto &pos) {
		auto total_energy = 0.;
		auto total_jac = vec();
		auto total_hess = Eigen::Matrix3d();

		// E = \sum_i E_i(x)
		// J = \sum_i J_i(x)
		// H = \sum_i H_i(x)
		auto local_id = 0;
		for (auto &T : assembles)
		{
			for (auto j = 0; j < 3; j++)
			{
				T[j] = pos[j]; // only filling the front point.
			}
			auto jac = decltype(total_jac)();
			auto hess = decltype(total_hess)();
			total_energy += wmtk::AMIPS_energy(T);
			wmtk::AMIPS_jacobian(T, jac);
			wmtk::AMIPS_hessian(T, hess);
            total_jac += jac;
            total_hess += hess;
			assert(!std::isnan(total_energy));
		}

		vec x = total_hess.ldlt().solve(total_jac);
		if (total_jac.isApprox(total_hess * x)) // a hacky PSD trick. TODO: change this.
			return -x;
		else
			return -total_jac;
	};
	auto compute_energy = [&assembles](const vec &pos) -> double {
		auto total_energy = 0.;
		for (auto &T : assembles)
		{
			for (auto j = 0; j < 3; j++)
			{
				T[j] = pos[j]; // only filling the front point x,y,z.
			}
			total_energy += wmtk::AMIPS_energy(T);
		}
		return total_energy;
	};
    auto linesearch = [&compute_energy](const vec &pos, const vec &dir, const int &max_iter) {
		auto lr = 0.8;
		auto old_energy = compute_energy(pos);
		for (auto iter = 1; iter <= max_iter; iter++)
		{
			vec newpos = pos + std::pow(lr, iter) * dir;
			if (compute_energy(newpos) < old_energy)
				return newpos; // TODO: armijo conditions.
		}
		return pos;
	};
	auto compute_new_valid_pos = [&linesearch, &newton_direction](const vec &old_pos) {
		auto current_pos = old_pos;
		auto line_search_iters = 12;
		auto newton_iters = 10;
		for (auto iter = 0; iter < newton_iters; iter++)
		{
			auto dir = newton_direction(current_pos);
			auto newpos = linesearch(current_pos, dir, line_search_iters);
			if ((newpos - current_pos).norm() < 1e-9) // barely moves
			{
				break;
			}
			current_pos = newpos;
		}
		return current_pos;
	};

	auto old_pos = m_vertex_attribute[v_id].m_posf;
	m_vertex_attribute[v_id].m_posf = compute_new_valid_pos(old_pos);

	// note: duplicate code snippets.
	for (auto &loc : locs)
	{
		auto t_id = loc.get_tid();
		if (is_inverted(t_id))
		{
			m_vertex_attribute[v_id].m_posf = old_pos;
			return false;
		}
	}
	for (auto &loc : locs)
	{
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
