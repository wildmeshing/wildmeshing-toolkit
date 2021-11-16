//
// Created by Yixin Hu on 11/3/21.
//

#include <wmtk/TetMesh.h>
#include <wmtk/Logger.hpp>

int wmtk::TetMesh::find_next_empty_slot_t()
{
	for (int i = m_t_empty_slot; i < m_tet_connectivity.size(); i++)
	{
		if (m_tet_connectivity[i].m_is_removed)
		{
			m_t_empty_slot = i + 1;
			return i;
		}
	}
	m_tet_connectivity.emplace_back();
	return m_tet_connectivity.size() - 1;
}

int wmtk::TetMesh::find_next_empty_slot_v()
{
	for (int i = m_v_empty_slot; i < m_vertex_connectivity.size(); i++)
	{
		if (m_vertex_connectivity[i].m_is_removed)
		{
			m_v_empty_slot = i + 1;
			return i;
		}
	}
	m_vertex_connectivity.emplace_back();
	return m_vertex_connectivity.size() - 1;
}

void wmtk::TetMesh::split_all_edges()
{
	std::vector<std::array<size_t, 2>> edges;
	for (int i = 0; i < m_tet_connectivity.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			std::array<size_t, 2> e = {{m_tet_connectivity[i][0], m_tet_connectivity[i][j + 1]}};
			if (e[0] > e[1])
				std::swap(e[0], e[1]);
			edges.push_back(e);
			e = {{m_tet_connectivity[i][j + 1], m_tet_connectivity[i][(j + 1) % 3 + 1]}};
			if (e[0] > e[1])
				std::swap(e[0], e[1]);
		}
	}
	vector_unique(edges);

	int cnt_suc = 0;
	for (const auto &e : edges)
	{
		// todo: convenient way to convert e into tuple?
		auto n12_t_ids = set_intersection(m_vertex_connectivity[e[0]].m_conn_tets,
										  m_vertex_connectivity[e[1]].m_conn_tets);
		int tid = n12_t_ids.front();
		Tuple loc(e[0], 0, 0, tid);
		loc.set_l_eid(*this, {e[0], e[1]});
		loc.set_l_fid(*this, {e[0], e[1]});
		if (split_edge(loc))
			cnt_suc++;
	}

	logger().info("{} {}", cnt_suc, edges.size());
}

bool wmtk::TetMesh::split_edge(const Tuple &loc0)
{
	if (!split_before(loc0))
		return false;

	// backup of everything
	auto loc1 = loc0;
	int v1_id = loc1.get_vid();
	auto loc2 = loc1.switch_vertex(*this);
	int v2_id = loc2.get_vid();
	//
	auto n12_t_ids = set_intersection(m_vertex_connectivity[v1_id].m_conn_tets,
									  m_vertex_connectivity[v2_id].m_conn_tets);
	std::vector<size_t> n12_v_ids;
	for (size_t t_id : n12_t_ids)
	{
		for (int j = 0; j < 4; j++)
			n12_v_ids.push_back(m_tet_connectivity[t_id][j]);
	}
	vector_unique(n12_v_ids);
	std::vector<std::pair<size_t, TetrahedronConnectivity>> old_tets(n12_t_ids.size());
	std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(n12_v_ids.size());
	for (size_t t_id : n12_t_ids)
		old_tets.push_back(std::make_pair(t_id, m_tet_connectivity[t_id]));
	for (size_t v_id : n12_v_ids)
		old_vertices.push_back(std::make_pair(v_id, m_vertex_connectivity[v_id]));

	// update connectivity
	int v_id = find_next_empty_slot_v();
	std::vector<size_t> new_t_ids;
	for (size_t t_id : n12_t_ids)
	{
		size_t new_t_id = find_next_empty_slot_t();
		new_t_ids.push_back(new_t_id);
		//
		int j = m_tet_connectivity[t_id].find(v1_id);
		m_tet_connectivity[new_t_id] = m_tet_connectivity[t_id];
		m_tet_connectivity[new_t_id][j] = v_id;
		//
		m_vertex_connectivity[v_id].m_conn_tets.push_back(t_id);
		m_vertex_connectivity[v_id].m_conn_tets.push_back(new_t_id);
		//
		for (int j = 0; j < 4; j++)
		{
			if (m_tet_connectivity[t_id][j] != v1_id && m_tet_connectivity[t_id][j] != v2_id)
				m_vertex_connectivity[m_tet_connectivity[t_id][j]].m_conn_tets.push_back(new_t_id);
		}
		//
		j = m_tet_connectivity[t_id].find(v2_id);
		m_tet_connectivity[t_id][j] = v_id;
		//
		vector_erase(m_vertex_connectivity[v1_id].m_conn_tets, t_id);
	}

	// possibly call the resize_attributes
	resize_attributes(m_vertex_connectivity.size(), m_tet_connectivity.size() * 6,
					  m_tet_connectivity.size() * 4, m_tet_connectivity.size());

	Tuple loc(v_id, 0, 0, old_tets.front().first);
	if (!split_after(loc))
	{
		m_vertex_connectivity[v_id].m_is_removed = true;
		for (int t_id : new_t_ids)
			m_tet_connectivity[t_id].m_is_removed = true;
		//
		for (int i = 0; i < old_tets.size(); i++)
		{
			int t_id = old_tets[i].first;
			m_tet_connectivity[t_id] = old_tets[i].second;
		}
		for (int i = 0; i < old_vertices.size(); i++)
		{
			int v_id = old_vertices[i].first;
			m_vertex_connectivity[v_id] = old_vertices[i].second;
		}

		return false;
	}

	// call invariants on all entities
	if (false) // if any invariant fails
	{
		// undo changes
		return false;
	}

	return true;
}
