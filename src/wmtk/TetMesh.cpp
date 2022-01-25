#include <wmtk/TetMesh.h>

#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <Tracy.hpp>

int wmtk::TetMesh::get_next_empty_slot_t()
{
    ZoneScoped;
    const auto it = m_tet_connectivity.emplace_back();
    const size_t size = std::distance(m_tet_connectivity.begin(), it) + 1;
    m_tet_connectivity[size - 1].hash = -1;
    p_edge_attrs->resize(size * 6);
    p_face_attrs->resize(size * 4);
    p_tet_attrs->resize(size);
    return size - 1;
}

int wmtk::TetMesh::get_next_empty_slot_v()
{
    ZoneScoped;
    const auto it = m_vertex_connectivity.emplace_back();
    const size_t size = std::distance(m_vertex_connectivity.begin(), it) + 1;
    p_vertex_attrs->resize(size);
    resize_vertex_mutex(size); // TODO: temp hack for mutex
    return size - 1;
}

wmtk::TetMesh::TetMesh()
{
    p_vertex_attrs = &vertex_attrs;
    p_edge_attrs = &edge_attrs;
    p_face_attrs = &face_attrs;
    p_tet_attrs = &tet_attrs;
}

void wmtk::TetMesh::init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets)
{
    ZoneScoped;
    m_vertex_connectivity.resize(n_vertices);
    m_tet_connectivity.resize(tets.size());
    for (int i = 0; i < tets.size(); i++) {
        m_tet_connectivity[i].m_indices = tets[i];
        for (int j = 0; j < 4; j++) {
            assert(tets[i][j] < m_vertex_connectivity.size());
            m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
        }
    }
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_edges() const
{
    ZoneScoped;
    std::vector<TetMesh::Tuple> edges;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 6; j++) {
            auto tup = tuple_from_edge(i, j);
            if (tup.eid(*this) == 6 * i + j) edges.emplace_back(tup);
        }
    }

    return edges;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_faces() const
{
    ZoneScoped;
    auto faces = std::vector<TetMesh::Tuple>();
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) {
            auto face_t = tuple_from_face(i, j);
            if (face_t.fid(*this) == 4 * i + j) faces.emplace_back(face_t);
        }
    }

    return faces;
}


bool wmtk::TetMesh::check_mesh_connectivity_validity() const
{
    ZoneScoped;
    std::vector<std::vector<size_t>> conn_tets(m_vertex_connectivity.size());
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) conn_tets[m_tet_connectivity[i][j]].push_back(i);
    }


    for (auto& tets : conn_tets) {
        auto tmp = tets;
        vector_unique(tets);
        assert(tmp == tets);
    }

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        assert(!m_vertex_connectivity[i].m_conn_tets.empty());
        assert(m_vertex_connectivity[i].m_conn_tets == conn_tets[i]);
    }

    // check is_removed
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++)
            assert(
                !m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed &&
                "m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed");
    }
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        for (int tid : m_vertex_connectivity[i].m_conn_tets)
            assert(!m_tet_connectivity[tid].m_is_removed && "m_tet_connectivity[tid].m_is_removed");
    }

    // check tuple
    for (size_t i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        Tuple loc = tuple_from_vertex(i);
        check_tuple_validity(loc);
        //
        Tuple locv = switch_vertex(loc);
        Tuple loce = switch_edge(loc);
        Tuple locf = switch_face(loc);
        auto loct = switch_tetrahedron(loc);
        check_tuple_validity(locv);
        check_tuple_validity(loce);
        check_tuple_validity(locf);
        if (loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }
    for (size_t i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        Tuple loc = tuple_from_tet(i);
        check_tuple_validity(loc);
        //
        Tuple locv = switch_vertex(loc);
        Tuple loce = switch_edge(loc);
        Tuple locf = switch_face(loc);
        auto loct = switch_tetrahedron(loc);
        check_tuple_validity(locv);
        check_tuple_validity(loce);
        check_tuple_validity(locf);
        if (loct.has_value()) {
            check_tuple_validity(loct.value());
        }
    }

    return true;
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_tets() const
{
    ZoneScoped;
    std::vector<TetMesh::Tuple> tets;
    for (auto i = 0; i < m_tet_connectivity.size(); i++) {
        auto& t = m_tet_connectivity[i];
        if (t.m_is_removed) continue;
        tets.emplace_back(tuple_from_tet(i));

        assert(tets.back().tid(*this) == i);
        assert(tets.back().is_valid(*this));
    }
    return tets;
}


std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_vertices() const
{
    ZoneScoped;
    std::vector<TetMesh::Tuple> verts;
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        auto& vc = m_vertex_connectivity[i];
        if (vc.m_is_removed) continue;
        assert(!vc.m_conn_tets.empty());

        verts.emplace_back(tuple_from_vertex(i));

        assert(verts.back().vid(*this) == i);
        assert(verts.back().is_valid(*this));
    }
    return verts;
}

bool wmtk::TetMesh::smooth_vertex(const Tuple& loc0)
{
    ZoneScoped;
    if (!smooth_before(loc0)) return false;
    start_protect_attributes();
    if (!smooth_after(loc0) || !invariants(get_one_ring_tets_for_vertex(loc0))) {
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();

    return true;
}


wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_edge(size_t tid, int local_eid) const
{
    ZoneScoped;
    assert(tid >= 0 && tid < m_tet_connectivity.size());
    assert(local_eid >= 0 && local_eid < m_local_edges.size());

    int vid = m_tet_connectivity[tid][m_local_edges[local_eid][0]];
    int fid = m_map_edge2face[local_eid];
    return Tuple(*this, vid, local_eid, fid, tid);
}

wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_face(size_t tid, int local_fid) const
{
    ZoneScoped;
    assert(tid >= 0 && tid < m_tet_connectivity.size());
    assert(local_fid >= 0 && local_fid < m_local_faces.size());

    int vid = m_tet_connectivity[tid][m_local_faces[local_fid][0]];
    int lvid1 = m_local_faces[local_fid][0];
    int lvid2 = m_local_faces[local_fid][1];
    int eid = -1;
    for (int i = 0; i < 6; i++) {
        if ((m_local_edges[i][0] == lvid1 && m_local_edges[i][1] == lvid2) ||
            (m_local_edges[i][0] == lvid2 && m_local_edges[i][1] == lvid1)) {
            eid = i;
            break;
        }
    }
    int fid = m_map_edge2face[local_fid];
    return Tuple(*this, vid, eid, local_fid, tid);
}

std::tuple<wmtk::TetMesh::Tuple, size_t> wmtk::TetMesh::tuple_from_face(
    const std::array<size_t, 3>& vids) const
{
    ZoneScoped;
    auto tmp = set_intersection(
        m_vertex_connectivity[vids[0]].m_conn_tets,
        m_vertex_connectivity[vids[1]].m_conn_tets);
    auto n12_t_ids = set_intersection(tmp, m_vertex_connectivity[vids[2]].m_conn_tets);
    assert(n12_t_ids.size() == 1 || n12_t_ids.size() == 2);

    // tid
    Tuple face;
    face.m_global_tid = n12_t_ids[0];
    if (n12_t_ids.size() > 1 && n12_t_ids[1] < n12_t_ids[0]) face.m_global_tid = n12_t_ids[1];
    // fid
    std::array<int, 3> f;
    for (int j = 0; j < 3; j++) {
        f[j] = m_tet_connectivity[face.m_global_tid].find(vids[j]);
    }
    std::sort(f.begin(), f.end());
    face.m_local_fid =
        std::find(m_local_faces.begin(), m_local_faces.end(), f) - m_local_faces.begin();
    // eid
    face.m_local_eid = m_local_edges_in_a_face[face.m_local_fid][0];
    // vid
    face.m_global_vid = m_tet_connectivity[face.m_global_tid][m_local_edges[face.m_local_eid][0]];

    size_t global_fid = face.m_global_tid * 4 + face.m_local_fid;

    return std::make_tuple(face, global_fid);
}

wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_vertex(size_t vid) const
{
    ZoneScoped;
    assert(vid >= 0 && vid < m_vertex_connectivity.size());

    int tid = m_vertex_connectivity[vid].m_conn_tets[0];
    int j = m_tet_connectivity[tid].find(vid);
    int eid = m_map_vertex2edge[j];
    int fid = m_map_edge2face[eid];
    return Tuple(*this, vid, eid, fid, tid);
}

wmtk::TetMesh::Tuple wmtk::TetMesh::tuple_from_tet(size_t tid) const
{
    ZoneScoped;
    assert(tid >= 0 && tid < m_tet_connectivity.size());

    int vid = m_tet_connectivity[tid][0];
    int eid = m_map_vertex2edge[0];
    int fid = m_map_edge2face[eid];
    return Tuple(*this, vid, eid, fid, tid);
}


std::array<wmtk::TetMesh::Tuple, 4> wmtk::TetMesh::oriented_tet_vertices(const Tuple& t) const
{
    ZoneScoped;
    std::array<Tuple, 4> vs;
    for (int j = 0; j < 4; j++) {
        vs[j].m_global_vid = m_tet_connectivity[t.m_global_tid][j];
        vs[j].m_local_eid = m_map_vertex2edge[j];
        vs[j].m_local_fid = m_map_edge2face[vs[j].m_local_eid];
        vs[j].m_global_tid = t.m_global_tid;
    }
    return vs;
}

std::array<wmtk::TetMesh::Tuple, 3> wmtk::TetMesh::get_face_vertices(const Tuple& t) const
{
    ZoneScoped;
    std::array<Tuple, 3> vs;
    vs[0] = t;
    vs[1] = switch_vertex(t);
    vs[2] = switch_vertex(switch_edge(t));
    return vs;
}

std::array<wmtk::TetMesh::Tuple, 6> wmtk::TetMesh::tet_edges(const Tuple& t) const
{
    ZoneScoped;
    std::array<Tuple, 6> es;
    for (int j = 0; j < 6; j++) {
        es[j].m_local_eid = j;
        es[j].m_local_fid = m_map_edge2face[j];

        es[j].m_global_vid = m_tet_connectivity[t.m_global_tid][m_local_edges[j][0]];
        es[j].m_global_tid = t.m_global_tid;
    }
    return es;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_one_ring_tets_for_vertex(const Tuple& t) const
{
    ZoneScoped;
    std::vector<Tuple> tets;
    for (int t_id : m_vertex_connectivity[t.m_global_vid].m_conn_tets) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_one_ring_vertices_for_vertex(
    const Tuple& t) const
{
    ZoneScoped;
    std::vector<size_t> v_ids;
    for (int t_id : m_vertex_connectivity[t.m_global_vid].m_conn_tets) {
        for (int j = 0; j < 4; j++) {
            v_ids.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    vector_unique(v_ids);
    vector_erase(v_ids, t.m_global_vid);
    std::vector<Tuple> vertices;
    for (auto v_id : v_ids) {
        vertices.push_back(tuple_from_vertex(v_id));
    }
    return vertices;
}

std::vector<size_t> wmtk::TetMesh::get_one_ring_vids_for_vertex(size_t vid) const
{
    ZoneScoped;
    std::vector<size_t> v_ids;
    for (int t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (int j = 0; j < 4; j++) {
            v_ids.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    vector_unique(v_ids);
    vector_erase(v_ids, vid);
    return v_ids;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_incident_tets_for_edge(const Tuple& t) const
{
    ZoneScoped;
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];

    auto tids = set_intersection(
        m_vertex_connectivity[v1_id].m_conn_tets,
        m_vertex_connectivity[v2_id].m_conn_tets);
    std::vector<Tuple> tets;
    for (int t_id : tids) {
        tets.push_back(tuple_from_tet(t_id));
    }
    return tets;
}

std::vector<wmtk::TetMesh::Tuple> wmtk::TetMesh::get_one_ring_tets_for_edge(const Tuple& t) const
{
    ZoneScoped;
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];

    auto tids = m_vertex_connectivity[v1_id].m_conn_tets;
    tids.insert(
        tids.end(),
        m_vertex_connectivity[v2_id].m_conn_tets.begin(),
        m_vertex_connectivity[v1_id].m_conn_tets.end());
    vector_unique(tids);

    std::vector<Tuple> tets;
    for (int t_id : tids) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}


void wmtk::TetMesh::consolidate_mesh()
{
    ZoneScoped;
    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(m_vertex_connectivity.size(), -1);
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(m_tet_connectivity.size(), -1);
    for (auto i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }

    v_cnt = 0;
    for (auto i = 0; i < m_vertex_connectivity.size(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            m_vertex_connectivity[v_cnt] = m_vertex_connectivity[i];
            p_vertex_attrs->move(i, v_cnt);
        }
        for (size_t& t_id : m_vertex_connectivity[v_cnt].m_conn_tets) t_id = map_t_ids[t_id];
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < m_tet_connectivity.size(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            m_tet_connectivity[t_cnt] = m_tet_connectivity[i];
            m_tet_connectivity[t_cnt].hash = 0;
            p_tet_attrs->move(i, t_cnt);

            for (auto j = 0; j < 4; j++) {
                p_face_attrs->move(i * 4 + j, t_cnt * 4 + j);
            }
            for (auto j = 0; j < 6; j++) {
                p_edge_attrs->move(i * 6 + j, t_cnt * 6 + j);
            }
        }
        for (size_t& v_id : m_tet_connectivity[t_cnt].m_indices) v_id = map_v_ids[v_id];
        t_cnt++;
    }

    m_vertex_connectivity.resize(v_cnt);
    m_tet_connectivity.resize(t_cnt);

    p_vertex_attrs->resize(v_cnt);
    p_edge_attrs->resize(6 * t_cnt);
    p_face_attrs->resize(4 * t_cnt);
    p_tet_attrs->resize(t_cnt);

    assert(check_mesh_connectivity_validity());
}

std::vector<std::array<size_t, 3>> wmtk::TetMesh::vertex_adjacent_boundary_faces(
    const Tuple& tup) const
{
    ZoneScoped;
    auto v = tup.vid(*this);
    auto result_faces = std::set<std::array<size_t, 3>>();
    for (auto t : m_vertex_connectivity[v].m_conn_tets) {
        auto& tet = m_tet_connectivity[t];
        for (auto j = 0; j < 4; j++) {
            if (tet[m_map_vertex2oppo_face[j]] == v) continue; // only consider those not connecting to it.
            auto& f = m_local_faces[j];
            auto face = std::array<size_t, 3>{{tet[f[0]], tet[f[1]], tet[f[2]]}};
            std::sort(face.begin(), face.end());
            auto it = result_faces.find(face);
            if (it == result_faces.end()) { // delete those appearing twice.
                result_faces.insert(face);
            } else {
                result_faces.erase(it);
            }
        }
    }
    return std::vector<std::array<size_t,3>>(result_faces.begin(), result_faces.end());
}
