#include <wmtk/TetMesh.h>

#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <tbb/parallel_for.h>

namespace wmtk {

int TetMesh::get_next_empty_slot_t()
{
    while (current_tet_size + MAX_THREADS >= m_tet_connectivity.size() ||
           tet_connectivity_synchronizing_flag) {
        if (tet_connectivity_lock.try_lock()) {
            if (current_tet_size + MAX_THREADS < m_tet_connectivity.size()) {
                tet_connectivity_lock.unlock();
                break;
            }
            tet_connectivity_synchronizing_flag = true;
            auto current_capacity = m_tet_connectivity.size();
            if (p_edge_attrs) {
                p_edge_attrs->resize(2 * current_capacity * 6);
            }
            if (p_face_attrs) {
                p_face_attrs->resize(2 * current_capacity * 4);
            }
            if (p_tet_attrs) {
                p_tet_attrs->resize(2 * current_capacity);
            }
            m_tet_connectivity.grow_to_at_least(2 * current_capacity);
            tet_connectivity_synchronizing_flag = false;
            tet_connectivity_lock.unlock();
            break;
        }
    }

    auto new_idx = current_tet_size++;
    m_tet_connectivity[new_idx].hash = -1;

    return new_idx;
}

int TetMesh::get_next_empty_slot_v()
{
    while (current_vert_size + MAX_THREADS >= m_vertex_connectivity.size() ||
           vertex_connectivity_synchronizing_flag) {
        if (vertex_connectivity_lock.try_lock()) {
            if (current_vert_size + MAX_THREADS < m_vertex_connectivity.size()) {
                vertex_connectivity_lock.unlock();
                break;
            }
            vertex_connectivity_synchronizing_flag = true;
            auto current_capacity = m_vertex_connectivity.size();
            if (p_vertex_attrs) {
                p_vertex_attrs->resize(2 * current_capacity);
            }
            resize_vertex_mutex(2 * current_capacity);
            m_vertex_connectivity.grow_to_at_least(2 * current_capacity);
            vertex_connectivity_synchronizing_flag = false;
            vertex_connectivity_lock.unlock();
            break;
        }
    }

    return current_vert_size++;
}

TetMesh::TetMesh()
{
    // p_vertex_attrs = &vertex_attrs;
    // p_edge_attrs = &edge_attrs;
    // p_face_attrs = &face_attrs;
    // p_tet_attrs = &tet_attrs;
}

void TetMesh::init(size_t n_vertices, const std::vector<std::array<size_t, 4>>& tets)
{
    m_vertex_connectivity.resize(n_vertices);
    m_tet_connectivity.resize(tets.size());
    current_vert_size = n_vertices;
    current_tet_size = tets.size();
    for (int i = 0; i < tets.size(); i++) {
        m_tet_connectivity[i].m_indices = tets[i];
        for (int j = 0; j < 4; j++) {
            assert(tets[i][j] < vert_capacity());
            m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
        }
    }

    // concurrent
    m_vertex_mutex.grow_to_at_least(n_vertices);

    // resize attributes
    if (p_vertex_attrs) {
        p_vertex_attrs->resize(n_vertices);
    }
    if (p_tet_attrs) {
        p_tet_attrs->resize(tets.size());
    }
    if (p_face_attrs) {
        p_face_attrs->resize(4 * tets.size());
    }
    if (p_edge_attrs) {
        p_edge_attrs->resize(6 * tets.size());
    }
}

void TetMesh::init_with_isolated_vertices(
    size_t n_vertices,
    const std::vector<std::array<size_t, 4>>& tets)
{
    m_vertex_connectivity.clear();
    m_vertex_connectivity.resize(n_vertices);
    m_tet_connectivity.clear();
    m_tet_connectivity.resize(tets.size());
    current_vert_size = n_vertices;
    current_tet_size = tets.size();
    for (size_t i = 0; i < tets.size(); i++) {
        m_tet_connectivity[i].m_indices = tets[i];
        for (int j = 0; j < 4; j++) {
            assert(tets[i][j] < vert_capacity());
            m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
        }
    }

    for (size_t i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_conn_tets.empty()) {
            m_vertex_connectivity[i].m_is_removed = true;
        }
    }

    // concurrent
    m_vertex_mutex.grow_to_at_least(n_vertices);

    // resize attributes
    if (p_vertex_attrs) {
        p_vertex_attrs->clear();
        p_vertex_attrs->resize(n_vertices);
    }
    if (p_tet_attrs) {
        p_tet_attrs->clear();
        p_tet_attrs->resize(tets.size());
    }
    if (p_face_attrs) {
        p_face_attrs->clear();
        p_face_attrs->resize(4 * tets.size());
    }
    if (p_edge_attrs) {
        p_edge_attrs->clear();
        p_edge_attrs->resize(6 * tets.size());
    }
}

void TetMesh::init(const MatrixXi& T)
{
    size_t n_vertices = T.maxCoeff() + 1;

    std::vector<std::array<size_t, 4>> tets;
    tets.resize(T.rows());

    for (int i = 0; i < T.rows(); ++i) {
        for (int j = 0; j < 4; ++j) {
            tets[i][j] = T(i, j);
        }
    }

    TetMesh::init_with_isolated_vertices(n_vertices, tets);
}


std::vector<TetMesh::Tuple> TetMesh::get_edges() const
{
    std::vector<std::tuple<size_t, size_t, TetMesh::Tuple>> edges;
    edges.reserve(tet_capacity() * 6);
    for (int i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 6; j++) {
            auto tup = tuple_from_edge(i, j);
            auto v0 = tup.vid(*this);
            auto v1 = tup.switch_vertex(*this).vid(*this);
            if (v0 > v1) std::swap(v0, v1);
            edges.emplace_back(v0, v1, tup);
        }
    }
    std::sort(edges.begin(), edges.end(), [](auto& a, auto& b) {
        return std::tie(std::get<0>(a), std::get<1>(a)) < std::tie(std::get<0>(b), std::get<1>(b));
    });
    edges.erase(
        std::unique(
            edges.begin(),
            edges.end(),
            [](auto& a, auto& b) {
                return std::tie(std::get<0>(a), std::get<1>(a)) ==
                       std::tie(std::get<0>(b), std::get<1>(b));
            }),
        edges.end());
    std::vector<TetMesh::Tuple> uniq_edges;
    uniq_edges.reserve(edges.size());
    for (auto& [v0, v1, e] : edges) uniq_edges.push_back(e);
    return uniq_edges;
}


void TetMesh::for_each_face(const std::function<void(const TetMesh::Tuple&)>& func)
{
    for (int i = 0; i < tet_capacity(); i++) {
        if (!tuple_from_tet(i).is_valid(*this)) continue;
        for (int j = 0; j < 4; j++) {
            auto tup = tuple_from_face(i, j);
            if (tup.fid(*this) == 4 * i + j) {
                func(tup);
            }
        }
    }
}


std::vector<TetMesh::Tuple> TetMesh::get_faces() const
{
    auto faces = std::vector<TetMesh::Tuple>();
    for (int i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) {
            auto face_t = tuple_from_face(i, j);
            if (face_t.fid(*this) == 4 * i + j) faces.emplace_back(face_t);
        }
    }

    return faces;
}


bool TetMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tets(vert_capacity());
    for (size_t i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++) conn_tets[m_tet_connectivity[i][j]].push_back(i);
    }


    for (auto& tets : conn_tets) {
        auto tmp = tets;
        vector_unique(tets);
        assert(tmp == tets);
    }

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        assert(!m_vertex_connectivity[i].m_conn_tets.empty());
        assert(m_vertex_connectivity[i].m_conn_tets == conn_tets[i]);
    }

    // check is_removed
    for (size_t i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 4; j++)
            assert(
                !m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed &&
                "m_vertex_connectivity[m_tet_connectivity[i][j]].m_is_removed");
    }
    for (size_t i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        for (int tid : m_vertex_connectivity[i].m_conn_tets)
            assert(!m_tet_connectivity[tid].m_is_removed && "m_tet_connectivity[tid].m_is_removed");
    }

    // check tuple
    for (size_t i = 0; i < vert_capacity(); i++) {
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
    for (size_t i = 0; i < tet_capacity(); i++) {
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


std::vector<TetMesh::Tuple> TetMesh::get_tets() const
{
    std::vector<TetMesh::Tuple> tets;
    for (auto i = 0; i < tet_capacity(); i++) {
        auto& t = m_tet_connectivity[i];
        if (t.m_is_removed) continue;
        tets.emplace_back(tuple_from_tet(i));

        assert(tets.back().tid(*this) == i);
        assert(tets.back().is_valid(*this));
    }
    return tets;
}


std::vector<TetMesh::Tuple> TetMesh::get_vertices() const
{
    std::vector<TetMesh::Tuple> verts;
    for (auto i = 0; i < vert_capacity(); i++) {
        auto& vc = m_vertex_connectivity[i];
        if (vc.m_is_removed) continue;
        assert(!vc.m_conn_tets.empty());

        verts.emplace_back(tuple_from_vertex(i));

        assert(verts.back().vid(*this) == i);
        assert(verts.back().is_valid(*this));
    }
    return verts;
}

bool TetMesh::smooth_vertex(const Tuple& loc0)
{
    if (!smooth_before(loc0)) return false;
    start_protect_attributes();
    if (!smooth_after(loc0) || !invariants(get_one_ring_tets_for_vertex(loc0))) {
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();

    return true;
}


TetMesh::Tuple TetMesh::tuple_from_edge(size_t tid, int local_eid) const
{
    assert(tid < tet_capacity());
    assert(local_eid >= 0 && local_eid < m_local_edges.size());

    int vid = m_tet_connectivity[tid][m_local_edges[local_eid][0]];
    int fid = m_map_edge2face[local_eid];
    return Tuple(*this, vid, local_eid, fid, tid);
}

TetMesh::Tuple TetMesh::tuple_from_face(size_t tid, int local_fid) const
{
    assert(tid < tet_capacity());
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

std::tuple<TetMesh::Tuple, size_t> TetMesh::tuple_from_face(const std::array<size_t, 3>& vids) const
{
    size_t v0_id = vids[0];
    size_t v1_id = vids[1];
    size_t v2_id = vids[2];

    size_t global_tid = std::numeric_limits<size_t>::max();

    // find lowest common tet id
    {
        const auto& t0 = m_vertex_connectivity[v0_id].m_conn_tets;
        const auto& t1 = m_vertex_connectivity[v1_id].m_conn_tets;
        const auto& t2 = m_vertex_connectivity[v2_id].m_conn_tets;
        size_t i0 = 0;
        size_t i1 = 0;
        size_t i2 = 0;

        if (t0.empty() || t1.empty() || t2.empty()) {
            return {Tuple(), -1};
        }

        while (true) {
            if (t0[i0] < t1[i1] || t0[i0] < t2[i2]) {
                i0++;
                if (i0 == t0.size()) {
                    return {Tuple(), -1};
                }
            }
            if (t1[i1] < t2[i2] || t1[i1] < t0[i0]) {
                i1++;
                if (i1 == t1.size()) {
                    return {Tuple(), -1};
                }
            }
            if (t2[i2] < t0[i0] || t2[i2] < t1[i1]) {
                i2++;
                if (i2 == t2.size()) {
                    return {Tuple(), -1};
                }
            }
            if (t0[i0] == t1[i1] && t0[i0] == t2[i2]) {
                global_tid = t0[i0];
                break;
            }
        }
    }

    //// alternative implementation but slower
    //{
    //    // make v0 the one with the fewest incident tets
    //    if (m_vertex_connectivity[v1_id].m_conn_tets.size() <
    //        m_vertex_connectivity[v0_id].m_conn_tets.size()) {
    //        std::swap(v0_id, v1_id);
    //    }
    //    if (m_vertex_connectivity[v2_id].m_conn_tets.size() <
    //        m_vertex_connectivity[v0_id].m_conn_tets.size()) {
    //        std::swap(v0_id, v2_id);
    //    }
    //
    //    for (const size_t tid : m_vertex_connectivity[v0_id].m_conn_tets) {
    //        bool v1_found = false;
    //        bool v2_found = false;
    //        for (const size_t vid : m_tet_connectivity[tid].m_indices) {
    //            if (vid == v1_id) {
    //                v1_found = true;
    //                if (v2_found) {
    //                    break;
    //                }
    //            }
    //            if (vid == v2_id) {
    //                v2_found = true;
    //                if (v1_found) {
    //                    break;
    //                }
    //            }
    //        }
    //        if (v1_found && v2_found) {
    //            // global_tid = std::min(global_tid, tid);
    //            global_tid = tid;
    //            break;
    //        }
    //    }
    //
    //    if (global_tid == std::numeric_limits<size_t>::max()) {
    //        return {Tuple(), -1};
    //    }
    //}

    // tid
    Tuple face;
    face.m_global_tid = global_tid;
    // fid
    std::array<int, 3> f;
    // for (int j = 0; j < 3; j++) {
    //     f[j] = m_tet_connectivity[face.m_global_tid].find(vids[j]);
    // }
    {
        const auto& vs = m_tet_connectivity[face.m_global_tid];
        int k = 0;
        for (int j = 0; j < 4; ++j) {
            if (vs[j] == vids[0] || vs[j] == vids[1] || vs[j] == vids[2]) {
                f[k++] = j;
            }
        }
    }
    std::sort(f.begin(), f.end());
    face.m_local_fid =
        std::find(m_local_faces.begin(), m_local_faces.end(), f) - m_local_faces.begin();
    // eid
    face.m_local_eid = m_local_edges_in_a_face[face.m_local_fid][0];
    // vid
    face.m_global_vid = m_tet_connectivity[face.m_global_tid][m_local_edges[face.m_local_eid][0]];

    size_t global_fid = face.m_global_tid * 4 + face.m_local_fid;

    face.m_hash = m_tet_connectivity[face.m_global_tid].hash;

    assert(face.is_valid(*this));
    assert(face.fid(*this) == global_fid);

    return std::make_tuple(face, global_fid);

    // auto tmp = set_intersection(
    //     m_vertex_connectivity[vids[0]].m_conn_tets,
    //     m_vertex_connectivity[vids[1]].m_conn_tets);
    // auto n12_t_ids = set_intersection(tmp, m_vertex_connectivity[vids[2]].m_conn_tets);
    // if (n12_t_ids.size() == 0 || n12_t_ids.size() > 2) {
    //     return {Tuple(), -1};
    // }

    //// tid
    // Tuple face;
    // face.m_global_tid = n12_t_ids[0];
    // if (n12_t_ids.size() > 1 && n12_t_ids[1] < n12_t_ids[0]) face.m_global_tid = n12_t_ids[1];
    //// fid
    // std::array<int, 3> f;
    // for (int j = 0; j < 3; j++) {
    //     f[j] = m_tet_connectivity[face.m_global_tid].find(vids[j]);
    // }
    // std::sort(f.begin(), f.end());
    // face.m_local_fid =
    //     std::find(m_local_faces.begin(), m_local_faces.end(), f) - m_local_faces.begin();
    //// eid
    // face.m_local_eid = m_local_edges_in_a_face[face.m_local_fid][0];
    //// vid
    // face.m_global_vid =
    // m_tet_connectivity[face.m_global_tid][m_local_edges[face.m_local_eid][0]];

    // size_t global_fid = face.m_global_tid * 4 + face.m_local_fid;

    // face.m_hash = m_tet_connectivity[face.m_global_tid].hash;

    // assert(face.is_valid(*this));
    // assert(face.fid(*this) == global_fid);

    // return std::make_tuple(face, global_fid);
}

TetMesh::Tuple TetMesh::tuple_from_edge(const std::array<size_t, 2>& vids) const
{
    auto tets = set_intersection(
        m_vertex_connectivity[vids[0]].m_conn_tets,
        m_vertex_connectivity[vids[1]].m_conn_tets);
    if (tets.empty()) return Tuple();

    auto tid = tets.front();
    auto local_ind = m_tet_connectivity[tid].m_indices;

    for (auto local_eid = 0; local_eid < 6; local_eid++) {
        auto [l0, l1] = m_local_edges[local_eid];
        auto v0 = local_ind[l0], v1 = local_ind[l1];
        if (v0 != vids[0] && v1 != vids[0]) continue;
        if (v0 != vids[1] && v1 != vids[1]) continue;
        return tuple_from_edge(tid, local_eid);
    }
    return Tuple();
}


TetMesh::Tuple TetMesh::tuple_from_vertex(size_t vid) const
{
    assert(vid < vert_capacity());
    if (m_vertex_connectivity[vid].m_is_removed) return Tuple();

    int tid = m_vertex_connectivity[vid].m_conn_tets[0];
    int j = m_tet_connectivity[tid].find(vid);
    int eid = m_map_vertex2edge[j];
    int fid = m_map_edge2face[eid];

    return Tuple(*this, vid, eid, fid, tid);
}

TetMesh::Tuple TetMesh::tuple_from_tet(size_t tid) const
{
    assert(tid < tet_capacity());
    if (m_tet_connectivity[tid].m_is_removed) return Tuple();

    int vid = m_tet_connectivity[tid][0];
    int eid = m_map_vertex2edge[0];
    int fid = m_map_edge2face[eid];
    return Tuple(*this, vid, eid, fid, tid);
}

TetMesh::Tuple TetMesh::tuple_from_vids(size_t vid0, size_t vid1, size_t vid2, size_t vid3) const
{
    const auto& vf0 = m_vertex_connectivity[vid0];
    const auto& vf1 = m_vertex_connectivity[vid1];
    const auto& vf2 = m_vertex_connectivity[vid2];
    const auto& vf3 = m_vertex_connectivity[vid3];

    const std::vector<size_t> tets01 = set_intersection(vf0.m_conn_tets, vf1.m_conn_tets);
    const std::vector<size_t> tets012 = set_intersection(tets01, vf2.m_conn_tets);
    const std::vector<size_t> tets0123 = set_intersection(tets012, vf3.m_conn_tets);

    if (tets0123.size() != 1) {
        log_and_throw_error("Cannot find tet with vids ({},{},{},{})", vid0, vid1, vid2, vid3);
    }

    const size_t tid = tets0123[0];

    const auto& tc = m_tet_connectivity[tid].m_indices;
    size_t local_vid = -1;
    for (int i = 0; i < 4; ++i) {
        if (tc[i] == vid0) {
            local_vid = i;
            break;
        }
    }
    assert(local_vid != -1);

    const size_t eid = m_tet_connectivity[tid].find_local_edge(vid0, vid1);
    const size_t fid = m_tet_connectivity[tid].find_local_face(vid0, vid1, vid2);

    return Tuple(*this, vid0, eid, fid, tid);
}

simplex::Tet TetMesh::simplex_from_tet(const Tuple& t) const
{
    const auto v = oriented_tet_vids(t.tid(*this));
    const simplex::Tet tet(v[0], v[1], v[2], v[3]);
    return tet;
}


std::array<TetMesh::Tuple, 4> TetMesh::oriented_tet_vertices(const Tuple& t) const
{
    std::array<Tuple, 4> vs;
    for (int j = 0; j < 4; j++) {
        vs[j].m_global_vid = m_tet_connectivity[t.m_global_tid][j];
        vs[j].m_local_eid = m_map_vertex2edge[j];
        vs[j].m_local_fid = m_map_edge2face[vs[j].m_local_eid];
        vs[j].m_global_tid = t.m_global_tid;
    }
    return vs;
}

std::array<size_t, 4> TetMesh::oriented_tet_vids(const Tuple& t) const
{
    return oriented_tet_vids(t.m_global_tid);
}

std::array<size_t, 4> TetMesh::oriented_tet_vids(const size_t tid) const
{
    return m_tet_connectivity[tid].m_indices;
}

std::array<TetMesh::Tuple, 3> TetMesh::get_face_vertices(const Tuple& t) const
{
    std::array<Tuple, 3> vs;
    vs[0] = t;
    vs[1] = switch_vertex(t);
    vs[2] = switch_vertex(switch_edge(t));
    return vs;
}

std::array<TetMesh::Tuple, 6> TetMesh::tet_edges(const Tuple& t) const
{
    std::array<Tuple, 6> es;
    for (int j = 0; j < 6; j++) {
        es[j].m_local_eid = j;
        es[j].m_local_fid = m_map_edge2face[j];

        es[j].m_global_vid = m_tet_connectivity[t.m_global_tid][m_local_edges[j][0]];
        es[j].m_global_tid = t.m_global_tid;
    }
    return es;
}

std::vector<size_t> TetMesh::get_one_ring_tids_for_vertex(const Tuple& t) const
{
    return get_one_ring_tids_for_vertex(t.m_global_vid);
}

std::vector<size_t> TetMesh::get_one_ring_tids_for_vertex(const size_t vid) const
{
    return m_vertex_connectivity[vid].m_conn_tets;
}


std::vector<TetMesh::Tuple> TetMesh::get_one_ring_tets_for_vertex(const Tuple& t) const
{
    std::vector<Tuple> tets;
    auto& tids = m_vertex_connectivity[t.m_global_vid].m_conn_tets;
    tets.reserve(tids.size());
    for (auto t_id : tids) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}

std::vector<TetMesh::Tuple> TetMesh::get_one_ring_vertices_for_vertex(const Tuple& t) const
{
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

std::vector<size_t> TetMesh::get_one_ring_vids_for_vertex(size_t vid) const
{
    std::vector<size_t> v_ids;
    v_ids.reserve(m_vertex_connectivity[vid].m_conn_tets.size() * 4);
    for (int t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (int j = 0; j < 4; j++) {
            v_ids.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    vector_unique(v_ids);
    vector_erase(v_ids, vid);
    return v_ids;
}

std::vector<size_t> TetMesh::get_one_ring_vids_for_vertex_adj(size_t vid) const
{
    std::vector<size_t> v_ids;
    v_ids.reserve(m_vertex_connectivity[vid].m_conn_tets.size() * 4);
    for (int t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (int j = 0; j < 4; j++) {
            v_ids.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    vector_unique(v_ids);
    vector_erase(v_ids, vid);
    return v_ids;
}

std::vector<size_t> TetMesh::get_one_ring_vids_for_vertex(size_t vid, std::vector<size_t>& cache)
{
    cache.clear();
    for (int t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (int j = 0; j < 4; j++) {
            cache.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    return cache;
}

std::vector<size_t> TetMesh::get_one_ring_vids_for_vertex_adj(
    size_t vid,
    std::vector<size_t>& cache)
{
    cache.clear();
    for (int t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (int j = 0; j < 4; j++) {
            cache.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    return cache;
}

std::vector<TetMesh::Tuple> TetMesh::get_incident_tets_for_edge(const Tuple& t) const
{
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];
    return get_incident_tets_for_edge(v1_id, v2_id);
}

std::vector<TetMesh::Tuple> TetMesh::get_incident_tets_for_edge(
    const size_t vid0,
    const size_t vid1) const
{
    auto tids = get_incident_tids_for_edge(vid0, vid1);
    std::vector<Tuple> tets;
    for (int t_id : tids) {
        tets.push_back(tuple_from_tet(t_id));
    }
    return tets;
}

std::vector<size_t> TetMesh::get_incident_tids_for_edge(const Tuple& t) const
{
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];
    return get_incident_tids_for_edge(v1_id, v2_id);
}

std::vector<size_t> TetMesh::get_incident_tids_for_edge(const size_t vid0, const size_t vid1) const
{
    auto tids = set_intersection(
        m_vertex_connectivity[vid0].m_conn_tets,
        m_vertex_connectivity[vid1].m_conn_tets);
    return tids;
}

std::vector<TetMesh::Tuple> TetMesh::get_one_ring_tets_for_edge(const Tuple& t) const
{
    int v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    int v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];

    auto tids = m_vertex_connectivity[v1_id].m_conn_tets;
    tids.insert(
        tids.end(),
        m_vertex_connectivity[v2_id].m_conn_tets.begin(),
        m_vertex_connectivity[v2_id].m_conn_tets.end());
    vector_unique(tids);

    std::vector<Tuple> tets;
    for (int t_id : tids) {
        tets.emplace_back(tuple_from_tet(t_id));
    }
    return tets;
}


void TetMesh::consolidate_mesh()
{
    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(vert_capacity(), -1);
    for (auto i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(tet_capacity(), -1);
    for (auto i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }

    v_cnt = 0;
    for (auto i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            m_vertex_connectivity[v_cnt] = m_vertex_connectivity[i];
            if (p_vertex_attrs) {
                p_vertex_attrs->move(i, v_cnt);
            }
        }
        for (size_t& t_id : m_vertex_connectivity[v_cnt].m_conn_tets) t_id = map_t_ids[t_id];
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            m_tet_connectivity[t_cnt] = m_tet_connectivity[i];
            m_tet_connectivity[t_cnt].hash = 0;
            if (p_tet_attrs) {
                p_tet_attrs->move(i, t_cnt);
            }

            if (p_face_attrs) {
                for (auto j = 0; j < 4; j++) {
                    p_face_attrs->move(i * 4 + j, t_cnt * 4 + j);
                }
            }
            if (p_edge_attrs) {
                for (auto j = 0; j < 6; j++) {
                    p_edge_attrs->move(i * 6 + j, t_cnt * 6 + j);
                }
            }
        }
        for (size_t& v_id : m_tet_connectivity[t_cnt].m_indices) v_id = map_v_ids[v_id];
        t_cnt++;
    }

    current_vert_size = v_cnt;
    current_tet_size = t_cnt;

    m_vertex_connectivity.resize(v_cnt);
    m_tet_connectivity.resize(t_cnt);

    if (p_vertex_attrs) {
        p_vertex_attrs->resize(v_cnt);
    }
    if (p_edge_attrs) {
        p_edge_attrs->resize(6 * t_cnt);
    }
    if (p_face_attrs) {
        p_face_attrs->resize(4 * t_cnt);
    }
    if (p_tet_attrs) {
        p_tet_attrs->resize(t_cnt);
    }

    assert(check_mesh_connectivity_validity());
}

std::vector<std::array<size_t, 3>> TetMesh::vertex_adjacent_boundary_faces(const Tuple& tup) const
{
    auto v = tup.vid(*this);
    auto result_faces = std::set<std::array<size_t, 3>>();
    for (const size_t t : m_vertex_connectivity[v].m_conn_tets) {
        const auto& tet = m_tet_connectivity[t];
        for (auto j = 0; j < 4; j++) {
            if (tet[m_map_vertex2oppo_face[j]] == v)
                continue; // only consider those not connecting to it.
            const auto& f = m_local_faces[j];
            std::array<size_t, 3> face{{tet[f[0]], tet[f[1]], tet[f[2]]}};
            std::sort(face.begin(), face.end());
            auto it = result_faces.find(face);
            if (it == result_faces.end()) { // delete those appearing twice.
                result_faces.insert(face);
            } else {
                result_faces.erase(it);
            }
        }
    }
    return std::vector<std::array<size_t, 3>>(result_faces.begin(), result_faces.end());
}

// concurrent

int TetMesh::release_vertex_mutex_in_stack()
{
    int num_released = 0;
    auto& stack = mutex_release_stack.local();
    for (int i = stack.size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(stack[i]);
        num_released++;
    }
    stack.clear();
    return num_released;
}

bool TetMesh::try_set_vertex_mutex_two_ring(const Tuple& v, int threadid)
{
    auto& stack = mutex_release_stack.local();
    for (auto v_one_ring : get_one_ring_vertices_for_vertex(v)) {
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            stack.push_back(v_one_ring.vid(*this));
            for (auto v_two_ring : get_one_ring_vertices_for_vertex(v_one_ring)) {
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    stack.push_back(v_two_ring.vid(*this));
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }
    }
    return true;
}

bool TetMesh::try_set_vertex_mutex_two_ring_vid(const Tuple& v, int threadid)
{
    auto& cache = get_one_ring_cache.local();
    auto& stack = mutex_release_stack.local();
    for (auto v_one_ring : get_one_ring_vids_for_vertex(v.vid(*this), cache)) {
        if (m_vertex_mutex[v_one_ring].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            {
                stack.push_back(v_one_ring);
            }
            for (auto v_two_ring : get_one_ring_vids_for_vertex(v_one_ring, cache)) {
                if (m_vertex_mutex[v_two_ring].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    stack.push_back(v_two_ring);
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }
    }
    return true;
}

bool TetMesh::try_set_vertex_mutex_two_ring_vid(size_t v, int threadid)
{
    auto& cache = get_one_ring_cache.local();
    auto& stack = mutex_release_stack.local();
    for (auto v_one_ring : get_one_ring_vids_for_vertex(v, cache)) {
        if (m_vertex_mutex[v_one_ring].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            stack.push_back(v_one_ring);
            for (auto v_two_ring : get_one_ring_vids_for_vertex(v_one_ring, cache)) {
                if (m_vertex_mutex[v_two_ring].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    stack.push_back(v_two_ring);
                } else {
                    return false;
                }
            }
        } else {
            return false;
        }
    }
    return true;
}


bool TetMesh::try_set_edge_mutex_two_ring(const Tuple& e, int threadid)
{
    const Tuple& v1 = e;
    auto& stack = mutex_release_stack.local();

    stack.reserve(128);

    // try v1
    auto acquire_lock = [&]() {
        if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
            if (try_set_vertex_mutex(v1, threadid)) {
                stack.push_back(v1.vid(*this));
            } else {
                return false;
            }
        }
        if (!v1.is_valid(*this)) {
            return false;
        }

        // try v2
        Tuple v2 = switch_vertex(v1);
        if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
            if (try_set_vertex_mutex(v2, threadid)) {
                stack.push_back(v2.vid(*this));
            } else {
                return false;
            }
        }
        if (!v2.is_valid(*this)) {
            return false;
        }

        // try v1 two ring
        return (
            try_set_vertex_mutex_two_ring_vid(v1, threadid) &&
            try_set_vertex_mutex_two_ring_vid(v2, threadid));
    };

    if (!acquire_lock()) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool TetMesh::try_set_face_mutex_two_ring(const Tuple& f, int threadid)
{
    Tuple v1 = f;
    bool release_flag = false;
    auto& stack = mutex_release_stack.local();


    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            stack.push_back(v1.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    Tuple v2 = switch_vertex(f);
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            stack.push_back(v2.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3
    Tuple v3 = switch_edge(v2).switch_vertex(*this);
    if (m_vertex_mutex[v3.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            stack.push_back(v3.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v3.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v3, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool TetMesh::try_set_face_mutex_two_ring(
    const Tuple& v1,
    const Tuple& v2,
    const Tuple& v3,
    int threadid)
{
    bool release_flag = false;
    auto& stack = mutex_release_stack.local();

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            stack.push_back(v1.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v1.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2
    // if (!vector_contains(stack, v2.vid(*this))) {
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            stack.push_back(v2.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v2.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3
    if (m_vertex_mutex[v3.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v3, threadid)) {
            stack.push_back(v3.vid(*this));
        } else {
            release_flag = true;
        }
    }
    if (!v3.is_valid(*this)) {
        release_flag = true;
    }
    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v3 two ring
    release_flag = !try_set_vertex_mutex_two_ring_vid(v3, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }
    return true;
}

bool TetMesh::try_set_face_mutex_two_ring(size_t v1, size_t v2, size_t v3, int threadid)
{
    bool release_flag = false;
    auto& stack = mutex_release_stack.local();

    auto try_all = [&]() {
        for (auto vv : {v1, v2, v3}) {
            if (m_vertex_mutex[vv].get_owner() != threadid) {
                if (try_set_vertex_mutex(vv, threadid)) {
                    stack.push_back(vv);
                } else {
                    return false;
                }
            }
        }

        for (auto vv : {v1, v2, v3}) {
            if (try_set_vertex_mutex_two_ring_vid(vv, threadid) == false) {
                return false;
            };
        }
        return true;
    };


    if (try_all() == false) {
        release_vertex_mutex_in_stack();
        return false;
    }

    return true;
}

bool TetMesh::try_set_vertex_mutex_one_ring(const Tuple& v, int threadid)
{
    auto& stack = mutex_release_stack.local();
    auto& cache = get_one_ring_cache.local();
    if (m_vertex_mutex[v.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v, threadid)) {
            stack.push_back(v.vid(*this));
            for (auto v_one_ring : get_one_ring_vids_for_vertex(v.vid(*this), cache)) {
                if (m_vertex_mutex[v_one_ring].get_owner() != threadid) {
                    if (try_set_vertex_mutex(v_one_ring, threadid)) {
                        stack.push_back(v_one_ring);
                    } else {
                        release_vertex_mutex_in_stack();
                        return false;
                    }
                }
            }
        } else {
            release_vertex_mutex_in_stack();
            return false;
        }
    }
    return true;
}

void TetMesh::for_each_edge(const std::function<void(const TetMesh::Tuple&)>& func)
{
    if (NUM_THREADS == 0) {
        for (int i = 0; i < tet_capacity(); i++) {
            if (!tuple_from_tet(i).is_valid(*this)) continue;
            for (int j = 0; j < 6; j++) {
                auto tup = tuple_from_edge(i, j);
                if (tup.eid(*this) == 6 * i + j) {
                    func(tup);
                }
            }
        }
    } else {
        tbb::task_arena arena(NUM_THREADS);
        arena.execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<int>(0, tet_capacity()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        if (!tuple_from_tet(i).is_valid(*this)) continue;
                        for (int j = 0; j < 6; j++) {
                            auto tup = tuple_from_edge(i, j);
                            if (tup.eid(*this) == 6 * i + j) {
                                func(tup);
                            }
                        }
                    }
                });
        });
    }
}


void TetMesh::for_each_tetra(const std::function<void(const TetMesh::Tuple&)>& func)
{
    if (NUM_THREADS == 0) {
        std::cout << "in serial for each tet" << std::endl;
        for (int i = 0; i < tet_capacity(); i++) {
            auto tup = tuple_from_tet(i);
            if (!tup.is_valid(*this)) continue;
            func(tup);
        }
    } else {
        std::cout << "in parallel for each tet" << std::endl;

        tbb::task_arena arena(NUM_THREADS);
        arena.execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<int>(0, tet_capacity()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        auto tup = tuple_from_tet(i);
                        if (!tup.is_valid(*this)) continue;
                        func(tup);
                    }
                });
        });
    }
}


void TetMesh::for_each_vertex(const std::function<void(const TetMesh::Tuple&)>& func)
{
    if (NUM_THREADS == 0) {
        std::cout << "in serial for each vertex" << std::endl;
        for (int i = 0; i < vert_capacity(); i++) {
            auto tup = tuple_from_vertex(i);
            if (!tup.is_valid(*this)) continue;
            func(tup);
        }
    } else {
        std::cout << "in parallel for each vertex" << std::endl;
        tbb::task_arena arena(NUM_THREADS);
        arena.execute([&] {
            tbb::parallel_for(
                tbb::blocked_range<int>(0, vert_capacity()),
                [&](tbb::blocked_range<int> r) {
                    for (int i = r.begin(); i < r.end(); i++) {
                        auto tup = tuple_from_vertex(i);
                        if (!tup.is_valid(*this)) continue;
                        func(tup);
                    }
                });
        });
    }
}

} // namespace wmtk