#include "TetMesh.h"

#include <wmtk/AttributeCollection.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/TupleUtils.hpp>

namespace wmtk {

// Atomically reserve `n` contiguous fresh tet slots from the preallocated storage.
// Returns the first index of the block, or -1 if the reservation would exceed the
// preallocated capacity -- in that case the caller must abort the operation. On the
// (rare) failure the counter is left advanced; those unusable slots are reclaimed at
// the next consolidate_mesh(). This is the only place the tet counter grows during
// operations, and it never resizes the storage (no more thread-safe growth).
long TetMesh::request_tet_slots(size_t n)
{
    if (n == 0) return (long)current_tet_size.load();
    const long cap = (long)m_tet_connectivity.size();
    // CAS loop: only advance the counter when there is room, so a failed request
    // never leaks slots (which would push current_tet_size past the storage size
    // and make later iteration go out of bounds).
    long first = current_tet_size.load(std::memory_order_relaxed);
    do {
        if (first + (long)n > cap) return -1;
    } while (!current_tet_size.compare_exchange_weak(
        first,
        first + (long)n,
        std::memory_order_acq_rel,
        std::memory_order_relaxed));
    // Reset the handed-out slots to a clean state. The old tbb::collector
    // shrank on consolidate and regrew fresh slots, so allocations were always
    // clean; the preallocated std::vector keeps stale data in the spare region, so
    // we must clear it here (matches old behaviour: default tet, hash = -1).
    for (long i = first; i < first + (long)n; ++i) {
        m_tet_connectivity[i] = TetrahedronConnectivity{};
        m_tet_connectivity[i].hash = -1;
    }
    return first;
}

long TetMesh::request_vert_slots(size_t n)
{
    if (n == 0) return (long)current_vert_size.load();
    const long cap = (long)m_vertex_connectivity.size();
    long first = current_vert_size.load(std::memory_order_relaxed);
    do {
        if (first + (long)n > cap) return -1;
    } while (!current_vert_size.compare_exchange_weak(
        first,
        first + (long)n,
        std::memory_order_acq_rel,
        std::memory_order_relaxed));
    // Reset the handed-out vertex slots to a clean state (see request_tet_slots).
    for (long i = first; i < first + (long)n; ++i) {
        m_vertex_connectivity[i] = VertexConnectivity{};
    }
    return first;
}

int TetMesh::get_next_empty_slot_t()
{
    return (int)request_tet_slots(1);
}

int TetMesh::get_next_empty_slot_v()
{
    return (int)request_vert_slots(1);
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
    // Preallocate spare capacity so operations can grab fresh slots without
    // resizing the storage. Only [0, live) is filled; the rest is spare capacity
    // that operations consume (and fail past). See reserved_capacity().
    const size_t vcap = reserved_capacity(n_vertices);
    const size_t tcap = reserved_capacity(tets.size());
    m_vertex_connectivity.resize(vcap);
    m_tet_connectivity.resize(tcap);
    current_vert_size = (long)n_vertices;
    current_tet_size = (long)tets.size();
    for (int i = 0; i < tets.size(); i++) {
        m_tet_connectivity[i].m_indices = tets[i];
        for (int j = 0; j < 4; j++) {
            assert(tets[i][j] < vert_capacity());
            m_vertex_connectivity[tets[i][j]].m_conn_tets.push_back(i);
        }
    }

    // concurrent
    resize_vertex_mutex(vcap);

    // resize attributes to the preallocated capacity
    if (p_vertex_attrs != nullptr) {
        p_vertex_attrs->resize(vcap);
    }
    if (p_tet_attrs != nullptr) {
        p_tet_attrs->resize(tcap);
    }
    if (p_face_attrs != nullptr) {
        p_face_attrs->resize(4 * tcap);
    }
    if (p_edge_attrs != nullptr) {
        p_edge_attrs->resize(6 * tcap);
    }
}

void TetMesh::init_with_isolated_vertices(
    size_t n_vertices,
    const std::vector<std::array<size_t, 4>>& tets)
{
    const size_t vcap = reserved_capacity(n_vertices);
    const size_t tcap = reserved_capacity(tets.size());
    m_vertex_connectivity.clear();
    m_vertex_connectivity.resize(vcap);
    m_tet_connectivity.clear();
    m_tet_connectivity.resize(tcap);
    current_vert_size = (long)n_vertices;
    current_tet_size = (long)tets.size();
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
    resize_vertex_mutex(vcap);

    // resize attributes to the preallocated capacity
    if (p_vertex_attrs) {
        p_vertex_attrs->clear();
        p_vertex_attrs->resize(vcap);
    }
    if (p_tet_attrs) {
        p_tet_attrs->clear();
        p_tet_attrs->resize(tcap);
    }
    if (p_face_attrs) {
        p_face_attrs->clear();
        p_face_attrs->resize(4 * tcap);
    }
    if (p_edge_attrs) {
        p_edge_attrs->clear();
        p_edge_attrs->resize(6 * tcap);
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
    // One entry per edge, not six. get_faces() right below already works this way: an
    // edge's canonical representative is the one whose eid() -- the lowest tet id
    // carrying it, times six, plus the local edge -- points back at this (tid, j). So the
    // duplicates can be rejected as they are generated instead of being collected,
    // sorted and uniqued afterwards.
    //
    // The old form built 6 * tet_capacity() entries of (v0, v1, Tuple) at 56 bytes each,
    // sorted all of them, and copied the survivors into a second vector. On a mesh with
    // 18M tets that is roughly 6 GB of staging plus a 108M-element sort, to produce a
    // result 6x smaller.
    //
    // The output is still ordered by (min vid, max vid), and the representative is now
    // well defined rather than whatever an unstable std::sort happened to leave first
    // among the six copies of each edge.
    std::vector<std::pair<std::pair<size_t, size_t>, TetMesh::Tuple>> edges;
    for (int i = 0; i < tet_capacity(); i++) {
        if (m_tet_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 6; j++) {
            const auto tup = tuple_from_edge(i, j);
            if (tup.eid(*this) != size_t(6 * i + j)) {
                continue; // some lower-numbered tet owns this edge
            }
            size_t v0 = tup.vid(*this);
            size_t v1 = tup.switch_vertex(*this).vid(*this);
            if (v0 > v1) std::swap(v0, v1);
            edges.emplace_back(std::make_pair(v0, v1), tup);
        }
    }
    std::sort(edges.begin(), edges.end(), [](const auto& a, const auto& b) {
        return a.first < b.first;
    });
    std::vector<TetMesh::Tuple> uniq_edges;
    uniq_edges.reserve(edges.size());
    for (auto& [key, e] : edges) uniq_edges.push_back(e);
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
    // Each tet contributes at most 4 faces and exactly one is the canonical
    // representative, so 2 * tet_capacity() is a close upper bound on a closed mesh and
    // an over-estimate on an open one. Without this the vector doubles its way up and
    // holds both buffers at the last reallocation -- 187M faces x 40 bytes is 7.5 GB of
    // payload on Thingi10K 338910, and the transient peak is far worse.
    faces.reserve(2 * tet_capacity());
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
        for (size_t tid : m_vertex_connectivity[i].m_conn_tets)
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
    tets.reserve(tet_capacity());
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
    verts.reserve(vert_capacity());
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

    size_t vid = m_tet_connectivity[tid][m_local_edges[local_eid][0]];
    size_t fid = m_map_edge2face[local_eid];
    return Tuple(*this, vid, local_eid, fid, tid);
}

TetMesh::Tuple TetMesh::tuple_from_face(size_t tid, int local_fid) const
{
    assert(tid < tet_capacity());
    assert(local_fid >= 0 && local_fid < m_local_faces.size());

    size_t vid = m_tet_connectivity[tid][m_local_faces[local_fid][0]];
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

size_t TetMesh::lowest_common_tet(const size_t v0_id, const size_t v1_id, const size_t v2_id) const
{
    const auto& t0 = m_vertex_connectivity[v0_id].m_conn_tets;
    const auto& t1 = m_vertex_connectivity[v1_id].m_conn_tets;
    const auto& t2 = m_vertex_connectivity[v2_id].m_conn_tets;

    // Scan the SMALLEST of the three fans and test the other two vertices against each
    // candidate tet, rather than merging all three in lockstep.
    //
    // The lockstep merge advances every pointer, so it costs O(|t0| + |t1| + |t2|) --
    // set by the LARGEST fan. That is fine when the fans are all ~20-30, which is the
    // valence of a well-shaped tet mesh, and it is why the merge was written that way.
    // It collapses when the fans are skewed. On Thingi10K model 71263 a split cascade
    // drove two vertices to 5494 and 10896 incident tets while the third had 10: the
    // merge walked ~16k entries to find a tet that 10 candidates would have located.
    // split_edge_before calls this four times per incident tet, so the pass stopped
    // making visible progress -- 98.5% of samples sat in this function.
    //
    // Scanning the smallest fan costs O(min fan) with a four-element check per candidate,
    // so it is comparable at normal valence and bounded by the *best* vertex rather than
    // the worst when the mesh degenerates.
    //
    // m_conn_tets is sorted, so scanning upward returns the lowest common tet id -- the
    // canonicalisation the global face id depends on, since a face shared by two tets
    // must get the same id from either side.
    const std::vector<size_t>* fan = &t0;
    size_t other_a = v1_id;
    size_t other_b = v2_id;
    if (t1.size() < fan->size()) {
        fan = &t1;
        other_a = v0_id;
        other_b = v2_id;
    }
    if (t2.size() < fan->size()) {
        fan = &t2;
        other_a = v0_id;
        other_b = v1_id;
    }

    for (const size_t tid : *fan) {
        const auto& conn = m_tet_connectivity[tid];
        if (conn.find(other_a) >= 0 && conn.find(other_b) >= 0) {
            return tid;
        }
    }
    return std::numeric_limits<size_t>::max();
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

        if (t0.empty() || t1.empty() || t2.empty()) {
            assert(false && "tuple_from_face: one of the vertices has no incident tets");
            return {Tuple(), -1};
        }

        global_tid = lowest_common_tet(v0_id, v1_id, v2_id);
        if (global_tid == std::numeric_limits<size_t>::max()) {
            assert(false && "tuple_from_face: no common tet found for the three vertices");
            return {Tuple(), -1};
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

std::tuple<TetMesh::Tuple, size_t> TetMesh::tuple_from_face(const simplex::Face& f) const
{
    return tuple_from_face(f.vertices());
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

    size_t tid = m_vertex_connectivity[vid].m_conn_tets[0];
    int j = m_tet_connectivity[tid].find(vid);
    int eid = m_map_vertex2edge[j];
    int fid = m_map_edge2face[eid];

    return Tuple(*this, vid, eid, fid, tid);
}

TetMesh::Tuple TetMesh::tuple_from_tet(size_t tid) const
{
    assert(tid < tet_capacity());
    if (m_tet_connectivity[tid].m_is_removed) return Tuple();

    size_t vid = m_tet_connectivity[tid][0];
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
    return simplex_from_tet(t.tid(*this));
}

simplex::Tet TetMesh::simplex_from_tet(const size_t tid) const
{
    const auto v = oriented_tet_vids(tid);
    const simplex::Tet tet(v[0], v[1], v[2], v[3]);
    return tet;
}

simplex::Face TetMesh::simplex_from_face(const Tuple& t) const
{
    const auto vs = get_face_vids(t);
    return simplex::Face(vs[0], vs[1], vs[2]);
}

simplex::Edge TetMesh::simplex_from_edge(const Tuple& t) const
{
    size_t v0 = t.vid(*this);
    size_t v1 = t.switch_vertex(*this).vid(*this);
    return simplex::Edge(v0, v1);
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

std::array<size_t, 3> TetMesh::get_face_vids(const Tuple& t) const
{
    std::array<size_t, 3> vs;
    vs[0] = t.vid(*this);
    vs[1] = t.switch_vertex(*this).vid(*this);
    vs[2] = t.switch_edge(*this).switch_vertex(*this).vid(*this);
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

const std::vector<size_t>& TetMesh::get_one_ring_tids_for_vertex(const Tuple& t) const
{
    return get_one_ring_tids_for_vertex(t.m_global_vid);
}

const std::vector<size_t>& TetMesh::get_one_ring_tids_for_vertex(const size_t vid) const
{
    // The fan IS the stored connectivity, so hand it back rather than copying it. Matches
    // TriMesh::get_one_ring_fids_for_vertex, which has always returned a reference.
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
    const auto& tids = m_vertex_connectivity[t.m_global_vid].m_conn_tets;
    std::vector<size_t> v_ids;
    v_ids.reserve(tids.size() * 4);
    for (size_t t_id : tids) {
        for (size_t j = 0; j < 4; j++) {
            v_ids.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    vector_unique(v_ids);
    vector_erase(v_ids, t.m_global_vid);
    std::vector<Tuple> vertices;
    vertices.reserve(v_ids.size());
    for (auto v_id : v_ids) {
        vertices.push_back(tuple_from_vertex(v_id));
    }
    return vertices;
}

std::vector<size_t> TetMesh::get_one_ring_vids_for_vertex(size_t vid) const
{
    std::vector<size_t> v_ids;
    v_ids.reserve(m_vertex_connectivity[vid].m_conn_tets.size() * 4);
    for (size_t t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (size_t j = 0; j < 4; j++) {
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
    for (size_t t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (size_t j = 0; j < 4; j++) {
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
    for (size_t t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (size_t j = 0; j < 4; j++) {
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
    for (size_t t_id : m_vertex_connectivity[vid].m_conn_tets) {
        for (size_t j = 0; j < 4; j++) {
            cache.push_back(m_tet_connectivity[t_id][j]);
        }
    }
    return cache;
}

std::vector<TetMesh::Tuple> TetMesh::get_incident_tets_for_edge(const Tuple& t) const
{
    size_t v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    size_t v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];
    return get_incident_tets_for_edge(v1_id, v2_id);
}

std::vector<TetMesh::Tuple> TetMesh::get_incident_tets_for_edge(
    const size_t vid0,
    const size_t vid1) const
{
    const auto tids = get_incident_tids_for_edge(vid0, vid1);
    std::vector<Tuple> tets;
    tets.reserve(tids.size());
    for (size_t t_id : tids) {
        tets.push_back(tuple_from_tet(t_id));
        assert(tets.back().is_valid(*this));
    }
    return tets;
}

std::vector<size_t> TetMesh::get_incident_tids_for_edge(const Tuple& t) const
{
    size_t v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    size_t v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];
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
    size_t v1_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][0]];
    size_t v2_id = m_tet_connectivity[t.m_global_tid][m_local_edges[t.m_local_eid][1]];

    auto tids = m_vertex_connectivity[v1_id].m_conn_tets;
    tids.insert(
        tids.end(),
        m_vertex_connectivity[v2_id].m_conn_tets.begin(),
        m_vertex_connectivity[v2_id].m_conn_tets.end());
    vector_unique(tids);

    std::vector<Tuple> tets;
    for (size_t t_id : tids) {
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

    // Re-establish spare capacity for the next round of operations (only [0,live)
    // is live; the rest is preallocated headroom that operations consume).
    const size_t vcap = reserved_capacity(v_cnt);
    const size_t tcap = reserved_capacity(t_cnt);

    m_vertex_connectivity.resize(vcap);
    m_tet_connectivity.resize(tcap);
    resize_vertex_mutex(vcap);

    if (p_vertex_attrs) {
        p_vertex_attrs->resize(vcap);
    }
    if (p_edge_attrs) {
        p_edge_attrs->resize(6 * tcap);
    }
    if (p_face_attrs) {
        p_face_attrs->resize(4 * tcap);
    }
    if (p_tet_attrs) {
        p_tet_attrs->resize(tcap);
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

int TetMesh::release_vertex_mutex_to(size_t mark)
{
    auto& stack = mutex_release_stack.local();
    int num_released = 0;
    while (stack.size() > mark) {
        unlock_vertex_mutex(stack.back());
        stack.pop_back();
        num_released++;
    }
    return num_released;
}

int TetMesh::release_vertex_mutex_in_stack()
{
    return release_vertex_mutex_to(0);
}

bool TetMesh::lock_vertex_ball(
    const size_t* seeds,
    size_t n_seeds,
    int threadid,
    int n,
    size_t mark)
{
    auto& stack = mutex_release_stack.local();
    auto& scr = m_ring_lock_scratch.local();

    stack.reserve(128);

    const size_t cap = m_vertex_connectivity.size();
    if (scr.stamp.size() < cap) {
        scr.stamp.resize(cap, 0);
    }
    if (++scr.epoch == 0) { // wrapped: every stale stamp would read as current
        std::fill(scr.stamp.begin(), scr.stamp.end(), 0);
        scr.epoch = 1;
    }
    const uint32_t epoch = scr.epoch;

    // Take `vid` into the ball. Already-owned vertices are marked but not re-locked, which is
    // what makes the BFS expand through them -- the flaw in the hand-written two-ring lockers
    // this replaces was to `continue` past them and never look at their neighbours.
    const auto claim = [&](size_t vid) {
        if (scr.stamp[vid] == epoch) {
            return true;
        }
        scr.stamp[vid] = epoch;
        if (m_vertex_mutex[vid].get_owner() == threadid) {
            return true;
        }
        if (!try_set_vertex_mutex(vid, threadid)) {
            return false;
        }
        stack.push_back(vid);
        return true;
    };

    scr.frontier.clear();
    for (size_t i = 0; i < n_seeds; ++i) {
        if (!claim(seeds[i]) || m_vertex_connectivity[seeds[i]].m_is_removed) {
            release_vertex_mutex_to(mark);
            return false;
        }
        scr.frontier.push_back(seeds[i]);
    }

    for (int depth = 0; depth < n; ++depth) {
        scr.next.clear();
        for (const size_t v : scr.frontier) {
            // Safe to read: `v` is held by this thread. Spelled out rather than calling
            // get_one_ring_vids_for_vertex(v, cache), which returns a copy of the cache and
            // would put a heap allocation on the per-vertex path of every lock acquisition.
            scr.one_ring.clear();
            for (const size_t t_id : m_vertex_connectivity[v].m_conn_tets) {
                for (size_t j = 0; j < 4; j++) {
                    scr.one_ring.push_back(m_tet_connectivity[t_id][j]);
                }
            }
            for (const size_t w : scr.one_ring) {
                if (scr.stamp[w] == epoch) {
                    continue;
                }
                if (!claim(w)) {
                    release_vertex_mutex_to(mark);
                    return false;
                }
                scr.next.push_back(w);
            }
        }
        if (scr.next.empty()) {
            break;
        }
        scr.frontier.swap(scr.next);
    }
    return true;
}

bool TetMesh::try_set_vertex_mutex_n_ring(size_t vid, int threadid, int n)
{
    return lock_vertex_ball(&vid, 1, threadid, n, mutex_release_stack.local().size());
}

bool TetMesh::try_set_vertex_mutex_n_ring(const Tuple& v, int threadid, int n)
{
    return try_set_vertex_mutex_n_ring(v.vid(*this), threadid, n);
}

bool TetMesh::try_set_edge_mutex_n_ring(const Tuple& e, int threadid, int n)
{
    const size_t mark = mutex_release_stack.local().size();

    // The second endpoint is read off the incident tet, so claim the first one and re-check
    // the edge before trusting that read.
    const size_t v1 = e.vid(*this);
    if (!try_set_vertex_mutex_n_ring(v1, threadid, 0) || !e.is_valid(*this)) {
        release_vertex_mutex_to(mark);
        return false;
    }

    const size_t seeds[2] = {v1, switch_vertex(e).vid(*this)};
    return lock_vertex_ball(seeds, 2, threadid, n, mark);
}

bool TetMesh::try_set_face_mutex_n_ring(size_t v1, size_t v2, size_t v3, int threadid, int n)
{
    const size_t seeds[3] = {v1, v2, v3};
    return lock_vertex_ball(seeds, 3, threadid, n, mutex_release_stack.local().size());
}

bool TetMesh::try_set_vertex_mutex_two_ring(const Tuple& v, int threadid)
{
    return try_set_vertex_mutex_n_ring(v, threadid, 2);
}

bool TetMesh::try_set_vertex_mutex_two_ring_vid(const Tuple& v, int threadid)
{
    return try_set_vertex_mutex_n_ring(v.vid(*this), threadid, 2);
}

bool TetMesh::try_set_vertex_mutex_two_ring_vid(size_t v, int threadid)
{
    return try_set_vertex_mutex_n_ring(v, threadid, 2);
}

bool TetMesh::try_set_edge_mutex_two_ring(const Tuple& e, int threadid)
{
    return try_set_edge_mutex_n_ring(e, threadid, 2);
}

bool TetMesh::try_set_face_mutex_two_ring(const Tuple& f, int threadid)
{
    const size_t mark = mutex_release_stack.local().size();

    // v2 and v3 are read off the incident tet, so claim v1 and re-check the face first.
    const size_t v1 = f.vid(*this);
    if (!try_set_vertex_mutex_n_ring(v1, threadid, 0) || !f.is_valid(*this)) {
        release_vertex_mutex_to(mark);
        return false;
    }

    const Tuple t2 = switch_vertex(f);
    const size_t seeds[3] = {
        v1,
        t2.vid(*this),
        t2.switch_edge(*this).switch_vertex(*this).vid(*this)};
    return lock_vertex_ball(seeds, 3, threadid, 2, mark);
}

bool TetMesh::try_set_face_mutex_two_ring(
    const Tuple& v1,
    const Tuple& v2,
    const Tuple& v3,
    int threadid)
{
    return try_set_face_mutex_n_ring(v1.vid(*this), v2.vid(*this), v3.vid(*this), threadid, 2);
}

bool TetMesh::try_set_face_mutex_two_ring(size_t v1, size_t v2, size_t v3, int threadid)
{
    return try_set_face_mutex_n_ring(v1, v2, v3, threadid, 2);
}

bool TetMesh::try_set_vertex_mutex_one_ring(const Tuple& v, int threadid)
{
    return try_set_vertex_mutex_n_ring(v, threadid, 1);
}

void TetMesh::for_each_edge(const std::function<void(const TetMesh::Tuple&)>& func)
{
    if (NUM_THREADS == 0) {
        // TODO: Try get_edges() here and see if it performs better.
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
        // TODO: This can probably be optimized by avoiding computing eid that often.
        threading::parallel_for(
            threading::range(0, tet_capacity()),
            [&](const threading::range& r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    if (!tuple_from_tet(i).is_valid(*this)) continue;
                    for (int j = 0; j < 6; j++) {
                        auto tup = tuple_from_edge(i, j);
                        if (tup.eid(*this) == 6 * i + j) {
                            func(tup);
                        }
                    }
                }
            },
            NUM_THREADS);
    }
}


void TetMesh::for_each_tetra(const std::function<void(const TetMesh::Tuple&)>& func)
{
    if (NUM_THREADS == 0) {
        // std::cout << "in serial for each tet" << std::endl;
        for (int i = 0; i < tet_capacity(); i++) {
            auto tup = tuple_from_tet(i);
            if (!tup.is_valid(*this)) continue;
            func(tup);
        }
    } else {
        // std::cout << "in parallel for each tet" << std::endl;

        threading::parallel_for(
            threading::range(0, tet_capacity()),
            [&](const threading::range& r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_tet(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            },
            NUM_THREADS);
    }
}


void TetMesh::for_each_vertex(const std::function<void(const TetMesh::Tuple&)>& func)
{
    if (NUM_THREADS == 0) {
        // std::cout << "in serial for each vertex" << std::endl;
        for (int i = 0; i < vert_capacity(); i++) {
            auto tup = tuple_from_vertex(i);
            if (!tup.is_valid(*this)) continue;
            func(tup);
        }
    } else {
        // std::cout << "in parallel for each vertex" << std::endl;
        threading::parallel_for(
            threading::range(0, vert_capacity()),
            [&](const threading::range& r) {
                for (size_t i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_vertex(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            },
            NUM_THREADS);
    }
}

} // namespace wmtk