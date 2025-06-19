#include <igl/is_edge_manifold.h>
#include <igl/writeDMAT.h>
#include <wmtk/TriMesh.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include "wmtk/utils/VectorUtils.h"

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/parallel_for.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

using namespace wmtk;

void TriMesh::Tuple::update_hash(const TriMesh& m)
{
    m_hash = m.m_tri_connectivity[m_fid].hash;
}

void TriMesh::Tuple::print_info()
{
    logger().trace("tuple: {} {} {}", m_vid, m_eid, m_fid);
}

size_t TriMesh::Tuple::eid(const TriMesh& m) const
{
    if (switch_face(m).has_value()) {
        size_t fid2 = switch_face(m)->fid(m);
        size_t min_fid = std::min(m_fid, fid2);
        if (min_fid == fid2) {
            int i = m.m_tri_connectivity[fid2].find(m_vid);
            int j = m.m_tri_connectivity[fid2].find(switch_vertex(m).vid(m));
            return min_fid * 3 + 3 - i - j;
        }
    }
    return m_fid * 3 + m_eid;
}


TriMesh::Tuple TriMesh::Tuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));

    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];

    Tuple loc = *this;
    switch (m_eid) {
    case 0:
        assert(m_vid == v1 || m_vid == v2);
        loc.m_vid = m_vid == v1 ? v2 : v1;
        break;
    case 1:
        assert(m_vid == v0 || m_vid == v2);
        loc.m_vid = m_vid == v0 ? v2 : v0;
        break;
    case 2:
        assert(m_vid == v0 || m_vid == v1);
        loc.m_vid = m_vid == v0 ? v1 : v0;
        break;
    default:;
    }
    assert(loc.is_valid(m));

    return loc;
}

TriMesh::Tuple TriMesh::Tuple::switch_edge(const TriMesh& m) const
{
    assert(is_valid(m));

    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    Tuple loc = *this;
    switch (lvid) {
    case 0:
        assert(m_eid == 1 || m_eid == 2);
        loc.m_eid = m_eid == 1 ? 2 : 1;
        break;
    case 1:
        assert(m_eid == 0 || m_eid == 2);
        loc.m_eid = m_eid == 0 ? 2 : 0;
        break;
    case 2:
        assert(m_eid == 0 || m_eid == 1);
        loc.m_eid = m_eid == 0 ? 1 : 0;
        break;
    default:;
    }
    assert(loc.is_valid(m));
    return loc;
}

std::optional<TriMesh::Tuple> TriMesh::Tuple::switch_face(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
    std::vector<size_t> v0_fids = m.m_vertex_connectivity[v0].m_conn_tris;
    std::vector<size_t> v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;

    std::sort(v0_fids.begin(), v0_fids.end());
    std::sort(v1_fids.begin(), v1_fids.end());
    std::vector<int> fids;
    std::set_intersection(
        v0_fids.begin(),
        v0_fids.end(),
        v1_fids.begin(),
        v1_fids.end(),
        std::back_inserter(fids)); // make sure this is correct
    assert(fids.size() == 1 || fids.size() == 2);

    if (fids.size() == 1) return {};

    Tuple loc = *this;

    // There is a triangle on the other side
    if (fids.size() == 2) {
        // Find the fid of the triangle on the other side
        size_t fid2 = fids[0] == m_fid ? fids[1] : fids[0];
        loc.m_fid = fid2;

        // Get sorted local indices of the two vertices in the new triangle
        size_t lv0_2 = m.m_tri_connectivity[fid2].find(v0);
        assert(lv0_2 == 0 || lv0_2 == 1 || lv0_2 == 2);
        size_t lv1_2 = m.m_tri_connectivity[fid2].find(v1);
        assert(lv1_2 == 0 || lv1_2 == 1 || lv1_2 == 2);

        if (lv0_2 > lv1_2) std::swap(lv0_2, lv1_2);

        // Assign the edge id depending on the table
        if (lv0_2 == 0 && lv1_2 == 1) {
            loc.m_eid = 2;
        } else if (lv0_2 == 1 && lv1_2 == 2) {
            loc.m_eid = 0;
        } else if (lv0_2 == 0 && lv1_2 == 2) {
            loc.m_eid = 1;
        } else {
            assert(false);
        }

        loc.update_hash(m);
    }
    assert(loc.is_valid(m));
    return loc;
}

bool TriMesh::Tuple::is_valid(const TriMesh& m) const
{
    if (m_fid + 1 == 0) return false;
    if (m.m_vertex_connectivity[m_vid].m_is_removed || m.m_tri_connectivity[m_fid].m_is_removed) {
        // assert(false);
        return false;
    }
    // Condition 3: tuple m_hash check
    if (m_hash != m.m_tri_connectivity[m_fid].hash) {
        // assert(false);
        return false;
    }
#ifndef NDEBUG
    //  Condition 0: Elements exist
    assert(m_vid < m.vert_capacity());
    assert(m_eid <= 2);
    assert(m_fid <= m.tri_capacity());

    // Condition 1: tid and vid are consistent
    const int lvid = m.m_tri_connectivity[m_fid].find(m_vid);
    assert(lvid == 0 || lvid == 1 || lvid == 2);

    // Condition 2: eid is valid
    const int v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];
    switch (m_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == v0 || m_vid == v2); break;
    case 2: assert(m_vid == v0 || m_vid == v1); break;
    default: assert(false);
    }
#endif

    return true;
}

// a valid mesh can have triangles that are is_removed == true
bool wmtk::TriMesh::check_mesh_connectivity_validity() const
{
    std::vector<std::vector<size_t>> conn_tris(vert_capacity());
    for (size_t i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 3; j++) conn_tris[m_tri_connectivity[i][j]].push_back(i);
    }

    for (unsigned i = 0; i < vert_capacity(); ++i)
        std::sort(conn_tris[i].begin(), conn_tris[i].end());

    // check conn_tets duplication, order, amount ...
    for (size_t i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;

        assert(
            m_vertex_connectivity[i].m_conn_tris == conn_tris[i] &&
            "m_vertex_connectivity[i].m_conn_tris!=conn_tris[i]");
    }
    return true;
}

bool wmtk::TriMesh::check_edge_manifold() const
{
    std::vector<size_t> count(tri_capacity() * 3, 0);
    auto faces = get_faces();
    for (auto f : faces) {
        for (int i = 0; i < 3; i++) {
            count[f.eid(*this)]++;
            if (count[f.eid(*this)] > 2) return false;
            f = (f.switch_vertex(*this)).switch_edge(*this);
        }
    }
    for (auto idx = 0; idx < tri_capacity() * 3; idx++) {
        if (count[idx] > 2) {
            return false;
        }
    }
    return true;
}

bool TriMesh::split_edge(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!split_edge_before(t)) return false;
    if (!t.is_valid(*this)) return false;
    // get local eid for return tuple construction
    auto eid = t.local_eid(*this);
    // get the vids
    size_t vid1 = t.vid(*this);
    size_t vid2 = switch_vertex(t).vid(*this);
    size_t fid1 = t.fid(*this);
    size_t fid1_vid3 =
        ((t.switch_vertex(*this)).switch_edge(*this)).switch_vertex(*this).vid(*this);

    for (auto vid : m_tri_connectivity[fid1].m_indices) {
        if ((vid != vid1) && (vid != vid2)) {
            assert(fid1_vid3 == vid);
            break;
        }
    }
    std::optional<size_t> fid2;
    if (t.switch_face(*this).has_value()) fid2 = (t.switch_face(*this).value()).fid(*this);
    std::optional<size_t> fid2_vid3;
    if (fid2.has_value()) {
        for (auto vid : m_tri_connectivity[fid2.value()].m_indices) {
            if ((vid != vid1) && (vid != vid2)) {
                fid2_vid3 = vid;
                break;
            }
        }
    }
    // record the vids that will be modified for roll backs on failure
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(3);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);
    old_vertices[2] = std::make_pair(fid1_vid3, m_vertex_connectivity[fid1_vid3]);
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(1);
    old_tris[0] = std::make_pair(fid1, m_tri_connectivity[fid1]);
    if (fid2.has_value()) {
        old_vertices.emplace_back(fid2_vid3.value(), m_vertex_connectivity[fid2_vid3.value()]);
        old_tris.emplace_back(fid2.value(), m_tri_connectivity[fid2.value()]);
    }

    size_t new_vid = get_next_empty_slot_v();
    size_t new_fid1 = get_next_empty_slot_t();
    std::optional<size_t> new_fid2;
    if (fid2.has_value()) new_fid2 = get_next_empty_slot_t();

    // first work on the vids
    // the old triangles are connected to the vertex of t
    // fid1_vid3
    m_vertex_connectivity[fid1_vid3].m_conn_tris.push_back(new_fid1);
    std::sort(
        m_vertex_connectivity[fid1_vid3].m_conn_tris.begin(),
        m_vertex_connectivity[fid1_vid3].m_conn_tris.end());
    // vid2
    vector_erase(m_vertex_connectivity[vid2].m_conn_tris, fid1);
    m_vertex_connectivity[vid2].m_conn_tris.push_back(new_fid1);
    if (fid2.has_value()) {
        vector_erase(m_vertex_connectivity[vid2].m_conn_tris, fid2.value());
        m_vertex_connectivity[vid2].m_conn_tris.push_back(new_fid2.value());
    }
    std::sort(
        m_vertex_connectivity[vid2].m_conn_tris.begin(),
        m_vertex_connectivity[vid2].m_conn_tris.end());
    // fid2_vid3
    if (fid2_vid3.has_value()) {
        m_vertex_connectivity[fid2_vid3.value()].m_conn_tris.push_back(new_fid2.value());
        std::sort(
            m_vertex_connectivity[fid2_vid3.value()].m_conn_tris.begin(),
            m_vertex_connectivity[fid2_vid3.value()].m_conn_tris.end());
    }

    // new_vid
    m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid1);
    m_vertex_connectivity[new_vid].m_conn_tris.push_back(new_fid1);
    if (fid2.has_value()) {
        m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid2.value());
        m_vertex_connectivity[new_vid].m_conn_tris.push_back(new_fid2.value());
    }
    std::sort(
        m_vertex_connectivity[new_vid].m_conn_tris.begin(),
        m_vertex_connectivity[new_vid].m_conn_tris.end());


    // now the triangles
    // need to update the hash
    // fid1 fid2 update m_indices and hash
    size_t j = m_tri_connectivity[fid1].find(vid2);
    m_tri_connectivity[fid1].m_indices[j] = new_vid;
    m_tri_connectivity[fid1].hash++;
    size_t i = m_tri_connectivity[fid1].find(vid1);
    size_t k = m_tri_connectivity[fid1].find(fid1_vid3);
    // new_fid1 m_indices in same order
    m_tri_connectivity[new_fid1].m_indices[i] = new_vid;
    m_tri_connectivity[new_fid1].m_indices[j] = vid2;
    m_tri_connectivity[new_fid1].m_indices[k] = fid1_vid3;
    m_tri_connectivity[new_fid1].hash++;

    if (fid2.has_value()) {
        j = m_tri_connectivity[fid2.value()].find(vid2);
        m_tri_connectivity[fid2.value()].m_indices[j] = new_vid;
        m_tri_connectivity[fid2.value()].hash++;
        i = m_tri_connectivity[fid2.value()].find(vid1);
        k = m_tri_connectivity[fid2.value()].find(fid2_vid3.value());
        // new_fid1 m_indices in same order
        m_tri_connectivity[new_fid2.value()].m_indices[i] = new_vid;
        m_tri_connectivity[new_fid2.value()].m_indices[j] = vid2;
        m_tri_connectivity[new_fid2.value()].m_indices[k] = fid2_vid3.value();
        m_tri_connectivity[new_fid2.value()].hash++;
    }
    // make the new tuple
    size_t new_fid = std::min(fid1, new_fid1);
    if (new_fid2.has_value()) new_fid = std::min(new_fid, new_fid2.value());
    int l = m_tri_connectivity[new_fid].find(new_vid);
    auto new_vertex = Tuple(new_vid, (l + 2) % 3, new_fid, *this);
    auto return_tuple = Tuple(vid1, eid, fid1, *this);
    assert(new_vertex.is_valid(*this));
    assert(return_tuple.is_valid(*this));

    new_tris = get_one_ring_tris_for_vertex(new_vertex);
    start_protect_attributes();

    // roll back if not successful
    if (!split_edge_after(return_tuple) || !invariants(new_tris)) {
        // rollback topo
        // restore old v, t
        for (auto old_v : old_vertices) m_vertex_connectivity[old_v.first] = old_v.second;
        for (auto old_t : old_tris) m_tri_connectivity[old_t.first] = old_t.second;
        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        m_tri_connectivity[new_fid1].m_is_removed = true;
        if (new_fid2.has_value()) m_tri_connectivity[new_fid2.value()].m_is_removed = true;
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_tris)
{
    if (!collapse_edge_before(loc0)) {
        return false;
    }
    // get fid for the return tuple
    // take the face that shares the same vertex the loc0 tuple is pointing to
    // or if that face doesn't exit
    // take the face that shares the same vertex of loc0
    auto new_fid =
        loc0.switch_vertex(*this).switch_edge(*this).switch_face(*this).has_value()
            ? (loc0.switch_vertex(*this).switch_edge(*this).switch_face(*this).value()).fid(*this)
            : (loc0.switch_edge(*this).switch_face(*this).value()).fid(*this);
    // get the vids
    size_t vid1 = loc0.vid(*this);
    size_t vid2 = switch_vertex(loc0).vid(*this);

    // record the vids that will be erased for roll backs on failure
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

    // get the fids
    auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;

    auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;

    // get the fids that will be modified
    auto n12_intersect_fids = set_intersection(n1_fids, n2_fids);
    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = loc0.fid(*this);
    TriMesh::Tuple loc1 = switch_face(loc0).value_or(loc0);
    size_t test_fid2 = loc1.fid(*this);
    //"faces at the edge is not correct"
    assert(
        vector_contains(n12_intersect_fids, test_fid1) &&
        vector_contains(n12_intersect_fids, test_fid2));
    // now mark the vertices as removed so the assertion for tuple validity in switch operations
    // won't fail
    m_vertex_connectivity[vid1].m_is_removed = true;
    m_vertex_connectivity[vid2].m_is_removed = true;
    for (size_t fid : n12_intersect_fids) {
        m_tri_connectivity[fid].m_is_removed = true;
    }

    std::vector<size_t> n12_union_fids;
    std::set_union(
        n1_fids.begin(),
        n1_fids.end(),
        n2_fids.begin(),
        n2_fids.end(),
        std::back_inserter(n12_union_fids));

    // record the fids that will be modified/erased for roll back on failure
    vector_unique(n12_union_fids);
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(n12_union_fids.size());

    for (int i = 0; i < old_tris.size(); i++) {
        size_t fid = n12_union_fids[i];
        old_tris[i] = std::make_pair(fid, m_tri_connectivity[fid]);
        m_tri_connectivity[fid].hash++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted
    size_t new_vid = get_next_empty_slot_v();
    for (size_t fid : n1_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid1);
            m_tri_connectivity[fid].m_indices[j] = new_vid;
        }
    }
    for (size_t fid : n2_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid2);
            m_tri_connectivity[fid].m_indices[j] = new_vid;
        }
    }

    // now work on vids
    // add in the new vertex

    for (size_t fid : n12_union_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else
            m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    }
    // This is sorting too, and it is important to sort
    vector_unique(m_vertex_connectivity[new_vid].m_conn_tris);

    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list
    std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
    for (size_t fid : n12_intersect_fids) {
        auto f_vids = m_tri_connectivity[fid].m_indices;
        for (size_t f_vid : f_vids) {
            if (f_vid != vid1 && f_vid != vid2) {
                same_edge_vid_fid.emplace_back(f_vid, fid);
                assert(vector_contains(m_vertex_connectivity[f_vid].m_conn_tris, fid));
                vector_erase(m_vertex_connectivity[f_vid].m_conn_tris, fid);
            }
        }
    }

    // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
    // update the old tuple version number
    // create an edge tuple for each changed edge
    // call back check will be done on this vector of tuples

    assert(m_vertex_connectivity[new_vid].m_conn_tris.size() != 0);

    const size_t gfid = m_vertex_connectivity[new_vid].m_conn_tris[0];
    int j = m_tri_connectivity[gfid].find(new_vid);
    auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, *this);
    int j_ret = m_tri_connectivity[new_fid].find(new_vid);
    auto return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, *this);
    assert(new_t.is_valid(*this));
    new_tris = get_one_ring_tris_for_vertex(new_t);

    start_protect_attributes();

    if (!collapse_edge_after(return_t) || !invariants(new_tris)) {
        // if call back check failed roll back
        // restore the changes for connected triangles and vertices
        // resotre the version-number
        // removed restore to false

        for (auto rollback : old_tris) {
            size_t fid = rollback.first;
            m_tri_connectivity[fid] = rollback.second;
        }
        for (auto rollback : old_vertices) {
            size_t vid = rollback.first;
            m_vertex_connectivity[vid] = rollback.second;
        }
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        for (auto vid_fid : same_edge_vid_fid) {
            size_t vid = vid_fid.first;
            size_t fid = vid_fid.second;
            m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
            std::sort(
                m_vertex_connectivity[vid].m_conn_tris.begin(),
                m_vertex_connectivity[vid].m_conn_tris.end());
        }
        for (size_t fid : n12_intersect_fids) {
            m_tri_connectivity[fid].m_is_removed = false;
        }

        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

bool TriMesh::swap_edge(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!swap_edge_before(t)) {
        return false;
    }

    // get the vids
    size_t vid1 = t.vid(*this);
    size_t vid2 = t.switch_vertex(*this).vid(*this);
    Tuple tmp_tuple;

    assert(t.switch_face(*this).has_value());
    tmp_tuple = switch_face(t).value();
    assert(tmp_tuple.is_valid(*this));
    tmp_tuple = tmp_tuple.switch_edge(*this);
    size_t vid3 = tmp_tuple.switch_vertex(*this).vid(*this);
    auto tmp_tuple2 = t.switch_edge(*this);
    assert(tmp_tuple2.is_valid(*this));
    size_t vid4 = tmp_tuple2.switch_vertex(*this).vid(*this);
    // record the vids that will be changed for roll backs on failure
    // namely the 4 vertices of the 2 triangles
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(4);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);
    old_vertices[2] = std::make_pair(vid3, m_vertex_connectivity[vid3]);
    old_vertices[3] = std::make_pair(vid4, m_vertex_connectivity[vid4]);

    // check if the triangles intersection is the one adjcent to the edge
    size_t test_fid1 = t.fid(*this);
    std::optional<size_t> test_fid2;
    if (!switch_face(t).has_value())
        return false; // can't sawp on boundary edge
    else
        test_fid2 = switch_face(t).value().fid(*this);
    assert(test_fid2.has_value());
    // record the fids that will be changed for roll backs on failure
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(2);
    old_tris[0] = std::make_pair(test_fid1, m_tri_connectivity[test_fid1]);
    old_tris[1] = std::make_pair(test_fid2.value(), m_tri_connectivity[test_fid2.value()]);

    // first work on triangles, there are only 2
    int j = m_tri_connectivity[test_fid1].find(vid2);
    m_tri_connectivity[test_fid1].m_indices[j] = vid3;
    m_tri_connectivity[test_fid1].hash++;

    j = m_tri_connectivity[test_fid2.value()].find(vid1);
    m_tri_connectivity[test_fid2.value()].m_indices[j] = vid4;
    m_tri_connectivity[test_fid2.value()].hash++;

    // then work on the vertices
    vector_erase(m_vertex_connectivity[vid1].m_conn_tris, test_fid2.value());
    vector_erase(m_vertex_connectivity[vid2].m_conn_tris, test_fid1);
    m_vertex_connectivity[vid3].m_conn_tris.push_back(test_fid1);
    vector_unique(m_vertex_connectivity[vid3].m_conn_tris);
    m_vertex_connectivity[vid4].m_conn_tris.push_back(test_fid2.value());
    vector_unique(m_vertex_connectivity[vid4].m_conn_tris);
    // change the tuple to the new edge tuple
    auto new_t = init_from_edge(vid4, vid3, test_fid2.value());

    assert(new_t.switch_vertex(*this).vid(*this) != vid1);
    assert(new_t.switch_vertex(*this).vid(*this) != vid2);
    assert(new_t.is_valid(*this));
    new_tris = {new_t, new_t.switch_face(*this).value()};
    start_protect_attributes();
    if (!swap_edge_after(new_t) || !invariants(new_tris)) {
        // restore the vertex and faces
        for (auto old_v : old_vertices) m_vertex_connectivity[old_v.first] = old_v.second;
        for (auto old_tri : old_tris) m_tri_connectivity[old_tri.first] = old_tri.second;
        rollback_protected_attributes();

        return false;
    }
    release_protect_attributes();
    return true;
}

bool TriMesh::smooth_vertex(const Tuple& loc0)
{
    ZoneScoped;
    if (!smooth_before(loc0)) return false;
    start_protect_attributes();
    if (!smooth_after(loc0) || !invariants(get_one_ring_tris_for_vertex(loc0))) {
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

void TriMesh::consolidate_mesh()
{
    auto v_cnt = 0;
    std::vector<size_t> map_v_ids(vert_capacity(), -1);
    for (auto i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        map_v_ids[i] = v_cnt;
        v_cnt++;
    }
    auto t_cnt = 0;
    std::vector<size_t> map_t_ids(tri_capacity(), -1);
    for (auto i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        map_t_ids[i] = t_cnt;
        t_cnt++;
    }
    v_cnt = 0;
    for (auto i = 0; i < vert_capacity(); i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;
        if (v_cnt != i) {
            assert(v_cnt < i);
            m_vertex_connectivity[v_cnt] = m_vertex_connectivity[i];
            if (p_vertex_attrs) p_vertex_attrs->move(i, v_cnt);
        }
        for (size_t& t_id : m_vertex_connectivity[v_cnt].m_conn_tris) t_id = map_t_ids[t_id];
        v_cnt++;
    }
    t_cnt = 0;
    for (int i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;

        if (t_cnt != i) {
            assert(t_cnt < i);
            m_tri_connectivity[t_cnt] = m_tri_connectivity[i];
            m_tri_connectivity[t_cnt].hash = 0;
            if (p_face_attrs) p_face_attrs->move(i, t_cnt);

            for (auto j = 0; j < 3; j++) {
                if (p_edge_attrs) p_edge_attrs->move(i * 3 + j, t_cnt * 3 + j);
            }
        }
        for (size_t& v_id : m_tri_connectivity[t_cnt].m_indices) v_id = map_v_ids[v_id];
        t_cnt++;
    }

    current_vert_size = v_cnt;
    current_tri_size = t_cnt;

    m_vertex_connectivity.resize(v_cnt);
    m_vertex_connectivity.shrink_to_fit();
    m_tri_connectivity.resize(t_cnt);
    m_tri_connectivity.shrink_to_fit();

    resize_mutex(vert_capacity());

    // Resize user class attributes
    if (p_vertex_attrs) p_vertex_attrs->resize(vert_capacity());
    if (p_edge_attrs) p_edge_attrs->resize(tri_capacity() * 3);
    if (p_face_attrs) p_face_attrs->resize(tri_capacity());

    assert(check_edge_manifold());
    assert(check_mesh_connectivity_validity());
}


std::vector<size_t> TriMesh::get_one_ring_vids_for_vertex_duplicate(const size_t& vid) const
{
    std::vector<size_t> one_ring;
    auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;

    one_ring.reserve(conn_tri.size() * 4);
    for (size_t tri : conn_tri) {
        for (auto j : m_tri_connectivity[tri].m_indices) {
            one_ring.push_back(j);
        }
    }

    return one_ring;
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_tris_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> one_ring;
    size_t vid = t.vid(*this);
    auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;
    one_ring.reserve(conn_tri.size());
    for (size_t tri : conn_tri) {
        int j = m_tri_connectivity[tri].find(vid);
        one_ring.emplace_back(vid, (j + 2) % 3, tri, *this);
        assert(one_ring[one_ring.size() - 1].is_valid(*this));
    }

    return one_ring;
}

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_edges_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<Tuple> one_ring_edges;
    std::vector<size_t> one_ring_vertices;
    size_t vid = t.vid(*this);
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    for (auto tri : one_ring_tris) {
        // find the vertex
        while (tri.vid(*this) != vid) {
            tri = tri.switch_vertex(*this).switch_edge(*this);
        }

        // push first edge if not there
        if (!vector_contains(one_ring_vertices, tri.switch_vertex(*this).vid(*this))) {
            one_ring_vertices.push_back(tri.switch_vertex(*this).vid(*this));
            one_ring_edges.push_back(tri.switch_vertex(*this));
        }

        // push second edge if not there
        tri = tri.switch_edge(*this);
        if (!vector_contains(one_ring_vertices, tri.switch_vertex(*this).vid(*this))) {
            one_ring_vertices.push_back(tri.switch_vertex(*this).vid(*this));
            one_ring_edges.push_back(tri.switch_vertex(*this));
        }
    }

    assert(one_ring_vertices.size() == one_ring_edges.size());

    return one_ring_edges;
}

std::array<wmtk::TriMesh::Tuple, 3> TriMesh::oriented_tri_vertices(
    const wmtk::TriMesh::Tuple& t) const
{
    std::array<TriMesh::Tuple, 3> incident_verts;
    size_t fid = t.fid(*this);
    auto indices = m_tri_connectivity[fid].m_indices;

    incident_verts[0] = Tuple(indices[0], 2, fid, *this);
    incident_verts[1] = Tuple(indices[1], 0, fid, *this);
    incident_verts[2] = Tuple(indices[2], 1, fid, *this);
    return incident_verts;
}

std::array<size_t, 3> TriMesh::oriented_tri_vids(const Tuple& t) const
{
    size_t fid = t.fid(*this);
    return oriented_tri_vids(fid);
}

std::array<size_t, 3> TriMesh::oriented_tri_vids(const int fid) const
{
    std::array<size_t, 3> incident_verts;
    auto indices = m_tri_connectivity[fid].m_indices;

    incident_verts[0] = indices[0];
    incident_verts[1] = indices[1];
    incident_verts[2] = indices[2];

    return incident_verts;
}

void TriMesh::init(size_t n_vertices, const std::vector<std::array<size_t, 3>>& tris)
{
    m_vertex_connectivity.resize(n_vertices);
    m_tri_connectivity.resize(tris.size());
    size_t hash_cnt = 0;
    for (int i = 0; i < tris.size(); i++) {
        m_tri_connectivity[i].m_indices = tris[i];

        m_tri_connectivity[i].hash = hash_cnt;
        for (int j = 0; j < 3; j++) {
            m_vertex_connectivity[tris[i][j]].m_conn_tris.push_back(i);
        }
    }
    current_vert_size = n_vertices;
    current_tri_size = tris.size();

    m_vertex_mutex.grow_to_at_least(n_vertices);

    // Resize user class attributes
    if (p_vertex_attrs) p_vertex_attrs->resize(vert_capacity());
    if (p_edge_attrs) p_edge_attrs->resize(tri_capacity() * 3);
    if (p_face_attrs) p_face_attrs->resize(tri_capacity());
}

void wmtk::TriMesh::init(const MatrixXi& F)
{
    size_t n_vertices = F.maxCoeff() + 1;

    std::vector<std::array<size_t, 3>> tris;
    tris.resize(F.rows());

    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            tris[i][j] = F(i, j);
        }
    }

    TriMesh::init(n_vertices, tris);
}

std::vector<TriMesh::Tuple> TriMesh::get_vertices() const
{
    const size_t n_vertices = vert_capacity();
    std::vector<Tuple> all_vertices_tuples;
    all_vertices_tuples.reserve(n_vertices);

    for (size_t i = 0; i < n_vertices; i++) {
        if (m_vertex_connectivity[i].m_is_removed) continue;

        const std::vector<size_t>& v_conn_fids = m_vertex_connectivity[i].m_conn_tris;
        size_t fid = *min_element(v_conn_fids.begin(), v_conn_fids.end());

        // get the 3 vid
        const std::array<size_t, 3> f_conn_verts = m_tri_connectivity[fid].m_indices;
        assert(i == f_conn_verts[0] || i == f_conn_verts[1] || i == f_conn_verts[2]);

        size_t eid = -1;

        // eid is the same as the lvid
        if (i == f_conn_verts[0]) eid = 2;
        if (i == f_conn_verts[1])
            eid = 0;
        else
            eid = 1;

        Tuple v_tuple = Tuple(i, eid, fid, *this);
        assert(v_tuple.is_valid(*this));
        all_vertices_tuples.push_back(v_tuple);
    }
    return all_vertices_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_faces() const
{
    std::vector<Tuple> all_faces_tuples;
    all_faces_tuples.reserve(tri_capacity());
    for (size_t i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        // get the 3 vid
        const std::array<size_t, 3>& f_conn_verts = m_tri_connectivity[i].m_indices;
        size_t vid = f_conn_verts[0];
        Tuple f_tuple = Tuple(vid, 2, i, *this);
        assert(f_tuple.is_valid(*this));
        all_faces_tuples.push_back(f_tuple);
    }
    return all_faces_tuples;
}

std::vector<TriMesh::Tuple> TriMesh::get_edges() const
{
    std::vector<TriMesh::Tuple> all_edges_tuples;
    all_edges_tuples.reserve(tri_capacity() * 3);
    for (int i = 0; i < tri_capacity(); i++) {
        if (m_tri_connectivity[i].m_is_removed) continue;
        for (int j = 0; j < 3; j++) {
            size_t l = (j + 2) % 3;
            auto tup = Tuple(m_tri_connectivity[i].m_indices[j], l, i, *this);
            if (tup.eid(*this) == 3 * i + l) all_edges_tuples.emplace_back(tup);
        }
    }

    return all_edges_tuples;
}

TriMesh::Tuple TriMesh::init_from_edge(size_t vid1, size_t vid2, size_t fid) const
{
    auto a = m_tri_connectivity[fid].find(vid1);
    auto b = m_tri_connectivity[fid].find(vid2);
    assert(a != -1 && b != -1);
    // 0,1 - >2, 1,2-> 0, 0,2->1
    return Tuple(vid1, 3 - (a + b), fid, *this);
}


size_t TriMesh::get_next_empty_slot_t()
{
    while (current_tri_size + MAX_THREADS >= m_tri_connectivity.size() ||
           tri_connectivity_synchronizing_flag) {
        if (tri_connectivity_lock.try_lock()) {
            if (current_tri_size + MAX_THREADS < m_tri_connectivity.size()) {
                tri_connectivity_lock.unlock();
                break;
            }
            tri_connectivity_synchronizing_flag = true;
            auto current_capacity = m_tri_connectivity.size();
            if (p_edge_attrs) p_edge_attrs->resize(2 * current_capacity * 3);
            if (p_face_attrs) p_face_attrs->resize(2 * current_capacity);
            m_tri_connectivity.grow_to_at_least(2 * current_capacity);
            tri_connectivity_synchronizing_flag = false;
            tri_connectivity_lock.unlock();
            break;
        }
    }

    return current_tri_size++;
}

size_t TriMesh::get_next_empty_slot_v()
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
            if (p_vertex_attrs) p_vertex_attrs->resize(2 * current_capacity);
            resize_mutex(2 * current_capacity);
            m_vertex_connectivity.grow_to_at_least(2 * current_capacity);
            vertex_connectivity_synchronizing_flag = false;
            vertex_connectivity_lock.unlock();
            break;
        }
    }

    return current_vert_size++;
}

bool TriMesh::swap_edge_before(const Tuple& t)
{
    if (!t.switch_face(*this).has_value()) return false;
    size_t v4 = ((t.switch_face(*this).value()).switch_edge(*this)).switch_vertex(*this).vid(*this);
    size_t v3 = ((t.switch_edge(*this)).switch_vertex(*this)).vid(*this);
    if (!set_intersection(
             m_vertex_connectivity[v4].m_conn_tris,
             m_vertex_connectivity[v3].m_conn_tris)
             .empty())
        return false;
    return true;
}


// link check, prerequisite for edge collapse
bool wmtk::TriMesh::check_link_condition(const Tuple& edge) const
{
    assert(edge.is_valid(*this));
    size_t vid1 = edge.vid(*this);
    size_t vid2 = switch_vertex(edge).vid(*this);
    auto vid1_ring = get_one_ring_edges_for_vertex(edge);
    auto vid2_ring = get_one_ring_edges_for_vertex(switch_vertex(edge));


    size_t dummy = std::numeric_limits<size_t>::max();

    std::vector<size_t> lk_vid1;
    std::vector<size_t> lk_vid2;


    std::vector<std::pair<size_t, size_t>> lk_e_vid1;
    std::vector<std::pair<size_t, size_t>> lk_e_vid2;

    for (auto e_vid : vid1_ring) {
        if (!e_vid.switch_face(*this).has_value()) {
            lk_vid1.push_back(dummy);
            lk_e_vid1.emplace_back(e_vid.vid(*this), dummy);
        }
        lk_vid1.push_back(e_vid.vid(*this));
    }
    std::vector<Tuple> vid1_tris = get_one_ring_tris_for_vertex(edge);
    for (auto v1_tri_t : vid1_tris) {
        auto indices = m_tri_connectivity[v1_tri_t.fid(*this)].m_indices;
        auto l = m_tri_connectivity[v1_tri_t.fid(*this)].find(vid1);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid1.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid1);

    for (auto e_vid : vid2_ring) {
        if (!e_vid.switch_face(*this).has_value()) {
            lk_vid2.push_back(dummy);
            lk_e_vid2.emplace_back(e_vid.vid(*this), dummy);
        }
        lk_vid2.push_back(e_vid.vid(*this));
    }
    std::vector<Tuple> vid2_tris = get_one_ring_tris_for_vertex(switch_vertex(edge));
    for (auto v2_tri_t : vid2_tris) {
        auto indices = m_tri_connectivity[v2_tri_t.fid(*this)].m_indices;
        auto l = m_tri_connectivity[v2_tri_t.fid(*this)].find(vid2);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid2.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid2);
    auto lk_vid12 = set_intersection(lk_vid1, lk_vid2);
    std::vector<size_t> lk_edge;
    lk_edge.push_back((edge.switch_edge(*this)).switch_vertex(*this).vid(*this));
    if (!edge.switch_face(*this).has_value())
        lk_edge.push_back(dummy);
    else
        lk_edge.push_back(
            ((edge.switch_face(*this).value()).switch_edge(*this)).switch_vertex(*this).vid(*this));
    vector_sort(lk_edge);
    bool v_link =
        (lk_vid12.size() == lk_edge.size() &&
         std::equal(lk_vid12.begin(), lk_vid12.end(), lk_edge.begin()));

    // check edge link condition
    // in 2d edge link for an edge is always empty

    bool e_link = true;
    std::vector<std::pair<size_t, size_t>> res;
    std::sort(lk_e_vid1.begin(), lk_e_vid1.end());
    std::sort(lk_e_vid2.begin(), lk_e_vid2.end());
    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));
    if (res.size() > 0) return false;

    return v_link;
}

int TriMesh::release_vertex_mutex_in_stack()
{
    int num_released = 0;
    for (int i = mutex_release_stack.local().size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(mutex_release_stack.local()[i]);
        num_released++;
    }
    mutex_release_stack.local().clear();
    return num_released;
}

bool TriMesh::try_set_vertex_mutex_two_ring(const Tuple& v, int threadid)
{
    for (auto v_one_ring : get_one_ring_edges_for_vertex(v)) {
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) continue;
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            mutex_release_stack.local().push_back(v_one_ring.vid(*this));
            for (auto v_two_ring : get_one_ring_edges_for_vertex(v_one_ring)) {
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) continue;
                if (try_set_vertex_mutex(v_two_ring, threadid)) {
                    mutex_release_stack.local().push_back(v_two_ring.vid(*this));
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

bool TriMesh::try_set_edge_mutex_two_ring(const Tuple& e, int threadid)
{
    Tuple v1 = e;
    bool release_flag = false;

    // try v1
    if (m_vertex_mutex[v1.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v1, threadid)) {
            mutex_release_stack.local().push_back(v1.vid(*this));
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
    Tuple v2 = switch_vertex(e);
    if (m_vertex_mutex[v2.vid(*this)].get_owner() != threadid) {
        if (try_set_vertex_mutex(v2, threadid)) {
            mutex_release_stack.local().push_back(v2.vid(*this));
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

    // try v1 two ring
    release_flag = !try_set_vertex_mutex_two_ring(v1, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    // try v2 two ring
    release_flag = !try_set_vertex_mutex_two_ring(v2, threadid);

    if (release_flag) {
        release_vertex_mutex_in_stack();
        return false;
    }

    return true;
}

bool wmtk::TriMesh::try_set_vertex_mutex_one_ring(const Tuple& v, int threadid)
{
    auto& stack = mutex_release_stack.local();
    auto vid = v.vid(*this);
    if (m_vertex_mutex[vid].get_owner() != threadid) {
        if (try_set_vertex_mutex(v, threadid)) {
            stack.push_back(vid);
            for (auto v_one_ring : get_one_ring_vids_for_vertex_duplicate(vid)) {
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

void wmtk::TriMesh::for_each_edge(const std::function<void(const TriMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, tri_capacity()),
            [&](const tbb::blocked_range<int>& r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    if (!tuple_from_tri(i).is_valid(*this)) continue;
                    for (int j = 0; j < 3; j++) {
                        auto tup = tuple_from_edge(i, j);
                        if (tup.eid(*this) == 3 * i + j) {
                            func(tup);
                        }
                    }
                }
            });
    });
}

void wmtk::TriMesh::for_each_vertex(const std::function<void(const TriMesh::Tuple&)>& func)
{
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

void wmtk::TriMesh::for_each_face(const std::function<void(const TriMesh::Tuple&)>& func)
{
    tbb::task_arena arena(NUM_THREADS);
    arena.execute([&] {
        tbb::parallel_for(
            tbb::blocked_range<int>(0, tri_capacity()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    auto tup = tuple_from_tri(i);
                    if (!tup.is_valid(*this)) continue;
                    func(tup);
                }
            });
    });
}