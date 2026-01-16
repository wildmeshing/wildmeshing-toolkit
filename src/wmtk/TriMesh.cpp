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
    /*
     * The edge ID is constructed from the min face ID incident to the edge and in there the local
     * edge ID:
     * eid = 3 * min_fid + local_eid
     */

    // const std::optional<Tuple> t_opp = switch_face(m);
    // if (t_opp.has_value()) {
    //     size_t fid2 = t_opp.value().fid(m);
    //     size_t min_fid = std::min(m_fid, fid2);
    //     if (min_fid == fid2) {
    //         int i = m.m_tri_connectivity[fid2].find(m_vid);
    //         int j = m.m_tri_connectivity[fid2].find(switch_vertex(m).vid(m));
    //         return min_fid * 3 + 3 - i - j;
    //     }
    // }
    // return m_fid * 3 + m_eid;

    const size_t v_opp = switch_vertex(m).vid(m);

    const std::vector<size_t>& fids = m.m_vertex_connectivity[m_vid].m_conn_tris;

    // find face that contain m_vid and v_opp
    for (const size_t f : fids) {
        const auto& f_vids = m.m_tri_connectivity[f].m_indices;

        int local_v0 = -1;
        int local_v1 = -1;
        for (size_t i = 0; i < f_vids.size(); ++i) {
            if (f_vids[i] == m_vid) {
                local_v0 = i;
            } else if (f_vids[i] == v_opp) {
                local_v1 = i;
            }
        }
        if (local_v1 == -1) {
            continue;
        }
        assert(local_v0 != -1);
        size_t local_eid = 3 - (local_v0 + local_v1);
        // fids are sorted --> return smallest fid
        return 3 * f + local_eid;
    }

    assert(false); // this statement should never be reached
    return m_fid * 3 + m_eid;
}


TriMesh::Tuple TriMesh::Tuple::switch_vertex(const TriMesh& m) const
{
    assert(is_valid(m));

    const size_t loc_v0 = m.m_tri_connectivity[m_fid][0];
    const size_t v1 = m.m_tri_connectivity[m_fid][1];
    const size_t v2 = m.m_tri_connectivity[m_fid][2];

    Tuple loc = *this;
    switch (m_eid) {
    case 0:
        assert(m_vid == v1 || m_vid == v2);
        loc.m_vid = m_vid == v1 ? v2 : v1;
        break;
    case 1:
        assert(m_vid == loc_v0 || m_vid == v2);
        loc.m_vid = m_vid == loc_v0 ? v2 : loc_v0;
        break;
    case 2:
        assert(m_vid == loc_v0 || m_vid == v1);
        loc.m_vid = m_vid == loc_v0 ? v1 : loc_v0;
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

    const size_t loc_v0 = m_vid;
    const size_t v1 = this->switch_vertex(m).m_vid;

    // Intersect the 1-ring of the two vertices in the edge pointed by the tuple
    std::vector<size_t> v0_fids = m.m_vertex_connectivity[loc_v0].m_conn_tris;
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
        size_t lv0_2 = m.m_tri_connectivity[fid2].find(loc_v0);
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

std::vector<TriMesh::Tuple> TriMesh::Tuple::switch_faces(const TriMesh& m) const
{
    assert(is_valid(m));

    std::vector<Tuple> faces;

    size_t loc_v0 = m_vid;
    size_t v1 = this->switch_vertex(m).m_vid;

    const std::vector<size_t>& v0_fids = m.m_vertex_connectivity[loc_v0].m_conn_tris;
    const std::vector<size_t>& v1_fids = m.m_vertex_connectivity[v1].m_conn_tris;
    // get the smaller vector of the two
    const std::vector<size_t>& fids = v0_fids.size() <= v1_fids.size() ? v0_fids : v1_fids;

    // find faces that contain v0 and v1
    for (const size_t f : fids) {
        if (f == m_fid) {
            continue;
        }

        const auto& f_vids = m.m_tri_connectivity[f].m_indices;

        int local_v0 = -1;
        int local_v1 = -1;
        for (size_t i = 0; i < f_vids.size(); ++i) {
            if (f_vids[i] == loc_v0) {
                local_v0 = i;
            } else if (f_vids[i] == v1) {
                local_v1 = i;
            }
        }
        if (local_v0 == -1 || local_v1 == -1) {
            continue;
        }
        assert(local_v0 != -1);
        size_t local_eid = 3 - (local_v0 + local_v1);
        faces.emplace_back(loc_v0, local_eid, f, m);
    }

    return faces;
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
    const int loc_v0 = m.m_tri_connectivity[m_fid][0];
    const int v1 = m.m_tri_connectivity[m_fid][1];
    const int v2 = m.m_tri_connectivity[m_fid][2];
    switch (m_eid) {
    case 0: assert(m_vid == v1 || m_vid == v2); break;
    case 1: assert(m_vid == loc_v0 || m_vid == v2); break;
    case 2: assert(m_vid == loc_v0 || m_vid == v1); break;
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
    if (!split_edge_before(t)) {
        return false;
    }
    if (!t.is_valid(*this)) {
        return false;
    }
    // get local eid for return tuple construction
    const size_t eid = t.local_eid(*this);
    const size_t t_fid = t.fid(*this);
    // get the vids
    const size_t vid1 = t.vid(*this);
    const size_t vid2 = t.switch_vertex(*this).vid(*this);
    std::vector<size_t> fids;
    fids.reserve(2);
    std::vector<size_t> vid3s;
    vid3s.reserve(2);

    // find neighboring faces sharing vid1 and vid2
    for (const size_t f : m_vertex_connectivity[vid1].m_conn_tris) {
        size_t v3;
        bool v2_found = false;
        for (const size_t v : m_tri_connectivity[f].m_indices) {
            if (v == vid2) {
                v2_found = true;
                continue;
            }
            if (v != vid1) {
                v3 = v;
            }
        }

        if (!v2_found) {
            continue;
        }
        fids.push_back(f);
        vid3s.push_back(v3);
    }
    assert(!(fids.empty() || vid3s.empty()));

    // record the vids that will be modified for roll backs on failure
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(1);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);
    for (size_t i = 0; i < vid3s.size(); ++i) {
        old_vertices.emplace_back(vid3s[i], m_vertex_connectivity[vid3s[i]]);
        old_tris.emplace_back(fids[i], m_tri_connectivity[fids[i]]);
    }

    const size_t new_vid = get_next_empty_slot_v();
    std::vector<size_t> new_fids(fids.size());
    for (size_t i = 0; i < fids.size(); ++i) {
        new_fids[i] = get_next_empty_slot_t();
    }

    // first work on the vids
    // the old triangles are connected to the vertex of t
    // fid1_vid3
    for (size_t i = 0; i < fids.size(); ++i) {
        m_vertex_connectivity[vid3s[i]].m_conn_tris.push_back(new_fids[i]);
        std::sort(
            m_vertex_connectivity[vid3s[i]].m_conn_tris.begin(),
            m_vertex_connectivity[vid3s[i]].m_conn_tris.end());
    }

    // vid2
    for (size_t i = 0; i < fids.size(); ++i) {
        vector_erase(m_vertex_connectivity[vid2].m_conn_tris, fids[i]);
        m_vertex_connectivity[vid2].m_conn_tris.push_back(new_fids[i]);
    }

    std::sort(
        m_vertex_connectivity[vid2].m_conn_tris.begin(),
        m_vertex_connectivity[vid2].m_conn_tris.end());

    // new_vid
    for (size_t i = 0; i < fids.size(); ++i) {
        m_vertex_connectivity[new_vid].m_conn_tris.push_back(fids[i]);
    }
    for (size_t i = 0; i < fids.size(); ++i) {
        m_vertex_connectivity[new_vid].m_conn_tris.push_back(new_fids[i]);
    }
    std::sort(
        m_vertex_connectivity[new_vid].m_conn_tris.begin(),
        m_vertex_connectivity[new_vid].m_conn_tris.end());


    // now the triangles
    // need to update the hash
    // fid1 fid2 update m_indices and hash
    for (size_t i = 0; i < fids.size(); ++i) {
        const size_t fid = fids[i];
        const size_t vid3 = vid3s[i];
        const size_t new_fid = new_fids[i];

        const size_t tj = m_tri_connectivity[fid].find(vid2);
        m_tri_connectivity[fid].m_indices[tj] = new_vid;
        m_tri_connectivity[fid].hash++;
        const size_t ti = m_tri_connectivity[fid].find(vid1);
        const size_t tk = m_tri_connectivity[fid].find(vid3);
        // new_fid1 m_indices in same order
        m_tri_connectivity[new_fid].m_indices[ti] = new_vid;
        m_tri_connectivity[new_fid].m_indices[tj] = vid2;
        m_tri_connectivity[new_fid].m_indices[tk] = vid3;
        m_tri_connectivity[new_fid].hash++;
    }

    // make the new tuple
    size_t min_fid = m_vertex_connectivity[new_vid].m_conn_tris[0]; // making use of sorted vector


    int l = m_tri_connectivity[min_fid].find(new_vid);
    auto new_vertex = Tuple(new_vid, (l + 2) % 3, min_fid, *this);
    auto return_tuple = Tuple(vid1, eid, t_fid, *this);
    assert(new_vertex.is_valid(*this));
    assert(return_tuple.is_valid(*this));

    new_tris = get_one_ring_tris_for_vertex(new_vertex);
    start_protect_attributes();

    // roll back if not successful
    if (!split_edge_after(return_tuple) || !invariants(new_tris)) {
        // rollback topo
        // restore old v, t
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        for (const auto& old_t : old_tris) {
            m_tri_connectivity[old_t.first] = old_t.second;
        }
        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        for (size_t i = 0; i < fids.size(); ++i) {
            m_tri_connectivity[new_fids[i]].m_is_removed = true;
        }
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

// bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_tris)
// {
//     if (!collapse_edge_before(loc0)) {
//         return false;
//     }
//     // get fid for the return tuple
//     // take the face that shares the same vertex the loc0 tuple is pointing to
//     // or if that face doesn't exit
//     // take the face that shares the same vertex of loc0
//     auto new_fid =
//         loc0.switch_vertex(*this).switch_edge(*this).switch_face(*this).has_value()
//             ?
//             (loc0.switch_vertex(*this).switch_edge(*this).switch_face(*this).value()).fid(*this)
//             : (loc0.switch_edge(*this).switch_face(*this).value()).fid(*this);
//     // get the vids
//     size_t vid1 = loc0.vid(*this);
//     size_t vid2 = switch_vertex(loc0).vid(*this);

//     // record the vids that will be erased for roll backs on failure
//     std::vector<std::pair<size_t, VertexConnectivity>> old_vertices(2);
//     old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
//     old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

//     // get the fids
//     auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;

//     auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;

//     // get the fids that will be modified
//     auto n12_intersect_fids = set_intersection(n1_fids, n2_fids);
//     // check if the triangles intersection is the one adjcent to the edge
//     size_t test_fid1 = loc0.fid(*this);
//     TriMesh::Tuple loc1 = switch_face(loc0).value_or(loc0);
//     size_t test_fid2 = loc1.fid(*this);
//     //"faces at the edge is not correct"
//     assert(
//         vector_contains(n12_intersect_fids, test_fid1) &&
//         vector_contains(n12_intersect_fids, test_fid2));
//     // now mark the vertices as removed so the assertion for tuple validity in switch operations
//     // won't fail
//     m_vertex_connectivity[vid1].m_is_removed = true;
//     m_vertex_connectivity[vid2].m_is_removed = true;
//     for (size_t fid : n12_intersect_fids) {
//         m_tri_connectivity[fid].m_is_removed = true;
//     }

//     std::vector<size_t> n12_union_fids;
//     std::set_union(
//         n1_fids.begin(),
//         n1_fids.end(),
//         n2_fids.begin(),
//         n2_fids.end(),
//         std::back_inserter(n12_union_fids));

//     // record the fids that will be modified/erased for roll back on failure
//     vector_unique(n12_union_fids);
//     std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(n12_union_fids.size());

//     for (int i = 0; i < old_tris.size(); i++) {
//         size_t fid = n12_union_fids[i];
//         old_tris[i] = std::make_pair(fid, m_tri_connectivity[fid]);
//         m_tri_connectivity[fid].hash++;
//     }
//     // modify the triangles
//     // the m_conn_tris needs to be sorted
//     size_t new_vid = get_next_empty_slot_v();
//     for (size_t fid : n1_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else {
//             int j = m_tri_connectivity[fid].find(vid1);
//             m_tri_connectivity[fid].m_indices[j] = new_vid;
//         }
//     }
//     for (size_t fid : n2_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else {
//             int j = m_tri_connectivity[fid].find(vid2);
//             m_tri_connectivity[fid].m_indices[j] = new_vid;
//         }
//     }

//     // now work on vids
//     // add in the new vertex

//     for (size_t fid : n12_union_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else
//             m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
//     }
//     // This is sorting too, and it is important to sort
//     vector_unique(m_vertex_connectivity[new_vid].m_conn_tris);

//     // remove the erased fids from the vertices' (the one of the triangles that is not the end
//     // points of the edge) connectivity list
//     std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
//     for (size_t fid : n12_intersect_fids) {
//         auto f_vids = m_tri_connectivity[fid].m_indices;
//         for (size_t f_vid : f_vids) {
//             if (f_vid != vid1 && f_vid != vid2) {
//                 same_edge_vid_fid.emplace_back(f_vid, fid);
//                 assert(vector_contains(m_vertex_connectivity[f_vid].m_conn_tris, fid));
//                 vector_erase(m_vertex_connectivity[f_vid].m_conn_tris, fid);
//             }
//         }
//     }

//     // ? ? tuples changes. this needs to be done before post check since checked are done on tuples
//     // update the old tuple version number
//     // create an edge tuple for each changed edge
//     // call back check will be done on this vector of tuples

//     assert(m_vertex_connectivity[new_vid].m_conn_tris.size() != 0);

//     const size_t gfid = m_vertex_connectivity[new_vid].m_conn_tris[0];
//     int j = m_tri_connectivity[gfid].find(new_vid);
//     auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, *this);
//     int j_ret = m_tri_connectivity[new_fid].find(new_vid);
//     auto return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, *this);
//     assert(new_t.is_valid(*this));
//     new_tris = get_one_ring_tris_for_vertex(new_t);

//     start_protect_attributes();

//     if (!collapse_edge_after(return_t) || !invariants(new_tris)) {
//         // if call back check failed roll back
//         // restore the changes for connected triangles and vertices
//         // resotre the version-number
//         // removed restore to false

//         for (auto rollback : old_tris) {
//             size_t fid = rollback.first;
//             m_tri_connectivity[fid] = rollback.second;
//         }
//         for (auto rollback : old_vertices) {
//             size_t vid = rollback.first;
//             m_vertex_connectivity[vid] = rollback.second;
//         }
//         m_vertex_connectivity[new_vid].m_conn_tris.clear();
//         m_vertex_connectivity[new_vid].m_is_removed = true;
//         for (auto vid_fid : same_edge_vid_fid) {
//             size_t vid = vid_fid.first;
//             size_t fid = vid_fid.second;
//             m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
//             std::sort(
//                 m_vertex_connectivity[vid].m_conn_tris.begin(),
//                 m_vertex_connectivity[vid].m_conn_tris.end());
//         }
//         for (size_t fid : n12_intersect_fids) {
//             m_tri_connectivity[fid].m_is_removed = false;
//         }

//         rollback_protected_attributes();
//         return false;
//     }
//     release_protect_attributes();
//     return true;
// }

bool TriMesh::collapse_edge(const Tuple& loc0, std::vector<Tuple>& new_tris)
{
    if (!collapse_edge_before(loc0)) {
        return false;
    }

    Tuple return_t;
    size_t new_vid;
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris;
    std::vector<std::pair<size_t, VertexConnectivity>> old_vertices;
    std::vector<std::pair<size_t, size_t>> same_edge_vid_fid;
    std::vector<size_t> n12_intersect_fids;

    collapse_edge_conn(
        loc0,
        new_tris,
        return_t,
        new_vid,
        old_tris,
        old_vertices,
        same_edge_vid_fid,
        n12_intersect_fids);

    start_protect_attributes();

    if (!collapse_edge_after(return_t) || !invariants(new_tris)) {
        // if call back check failed roll back
        // restore the changes for connected triangles and vertices
        // resotre the version-number
        // removed restore to false

        // for (auto rollback : old_tris) {
        //     size_t fid = rollback.first;
        //     m_tri_connectivity[fid] = rollback.second;
        // }
        // for (auto rollback : old_vertices) {
        //     size_t vid = rollback.first;
        //     m_vertex_connectivity[vid] = rollback.second;
        // }
        // m_vertex_connectivity[new_vid].m_conn_tris.clear();
        // m_vertex_connectivity[new_vid].m_is_removed = true;
        // for (auto vid_fid : same_edge_vid_fid) {
        //     size_t vid = vid_fid.first;
        //     size_t fid = vid_fid.second;
        //     m_vertex_connectivity[vid].m_conn_tris.push_back(fid);
        //     std::sort(
        //         m_vertex_connectivity[vid].m_conn_tris.begin(),
        //         m_vertex_connectivity[vid].m_conn_tris.end());
        // }
        // for (size_t fid : n12_intersect_fids) {
        //     m_tri_connectivity[fid].m_is_removed = false;
        // }
        collapse_edge_rollback(
            new_vid,
            old_tris,
            old_vertices,
            same_edge_vid_fid,
            n12_intersect_fids);

        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}

void TriMesh::collapse_edge_rollback(
    size_t& new_vid,
    std::vector<std::pair<size_t, TriangleConnectivity>>& old_tris,
    std::vector<std::pair<size_t, VertexConnectivity>>& old_vertices,
    std::vector<std::pair<size_t, size_t>>& same_edge_vid_fid,
    std::vector<size_t>& n12_intersect_fids)
{
    for (auto rollback : old_tris) {
        size_t fid = rollback.first;
        m_tri_connectivity[fid] = rollback.second;
    }
    for (auto rollback : old_vertices) {
        size_t vid = rollback.first;
        m_vertex_connectivity[vid] = rollback.second;
    }

    // CHANGE: no new vertex
    // m_vertex_connectivity[new_vid].m_conn_tris.clear();
    // m_vertex_connectivity[new_vid].m_is_removed = true;


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
}


void TriMesh::collapse_edge_conn(
    const Tuple& loc0,
    std::vector<Tuple>& new_tris,
    Tuple& return_t,
    size_t& new_vid,
    std::vector<std::pair<size_t, TriangleConnectivity>>& old_tris,
    std::vector<std::pair<size_t, VertexConnectivity>>& old_vertices,
    std::vector<std::pair<size_t, size_t>>& same_edge_vid_fid,
    std::vector<size_t>& n12_intersect_fids)
{
    // get fid for the return tuple
    // take the face that shares the same vertex the loc0 tuple is pointing to
    // or if that face doesn't exit
    // take the face that shares the same vertex of loc0
    const SmartTuple smart_loc0(*this, loc0);
    const SmartTuple opp_edge = smart_loc0.switch_vertex().switch_edge();
    const auto opp_edge_opp_faces = opp_edge.tuple().switch_faces(*this);
    size_t new_fid = -1;
    if (!opp_edge_opp_faces.empty()) {
        new_fid = opp_edge_opp_faces[0].fid(*this);
    } else {
        const SmartTuple inc_edge = smart_loc0.switch_edge();
        const auto inc_edge_opp_faces = inc_edge.tuple().switch_faces(*this);
        assert(!inc_edge_opp_faces.empty()); // should exist due to link condition
        new_fid = inc_edge_opp_faces[0].fid(*this);
    }

    // get the vids
    const size_t vid1 = loc0.vid(*this);
    const size_t vid2 = switch_vertex(loc0).vid(*this);

    // record the vids that will be erased for roll backs on failure
    old_vertices.resize(2);
    old_vertices[0] = std::make_pair(vid1, m_vertex_connectivity[vid1]);
    old_vertices[1] = std::make_pair(vid2, m_vertex_connectivity[vid2]);

    // get the fids
    const auto n1_fids = m_vertex_connectivity[vid1].m_conn_tris;
    const auto n2_fids = m_vertex_connectivity[vid2].m_conn_tris;

    // get the fids that will be modified
    n12_intersect_fids = set_intersection(n1_fids, n2_fids);
#ifndef NDEBUG
    {
        // check if the triangles intersection is the one adjcent to the edge
        assert(vector_contains(n12_intersect_fids, loc0.fid(*this)));
        for (const Tuple& loc1 : loc0.switch_faces(*this)) {
            assert(vector_contains(n12_intersect_fids, loc1.fid(*this)));
        }
        //"faces at the edge is not correct"
    }
#endif // !NDEBUG

    // now mark the vertices as removed so the assertion for tuple validity in switch operations
    // won't fail

    // CHANGE: preserve v2 now, do not generate new vertex for collapse
    m_vertex_connectivity[vid1].m_is_removed = true;
    // m_vertex_connectivity[vid2].m_is_removed = true;
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
    old_tris.resize(n12_union_fids.size());

    for (int i = 0; i < old_tris.size(); i++) {
        size_t fid = n12_union_fids[i];
        old_tris[i] = std::make_pair(fid, m_tri_connectivity[fid]);
        m_tri_connectivity[fid].hash++;
    }
    // modify the triangles
    // the m_conn_tris needs to be sorted

    // CHANGE: new_vid now is v2
    // new_vid = get_next_empty_slot_v();
    new_vid = vid2;

    for (size_t fid : n1_fids) {
        if (m_tri_connectivity[fid].m_is_removed)
            continue;
        else {
            int j = m_tri_connectivity[fid].find(vid1);

            // CHANGE: new_vid now is v2
            // m_tri_connectivity[fid].m_indices[j] = new_vid;
            m_tri_connectivity[fid].m_indices[j] = vid2;
        }
    }
    // CHANGE: no need to update
    // for (size_t fid : n2_fids) {
    //     if (m_tri_connectivity[fid].m_is_removed)
    //         continue;
    //     else {
    //         int j = m_tri_connectivity[fid].find(vid2);
    //         m_tri_connectivity[fid].m_indices[j] = new_vid;
    //     }
    // }

    // now work on vids
    // add in the new vertex

    // CHANGE: change vf connectivity f v2 instead of adding new

    // add preseved v1 faces to v2
    // clear v2 conn first
    m_vertex_connectivity[vid2].m_conn_tris.clear();
    for (size_t fid : n12_union_fids) {
        if (m_tri_connectivity[fid].m_is_removed) {
            continue;
        } else {
            m_vertex_connectivity[vid2].m_conn_tris.push_back(fid);
        }
    }

    // this is sorting
    vector_unique(m_vertex_connectivity[vid2].m_conn_tris);

    // for (size_t fid : n12_union_fids) {
    //     if (m_tri_connectivity[fid].m_is_removed)
    //         continue;
    //     else
    //         m_vertex_connectivity[new_vid].m_conn_tris.push_back(fid);
    // }
    // // This is sorting too, and it is important to sort
    // vector_unique(m_vertex_connectivity[new_vid].m_conn_tris);

    // remove the erased fids from the vertices' (the one of the triangles that is not the end
    // points of the edge) connectivity list

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

    // CHANGE: new_vid to vid2,


    const size_t gfid = m_vertex_connectivity[new_vid].m_conn_tris[0];
    int j = m_tri_connectivity[gfid].find(new_vid);
    auto new_t = Tuple(new_vid, (j + 2) % 3, gfid, *this);
    int j_ret = m_tri_connectivity[new_fid].find(new_vid);
    return_t = Tuple(new_vid, (j_ret + 2) % 3, new_fid, *this);
    assert(new_t.is_valid(*this));
    new_tris = get_one_ring_tris_for_vertex(new_t);
}


bool TriMesh::swap_edge(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!swap_edge_before(t)) {
        return false;
    }

    // get the vids
    size_t vid1 = t.vid(*this);
    size_t vid2 = t.switch_vertex(*this).vid(*this);

    const auto t_opps = t.switch_faces(*this);
    if (t_opps.size() != 1) {
        // should be already checked in swap_edge_before
        return false; // can't sawp on boundary or non-manifold edge
    }

    Tuple tmp_tuple;
    tmp_tuple = t_opps[0];
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
    size_t test_fid2 = t_opps[0].fid(*this);
    // record the fids that will be changed for roll backs on failure
    std::vector<std::pair<size_t, TriangleConnectivity>> old_tris(2);
    old_tris[0] = std::make_pair(test_fid1, m_tri_connectivity[test_fid1]);
    old_tris[1] = std::make_pair(test_fid2, m_tri_connectivity[test_fid2]);

    // first work on triangles, there are only 2
    int j = m_tri_connectivity[test_fid1].find(vid2);
    m_tri_connectivity[test_fid1].m_indices[j] = vid3;
    m_tri_connectivity[test_fid1].hash++;

    j = m_tri_connectivity[test_fid2].find(vid1);
    m_tri_connectivity[test_fid2].m_indices[j] = vid4;
    m_tri_connectivity[test_fid2].hash++;

    // then work on the vertices
    vector_erase(m_vertex_connectivity[vid1].m_conn_tris, test_fid2);
    vector_erase(m_vertex_connectivity[vid2].m_conn_tris, test_fid1);
    m_vertex_connectivity[vid3].m_conn_tris.push_back(test_fid1);
    vector_unique(m_vertex_connectivity[vid3].m_conn_tris);
    m_vertex_connectivity[vid4].m_conn_tris.push_back(test_fid2);
    vector_unique(m_vertex_connectivity[vid4].m_conn_tris);
    // change the tuple to the new edge tuple
    auto new_t = tuple_from_edge(vid4, vid3, test_fid2);

    assert(new_t.switch_vertex(*this).vid(*this) != vid1);
    assert(new_t.switch_vertex(*this).vid(*this) != vid2);
    assert(new_t.is_valid(*this));
    new_tris = {new_t, new_t.switch_faces(*this)[0]};
    start_protect_attributes();
    if (!swap_edge_after(new_t) || !invariants(new_tris)) {
        // restore the vertex and faces
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        for (const auto& old_tri : old_tris) {
            m_tri_connectivity[old_tri.first] = old_tri.second;
        }
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

bool TriMesh::split_face(const Tuple& t, std::vector<Tuple>& new_tris)
{
    if (!split_face_before(t)) {
        return false;
    }
    if (!t.is_valid(*this)) {
        return false;
    }

    // get local eid for return tuple construction
    const size_t local_eid = t.local_eid(*this);

    /**
     *
     *
     *         v2
     *         /|\
     *        / | \
     *       /  |  \
     *      /f1 ^ f0\
     *     /  /   \  \
     *    //   f2    \\
     *  v0 ----------- v1
     *
     */

    const size_t fid = t.fid(*this);

    std::array<size_t, 3> vid;
    vid[0] = t.vid(*this);
    vid[1] = t.switch_vertex(*this).vid(*this);
    vid[2] = t.switch_edge(*this).switch_vertex(*this).vid(*this);

    // record the vids that will be modified for roll backs on failure
    std::array<std::pair<size_t, VertexConnectivity>, 3> old_vertices;
    old_vertices[0] = {vid[0], m_vertex_connectivity[vid[0]]};
    old_vertices[1] = {vid[1], m_vertex_connectivity[vid[1]]};
    old_vertices[2] = {vid[2], m_vertex_connectivity[vid[2]]};
    std::pair<size_t, TriangleConnectivity> old_tri;
    old_tri = std::make_pair(fid, m_tri_connectivity[fid]);

    const auto conn_tris = [this, &vid](size_t i) -> std::vector<size_t>& {
        return m_vertex_connectivity[vid[i]].m_conn_tris;
    };

    // update vertex connectivity
    const size_t new_vid = get_next_empty_slot_v();
    const size_t new_fid1 = get_next_empty_slot_t();
    const size_t new_fid2 = get_next_empty_slot_t();

    vector_erase(conn_tris(0), fid);
    conn_tris(0).emplace_back(new_fid1);
    conn_tris(0).emplace_back(new_fid2);
    std::sort(conn_tris(0).begin(), conn_tris(0).end());
    conn_tris(1).emplace_back(new_fid2);
    std::sort(conn_tris(1).begin(), conn_tris(1).end());
    conn_tris(2).emplace_back(new_fid1);
    std::sort(conn_tris(2).begin(), conn_tris(2).end());

    m_vertex_connectivity[new_vid].m_conn_tris.reserve(3);
    m_vertex_connectivity[new_vid].m_conn_tris.emplace_back(fid);
    m_vertex_connectivity[new_vid].m_conn_tris.emplace_back(new_fid1);
    m_vertex_connectivity[new_vid].m_conn_tris.emplace_back(new_fid2);
    std::sort(
        m_vertex_connectivity[new_vid].m_conn_tris.begin(),
        m_vertex_connectivity[new_vid].m_conn_tris.end());

    // now the triangles
    // need to update the hash of fid
    const size_t i = m_tri_connectivity[fid].find(vid[0]);
    const size_t j = m_tri_connectivity[fid].find(vid[1]);
    const size_t k = m_tri_connectivity[fid].find(vid[2]);
    m_tri_connectivity[fid].m_indices[i] = new_vid;
    m_tri_connectivity[fid].hash++;
    // new_fid1/2 m_indices in same order
    m_tri_connectivity[new_fid1].m_indices[i] = vid[0];
    m_tri_connectivity[new_fid1].m_indices[j] = new_vid;
    m_tri_connectivity[new_fid1].m_indices[k] = vid[2];
    m_tri_connectivity[new_fid2].m_indices[i] = vid[0];
    m_tri_connectivity[new_fid2].m_indices[j] = vid[1];
    m_tri_connectivity[new_fid2].m_indices[k] = new_vid;

    // make the new tuple
    Tuple new_vertex_tuple(new_vid, (k + 2) % 3, new_fid2, *this);
    Tuple return_tuple(vid[0], local_eid, new_fid2, *this);
    assert(return_tuple.is_valid(*this));

    new_tris = get_one_ring_tris_for_vertex(new_vertex_tuple);
    start_protect_attributes();

    // roll back if not successful
    if (!split_face_after(return_tuple) || !invariants(new_tris)) {
        // rollback topo
        // restore old v, t
        for (const auto& old_v : old_vertices) {
            m_vertex_connectivity[old_v.first] = old_v.second;
        }
        m_tri_connectivity[old_tri.first] = old_tri.second;

        // erase new_vid new_fids
        m_vertex_connectivity[new_vid].m_conn_tris.clear();
        m_vertex_connectivity[new_vid].m_is_removed = true;
        m_tri_connectivity[new_fid1].m_is_removed = true;
        m_tri_connectivity[new_fid2].m_is_removed = true;
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

std::vector<wmtk::TriMesh::Tuple> TriMesh::get_one_ring_vertices_for_vertex(
    const wmtk::TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> one_ring;
    size_t vid = t.vid(*this);
    auto& conn_tri = m_vertex_connectivity[vid].m_conn_tris;

    std::unordered_map<size_t, bool> visited;
    visited[vid] = true; // prevent pushing t in to one ring

    one_ring.reserve(conn_tri.size() * 4);
    for (size_t tri : conn_tri) {
        for (auto j : m_tri_connectivity[tri].m_indices) {
            if (visited.find(j) != visited.end()) {
                // skip if already pushed
                continue;
            } else {
                visited[j] = true;
                one_ring.push_back(tuple_from_vertex(j));
            }
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
    for (Tuple tri : one_ring_tris) {
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

    /**
     * The code below is a faster implementation but it did not give the exact same
     * result for QSLIM. Leaving it commented out for now.
     */

    // const size_t vid = t.vid(*this);
    //
    // std::map<size_t, Tuple> vid_tup;
    // for (const size_t fid : m_vertex_connectivity[vid].m_conn_tris) {
    //     const auto& vids = m_tri_connectivity[fid].m_indices;
    //     const int loc_v0 = m_tri_connectivity[fid].find(vid);
    //     assert(loc_v0 >= 0);
    //     const int loc_v1 = (loc_v0 + 1) % 3;
    //     const size_t v1 = vids[loc_v1];
    //     if (vid_tup.count(v1) == 0) {
    //         const int e1 = 3 - (loc_v0 + loc_v1);
    //         vid_tup[v1] = Tuple(v1, e1, fid, *this);
    //     }
    //
    //     const int loc_v2 = (loc_v0 + 2) % 3;
    //     const size_t v2 = vids[loc_v2];
    //     if (vid_tup.count(v2) == 0) {
    //         const int e2 = 3 - (loc_v0 + loc_v1);
    //         vid_tup[v2] = Tuple(v2, e2, fid, *this);
    //     }
    // }
    //
    // std::vector<Tuple> one_ring_edges;
    // one_ring_edges.reserve(vid_tup.size());
    // for (const auto& [v, edge_tuple] : vid_tup) {
    //     one_ring_edges.emplace_back(edge_tuple);
    // }

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

TriMesh::Tuple TriMesh::tuple_from_edge(size_t vid1, size_t vid2, size_t fid) const
{
    auto a = m_tri_connectivity[fid].find(vid1);
    auto b = m_tri_connectivity[fid].find(vid2);
    assert(a != -1 && b != -1);
    // 0,1 - >2, 1,2-> 0, 0,2->1
    return Tuple(vid1, 3 - (a + b), fid, *this);
}

TriMesh::Tuple wmtk::TriMesh::tuple_from_vids(size_t vid0, size_t vid1, size_t vid2) const
{
    const auto& vf0 = m_vertex_connectivity[vid0];
    const auto& vf1 = m_vertex_connectivity[vid1];
    const auto& vf2 = m_vertex_connectivity[vid2];

    const std::vector<size_t> tris01 = set_intersection(vf0.m_conn_tris, vf1.m_conn_tris);
    const std::vector<size_t> tris012 = set_intersection(tris01, vf2.m_conn_tris);

    if (tris012.size() != 1) {
        log_and_throw_error("Cannot find face with vids ({},{},{})", vid0, vid1, vid2);
    }

    const size_t fid = tris012[0];

    const auto& tc = m_tri_connectivity[fid].m_indices;
    size_t local_vid = -1;
    for (int i = 0; i < 3; ++i) {
        if (tc[i] == vid0) {
            local_vid = i;
            break;
        }
    }
    assert(local_vid != -1);

    const size_t local_vid_next = (local_vid + 1) % 3;
    const size_t local_vid_prev = (local_vid + 2) % 3;
    if (tc[local_vid_next] == vid1) {
        return Tuple(vid0, local_vid_prev, fid, *this);
    } else {
        assert(tc[local_vid_prev] == vid1);
        return Tuple(vid0, local_vid_next, fid, *this);
    }

    Tuple t;


    return Tuple();
}

simplex::Vertex wmtk::TriMesh::simplex_from_vertex(const Tuple& t) const
{
    return simplex::Vertex(t.vid(*this));
}

simplex::Edge wmtk::TriMesh::simplex_from_edge(const Tuple& t) const
{
    return simplex::Edge(t.vid(*this), t.switch_vertex(*this).vid(*this));
}

simplex::Face wmtk::TriMesh::simplex_from_face(const Tuple& t) const
{
    const auto vs = oriented_tri_vids(t.fid(*this));
    return simplex::Face(vs[0], vs[1], vs[2]);
}

TriMesh::Tuple wmtk::TriMesh::tuple_from_simplex(const simplex::Face& s) const
{
    const auto& v = s.vertices();
    return tuple_from_vids(v[0], v[1], v[2]);
}

simplex::RawSimplexCollection wmtk::TriMesh::simplex_incident_triangles(
    const simplex::Vertex& v) const
{
    const auto fids = m_vertex_connectivity[v.vertices()[0]].m_conn_tris;
    simplex::RawSimplexCollection sc;

    for (const size_t fid : fids) {
        const auto vids = oriented_tri_vids(fid);
        sc.add(simplex::Face(vids[0], vids[1], vids[2]));
    }
    sc.sort_and_clean();
    return sc;
}

simplex::RawSimplexCollection wmtk::TriMesh::simplex_incident_triangles(
    const simplex::Edge& e) const
{
    const simplex::Vertex loc_v0(e.vertices()[0]);
    const simplex::Vertex v1(e.vertices()[1]);

    const auto sc0 = simplex_incident_triangles(loc_v0);
    const auto sc1 = simplex_incident_triangles(v1);

    return simplex::RawSimplexCollection::get_intersection(sc0, sc1);
}

simplex::RawSimplexCollection wmtk::TriMesh::simplex_link_vertices(const simplex::Vertex& v) const
{
    const auto tris = simplex_incident_triangles(v);
    simplex::RawSimplexCollection sc;
    sc.reserve_vertices(tris.faces().size() * 2);
    for (const simplex::Face& f : tris.faces()) {
        for (const size_t vid : f.vertices()) {
            if (vid != v.vertices()[0]) {
                sc.add(simplex::Vertex(vid));
            }
        }
    }
    sc.sort_and_clean();

    return sc;
}

simplex::RawSimplexCollection wmtk::TriMesh::simplex_link_vertices(const simplex::Edge& e) const
{
    const auto tris = simplex_incident_triangles(e);
    simplex::RawSimplexCollection sc;
    sc.reserve_vertices(tris.faces().size());
    for (const simplex::Face& f : tris.faces()) {
        sc.add(f.opposite_vertex(e));
    }
    sc.sort_and_clean();

    return sc;
}

simplex::RawSimplexCollection wmtk::TriMesh::simplex_link_edges(const simplex::Vertex& v) const
{
    const auto tris = simplex_incident_triangles(v);
    simplex::RawSimplexCollection sc;
    sc.reserve_edges(tris.faces().size());
    for (const simplex::Face& f : tris.faces()) {
        sc.add(f.opposite_edge(v));
    }
    sc.sort_and_clean();

    return sc;
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
    const auto opps = t.switch_faces(*this);
    if (opps.size() != 1) {
        return false;
    }
    // if (!t.switch_face(*this).has_value()) return false;
    // size_t v4 =
    // ((t.switch_face(*this).value()).switch_edge(*this)).switch_vertex(*this).vid(*this);
    const size_t v3 = ((t.switch_edge(*this)).switch_vertex(*this)).vid(*this);
    const size_t v4 = opps[0].switch_edge(*this).switch_vertex(*this).vid(*this);
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
    const auto vid1_ring = get_one_ring_edges_for_vertex(edge);
    const auto vid2_ring = get_one_ring_edges_for_vertex(switch_vertex(edge));


    size_t dummy = std::numeric_limits<size_t>::max();

    std::vector<size_t> lk_vid1; // link vertices of vid1
    std::vector<size_t> lk_vid2; // link vertices of vid2

    std::vector<std::pair<size_t, size_t>> lk_e_vid1; // link edges of vid1
    std::vector<std::pair<size_t, size_t>> lk_e_vid2; // link edges of vid2

    for (const Tuple& e_vid : vid1_ring) {
        if (e_vid.switch_faces(*this).empty()) {
            lk_vid1.push_back(dummy);
            lk_e_vid1.emplace_back(e_vid.vid(*this), dummy);
        }
        lk_vid1.push_back(e_vid.vid(*this));
    }
    // collect link edges
    std::vector<Tuple> vid1_tris = get_one_ring_tris_for_vertex(edge);
    for (const Tuple& v1_tri_t : vid1_tris) {
        auto indices = m_tri_connectivity[v1_tri_t.fid(*this)].m_indices;
        auto l = m_tri_connectivity[v1_tri_t.fid(*this)].find(vid1);
        assert(l != -1);
        auto i0 = indices[(l + 1) % 3], i1 = indices[(l + 2) % 3];
        lk_e_vid1.emplace_back(std::min(i0, i1), std::max(i0, i1));
    }
    vector_unique(lk_vid1);

    for (const Tuple& e_vid : vid2_ring) {
        if (e_vid.switch_faces(*this).empty()) {
            lk_vid2.push_back(dummy);
            lk_e_vid2.emplace_back(e_vid.vid(*this), dummy);
        }
        lk_vid2.push_back(e_vid.vid(*this));
    }
    // collect link edges
    std::vector<Tuple> vid2_tris = get_one_ring_tris_for_vertex(switch_vertex(edge));
    for (const Tuple& v2_tri_t : vid2_tris) {
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
    const auto edge_opps = edge.switch_faces(*this);
    if (edge_opps.empty()) {
        // edge is boundary
        lk_edge.push_back(dummy);
    } else {
        for (const Tuple& opp : edge_opps) {
            lk_edge.push_back(opp.switch_edge(*this).switch_vertex(*this).vid(*this));
        }
    }
    vector_sort(lk_edge);
    bool v_link =
        (lk_vid12.size() == lk_edge.size() &&
         std::equal(lk_vid12.begin(), lk_vid12.end(), lk_edge.begin()));

    // check edge link condition
    // in 2d edge link for an edge is always empty

    if (!v_link) {
        return false;
    }

    // bool e_link = true;
    std::vector<std::pair<size_t, size_t>> res;
    std::sort(lk_e_vid1.begin(), lk_e_vid1.end());
    std::sort(lk_e_vid2.begin(), lk_e_vid2.end());
    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));
    if (res.size() > 0) {
        return false;
    }

    return true;
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
    for (const Tuple& v_one_ring : get_one_ring_edges_for_vertex(v)) {
        if (m_vertex_mutex[v_one_ring.vid(*this)].get_owner() == threadid) {
            continue;
        }
        if (try_set_vertex_mutex(v_one_ring, threadid)) {
            mutex_release_stack.local().push_back(v_one_ring.vid(*this));
            for (const Tuple& v_two_ring : get_one_ring_edges_for_vertex(v_one_ring)) {
                if (m_vertex_mutex[v_two_ring.vid(*this)].get_owner() == threadid) {
                    continue;
                }
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

bool TriMesh::try_set_face_mutex_one_ring(const Tuple& f, int threadid)
{
    const auto verts = oriented_tri_vertices(f);
    for (const Tuple& v : verts) {
        if (m_vertex_mutex[v.vid(*this)].get_owner() != threadid) {
            release_vertex_mutex_in_stack();
            return false;
        }
        if (try_set_vertex_mutex(v, threadid)) {
            mutex_release_stack.local().push_back(v.vid(*this));
        } else {
            release_vertex_mutex_in_stack();
            return false;
        }
        if (!v.is_valid(*this)) {
            release_vertex_mutex_in_stack();
            return false;
        }
    }

    for (const Tuple& v : verts) {
        if (!try_set_vertex_mutex_one_ring(v, threadid)) {
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