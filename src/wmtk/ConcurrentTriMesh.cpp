#include <wmtk/ConcurrentTriMesh.h>

#include <wmtk/utils/TupleUtils.hpp>

using namespace wmtk;

// bool ConcurrentTriMesh::collapse_edge(const Tuple& loc0, Tuple& new_t)
// {
//     if (!collapse_before(loc0)) return false;
//     // get the vids
//     size_t vid1 = loc0.get_vid();
//     size_t vid2 = switch_vertex(loc0).get_vid();

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
//     size_t test_fid1 = loc0.get_fid();
//     TriMesh::Tuple loc1 = switch_face(loc0).value_or(loc0);
//     size_t test_fid2 = loc1.get_fid();
//     assert(
//         ("faces at the edge is not correct",
//          (vector_contains(n12_intersect_fids, test_fid1) &&
//           vector_contains(n12_intersect_fids, test_fid2))));
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
//     add_new_vertex_vertex();
//     for (size_t fid : n1_fids) {
//         if (m_tri_connectivity[fid].m_is_removed)
//             continue;
//         else {
//             int j = m_tri_connectivity[fid].find(vid1);
//             m_tri_connectivity[fid].m_indices[j] = new_vid;
//         }
//     }
//     for (size_t fid : n2_fids) {
//         if (vector_contains(n12_intersect_fids, fid))
//             continue;
//         else {
//             int j = m_tri_connectivity[fid].find(vid2);
//             m_tri_connectivity[fid].m_indices[j] = new_vid;
//         }
//     }

//     // now work on vids
//     // add in the new vertex
//     if (new_vid < m_vertex_connectivity.size() - 1) {
//         assert(m_vertex_connectivity[new_vid].m_is_removed);
//         m_vertex_connectivity[new_vid].m_conn_tris.clear();
//     }

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

//     size_t fid = m_vertex_connectivity[new_vid].m_conn_tris[0];
//     int j = m_tri_connectivity[fid].find(new_vid);
//     new_t = Tuple(new_vid, (j + 2) % 3, fid, *this);
//     if (!collapse_after(new_t)) {
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

//         // by the end the new_t and old t both exist and both valid
//         return false;
//     }
//     return true;
// }