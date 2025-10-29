#include "c1_multimesh.hpp"

#include "c1_utils.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <igl/predicates/predicates.h>
#include <igl/write_triangle_mesh.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/GeoUtils.h>
// clang-format on

#include <algorithm>
#include <iterator>
#include <limits>
#include <set>

namespace wmtk::components::c1_simplification {

/////////////////////////////////////
///////// TetMesh functions /////////
/////////////////////////////////////

void MMTetMesh::init_from_eigen(const MatrixXd& V, const MatrixXi& T)
{
    // initialize connectivity
    std::cout << 0 << std::endl;
    init(T);
    std::cout << 1 << std::endl;

    // initialize attributes
    v_attrs.resize(V.rows());

    for (size_t i = 0; i < vert_capacity(); ++i) {
        v_attrs[i].pos = V.row(i);
    }
}

bool MMTetMesh::is_inverted(const Tuple& t) const
{
    auto vs = oriented_tet_vertices(t);
    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(
        v_attrs[vs[0].vid(*this)].pos,
        v_attrs[vs[1].vid(*this)].pos,
        v_attrs[vs[2].vid(*this)].pos,
        v_attrs[vs[3].vid(*this)].pos);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result < 0) // TODO::check this sign
        return false;
    return true;
}

bool MMTetMesh::collapse_edge_before(const Tuple& t)
{
    // TODO: test use, remove this
    return true;

    auto& VA = v_attrs;
    auto& cache = t_cache;

    cache.changed_tids.clear();

    size_t v1_id = t.vid(*this);
    auto loc1 = switch_vertex(t);
    size_t v2_id = loc1.vid(*this);

    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    auto n1_locs = get_one_ring_tets_for_vertex(t);
    auto n12_locs = get_incident_tets_for_edge(t); // todo: duplicated computation

    std::set<size_t> n1_loc_tids, n12_loc_tids;

    for (auto& l : n1_locs) {
        n1_loc_tids.insert(l.tid(*this));
    }
    for (auto& l : n12_locs) {
        n12_loc_tids.insert(l.tid(*this));
    }

    std::set<size_t> changed_tids;

    std::set_difference(
        n1_loc_tids.begin(),
        n1_loc_tids.end(),
        n12_loc_tids.begin(),
        n12_loc_tids.end(),
        std::inserter(changed_tids, changed_tids.begin()));

    std::vector<size_t> changed_tids_vec(changed_tids.begin(), changed_tids.end());

    cache.changed_tids = changed_tids_vec;

    return true;
}

bool MMTetMesh::collapse_edge_after(const Tuple& t)
{
    // TODO: test use, remove this
    return true;

    auto& VA = v_attrs;
    auto& cache = t_cache;
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    if (!TetMesh::collapse_edge_after(t)) {
        return false;
    }

    for (size_t tid : cache.changed_tids) {
        auto tet = tuple_from_tet(tid);

        if (is_inverted(tet)) {
            return false;
        }
    }

    return true;
}

////////////////////////////////////
///////// UVMesh functions /////////
////////////////////////////////////

void MMUVMesh::init_from_eigen(const MatrixXd& V, const MatrixXi& F)
{
    // initialize connectivity
    init(F);

    // initialize attributes
    v_attrs.resize(V.rows());

    for (size_t i = 0; i < vert_capacity(); ++i) {
        v_attrs[i].pos = V.row(i);
    }
}

bool MMUVMesh::is_inverted(const Tuple& t) const
{
    auto vs = oriented_tri_vertices(t);
    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(
        v_attrs[vs[0].vid(*this)].pos,
        v_attrs[vs[1].vid(*this)].pos,
        v_attrs[vs[2].vid(*this)].pos);
    int result;
    if (res == igl::predicates::Orientation::POSITIVE)
        result = 1;
    else if (res == igl::predicates::Orientation::NEGATIVE)
        result = -1;
    else
        result = 0;

    if (result > 0) // TODO::check this sign
        return false;
    return true;
}

bool MMUVMesh::collapse_edge_before(const Tuple& t)
{
    // TODO: test use, remove this
    return true;

    if (is_boundary_vertex(t)) {
        // boundary vertex cannot be collapsed
        // TODO: change to weaker
        return false;
    }

    // TODO: if seam connect together, only collapse to the shared vertex is allowed

    // cache vertex uv pos
    uv_cache.old_uv_pos = v_attrs[t.vid(*this)].pos;

    return true;
}

bool MMUVMesh::collapse_edge_after(const Tuple& t)
{
    // TODO: test use, remove this
    return true;

    for (const auto& f : get_one_ring_tris_for_vertex(t)) {
        if (is_inverted(f)) {
            return false;
        }
    }

    return true;
}


/////////////////////////////////////////
///////// SurfaceMesh functions /////////
/////////////////////////////////////////

void MMSurfaceMesh::init_from_eigen_with_map_and_dofs(
    const MatrixXd& V,
    const MatrixXi& F,
    const std::map<int64_t, int64_t>& s2t_vid_map,
    const std::vector<Vector3d>& vgrads,
    const std::vector<std::array<Vector3d, 3>>& egrads)
{
    // initialize connectivity
    init(F);

    // initialize attributes
    v_attrs.resize(V.rows());
    e_attrs.resize(tri_capacity() * 3);

    for (size_t i = 0; i < vert_capacity(); ++i) {
        v_attrs[i].pos = V.row(i); // assign pos
        v_attrs[i].grad = vgrads[i]; // assign grad
        v_attrs[i].vid_in_tet = s2t_vid_map.at(i); // assign mapping to tet vertices
    }

    // assign local edge midgrad to global edge attributes
    for (size_t i = 0; i < tri_capacity(); ++i) {
        const std::array<Tuple, 3> es = {
            {tuple_from_edge(1, 2, i),
             tuple_from_edge(2, 0, i),
             tuple_from_edge(0, 1, i)}}; // using same local order in tuple

        for (int k = 0; k < 3; ++k) {
            size_t eid = es[k].eid(*this);
            e_attrs[eid] = egrads[i][k];
        }
    }
}

TetMesh::Tuple MMSurfaceMesh::map_to_tet_edge_tuple(const Tuple& e)
{
    // get two vertex ids and map to vid in tet
    size_t v0 = e.vid(*this);
    size_t v1 = e.switch_vertex(*this).vid(*this);

    std::cout << v0 << " " << v1 << std::endl;

    size_t tv0 = v_attrs[v0].vid_in_tet;
    size_t tv1 = v_attrs[v1].vid_in_tet;

    TetMesh::Tuple te = tetmesh_ptr->tuple_from_edge(
        {{tv0, tv1}}); // this only returns the same edge, not guarantee the same vertex
    if (te.vid(*tetmesh_ptr) != tv0) {
        te = te.switch_vertex(*tetmesh_ptr);
    }

    return te;
}

TetMesh::Tuple MMSurfaceMesh::map_to_tet_vertex_tuple(const Tuple& v)
{
    TetMesh::Tuple tv = tetmesh_ptr->tuple_from_vertex(v_attrs[v.vid(*this)].vid_in_tet);
    return tv;
}

std::vector<TriMesh::Tuple> MMSurfaceMesh::map_to_uv_edge_tuples(const Tuple& e)
{
    std::vector<Tuple> uv_edge_tuples;
    // get first uv edge
    Tuple v0 = e;
    Tuple v1 = e.switch_vertex(*this);

    auto vid0 = v0.vid(*this);
    auto vid1 = v1.vid(*this);

    auto tri_vids = oriented_tri_vids(e);

    int lid0 = -1;
    int lid1 = -1;

    for (int k = 0; k < 3; ++k) {
        if (tri_vids[k] == vid0) {
            lid0 = k;
        }
        if (tri_vids[k] == vid1) {
            lid1 = k;
        }
    }

    assert(lid0 != -1);
    assert(lid1 != -1);

    auto uv_vids = uvmesh_ptr->oriented_tri_vids(e);
    auto uv_vid0 = uv_vids[lid0];
    auto e0_lid = e.local_eid(*this);

    Tuple uv_e0_tuple(uv_vid0, e0_lid, e.fid(*this), *uvmesh_ptr);

    // push first uv edge into result vector
    uv_edge_tuples.push_back(uv_e0_tuple);

    // skip if is boundary on surface or not seam in uv
    if (is_boundary_edge(e) || !uvmesh_ptr->is_boundary_edge(uv_e0_tuple)) {
        return uv_edge_tuples;
    }

    // compute another edge if not boundary and in uv it is a boundary (seam)
    Tuple e1 = e.switch_face(*this).value();
    auto e1_lid = e1.local_eid(*this);

    auto tri_vids_other = oriented_tri_vids(e1);
    auto uv_vids_other = uvmesh_ptr->oriented_tri_vids(e1);

    int lid0_other = -1;
    int lid1_other = -1;

    for (int k = 0; k < 3; ++k) {
        if (tri_vids_other[k] == vid0) {
            lid0_other = k;
        }
        if (tri_vids_other[k] == vid1) {
            lid1_other = k;
        }
    }

    assert(lid0_other != -1);
    assert(lid0_other != -1);

    auto uv_vid0_other = uv_vids_other[lid0_other];

    Tuple uv_e1_tuple(uv_vid0_other, e1_lid, e1.fid(*this), *uvmesh_ptr);
    uv_edge_tuples.push_back(uv_e1_tuple);

    return uv_edge_tuples;
}


std::vector<TriMesh::Tuple> MMSurfaceMesh::map_to_uv_vertex_tuples(const Tuple& v)
{
    std::cout << "a" << std::endl;

    auto face_one_ring = get_one_ring_tris_for_vertex(v);
    std::vector<Tuple> uv_face_one_ring;

    std::cout << "b" << std::endl;

    size_t vid = v.vid(*this);
    std::cout << "c" << std::endl;


    int cnt = 0;
    for (const auto& f : face_one_ring) {
        size_t fid = f.fid(*this);
        size_t leid = f.local_eid(*this);
        std::cout << cnt << " fid: " << fid << " leid: " << leid << std::endl;

        auto tri_vids = oriented_tri_vids(fid);
        std::cout << "pass ori: " << tri_vids[0] << " " << tri_vids[1] << " " << tri_vids[2]
                  << std::endl;

        int lvid = -1;
        for (int k = 0; k < 3; ++k) {
            if (tri_vids[k] == vid) {
                lvid = k;
            }
        }

        std::cout << "lvid: " << lvid << std::endl;

        assert(lvid != -1);

        auto uv_tri_vids = uvmesh_ptr->oriented_tri_vids(fid);
        std::cout << "pass ori uv: " << uv_tri_vids[0] << " " << uv_tri_vids[1] << " "
                  << uv_tri_vids[2] << std::endl;

        auto uv_vid = uv_tri_vids[lvid];
        std::cout << "uv_vid: " << uv_vid << std::endl;


        Tuple uv_v_tuple = Tuple(uv_vid, leid, fid, *uvmesh_ptr);
        uv_face_one_ring.push_back(uv_v_tuple);
        cnt++;
    }
    std::cout << "d" << std::endl;


    // wmtk::unique_vertex_tuples(*uvmesh_ptr, uv_face_one_ring);

    return uv_face_one_ring;
}

Eigen::Matrix<double, 12, 3> MMSurfaceMesh::assemble_dofs(const size_t& fid)
{
    // TODO::implement this
    return Eigen::Matrix<double, 12, 3>();
};

bool MMSurfaceMesh::collapse_edge_before(const Tuple& t)
{
    // TODO: test use, remove this
    return true;

    // // uv before
    // auto uv_e_tuples = map_to_uv_edge_tuples(t);
    // for (const auto& uv_e : uv_e_tuples) {
    //     if (!uvmesh_ptr->collapse_edge_before(uv_e)) {
    //         return false;
    //     }
    // }

    // // tet before
    // auto t_e_tuple = map_to_tet_edge_tuple(t);
    // if (!tetmesh_ptr->collapse_edge_before(t_e_tuple)) {
    //     return false;
    // }

    // surface_before
    s_cache.old_pos = v_attrs[t.vid(*this)].pos;

    return true;
}

bool MMSurfaceMesh::collapse_edge_after(const Tuple& t)
{
    // TODO: test use, remove this
    return true;

    if (!TriMesh::collapse_edge_after(t)) {
        return false;
    }

    // // uv after
    auto uv_v_tuples = map_to_uv_vertex_tuples(t);
    // for (const auto& uv_v : uv_v_tuples) {
    //     if (!uvmesh_ptr->collapse_edge_after(uv_v)) {
    //         return false;
    //     }
    // }

    // // tet after
    // auto t_v_tuple = map_to_tet_vertex_tuple(t);
    // if (!tetmesh_ptr->collapse_edge_after(t_v_tuple)) {
    //     return false;
    // }

    // find old vertex in which new triangle
    Vector2d old_uv_pos = uvmesh_ptr->uv_cache.old_uv_pos;

    size_t fid_involved = -1;
    Vector2d a, b, c;
    bool found_flag = false;
    for (const auto& uv_v : uv_v_tuples) {
        auto uv_tris_one_ring = uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v);

        for (const auto& uv_tri : uv_tris_one_ring) {
            auto uv_tri_vids = uvmesh_ptr->oriented_tri_vids(uv_tri);

            Vector2d p0, p1, p2;
            p0 = uvmesh_ptr->v_attrs[uv_tri_vids[0]].pos;
            p1 = uvmesh_ptr->v_attrs[uv_tri_vids[1]].pos;
            p2 = uvmesh_ptr->v_attrs[uv_tri_vids[2]].pos;

            if (point_in_tri(old_uv_pos, p0, p1, p2)) {
                fid_involved = uv_tri.fid(*uvmesh_ptr);
                a = p0;
                b = p1;
                c = p2;
                found_flag = true;
                break;
            }
        }

        if (found_flag) break;
    }

    assert(found_flag);

    // check deviation
    // compute bc in uv tri
    // TODO: carefully check the order of vertices for bary here
    auto bary_coord = barycentric_coord_in_tri(old_uv_pos, a, b, c);
    auto dofs = assemble_dofs(fid_involved);

    Eigen::Vector3d new_pos = CT_eval(bary_coord[0], bary_coord[1], dofs);

    if ((new_pos - s_cache.old_pos).norm() > deviation_threshold) {
        return false;
    }

    return true;
}

bool MMSurfaceMesh::multimesh_collapse_edge(const Tuple& t)
{
    // before checks
    // uv before
    auto uv_e_tuples = map_to_uv_edge_tuples(t);
    for (const auto& uv_e : uv_e_tuples) {
        if (!uvmesh_ptr->collapse_edge_before(uv_e)) {
            wmtk::logger().info("collapse rejected by uv before");
            return false;
        }
    }

    // surface before
    if (!collapse_edge_before(t)) {
        wmtk::logger().info("collapse rejected by surface before");

        return false;
    }

    // tet before
    auto t_e_tuple = map_to_tet_edge_tuple(t);
    if (!tetmesh_ptr->collapse_edge_before(t_e_tuple)) {
        wmtk::logger().info("collapse rejected by tet before");

        return false;
    }

    // connectivity change
    // first call tet one because it involves link condition check
    // the link condition check return false before actual connectivity change of the tetmesh so it
    // is safe

    // tet connectivity change
    // variables prefix with tet_
    std::vector<TetMesh::Tuple> tet_new_edges;
    size_t tet_v1_id;
    TetMesh::Tuple tet_new_loc;
    std::map<size_t, wmtk::TetMesh::VertexConnectivity> tet_rollback_vert_conn;
    std::vector<size_t> tet_n1_t_ids;
    std::vector<size_t> tet_new_tet_id;
    std::vector<MMTetMesh::TetrahedronConnectivity> tet_old_tets;

    if (!tetmesh_ptr->collapse_edge_conn(
            t_e_tuple,
            tet_new_edges,
            tet_v1_id,
            tet_new_loc,
            tet_rollback_vert_conn,
            tet_n1_t_ids,
            tet_new_tet_id,
            tet_old_tets)) {
        return false; // don't need roll back
    }

    // suface mesh connectivity change
    // variables prefix with s_
    std::vector<Tuple> s_new_tris;
    Tuple s_return_t;
    size_t s_new_vid;
    std::vector<std::pair<size_t, TriangleConnectivity>> s_old_tris;
    std::vector<std::pair<size_t, VertexConnectivity>> s_old_vertices;
    std::vector<std::pair<size_t, size_t>> s_same_edge_vid_fid;
    std::vector<size_t> s_n12_intersect_fids;

    collapse_edge_conn(
        t,
        s_new_tris,
        s_return_t,
        s_new_vid,
        s_old_tris,
        s_old_vertices,
        s_same_edge_vid_fid,
        s_n12_intersect_fids);

    // uv mesh connectivity change
    // variables prefix with uv_
    std::vector<std::vector<Tuple>> uv_new_tris(uv_e_tuples.size());
    std::vector<Tuple> uv_return_t(uv_e_tuples.size());
    std::vector<size_t> uv_new_vid(uv_e_tuples.size());
    std::vector<std::vector<std::pair<size_t, TriangleConnectivity>>> uv_old_tris(
        uv_e_tuples.size());
    std::vector<std::vector<std::pair<size_t, VertexConnectivity>>> uv_old_vertices(
        uv_e_tuples.size());
    std::vector<std::vector<std::pair<size_t, size_t>>> uv_same_edge_vid_fid(uv_e_tuples.size());
    std::vector<std::vector<size_t>> uv_n12_intersect_fids(uv_e_tuples.size());

    for (int i = 0; i < uv_e_tuples.size(); ++i) {
        uvmesh_ptr->collapse_edge_conn(
            uv_e_tuples[i],
            uv_new_tris[i],
            uv_return_t[i],
            uv_new_vid[i],
            uv_old_tris[i],
            uv_old_vertices[i],
            uv_same_edge_vid_fid[i],
            uv_n12_intersect_fids[i]);
    }

    // after and roll back
    this->start_protect_attributes();
    uvmesh_ptr->start_protect_attributes();
    tetmesh_ptr->start_protect_attributes();

    bool rollback_flag = false;
    // if any after check failed, rollback_flag is set to true
    // surface check
    rollback_flag =
        rollback_flag || !this->collapse_edge_after(s_return_t) || !this->invariants(s_new_tris);

    // uv check
    for (int i = 0; i < uv_e_tuples.size(); ++i) {
        if (rollback_flag) {
            // already set to true, break
            break;
        }
        rollback_flag = rollback_flag || !uvmesh_ptr->collapse_edge_after(uv_return_t[i]) ||
                        !uvmesh_ptr->invariants(uv_new_tris[i]);
    }

    // tet check
    if (!rollback_flag) {
        // check only if rollback_flag is false
        rollback_flag =
            rollback_flag || !tetmesh_ptr->collapse_edge_check_topology(tet_new_tet_id) ||
            !tetmesh_ptr->collapse_edge_after(tet_new_loc) ||
            !tetmesh_ptr->invariants(tetmesh_ptr->get_one_ring_tets_for_vertex(tet_new_loc));
    }

    // rollback
    if (rollback_flag) {
        // surface rollback
        this->collapse_edge_rollback(
            s_new_vid,
            s_old_tris,
            s_old_vertices,
            s_same_edge_vid_fid,
            s_n12_intersect_fids);

        // uv rollback
        // TODO: check inside one ring effect
        for (int i = 0; i < uv_e_tuples.size(); ++i) {
            uvmesh_ptr->collapse_edge_rollback(
                uv_new_vid[i],
                uv_old_tris[i],
                uv_old_vertices[i],
                uv_same_edge_vid_fid[i],
                uv_n12_intersect_fids[i]);
        }

        // tet rollback
        tetmesh_ptr->collapse_edge_rollback(
            tet_v1_id,
            tet_rollback_vert_conn,
            tet_n1_t_ids,
            tet_new_tet_id,
            tet_old_tets);

        return false;
    }


    this->release_protect_attributes();
    uvmesh_ptr->release_protect_attributes();
    tetmesh_ptr->release_protect_attributes();

    // post process of return values
    // tetmesh
    for (size_t t_id : tet_new_tet_id) {
        for (int j = 0; j < 6; j++) {
            tet_new_edges.push_back(tetmesh_ptr->tuple_from_edge(t_id, j));
        }
    }
    // unique_edge_tuples(*tetmesh_ptr, tet_new_edges);

    return true;
}
} // namespace wmtk::components::c1_simplification