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
#include <cmath>
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
    // return true;

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
    // return true;

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
    // return true;

    // link condition
    if (!TriMesh::collapse_edge_before(t)) {
        wmtk::logger().debug("rejected by uvmesh mesh link condition");
        return false;
    }

    auto v1 = t;
    auto v2 = t.switch_vertex(*this);

    if (v_attrs[v1.vid(*this)].is_cone) {
        return false;
    }

    if (is_boundary_vertex(v1) && !is_boundary_vertex(v2)) {
        return false;
    }

    // if (is_boundary_vertex(t) || is_boundary_vertex(t.switch_vertex(*this))) {
    //     // boundary vertex cannot be collapsed
    //     // TODO: change to weaker
    //     return false;
    // }


    // TODO: if seam connect together, only collapse to the shared vertex is allowed

    // TODO: check result vertex valance = valance1 + valance2 - 2 < threshold


    // TODO: isolate updates for different vertex
    // cache vertex uv pos
    uv_cache.old_uv_pos = v_attrs[t.vid(*this)].pos;
    uv_cache.v2_pos = v_attrs[t.switch_vertex(*this).vid(*this)].pos;
    return true;
}

bool MMUVMesh::collapse_edge_after(const Tuple& t)
{
    // TODO: test use, remove this
    // return true;
    // v_attrs[t.vid(*this)].pos = uv_cache.v2_pos;

    // TODO: update position in surface after, only check invertion and quality here. call this
    // after surface after

    for (const auto& f : get_one_ring_tris_for_vertex(t)) {
        if (is_inverted(f)) {
            wmtk::logger().debug("collapse rejected by uv invertion");
            return false;
        }

        // check circumradius to inradius
        auto vs = oriented_tri_vertices(f);
        double a = (v_attrs[vs[0].vid(*this)].pos - v_attrs[vs[1].vid(*this)].pos).norm();
        double b = (v_attrs[vs[1].vid(*this)].pos - v_attrs[vs[2].vid(*this)].pos).norm();
        double c = (v_attrs[vs[2].vid(*this)].pos - v_attrs[vs[0].vid(*this)].pos).norm();
        double s = (a + b + c) / 2.0;

        double ratio = (8 * (s - a) * (s - b) * (s - c)) / (a * b * c);
        if (ratio < 0.1) {
            wmtk::logger().debug("collapse rejected by small aspect ratio {}", ratio);
            return false;
        }
    }

    return true;
}

void MMUVMesh::output_uv_mesh(const std::string& file, bool consolidate)
{
    if (consolidate) {
        consolidate_mesh();
    }

    const auto& vertices = get_vertices();
    const auto& tris = get_faces();

    std::ofstream output(file);

    size_t prev_id = -1;

    for (size_t i = 0; i < vertices.size(); ++i) {
        if (vertices[i].vid(*this) > prev_id + 1) {
            for (size_t k = 0; k < vertices[i].vid(*this) - prev_id - 1; ++k) {
                output << "v 0 0 0" << std::endl;
            }
        }
        output << "v" << " " << v_attrs[vertices[i].vid(*this)].pos[0] << " "
               << v_attrs[vertices[i].vid(*this)].pos[1] << " " << 0 << std::endl;
        prev_id = vertices[i].vid(*this);
    }

    for (size_t i = 0; i < tris.size(); ++i) {
        auto vs = oriented_tri_vertices(tris[i]);
        output << "f" << " " << vs[0].vid(*this) + 1 << " " << vs[1].vid(*this) + 1 << " "
               << vs[2].vid(*this) + 1 << std::endl;
    }
}


/////////////////////////////////////////
///////// SurfaceMesh functions /////////
/////////////////////////////////////////

void MMSurfaceMesh::init_from_eigen_with_map_and_dofs(
    const MatrixXd& V,
    const MatrixXi& F,
    const std::map<int64_t, int64_t>& s2t_vid_map,
    const std::vector<Eigen::Matrix<double, 12, 3>>& dofs,
    const std::vector<size_t>& cone_vids)
// const std::vector<Vector3d>& vgrads,
// const std::vector<std::array<Vector3d, 3>>& egrads)
{
    // initialize connectivity
    init(F);

    // initialize attributes
    v_attrs.resize(V.rows());
    e_attrs.resize(tri_capacity() * 3);
    f_attrs.resize(tri_capacity());

    for (size_t i = 0; i < vert_capacity(); ++i) {
        v_attrs[i].pos = V.row(i); // assign pos
        // v_attrs[i].grad = vgrads[i]; // assign grad
        if (tetmesh_ptr != nullptr) {
            v_attrs[i].vid_in_tet = s2t_vid_map.at(i); // assign mapping to tet vertices
        }
    }

    // initialize tracked vertices
    auto vertices = get_vertices();
    for (size_t i = 0; i < vertices.size(); ++i) {
        auto uv_v_tuples = map_to_uv_vertex_tuples(vertices[i]);

        // use the first uv position
        assert(uv_v_tuples.size() > 0);
        auto uv_pos = uvmesh_ptr->v_attrs[uv_v_tuples[0].vid(*uvmesh_ptr)].pos;
        auto fid = uv_v_tuples[0].fid(*uvmesh_ptr);

        // assign tracked v to the tuple fid
        tracked_vertices.emplace_back(uv_pos, v_attrs[i].pos, i, fid);

        // tracked_vid_to_fid_map[i] = fid;
        if (tracked_fid_to_vids_map.find(fid) != tracked_fid_to_vids_map.end()) {
            tracked_fid_to_vids_map[fid].push_back(i);
        } else {
            tracked_fid_to_vids_map[fid] = {i};
        }
    }

    // // assign local edge midgrad to global edge attributes
    // for (size_t i = 0; i < tri_capacity(); ++i) {
    //     const std::array<Tuple, 3> es = {
    //         {tuple_from_edge(1, 2, i),
    //          tuple_from_edge(2, 0, i),
    //          tuple_from_edge(0, 1, i)}}; // using same local order in tuple

    //     for (int k = 0; k < 3; ++k) {
    //         size_t eid = es[k].eid(*this);
    //         e_attrs[eid] = egrads[i][k];
    //     }
    // }
    for (size_t i = 0; i < tri_capacity(); ++i) {
        f_attrs[i].dofs = dofs[i];
    }

    for (const auto& vid : cone_vids) {
        v_attrs[vid].is_cone = true;
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
    // std::cout << "a" << std::endl;

    auto face_one_ring = get_one_ring_tris_for_vertex(v);
    std::vector<Tuple> uv_face_one_ring;

    // std::cout << "b" << std::endl;

    size_t vid = v.vid(*this);
    // std::cout << "c" << std::endl;


    int cnt = 0;
    for (const auto& f : face_one_ring) {
        size_t fid = f.fid(*this);
        size_t leid = f.local_eid(*this);
        // std::cout << cnt << " fid: " << fid << " leid: " << leid << std::endl;

        auto tri_vids = oriented_tri_vids(fid);
        // std::cout << "pass ori: " << tri_vids[0] << " " << tri_vids[1] << " " << tri_vids[2]
        //           << std::endl;

        int lvid = -1;
        for (int k = 0; k < 3; ++k) {
            if (tri_vids[k] == vid) {
                lvid = k;
            }
        }

        // std::cout << "lvid: " << lvid << std::endl;

        assert(lvid != -1);

        auto uv_tri_vids = uvmesh_ptr->oriented_tri_vids(fid);
        // std::cout << "pass ori uv: " << uv_tri_vids[0] << " " << uv_tri_vids[1] << " "
        //           << uv_tri_vids[2] << std::endl;

        auto uv_vid = uv_tri_vids[lvid];
        // std::cout << "uv_vid: " << uv_vid << std::endl;


        Tuple uv_v_tuple = Tuple(uv_vid, leid, fid, *uvmesh_ptr);
        uv_face_one_ring.push_back(uv_v_tuple);
        cnt++;
    }
    // std::cout << "d" << std::endl;


    // wmtk::unique_vertex_tuples(*uvmesh_ptr, uv_face_one_ring);
    std::vector<Tuple> unique_uv_vertex_tuples;
    std::map<size_t, bool> exist_map;

    for (const auto& v : uv_face_one_ring) {
        size_t vid = v.vid(*this);
        if (exist_map.find(vid) == exist_map.end()) {
            // push back only if not visited
            exist_map[vid] = true;
            unique_uv_vertex_tuples.push_back(v);
        }
    }

    return unique_uv_vertex_tuples;
}

TriMesh::Tuple MMSurfaceMesh::map_to_equivalent_uv_tuple(const Tuple& t)
{
    size_t fid = t.fid(*this);
    size_t leid = t.local_eid(*this);

    size_t vid = t.vid(*this);
    int lvid = -1;

    const auto& tri_verts = oriented_tri_vids(fid);
    for (int k = 0; k < 3; ++k) {
        if (tri_verts[k] == vid) {
            lvid = k;
            break;
        }
    }

    assert(lvid > -1);

    const auto& uv_verts = uvmesh_ptr->oriented_tri_vids(fid);
    size_t uv_vid = uv_verts[lvid];

    return Tuple(uv_vid, leid, fid, *uvmesh_ptr);
}

Eigen::Matrix<double, 12, 3> MMSurfaceMesh::assemble_dofs(const size_t& fid)
{
    // TODO::implement this
    return f_attrs[fid].dofs;
    // return Eigen::Matrix<double, 12, 3>();
};

void MMSurfaceMesh::clear_info_cache()
{
    s_cache.layout_parts_map.clear();
    s_cache.tracked_vertex_info_cache.clear();
    s_cache.collapse_area_fid_involved.clear();

    s_cache.deleted_fid_2 = -1;
    s_cache.deleted_vid_2 = -1;
    s_cache.uv_collapse_to_v2_id = -1;
}

bool MMSurfaceMesh::collapse_edge_before(const Tuple& t)
{
    // TODO: test use, remove this
    // return true;
    clear_info_cache();

    if (v_attrs[t.vid(*this)].is_cone || v_attrs[t.switch_vertex(*this).vid(*this)].is_cone) {
        wmtk::logger().debug("rejected by cone");
        return false;
    }

    // link condition
    if (!TriMesh::collapse_edge_before(t)) {
        wmtk::logger().debug("rejected by surface mesh link condition");
        return false;
    }

    // check if map to multiple uv edges and has same v0
    const auto& uv_e_tuples = map_to_uv_edge_tuples(t);
    const auto& uv_v_tuples = map_to_uv_vertex_tuples(t);

    if (uv_e_tuples.size() > 1) {
        assert(uv_e_tuples.size() == 2);
        if (uv_e_tuples[0].vid(*uvmesh_ptr) == uv_e_tuples[1].vid(*uvmesh_ptr)) {
            wmtk::logger().debug("rejected by collapse from same vertex in two connected uv edges");
            return false;
        }

        if (uv_e_tuples[0].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr) ==
            uv_e_tuples[1].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr)) {
            wmtk::logger().debug("rejected by collapse to same vertex in two connected uv edges");
            return false;
        }
    }

    // surface_before
    s_cache.old_pos = v_attrs[t.vid(*this)].pos;
    s_cache.v2_pos = v_attrs[t.switch_vertex(*this).vid(*this)].pos;

    s_cache.deleted_fid_1 = t.fid(*this);
    if (!is_boundary_edge(t)) {
        s_cache.deleted_fid_2 = t.switch_face(*this).value().fid(*this);
    }

    // deleted vertex ids on at most two uv edges
    s_cache.deleted_vid_1 = uv_e_tuples[0].vid(*uvmesh_ptr);
    s_cache.uv_collapse_to_v1_id = uv_e_tuples[0].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);

    if (uv_e_tuples.size() > 1) {
        s_cache.deleted_vid_2 = uv_e_tuples[1].vid(*uvmesh_ptr);
        s_cache.uv_collapse_to_v2_id = uv_e_tuples[1].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);
    } else {
        s_cache.deleted_vid_2 = -1;
        s_cache.uv_collapse_to_v2_id = -1;
    }

    // collapse area fids
    auto f_involved = get_one_ring_tris_for_vertex(t);
    std::vector<size_t> f_involved_vec;
    for (const auto& f_tuple : f_involved) {
        s_cache.collapse_area_fid_involved[f_tuple.fid(*this)] = true;
        f_involved_vec.push_back(f_tuple.fid(*this));
    }

    // sanity check
    std::vector<size_t> uv_f_involved;
    for (const auto& uv_v : uv_v_tuples) {
        auto uv_v_one_ring_face = uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v);
        for (const auto& uv_f : uv_v_one_ring_face) {
            uv_f_involved.push_back(uv_f.fid(*uvmesh_ptr));
        }
    }

    assert(f_involved_vec.size() == uv_f_involved.size());
    for (const size_t& uv_fid : uv_f_involved) {
        assert(
            s_cache.collapse_area_fid_involved.find(uv_fid) !=
            s_cache.collapse_area_fid_involved.end());
    }


    // store same edge tuple info
    // the first uv edge has the same face as input edge
    s_cache.ref_uv_v1_id = uv_e_tuples[0].vid(*uvmesh_ptr);
    s_cache.ref_uv_v2_id = uv_e_tuples[0].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);

    s_cache.v1_uv_pos = uvmesh_ptr->v_attrs[uv_e_tuples[0].vid(*uvmesh_ptr)].pos;
    s_cache.v2_uv_pos =
        uvmesh_ptr->v_attrs[uv_e_tuples[0].switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr)].pos;

    // layout parts
    int processed_part_cnt = 1;

    // add first part
    s_cache.layout_parts_map[s_cache.ref_uv_v1_id] =
        LayoutPartInfo(s_cache.ref_uv_v1_id, s_cache.v2_uv_pos, Vector2d(0, 0), 0);

    // vector from ref_v1 to ref_v2
    const auto collapse_vec = s_cache.v2_uv_pos - s_cache.v1_uv_pos;

    // clockwise traverse
    auto cur_tuple = t;
    Tuple prev_uv_tuple;
    double accumulated_rot = 0;
    while (processed_part_cnt < uv_v_tuples.size()) {
        // first one must be in the map, so uninitialized prev_uv_tuple is safe
        auto uv_tuple = map_to_equivalent_uv_tuple(cur_tuple);
        if (s_cache.layout_parts_map.find(uv_tuple.vid(*uvmesh_ptr)) !=
            s_cache.layout_parts_map.end()) {
            // already processed this part, skip, update cur/prev tuple
            if (!is_boundary_edge(cur_tuple)) {
                cur_tuple = cur_tuple.switch_face(*this).value().switch_edge(*this);
                prev_uv_tuple = uv_tuple;
            } else {
                break;
            }
        } else {
            // compute a new part
            size_t uv_vid = uv_tuple.vid(*uvmesh_ptr);

            // prev ref edge
            const auto& prev_v1 = prev_uv_tuple.vid(*uvmesh_ptr);
            const auto& prev_v2 = prev_uv_tuple.switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);
            const auto& prev_v1_pos = uvmesh_ptr->v_attrs[prev_v1].pos;
            const auto& prev_v2_pos = uvmesh_ptr->v_attrs[prev_v2].pos;

            Vector2d prev_edge_vec = prev_v2_pos - prev_v1_pos;

            // cur ref edge, do a swtich edge on cur_tuple
            auto cur_uv_edge_tuple = uv_tuple.switch_edge(*uvmesh_ptr);
            const auto& cur_v1 = cur_uv_edge_tuple.vid(*uvmesh_ptr);
            const auto& cur_v2 = cur_uv_edge_tuple.switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);
            const auto& cur_v1_pos = uvmesh_ptr->v_attrs[cur_v1].pos;
            const auto& cur_v2_pos = uvmesh_ptr->v_attrs[cur_v2].pos;

            Vector2d cur_edge_vec = cur_v2_pos - cur_v1_pos;

            // use atan2 to compute angle from prev edge to cur edge
            double angle =
                atan2(cur_edge_vec[1], cur_edge_vec[0]) - atan2(prev_edge_vec[1], prev_edge_vec[0]);

            // accumulate angle from original ref
            accumulated_rot += angle;

            // construct new layout part
            LayoutPartInfo part_info;
            part_info.ref_uv_vid = uv_vid;
            part_info.rotation = accumulated_rot;
            part_info.translation = cur_v1_pos - prev_v1_pos;

            Eigen::Matrix2d R;
            R << cos(accumulated_rot), -sin(accumulated_rot), sin(accumulated_rot),
                cos(accumulated_rot);

            part_info.new_pos = cur_v1_pos + R * collapse_vec;

            // add new part to map
            s_cache.layout_parts_map[uv_vid] = part_info;

            processed_part_cnt++;
        }
    }

    // TODO: ccw traverse for surface mesh with boundaries, first test without this

    // TODO: cache all effected tracked vertex
    std::vector<size_t> effected_tracked_vids;

    // get real one ring by computing each one ring
    for (const auto& uv_v : uv_v_tuples) {
        auto uv_vid = uv_v.vid(*uvmesh_ptr);
        auto uv_one_ring_faces = uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v);

        for (const auto& f : uv_one_ring_faces) {
            for (const auto& vid : tracked_fid_to_vids_map[f.fid(*uvmesh_ptr)]) {
                effected_tracked_vids.push_back(vid);

                TrackedVertexInfo tvinfo;
                tvinfo.vid = vid;
                tvinfo.ref_uv_vid = uv_vid;
                tvinfo.uv_pos = tracked_vertices[vid].uv_pos;
                tvinfo.ref_uv_pos = uvmesh_ptr->v_attrs[uv_vid].pos;
                tvinfo.translation = s_cache.layout_parts_map[uv_vid].translation;
                tvinfo.rotation = s_cache.layout_parts_map[uv_vid].rotation;

                s_cache.tracked_vertex_info_cache.push_back(tvinfo);
            }
        }
    }


    return true;
}

bool MMSurfaceMesh::collapse_edge_after(const Tuple& t)
{
    // TODO: test use, remove this
    // return true;

    if (!TriMesh::collapse_edge_after(t)) {
        return false;
    }

    // update uv position
    for (const auto& pair : s_cache.layout_parts_map) {
        const auto& old_uv_vid = pair.first;
        const auto& part = pair.second;
        if (old_uv_vid != s_cache.deleted_vid_1 && old_uv_vid != s_cache.deleted_vid_2) {
            // update uv vertex that is affected by the collapse but not deleted
            uvmesh_ptr->v_attrs[old_uv_vid].pos = part.new_pos;
        } else {
            if (old_uv_vid == s_cache.deleted_vid_1) {
                // auto uv_tuple_to_update = map_to_equivalent_uv_tuple(t);
                // uvmesh_ptr->v_attrs[uv_tuple_to_update.vid(*uvmesh_ptr)].pos = part.new_pos;
                uvmesh_ptr->v_attrs[s_cache.uv_collapse_to_v1_id].pos = part.new_pos;
            } else {
                // TODO: add position for the other vertex
                assert(old_uv_vid == s_cache.deleted_vid_2);
                uvmesh_ptr->v_attrs[s_cache.uv_collapse_to_v2_id].pos = part.new_pos;
                continue;
            }
        }
    }

    // check uv invertion
    auto uv_v_tuples_after = map_to_uv_vertex_tuples(t);

    for (const auto& uv_v_after : uv_v_tuples_after) {
        auto uv_f_after = uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v_after);
        for (const auto& uv_f : uv_f_after) {
            if (uvmesh_ptr->is_inverted(uv_f)) {
                wmtk::logger().debug("collapse rejected by uv invertion in surface after");
                return false;
            }
        }
    }

    // delete removed faces in map
    tracked_fid_to_vids_map.erase(s_cache.deleted_fid_1);
    if (s_cache.deleted_fid_2 != -1) {
        tracked_fid_to_vids_map.erase(s_cache.deleted_fid_2);
    }

    // clear modified face to tracked vertex in map
    for (const auto& uv_v_after : uv_v_tuples_after) {
        for (const auto& uv_f : uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v_after)) {
            // delete effected track id in kept faces
            for (const auto& tvinfo : s_cache.tracked_vertex_info_cache) {
                const auto& tracked_vid = tvinfo.vid;
                auto it = std::find(
                    tracked_fid_to_vids_map[uv_f.fid(*uvmesh_ptr)].begin(),
                    tracked_fid_to_vids_map[uv_f.fid(*uvmesh_ptr)].end(),
                    tracked_vid);
                if (it != tracked_fid_to_vids_map[uv_f.fid(*uvmesh_ptr)].end()) {
                    tracked_fid_to_vids_map[uv_f.fid(*uvmesh_ptr)].erase(it);
                }
            }
        }
    }

    // find new uv position for tracked vertices


    for (const auto& tvinfo : s_cache.tracked_vertex_info_cache) {
        const auto& tracked_vid = tvinfo.vid;
        const auto& ref_uv_vid = tvinfo.ref_uv_vid;
        const auto& tracked_pos = tvinfo.uv_pos;
        const auto& ref_uv_pos = tvinfo.ref_uv_pos;
        const auto& rotation = tvinfo.rotation;

        Vector2d tracked_v_chart_pos; // tracked position in vertex chart. rotate and translate if
                                      // not in the first part
        Eigen::Matrix2d R_inv;
        R_inv << cos(-rotation), -sin(-rotation), sin(-rotation), cos(-rotation);

        tracked_v_chart_pos = R_inv * (tracked_pos - ref_uv_pos) + s_cache.v1_uv_pos;

        std::ofstream point_file("point.obj");

        point_file << "v " << tracked_v_chart_pos[0] << " " << tracked_v_chart_pos[1] << " 0"
                   << std::endl;
        point_file.close();

        std::ofstream vchart_file("v_chart.obj");
        std::ofstream v_tri_file("v_tris.obj");
        int tri_cnt = 0;

        bool found_flag = false;

        // use tracked pos in chart and traverse through charts
        for (const auto& uv_v_after : uv_v_tuples_after) {
            const auto& uv_v_after_vid = uv_v_after.vid(*uvmesh_ptr);
            const auto& uv_v_after_pos = uvmesh_ptr->v_attrs[uv_v_after_vid].pos;

            // find related part
            LayoutPartInfo part;
            if (s_cache.layout_parts_map.find(uv_v_after_vid) != s_cache.layout_parts_map.end()) {
                part = s_cache.layout_parts_map.at(uv_v_after_vid);
                // } else if (uv_v_after_vid == s_cache.ref_uv_v2_id) {
                // } else if (uv_v_after_vid == map_to_equivalent_uv_tuple(t).vid(*uvmesh_ptr)) {
            } else if (uv_v_after_vid == s_cache.uv_collapse_to_v1_id) {
                part = s_cache.layout_parts_map.at(s_cache.ref_uv_v1_id);
            } else if (uv_v_after_vid == s_cache.uv_collapse_to_v2_id) {
                // other side edge
                assert(s_cache.deleted_vid_2 != -1);
                part = s_cache.layout_parts_map.at(s_cache.deleted_vid_2);
            } else {
                // vertex not involved, skip
                continue;
            }

            // rotation matrix for the triangles in this part
            Eigen::Matrix2d R_inv_part, R_part;
            R_inv_part << cos(-part.rotation), -sin(-part.rotation), sin(-part.rotation),
                cos(-part.rotation);
            R_part << cos(part.rotation), -sin(part.rotation), sin(part.rotation),
                cos(part.rotation);

            // traverse tris in this part with rotation and translation
            const auto& uv_v_after_one_ring_tris =
                uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v_after);
            for (const auto& tri_tuple : uv_v_after_one_ring_tris) {
                size_t tri_fid = tri_tuple.fid(*uvmesh_ptr);
                // if (s_cache.collapse_area_fid_involved.find(tri_fid) ==
                //     s_cache.collapse_area_fid_involved.end()) {
                //     // skip faces that are not involved if not collapsed edge
                //     continue;
                // }

                auto v0 = tri_tuple.vid(*uvmesh_ptr);
                auto v1 = tri_tuple.switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);
                auto v2 =
                    tri_tuple.switch_edge(*uvmesh_ptr).switch_vertex(*uvmesh_ptr).vid(*uvmesh_ptr);
                Vector2d vec01 = uvmesh_ptr->v_attrs[v1].pos - uvmesh_ptr->v_attrs[v0].pos;
                Vector2d vec02 = uvmesh_ptr->v_attrs[v2].pos - uvmesh_ptr->v_attrs[v0].pos;

                // move and rotate the triangle to v2
                Vector2d p0_chart = s_cache.v2_uv_pos;
                Vector2d p1_chart = p0_chart + R_inv_part * vec01;
                Vector2d p2_chart = p0_chart + R_inv_part * vec02;

                // std::cout << "tri: " << std::endl;
                v_tri_file << "v " << uvmesh_ptr->v_attrs[v0].pos[0] << " "
                           << uvmesh_ptr->v_attrs[v0].pos[1] << " 0" << std::endl;
                v_tri_file << "v " << uvmesh_ptr->v_attrs[v1].pos[0] << " "
                           << uvmesh_ptr->v_attrs[v1].pos[1] << " 0" << std::endl;
                v_tri_file << "v " << uvmesh_ptr->v_attrs[v2].pos[0] << " "
                           << uvmesh_ptr->v_attrs[v2].pos[1] << " 0" << std::endl;

                vchart_file << "v " << p0_chart[0] << " " << p0_chart[1] << " 0" << std::endl;
                vchart_file << "v " << p1_chart[0] << " " << p1_chart[1] << " 0" << std::endl;
                vchart_file << "v " << p2_chart[0] << " " << p2_chart[1] << " 0" << std::endl;
                tri_cnt++;

                if (point_in_tri(tracked_v_chart_pos, p0_chart, p1_chart, p2_chart)) {
                    if (!found_flag) {
                        found_flag = true;

                        // update track vertex position
                        auto& tracked_v = tracked_vertices[tracked_vid];
                        tracked_v.uv_pos =
                            uv_v_after_pos + R_part * (tracked_v_chart_pos - s_cache.v2_uv_pos);

                        // update face id
                        tracked_v.in_tri_id = tri_tuple.fid(*uvmesh_ptr);

                        // update tracked_fid_to_vids_map
                        tracked_fid_to_vids_map[tri_tuple.fid(*uvmesh_ptr)].push_back(tracked_vid);

                        // break;
                    }
                }
            }
            if (found_flag) {
                // TODO: add break for efficiency
                // no break for v_chart
                // break;
            }
        }
        for (int i = 0; i < tri_cnt; ++i) {
            vchart_file << "f " << i * 3 + 1 << " " << i * 3 + 2 << " " << i * 3 + 3 << std::endl;
            v_tri_file << "f " << i * 3 + 1 << " " << i * 3 + 2 << " " << i * 3 + 3 << std::endl;
        }

        vchart_file.close();
        v_tri_file.close();

        assert(found_flag);
    }


    // // old code to deprecate
    // // find old vertex in which new triangle
    // Vector2d old_uv_pos = uvmesh_ptr->uv_cache.old_uv_pos;

    // size_t fid_involved = -1;
    // Vector2d a, b, c;
    // bool found_flag = false;
    // for (const auto& uv_v : uv_v_tuples) {
    //     auto uv_tris_one_ring = uvmesh_ptr->get_one_ring_tris_for_vertex(uv_v);

    //     for (const auto& uv_tri : uv_tris_one_ring) {
    //         auto uv_tri_vids = uvmesh_ptr->oriented_tri_vids(uv_tri);

    //         Vector2d p0, p1, p2;
    //         p0 = uvmesh_ptr->v_attrs[uv_tri_vids[0]].pos;
    //         p1 = uvmesh_ptr->v_attrs[uv_tri_vids[1]].pos;
    //         p2 = uvmesh_ptr->v_attrs[uv_tri_vids[2]].pos;

    //         if (point_in_tri(old_uv_pos, p0, p1, p2)) {
    //             fid_involved = uv_tri.fid(*uvmesh_ptr);
    //             a = p0;
    //             b = p1;
    //             c = p2;
    //             found_flag = true;
    //             break;
    //         }
    //     }

    //     if (found_flag) break;
    // }

    // assert(found_flag);

    // // check deviation
    // // compute bc in uv tri
    // // TODO: carefully check the order of vertices for bary here
    // auto bary_coord = barycentric_coord_in_tri(old_uv_pos, a, b, c);
    // auto dofs = assemble_dofs(fid_involved);

    // Eigen::Vector3d new_pos = CT_eval(bary_coord[0], bary_coord[1], dofs);

    // if ((new_pos - s_cache.old_pos).norm() > deviation_threshold) {
    //     wmtk::logger().debug(
    //         "collpase rejected by deviation: {}, threshold: {}",
    //         (new_pos - s_cache.old_pos).norm(),
    //         deviation_threshold);
    //     return false;
    // }


    // update 3d position

    v_attrs[t.vid(*this)].pos = s_cache.v2_pos;

    return true;
}

bool MMSurfaceMesh::multimesh_collapse_edge(const Tuple& t)
{
    // backup of tracked vertices
    auto tracked_vertices_copy = tracked_vertices;
    auto tracked_fid_to_vids_map_copy = tracked_fid_to_vids_map;
    auto other_vid = t.switch_vertex(*this).vid(*this);

    // before checks
    // uv before
    auto uv_e_tuples = map_to_uv_edge_tuples(t);
    for (const auto& uv_e : uv_e_tuples) {
        if (!uvmesh_ptr->collapse_edge_before(uv_e)) {
            wmtk::logger().debug("collapse rejected by uv before");
            return false;
        }
    }

    // surface before
    if (!collapse_edge_before(t)) {
        wmtk::logger().debug("collapse rejected by surface before");

        return false;
    }

    // tet before
    // tet connectivity change
    // variables prefix with tet_
    std::vector<TetMesh::Tuple> tet_new_edges;
    size_t tet_v1_id;
    TetMesh::Tuple tet_new_loc;
    std::map<size_t, wmtk::TetMesh::VertexConnectivity> tet_rollback_vert_conn;
    std::vector<size_t> tet_n1_t_ids;
    std::vector<size_t> tet_new_tet_id;
    std::vector<MMTetMesh::TetrahedronConnectivity> tet_old_tets;
    if (tetmesh_ptr != nullptr) {
        auto t_e_tuple = map_to_tet_edge_tuple(t);
        if (!tetmesh_ptr->collapse_edge_before(t_e_tuple)) {
            wmtk::logger().debug("collapse rejected by tet before");

            return false;
        }


        // connectivity change
        // first call tet one because it involves link condition check
        // the link condition check return false before actual connectivity change of the tetmesh so
        // it is safe


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
    if (tetmesh_ptr != nullptr) {
        tetmesh_ptr->start_protect_attributes();
    }

    bool rollback_flag = false;
    // if any after check failed, rollback_flag is set to true

    // surface check
    if (!rollback_flag) {
        rollback_flag = rollback_flag || !this->collapse_edge_after(s_return_t) ||
                        !this->invariants(s_new_tris);
    }

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
    if (tetmesh_ptr != nullptr) {
        if (!rollback_flag) {
            // check only if rollback_flag is false
            rollback_flag =
                rollback_flag || !tetmesh_ptr->collapse_edge_check_topology(tet_new_tet_id) ||
                !tetmesh_ptr->collapse_edge_after(tet_new_loc) ||
                !tetmesh_ptr->invariants(tetmesh_ptr->get_one_ring_tets_for_vertex(tet_new_loc));
        }
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
        if (tetmesh_ptr != nullptr) {
            tetmesh_ptr->collapse_edge_rollback(
                tet_v1_id,
                tet_rollback_vert_conn,
                tet_n1_t_ids,
                tet_new_tet_id,
                tet_old_tets);
        }
    }


    this->release_protect_attributes();
    uvmesh_ptr->release_protect_attributes();
    if (tetmesh_ptr != nullptr) {
        tetmesh_ptr->release_protect_attributes();
    }

    if (rollback_flag) {
        // rollback tracked vertices
        tracked_fid_to_vids_map = tracked_fid_to_vids_map_copy;
        tracked_vertices = tracked_vertices_copy;
        return false;
    }

    // post process of return values
    // TODO: add return reference
    if (tetmesh_ptr != nullptr) {
        // tetmesh
        for (size_t t_id : tet_new_tet_id) {
            for (int j = 0; j < 6; j++) {
                tet_new_edges.push_back(tetmesh_ptr->tuple_from_edge(t_id, j));
            }
        }
        // unique_edge_tuples(*tetmesh_ptr, tet_new_edges);

        std::vector<TetMesh::Tuple> unique_tet_new_edges;
        std::map<size_t, bool> exist_map;

        for (const auto& te : tet_new_edges) {
            size_t teid = te.eid(*tetmesh_ptr);
            if (exist_map.find(teid) == exist_map.end()) {
                // push back only if not visited
                exist_map[teid] = true;
                unique_tet_new_edges.push_back(te);
            }
        }
    }

    return true;
}


void MMSurfaceMesh::output_surface_mesh(const std::string& file)
{
    // TODO: remove consolidate
    consolidate_mesh();

    const auto& vertices = get_vertices();
    const auto& tris = get_faces();

    std::ofstream output(file);

    for (size_t i = 0; i < vertices.size(); ++i) {
        output << "v" << " " << v_attrs[vertices[i].vid(*this)].pos[0] << " "
               << v_attrs[vertices[i].vid(*this)].pos[1] << " "
               << v_attrs[vertices[i].vid(*this)].pos[2] << std::endl;
    }

    for (size_t i = 0; i < tris.size(); ++i) {
        auto vs = oriented_tri_vertices(tris[i]);
        output << "f" << " " << vs[0].vid(*this) + 1 << " " << vs[1].vid(*this) + 1 << " "
               << vs[2].vid(*this) + 1 << std::endl;
    }
}

void MMSurfaceMesh::output_tracked_vertices(const std::string& filename)
{
    std::ofstream output(filename);
    for (size_t i = 0; i < tracked_vertices.size(); ++i) {
        output << "v " << tracked_vertices[i].uv_pos[0] << " " << tracked_vertices[i].uv_pos[1]
               << " 0" << std::endl;
    }
}

} // namespace wmtk::components::c1_simplification