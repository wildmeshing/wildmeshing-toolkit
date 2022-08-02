
#include <wmtk/TetMesh.h>
#include <wmtk/TetMeshCutTable.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <bitset>


void wmtk::TetMesh::triangle_insertion(
    const std::vector<Tuple>& intersected_tets,
    const std::vector<Tuple>& intersected_edges,
    std::vector<size_t>& new_vids,
    std::vector<size_t>& new_center_vids,
    std::vector<std::array<size_t, 4>>& center_split_tets)
{
    std::vector<size_t> new_tids;

    /// get all tets
    std::vector<size_t> intersected_tids;
    intersected_tids.reserve(intersected_tets.size());
    for (auto& loc : intersected_tets) {
        intersected_tids.push_back(loc.tid(*this));
    }

    std::vector<bool> mark_surface(intersected_tids.size(), true);
    std::map<std::array<size_t, 2>, size_t> map_edge2vid;
    int cnt = 0;
    std::vector<size_t> surrounding_tids;

    // gather tets adjacent to the edges
    for (auto& loc : intersected_edges) {
        size_t v1_id = loc.vid(*this);
        auto tmp = switch_vertex(loc);
        size_t v2_id = tmp.vid(*this);
        std::array<size_t, 2> e = {{v1_id, v2_id}};
        if (e[0] > e[1]) std::swap(e[0], e[1]);

        std::vector<size_t> tids = set_intersection(
            m_vertex_connectivity[e[0]].m_conn_tets,
            m_vertex_connectivity[e[1]].m_conn_tets);
        surrounding_tids.insert(surrounding_tids.end(), tids.begin(), tids.end());

        auto new_vid = get_next_empty_slot_v();
        map_edge2vid[e] = new_vid;
        new_vids.push_back(new_vid);
        cnt++;
    }
    vector_unique(surrounding_tids);
    std::vector<size_t> diff_tids;
    std::sort(intersected_tids.begin(), intersected_tids.end());
    std::set_difference(
        surrounding_tids.begin(),
        surrounding_tids.end(),
        intersected_tids.begin(),
        intersected_tids.end(),
        std::back_inserter(diff_tids));

    //
    mark_surface.resize(diff_tids.size() + intersected_tids.size(), false);
    intersected_tids.insert(intersected_tids.end(), diff_tids.begin(), diff_tids.end());
    assert(mark_surface.size() == intersected_tids.size());

    /// track surface before
    std::vector<Tuple> old_faces;
    std::vector<std::array<size_t, 5>> old_face_vids; // vids, tid, l_fid
    for (size_t tid : intersected_tids) {
        for (int j = 0; j < 4; j++) {
            std::array<int, 3> l_f = {{(j + 1) % 4, (j + 2) % 4, (j + 3) % 4}};
            std::sort(l_f.begin(), l_f.end());
            size_t l_fid =
                std::find(m_local_faces.begin(), m_local_faces.end(), l_f) - m_local_faces.begin();
            //
            old_face_vids.push_back(
                {{m_tet_connectivity[tid][(j + 1) % 4],
                  m_tet_connectivity[tid][(j + 2) % 4],
                  m_tet_connectivity[tid][(j + 3) % 4],
                  tid,
                  l_fid}});
            std::sort(old_face_vids.back().begin(), old_face_vids.back().begin() + 3);
        }
    }
    // unique old_face_vids
    std::sort(
        old_face_vids.begin(),
        old_face_vids.end(),
        [](const std::array<size_t, 5>& v1, const std::array<size_t, 5>& v2) {
            return std::tie(v1[0], v1[1], v1[2]) < std::tie(v2[0], v2[1], v2[2]);
        });
    auto it = std::unique(
        old_face_vids.begin(),
        old_face_vids.end(),
        [](const std::array<size_t, 5>& v1, const std::array<size_t, 5>& v2) {
            return std::tie(v1[0], v1[1], v1[2]) == std::tie(v2[0], v2[1], v2[2]);
        });
    old_face_vids.erase(it, old_face_vids.end());
    //
    for (auto& info : old_face_vids) {
        old_faces.push_back(tuple_from_face(info[3], info[4]));
    }
    if (!triangle_insertion_before(old_faces)) return; // remember old_faces vids in cache

    ///subdivide
    std::map<std::array<size_t, 3>, std::vector<std::array<size_t, 5>>>
        new_face_vids; // note: vids of the face, tid, l_fid
    subdivide_tets(
        intersected_tids,
        mark_surface,
        map_edge2vid,
        new_face_vids,
        new_vids,
        new_tids,
        new_center_vids, 
        center_split_tets);

    /// track surface after
    std::vector<std::vector<Tuple>> new_faces(old_faces.size() + 1);
    for (auto& info : new_face_vids) {
        auto it1 = std::find_if(
            old_face_vids.begin(),
            old_face_vids.end(),
            [&info](const std::array<size_t, 5>& v1) {
                return std::array<size_t, 3>({{v1[0], v1[1], v1[2]}}) == info.first;
            });
        int i = it1 - old_face_vids.begin(); // already handled special case here
        assert(i < new_faces.size());
        //

        for (auto& f_info : info.second) {
            new_faces[i].push_back(tuple_from_face(f_info[3], f_info[4]));
        }
    }
    // TODO: no invariants
    if (!triangle_insertion_after(new_faces)) return;
}

void wmtk::TetMesh::subdivide_tets(
    const std::vector<size_t> intersected_tids,
    const std::vector<bool>& mark_surface,
    const std::map<std::array<size_t, 2>, size_t>& map_edge2vid,
    std::map<std::array<size_t, 3>, std::vector<std::array<size_t, 5>>>& new_face_vids,
    const std::vector<size_t>& new_vids,
    std::vector<size_t>& new_tids,
    std::vector<size_t>& new_center_vids,
    std::vector<std::array<size_t, 4>>& center_split_tets)
{
    /// insert new vertices

    /// record infos
    std::vector<size_t> vids;
    for (size_t t_id : intersected_tids) {
        for (int j = 0; j < 4; j++) vids.push_back(m_tet_connectivity[t_id][j]);
    }
    vector_unique(vids);
    //
    std::vector<size_t> tids;
    for (size_t vid : vids) {
        tids.insert(
            tids.end(),
            m_vertex_connectivity[vid].m_conn_tets.begin(),
            m_vertex_connectivity[vid].m_conn_tets.end());
    }
    vector_unique(tids);

    /// insert new tets
    size_t old_t_size = tet_capacity();
    for (int i = 0; i < intersected_tids.size(); i++) {
        size_t t_id = intersected_tids[i];
        std::array<int, 6> new_v_ids = {{-1, -1, -1, -1, -1, -1}};
        for (int j = 0; j < 6; j++) {
            std::array<size_t, 2> e = {
                {m_tet_connectivity[t_id][m_local_edges[j][0]],
                 m_tet_connectivity[t_id][m_local_edges[j][1]]}};
            if (e[0] > e[1]) std::swap(e[0], e[1]);
            auto it = map_edge2vid.find(e);
            if (it != map_edge2vid.end()) new_v_ids[j] = it->second;
        }

        subdivide_a_tet(
            t_id,
            new_v_ids,
            mark_surface[i],
            new_face_vids,
            new_tids,
            new_center_vids,
            center_split_tets);
    }

    /// update conn_tets
    {
        for (auto new_vid : new_vids) {
            vids.push_back(new_vid);
        }

        for (auto new_vid : new_center_vids) {
            vids.push_back(new_vid);
        }

        for (auto new_tid : new_tids) {
            tids.push_back(new_tid);
        }


        std::map<size_t, std::vector<size_t>> new_conn_tets;
        for (int vid : vids) {
            new_conn_tets[vid] = {};
        }
        //
        for (size_t tid : tids) {
            for (int j = 0; j < 4; j++) {
                size_t vid = m_tet_connectivity[tid][j];
                auto it = new_conn_tets.find(vid);
                if (it != new_conn_tets.end()) {
                    it->second.push_back(tid);
                }
            }
        }
        //
        for (int vid : vids) {
            m_vertex_connectivity[vid].m_conn_tets = new_conn_tets[vid];
            std::sort(
                m_vertex_connectivity[vid].m_conn_tets.begin(),
                m_vertex_connectivity[vid].m_conn_tets.end());
        }
    }
}

void wmtk::TetMesh::subdivide_a_tet(
    size_t t_id,
    const std::array<int, 6>& new_v_ids,
    bool mark_surface,
    std::map<std::array<size_t, 3>, std::vector<std::array<size_t, 5>>>& new_face_vids,
    std::vector<size_t>& new_tids,
    std::vector<size_t>& new_center_vids,
    std::vector<std::array<size_t, 4>>& center_split_tets)
{
    using namespace Eigen;

    std::map<std::array<int, 2>, int> get_local_e_id;
    for (int i = 0; i < m_local_edges.size(); i++) {
        get_local_e_id[m_local_edges[i]] = i;
        get_local_e_id[{{m_local_edges[i][1], m_local_edges[i][0]}}] = i;
    }

    std::vector<size_t> all_v_ids = {
        m_tet_connectivity[t_id][0],
        m_tet_connectivity[t_id][1],
        m_tet_connectivity[t_id][2],
        m_tet_connectivity[t_id][3]};
    std::array<int, 6> local_new_v_ids = {{-1, -1, -1, -1, -1, -1}};

    std::bitset<6> config_bits;
    int cnt = 0;
    for (int i = 0; i < new_v_ids.size(); i++) {
        if (new_v_ids[i] >= 0) {
            all_v_ids.push_back(new_v_ids[i]);
            config_bits.set(i);
            local_new_v_ids[i] = 4 + cnt;
            cnt++;
        }
    }

    int config_id = (int)(config_bits.to_ulong());
    const auto configs = CutTable::get_tet_confs(config_id);
    assert(!configs.empty());
    // note: config with centroid only happens on open boundary preservation

    std::vector<Vector2i> my_diags;
    for (int j = 0; j < 4; j++) { // 4 faces of the tet
        //        int cnt_diags = 0;
        std::vector<std::array<size_t, 2>> tmp_diags; // global ids
        std::vector<std::array<int, 2>> tmp_local_diags; // global ids
        for (int k = 0; k < 3; k++) {
            int l_eid = get_local_e_id[{{m_local_faces[j][k], m_local_faces[j][(k + 1) % 3]}}];
            if (new_v_ids[l_eid] >= 0) {
                tmp_diags.push_back(
                    {{(size_t)new_v_ids[l_eid],
                      m_tet_connectivity[t_id][m_local_faces[j][(k + 2) % 3]]}});
                tmp_local_diags.push_back(
                    {{local_new_v_ids[l_eid], m_local_faces[j][(k + 2) % 3]}});
                //                if (new_v_ids[l_eid] > diag[1])//todo: buggy
                //                    diag << m_local_faces[j][(k + 2) % 3], local_new_v_ids[l_eid];
                //                cnt_diags++;
            }
        }
        if (tmp_diags.size() < 2) continue;

        Vector2i diag;
        if (tmp_diags[0][0] > tmp_diags[1][0])
            diag << tmp_local_diags[0][0], tmp_local_diags[0][1];
        else
            diag << tmp_local_diags[1][0], tmp_local_diags[1][1];

        if (diag[0] > diag[1]) std::swap(diag[0], diag[1]);
        my_diags.push_back(diag);
    }
    std::sort(my_diags.begin(), my_diags.end(), [](const Vector2i& a, const Vector2i& b) {
        return std::make_tuple(a[0], a[1]) < std::make_tuple(b[0], b[1]);
    });

    int diag_config_id = -1;
    if (!my_diags.empty()) {
        const auto& all_diags = CutTable::get_diag_confs(config_id);
        for (int i = 0; i < all_diags.size(); i++) {
            if (my_diags != all_diags[i]) continue;
            diag_config_id = i;
            break;
        }
    } else
        diag_config_id = 0;
    assert(diag_config_id >= 0);

    // note: in some cases, we have to add centroid since the current setting of diags enforces that

    const auto& config = CutTable::get_tet_conf(config_id, diag_config_id);
    const auto& new_is_surface_fs = CutTable::get_surface_conf(config_id, diag_config_id);
    const auto& old_local_f_ids = CutTable::get_face_id_conf(config_id, diag_config_id);
    auto is_add_centroid = false;
    auto old_tet = m_tet_connectivity[t_id].m_indices;

    for (int i = 0; i < config.size(); i++) {
        const auto& t = config[i];
        TetrahedronConnectivity tet;
        for (int j = 0; j < 4; j++) {
            if (!is_add_centroid && t[j] >= 4 + config_bits.count()) {
                auto vid = get_next_empty_slot_v();
                new_center_vids.push_back(vid);
                all_v_ids.push_back(vid);
                center_split_tets.push_back(m_tet_connectivity[t_id].m_indices);

                is_add_centroid = true;
            }
            tet[j] = all_v_ids[t[j]];
        }
        size_t new_t_id = t_id;
        if (i < config.size() - 1) {
            new_t_id = get_next_empty_slot_t();
            new_tids.push_back(new_t_id);
        }
        m_tet_connectivity[new_t_id] = tet;


        /// track surface
        for (int j = 0; j < 4; j++) {
            std::vector<size_t> old_f_vids;
            if (old_local_f_ids[i][j] >= 0) { // old faces
                int old_j = old_local_f_ids[i][j];
                old_f_vids = {
                    old_tet[(old_j + 1) % 4],
                    old_tet[(old_j + 2) % 4],
                    old_tet[(old_j + 3) % 4]};
                std::sort(old_f_vids.begin(), old_f_vids.end());
            }
            //
            if (mark_surface && new_is_surface_fs[i][j]) { // new faces
                old_f_vids = {0, 0, 0}; // get empty old face map to new faces
            }

            if (!old_f_vids.empty()) {
                std::array<int, 3> l_f = {{(j + 1) % 4, (j + 2) % 4, (j + 3) % 4}};
                std::sort(l_f.begin(), l_f.end());
                size_t l_fid =
                    std::find(m_local_faces.begin(), m_local_faces.end(), l_f) -
                    m_local_faces
                        .begin(); // note tuple_from_face use the l_fid corresponds to m_local_faces
                //
                std::array<size_t, 5> new_f_vids = {
                    {tet[(j + 1) % 4], tet[(j + 2) % 4], tet[(j + 3) % 4], new_t_id, l_fid}};
                std::sort(new_f_vids.begin(), new_f_vids.begin() + 3);
                new_face_vids[{{old_f_vids[0], old_f_vids[1], old_f_vids[2]}}].push_back(
                    new_f_vids);
            }

            // note: new face_id has higher priority than old ones
            // note: non-cut-through tet does not track surface!!!
        }
    }

    for (auto& info : new_face_vids) { // erase duplicates <-- must have duplicates
        std::sort(
            info.second.begin(),
            info.second.end(),
            [](const std::array<size_t, 5>& v1, const std::array<size_t, 5>& v2) {
                return std::make_tuple(v1[0], v1[1], v1[2]) < std::make_tuple(v2[0], v2[1], v2[2]);
            });
        auto it = std::unique(
            info.second.begin(),
            info.second.end(),
            [](const std::array<size_t, 5>& v1, const std::array<size_t, 5>& v2) {
                return std::make_tuple(v1[0], v1[1], v1[2]) == std::make_tuple(v2[0], v2[1], v2[2]);
            });
        info.second.erase(it, info.second.end());
    }
}

bool wmtk::TetMesh::insert_point(const Tuple& t, std::vector<Tuple>& new_tets)
{
    ZoneScoped;
    if (!insert_point_before(t)) return false;
    start_protect_attributes();
    if (!insert_point_after(new_tets) || !invariants(new_tets)) {
        rollback_protected_attributes();
        return false;
    }
    release_protect_attributes();
    return true;
}