#include "Split.h"
using namespace adaptive_tessellation;
using namespace wmtk;


// every edge is split if it is longer than 4/5 L

auto split_renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) {
        assert(e.is_valid(m));
        optup.emplace_back(op, e);
    }
    return optup;
};

double AdaptiveTessellation::get_quadrics_area_accuracy_error_for_split(
    const Tuple& edge_tuple) const
{
    double energy = get_one_ring_quadrics_error_for_vertex(edge_tuple);
    energy += get_one_ring_quadrics_error_for_vertex(edge_tuple.switch_vertex(*this));
    // energy +=
    //     get_one_ring_quadrics_error_for_vertex(edge_tuple.switch_edge(*this).switch_vertex(*this));
    if (edge_tuple.switch_face(*this).has_value()) {
        // energy += get_one_ring_quadrics_error_for_vertex(
        //     edge_tuple.switch_face(*this).value().switch_edge(*this).switch_vertex(*this));
    } else {
        if (is_seam_edge(edge_tuple)) {
            // energy += get_one_ring_quadrics_error_for_vertex(
            //     get_oriented_mirror_edge(edge_tuple).switch_edge(*this).switch_vertex(*this));
            energy += get_one_ring_quadrics_error_for_vertex(
                get_oriented_mirror_edge(edge_tuple).switch_vertex(*this));
            energy += get_one_ring_quadrics_error_for_vertex(get_oriented_mirror_edge(edge_tuple));
        } // else
        // energy *= 2;
    }
    return energy;
}

void AdaptiveTessellationPairedSplitEdgeOperation::mark_failed()
{
    split_edge.mark_failed();
    mirror_split_edge.mark_failed();
}

template <typename Executor>
void addPairedCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationPairedSplitEdgeOperation>());
}

template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationSplitEdgeOperation>());
}

bool AdaptiveTessellationSplitEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    if (wmtk::TriMeshSplitEdgeOperation::before(m, t)) {
        // record the operation cache
        op_cache.local().v1 = t.vid(m);
        op_cache.local().v2 = t.switch_vertex(m).vid(m);
        op_cache.local().curve_id = m.edge_attrs[t.eid(m)].curve_id;
        op_cache.local().before_curve_ids.resize(0); // clear the cache
        // edge 2
        op_cache.local().before_curve_ids.emplace_back(
            m.edge_attrs[t.switch_edge(m).eid(m)].curve_id);
        // edge 1
        op_cache.local().before_curve_ids.emplace_back(
            m.edge_attrs[t.switch_vertex(m).switch_edge(m).eid(m)].curve_id);
        if (t.switch_face(m).has_value()) {
            // edge 5
            op_cache.local().before_curve_ids.emplace_back(
                m.edge_attrs[t.switch_face(m).value().switch_edge(m).eid(m)].curve_id);
            // edge 4
            op_cache.local().before_curve_ids.emplace_back(
                m.edge_attrs[t.switch_face(m).value().switch_vertex(m).switch_edge(m).eid(m)]
                    .curve_id);
        }
        // record the mesh cache
        m.cache.local().partition_id = m.vertex_attrs[t.vid(m)].partition_id;
        if (m.is_boundary_vertex(t))
            m.vertex_attrs[op_cache.local().v1].boundary_vertex = true;
        else
            m.vertex_attrs[op_cache.local().v1].boundary_vertex = false;
        if (m.is_boundary_vertex(t.switch_vertex(m)))
            m.vertex_attrs[op_cache.local().v2].boundary_vertex = true;
        else
            m.vertex_attrs[op_cache.local().v2].boundary_vertex = false;
        return true;
    }
    return false;
}

TriMeshOperation::ExecuteReturnData AdaptiveTessellationSplitEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    assert(m.check_mesh_connectivity_validity());
    TriMeshSplitEdgeOperation::ExecuteReturnData ret_data =
        TriMeshSplitEdgeOperation::execute(m, t);
    return_edge_tuple = ret_data.tuple;

    return ret_data;
}

bool AdaptiveTessellationSplitEdgeOperation::after(
    AdaptiveTessellation& m,
    wmtk::TriMeshOperation::ExecuteReturnData& ret_data)
{
    assert(bool(*this));
    if (wmtk::TriMeshSplitEdgeOperation::after(m, ret_data)) {
        assert(bool(*this));
        const Eigen::Vector2d p =
            (m.vertex_attrs[op_cache.local().v1].pos + m.vertex_attrs[op_cache.local().v2].pos) /
            2.0;
        auto vid = return_edge_tuple.switch_vertex(m).vid(m);
        // update the vertex_attrs
        m.vertex_attrs[vid].pos = p;
        m.vertex_attrs[vid].partition_id = m.cache.local().partition_id;
        m.vertex_attrs[vid].curve_id = m.vertex_attrs[op_cache.local().v1].curve_id;
        if (m.vertex_attrs[op_cache.local().v1].boundary_vertex &&
            m.vertex_attrs[op_cache.local().v2].boundary_vertex) {
            m.vertex_attrs[vid].boundary_vertex = true;
            m.vertex_attrs[vid].t =
                m.mesh_parameters.m_boundary.uv_to_t(m.vertex_attrs[vid].pos).second;
        }
        // update the edge_attrs
        m.edge_attrs[return_edge_tuple.eid(m)].curve_id = op_cache.local().curve_id;
        // edge 6
        auto edge_6 =
            return_edge_tuple.switch_vertex(m).switch_edge(m).switch_face(m).value().switch_edge(m);
        m.edge_attrs[edge_6.eid(m)].curve_id = op_cache.local().curve_id;
        // edge 2'
        m.edge_attrs[return_edge_tuple.switch_edge(m).eid(m)].curve_id =
            op_cache.local().before_curve_ids[0];
        // edge 1'
        auto edge_1_prime = return_edge_tuple.switch_vertex(m)
                                .switch_edge(m)
                                .switch_face(m)
                                .value()
                                .switch_edge(m)
                                .switch_vertex(m)
                                .switch_edge(m);
        m.edge_attrs[edge_1_prime.eid(m)].curve_id = op_cache.local().before_curve_ids[1];
        // nullify the middle interior edge outdated edge_attrs data
        m.edge_attrs[return_edge_tuple.switch_vertex(m).switch_edge(m).eid(m)].curve_id =
            std::nullopt;
        m.edge_attrs[return_edge_tuple.switch_vertex(m).switch_edge(m).switch_face(m).value().eid(
                         m)]
            .curve_id = std::nullopt;
        // check if it has a neighbore face whose edge_attrs data also need to be updated/nullified
        if (return_edge_tuple.switch_face(m).has_value()) {
            assert(op_cache.local().before_curve_ids.size() == 4);
            // edge 4'
            m.edge_attrs[return_edge_tuple.switch_face(m).value().switch_edge(m).eid(m)].curve_id =
                op_cache.local().before_curve_ids[2];
            // edge 5'
            auto edge_5_prime = return_edge_tuple.switch_face(m)
                                    .value()
                                    .switch_vertex(m)
                                    .switch_edge(m)
                                    .switch_face(m)
                                    .value()
                                    .switch_edge(m)
                                    .switch_vertex(m)
                                    .switch_edge(m);
            m.edge_attrs[edge_5_prime.eid(m)].curve_id = op_cache.local().before_curve_ids[3];
            // nullify the middle interior edge outdated edge_attrs data
            m.edge_attrs
                [return_edge_tuple.switch_face(m).value().switch_vertex(m).switch_edge(m).eid(m)]
                    .curve_id = std::nullopt;
            m.edge_attrs[return_edge_tuple.switch_face(m)
                             .value()
                             .switch_vertex(m)
                             .switch_edge(m)
                             .switch_face(m)
                             .value()
                             .eid(m)]
                .curve_id = std::nullopt;
        }
    }
    assert(bool(*this));
    // update the face_attrs (accuracy error)
    if (!m.mesh_parameters.m_ignore_embedding) {
        assert(bool(*this));
        auto modified_tris = modified_tuples(m);
        assert(modified_tris.size() == 2 || modified_tris.size() == 4);
        if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
            // get a vector of new traingles uvs
            std::vector<std::array<float, 6>> modified_tris_uv(modified_tris.size());
            for (int i = 0; i < modified_tris.size(); i++) {
                auto tri = modified_tris[i];
                auto verts = m.oriented_tri_vids(tri);
                std::array<float, 6> tri_uv;
                for (int i = 0; i < 3; i++) {
                    tri_uv[i * 2] = m.vertex_attrs[verts[i]].pos(0);
                    tri_uv[i * 2 + 1] = m.vertex_attrs[verts[i]].pos(1);
                }
                modified_tris_uv[i] = tri_uv;
            }
            std::vector<float> renewed_errors(modified_tris.size());
            m.m_texture_integral.get_error_per_triangle(modified_tris_uv, renewed_errors);
            m.set_faces_cached_distance_integral(modified_tris, renewed_errors);
        } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
            std::vector<wmtk::Quadric<double>> compressed_quadrics(modified_tris.size());
            m.m_quadric_integral.get_quadric_per_triangle(
                modified_tris.size(),
                [&](int f) -> std::array<float, 6> {
                    // Get triangle uv positions
                    std::array<Tuple, 3> local_tuples = m.oriented_tri_vertices(modified_tris[f]);
                    const Eigen::Vector2f& p0 =
                        m.vertex_attrs[local_tuples[0].vid(m)].pos.cast<float>();
                    const Eigen::Vector2f& p1 =
                        m.vertex_attrs[local_tuples[1].vid(m)].pos.cast<float>();
                    const Eigen::Vector2f& p2 =
                        m.vertex_attrs[local_tuples[2].vid(m)].pos.cast<float>();
                    return {p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y()};
                },
                compressed_quadrics);
            m.set_faces_quadrics(modified_tris, compressed_quadrics);
        }
    }

    return ret_data.success;
}

bool AdaptiveTessellationPairedSplitEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    static std::atomic_int cnt = 0;
    assert(t.is_valid(m));
    // TODO is this thread safe?
    mirror_edge_tuple = std::nullopt; // reset the mirror edge tuple
    paired_op_cache.local().before_sibling_edges.resize(0); // clear the sibling edge cache
    //// === initiate the paired edge cache  === ////
    paired_op_cache.local().before_sibling_edges.emplace_back(m.get_sibling_edge_opt(t));
    paired_op_cache.local().before_sibling_edges.emplace_back(
        m.get_sibling_edge_opt(t.switch_vertex(m).switch_edge(m)));
    paired_op_cache.local().before_sibling_edges.emplace_back(
        m.get_sibling_edge_opt(t.switch_edge(m).switch_vertex(m)));
    assert(paired_op_cache.local().before_sibling_edges.size() == 3);
    // if it's a seam edge
    if (m.is_seam_edge(t)) {
        mirror_edge_tuple = std::make_optional<wmtk::TriMesh::Tuple>(m.get_oriented_mirror_edge(t));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge_opt(mirror_edge_tuple.value()));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge_opt(mirror_edge_tuple.value().switch_vertex(m).switch_edge(m)));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge_opt(mirror_edge_tuple.value().switch_edge(m).switch_vertex(m)));
        assert(paired_op_cache.local().before_sibling_edges.size() == 6);
    } // else if it is an interior edge
    else if (t.switch_face(m).has_value()) {
        assert(!m.is_seam_edge(t));
        wmtk::TriMesh::Tuple switch_face_oriented = t.switch_face(m).value().switch_vertex(m);
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge_opt(switch_face_oriented));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge_opt(switch_face_oriented.switch_vertex(m).switch_edge(m)));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge_opt(switch_face_oriented.switch_edge(m).switch_vertex(m)));
        assert(paired_op_cache.local().before_sibling_edges.size() == 6);
    }

    bool split_edge_success = split_edge.before(m, t);
    if (!split_edge_success) return false;
    assert(m.is_seam_edge(t) == mirror_edge_tuple.has_value());
    bool split_mirror_edge_success =
        m.is_seam_edge(t) ? mirror_split_edge.before(m, mirror_edge_tuple.value()) : true;
    if (!m.mesh_parameters.m_do_not_output) {
        wmtk::logger().info("========= !!! current iteration {}", cnt);
        if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
            m.write_obj_displaced(
                m.mesh_parameters.m_output_folder + fmt::format("/area_split_{:04d}.obj", cnt));
            m.write_vtk(
                m.mesh_parameters.m_output_folder + fmt::format("/area_split_{:04d}.vtu", cnt));
        }
        // m.write_vtk(m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}.vtu", cnt));
        // m.write_perface_vtk(
        // m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}_face.vtu", cnt));
        else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
            m.write_obj_displaced(
                m.mesh_parameters.m_output_folder + fmt::format("/quadrics_split_{:04d}.obj", cnt));
            m.write_vtk(
                m.mesh_parameters.m_output_folder + fmt::format("/quadrics_split_{:04d}.vtu", cnt));
            // m.write_obj(m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}_2d.obj", cnt));}
        }
    }
    cnt++;
    assert(
        (paired_op_cache.local().before_sibling_edges.size() == 3 ||
         paired_op_cache.local().before_sibling_edges.size() == 6));

    return split_edge_success && split_mirror_edge_success;
}

wmtk::TriMeshOperation::ExecuteReturnData AdaptiveTessellationPairedSplitEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    bool old_t_is_seam = m.is_seam_edge(t);
    bool old_t_is_boundary = m.is_boundary_edge(t) && (!old_t_is_seam);
    assert(m.check_mesh_connectivity_validity());
    assert(m.is_seam_edge(t) == mirror_edge_tuple.has_value());
    TriMeshOperation::ExecuteReturnData ret_data = split_edge.execute(m, t);
    if (!ret_data.success) return ret_data;
    assert(bool(split_edge));
    split_edge.return_edge_tuple = ret_data.tuple;
    assert(split_edge.return_edge_tuple.vid(m) == t.vid(m));
    if (old_t_is_seam) {
        wmtk::logger().info("execute seam");
        assert(t.vid(m) == m.get_oriented_mirror_edge(mirror_edge_tuple.value()).vid(m));
        TriMeshOperation::ExecuteReturnData mirror_ret_data =
            mirror_split_edge.execute(m, mirror_edge_tuple.value());
        if (!mirror_ret_data.success) return ret_data;
        mirror_split_edge.return_edge_tuple = mirror_ret_data.tuple;
        assert(mirror_split_edge.return_edge_tuple.vid(m) == mirror_edge_tuple.value().vid(m));
        for (auto& nt : mirror_ret_data.new_tris) ret_data.new_tris.emplace_back(nt);
        assert(ret_data.success && mirror_ret_data.success);
    }
    /////===== populate the after sibling edges for easy update in after =====/////
    // edge naming referring to the ascii in .h file

    paired_op_cache.local().after_sibling_edges.resize(0);
    // 0'
    assert(t.vid(m) == split_edge.return_edge_tuple.vid(m));
    paired_op_cache.local().after_sibling_edges.emplace_back(split_edge.return_edge_tuple);
    // 1'
    auto edge_1_prime = split_edge.return_edge_tuple.switch_vertex(m)
                            .switch_edge(m)
                            .switch_face(m)
                            .value()
                            .switch_edge(m)
                            .switch_vertex(m)
                            .switch_edge(m);
    assert(edge_1_prime.is_valid(m));
    assert(
        split_edge.return_edge_tuple.switch_edge(m).switch_vertex(m).vid(m) ==
        edge_1_prime.switch_vertex(m).vid(m));
    paired_op_cache.local().after_sibling_edges.emplace_back(edge_1_prime);
    // 2'
    auto edge_2_prime = split_edge.return_edge_tuple.switch_edge(m).switch_vertex(m);
    assert(edge_2_prime.is_valid(m));
    assert(edge_2_prime.switch_vertex(m).vid(m) == split_edge.return_edge_tuple.vid(m));
    paired_op_cache.local().after_sibling_edges.emplace_back(edge_2_prime);
    // 6
    auto edge_6 = split_edge.return_edge_tuple.switch_vertex(m)
                      .switch_edge(m)
                      .switch_face(m)
                      .value()
                      .switch_edge(m);
    assert(edge_6.is_valid(m));
    assert(edge_6.vid(m) == split_edge.return_edge_tuple.switch_vertex(m).vid(m));
    // old t is boundary edge but not seam
    if (old_t_is_boundary) {
        assert(!mirror_edge_tuple.has_value());
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_6);
        assert(edge_6.is_valid(m));
        assert(paired_op_cache.local().after_sibling_edges.size() == 4);
        return ret_data;
    }
    // old t is mirror edge
    if (old_t_is_seam) {
        assert(mirror_edge_tuple.has_value());
        // 3'
        assert(mirror_split_edge.return_edge_tuple.vid(m) == mirror_edge_tuple.value().vid(m));
        assert(mirror_split_edge.return_edge_tuple.is_valid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(
            mirror_split_edge.return_edge_tuple);
        // 4'
        auto edge_4_prime = mirror_split_edge.return_edge_tuple.switch_vertex(m)
                                .switch_edge(m)
                                .switch_face(m)
                                .value()
                                .switch_edge(m)
                                .switch_vertex(m)
                                .switch_edge(m);
        assert(edge_4_prime.is_valid(m));
        assert(
            edge_4_prime.switch_vertex(m).vid(m) ==
            mirror_split_edge.return_edge_tuple.switch_edge(m).switch_vertex(m).vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_4_prime);
        // 5'
        auto edge_5_prime = mirror_split_edge.return_edge_tuple.switch_edge(m).switch_vertex(m);
        assert(edge_5_prime.is_valid(m));
        assert(edge_5_prime.switch_vertex(m).vid(m) == mirror_split_edge.return_edge_tuple.vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_5_prime);
        // 6
        assert(edge_6.is_valid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_6);
        // 7
        auto edge_7 = mirror_split_edge.return_edge_tuple.switch_vertex(m)
                          .switch_edge(m)
                          .switch_face(m)
                          .value()
                          .switch_edge(m);
        assert(edge_7.is_valid(m));
        assert(edge_7.vid(m) == mirror_split_edge.return_edge_tuple.switch_vertex(m).vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_7);
        assert(paired_op_cache.local().after_sibling_edges.size() == 8);
        return ret_data;
    }
    // old t is interior edge
    else {
        assert(!mirror_edge_tuple.has_value());
        assert(!old_t_is_seam);
        // 3'
        auto edge_3_prime = edge_6.switch_face(m).value().switch_vertex(m);
        assert(edge_3_prime.is_valid(m));
        assert(
            edge_3_prime.switch_vertex(m).vid(m) ==
            split_edge.return_edge_tuple.switch_vertex(m).vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_3_prime);
        // 4'
        auto edge_4_prime = edge_3_prime.switch_vertex(m)
                                .switch_edge(m)
                                .switch_face(m)
                                .value()
                                .switch_edge(m)
                                .switch_vertex(m)
                                .switch_edge(m);
        assert(edge_4_prime.is_valid(m));
        assert(edge_4_prime.vid(m) == split_edge.return_edge_tuple.vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_4_prime);
        // 5'
        auto edge_5_prime = edge_3_prime.switch_edge(m).switch_vertex(m);
        assert(edge_5_prime.is_valid(m));
        assert(
            edge_5_prime.switch_edge(m).switch_vertex(m).vid(m) ==
            split_edge.return_edge_tuple.switch_vertex(m).vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_5_prime);
        // 6
        assert(edge_6.is_valid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_6);
        // 7
        auto edge_7 = split_edge.return_edge_tuple.switch_face(m).value().switch_vertex(m);
        assert(edge_7.is_valid(m));
        assert(edge_7.switch_vertex(m).vid(m) == split_edge.return_edge_tuple.vid(m));
        paired_op_cache.local().after_sibling_edges.emplace_back(edge_7);
        assert(paired_op_cache.local().after_sibling_edges.size() == 8);
    }
    return ret_data;
}

bool AdaptiveTessellationPairedSplitEdgeOperation::after(
    AdaptiveTessellation& m,
    wmtk::TriMeshOperation::ExecuteReturnData& ret_data)
{
    assert(bool(split_edge));
    split_edge.after(m, ret_data);
    if (!ret_data.success) return false;
    // nullify the inside edges old mirror info
    m.face_attrs[split_edge.return_edge_tuple.switch_vertex(m).switch_edge(m).fid(m)]
        .mirror_edges[split_edge.return_edge_tuple.switch_vertex(m).switch_edge(m).local_eid(m)] =
        std::nullopt;
    m.face_attrs
        [split_edge.return_edge_tuple.switch_vertex(m).switch_edge(m).switch_face(m).value().fid(m)]
            .mirror_edges[split_edge.return_edge_tuple.switch_vertex(m)
                              .switch_edge(m)
                              .switch_face(m)
                              .value()
                              .local_eid(m)] = std::nullopt;

    // old t is seam edge
    if (mirror_edge_tuple.has_value()) {
        assert(paired_op_cache.local().before_sibling_edges.size() == 6);
        assert(paired_op_cache.local().after_sibling_edges.size() == 8);
        ret_data.success &=
            mirror_split_edge.after(m, ret_data); // after doesn't use contents of ret_data
        // enforce the mirror_split_edge t to be the same as the primary t
        m.vertex_attrs[mirror_split_edge.return_edge_tuple.vid(m)].t =
            m.vertex_attrs[split_edge.return_edge_tuple.vid(m)].t;
        // now do the siling edge tranfering
        if (!ret_data.success) return false;
        // it's a seam edge update mirror edge data using the sibling edges
        // edge naming referring to the ascii in .h file
        assert(paired_op_cache.local().after_sibling_edges[7].is_valid(m));
        assert(paired_op_cache.local().after_sibling_edges[6].is_valid(m));
        m.set_mirror_edge_data(
            split_edge.return_edge_tuple,
            paired_op_cache.local().after_sibling_edges[7]);
        m.set_mirror_edge_data(
            paired_op_cache.local().after_sibling_edges[7],
            split_edge.return_edge_tuple);
        m.set_mirror_edge_data(
            mirror_split_edge.return_edge_tuple,
            paired_op_cache.local().after_sibling_edges[6]);
        m.set_mirror_edge_data(
            paired_op_cache.local().after_sibling_edges[6],
            mirror_split_edge.return_edge_tuple);
    }

    // now we update 1_prime, 2_prime, s1, s2 mirror data if they are seam edges
    if (paired_op_cache.local().before_sibling_edges[1].has_value() &&
        m.is_seam_edge(paired_op_cache.local().before_sibling_edges[1].value())) {
        assert(paired_op_cache.local().after_sibling_edges[1].is_valid(m));
        // s1
        m.set_mirror_edge_data(
            paired_op_cache.local().before_sibling_edges[1].value(),
            paired_op_cache.local().after_sibling_edges[1]);
        // 1_prime
        m.set_mirror_edge_data(
            paired_op_cache.local().after_sibling_edges[1],
            paired_op_cache.local().before_sibling_edges[1].value());
    }
    if (paired_op_cache.local().before_sibling_edges[2].has_value() &&
        m.is_seam_edge(paired_op_cache.local().before_sibling_edges[2].value())) {
        assert(paired_op_cache.local().after_sibling_edges[2].is_valid(m));
        // s2
        m.set_mirror_edge_data(
            paired_op_cache.local().before_sibling_edges[2].value(),
            paired_op_cache.local().after_sibling_edges[2]);
        // 2_prime
        m.set_mirror_edge_data(
            paired_op_cache.local().after_sibling_edges[2],
            paired_op_cache.local().before_sibling_edges[2].value());
    }

    // now we update 4_prime, 5_prime, s4, s5 mirror data if s4, s5 exist and if they are seam
    // edges
    if (paired_op_cache.local().before_sibling_edges.size() == 3) return ret_data.success;
    assert(paired_op_cache.local().before_sibling_edges.size() == 6);
    assert(paired_op_cache.local().after_sibling_edges.size() == 8);
    if (paired_op_cache.local().before_sibling_edges[4].has_value() &&
        m.is_seam_edge(paired_op_cache.local().before_sibling_edges[4].value())) {
        assert(paired_op_cache.local().after_sibling_edges[4].is_valid(m));
        // s4
        m.set_mirror_edge_data(
            paired_op_cache.local().before_sibling_edges[4].value(),
            paired_op_cache.local().after_sibling_edges[4]);
        // 4_prime
        m.set_mirror_edge_data(
            paired_op_cache.local().after_sibling_edges[4],
            paired_op_cache.local().before_sibling_edges[4].value());
    }
    if (paired_op_cache.local().before_sibling_edges[5].has_value() &&
        m.is_seam_edge(paired_op_cache.local().before_sibling_edges[5].value())) {
        assert(paired_op_cache.local().after_sibling_edges[5].is_valid(m));
        // s5
        m.set_mirror_edge_data(
            paired_op_cache.local().before_sibling_edges[5].value(),
            paired_op_cache.local().after_sibling_edges[5]);
        // 5_prime
        m.set_mirror_edge_data(
            paired_op_cache.local().after_sibling_edges[5],
            paired_op_cache.local().before_sibling_edges[5].value());
    }
    // nullify the 2 inside edges' corresponding mirror edge data in face_attrs
    m.face_attrs[paired_op_cache.local().after_sibling_edges[3].switch_vertex(m).switch_edge(m).fid(
                     m)]
        .mirror_edges[paired_op_cache.local()
                          .after_sibling_edges[3]
                          .switch_vertex(m)
                          .switch_edge(m)
                          .local_eid(m)] = std::nullopt;
    m.face_attrs[paired_op_cache.local().after_sibling_edges[7].switch_edge(m).switch_vertex(m).fid(
                     m)]
        .mirror_edges[paired_op_cache.local()
                          .after_sibling_edges[7]
                          .switch_edge(m)
                          .switch_vertex(m)
                          .local_eid(m)] = std::nullopt;
    return ret_data.success;
}

void AdaptiveTessellation::split_all_edges()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](const auto& tup) {
        assert(tup.is_valid(*this));
        collect_tuples.emplace_back(tup);
    });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_split", t);
    wmtk::logger().info("=======split==========");

    wmtk::logger().info("size for edges to be split is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        addPairedCustomOps(executor);
        executor.renew_neighbor_tuples = split_renew;
        executor.priority = [&](auto& m, auto _, auto& e) {
            double priority = 0.;
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                // priority already scaled by 2d edge length
                priority = m.get_cached_area_accuracy_error_for_split(e) * m.get_length2d(e);
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                // error is not scaled by 2d edge length
                priority = m.get_quadrics_area_accuracy_error_for_split(e);
            } else
                priority = m.mesh_parameters.m_get_length(e);
            return priority;
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            double total_error = 0.;
            double unscaled_total_error = 0.;

            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                unscaled_total_error = m.get_cached_area_accuracy_error_for_split(tup);
                total_error = unscaled_total_error * m.get_length2d(tup);
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                unscaled_total_error = m.get_quadrics_area_accuracy_error_for_split(tup);
                total_error = unscaled_total_error;
            } else {
                total_error = m.mesh_parameters.m_get_length(tup);
                unscaled_total_error = total_error;
            }
            // check if out of date
            if (!is_close(total_error, weight)) {
                wmtk::logger().info("weight not up to date");
                wmtk::logger().info("the v1 {}, v2 {}", tup.vid(m), tup.switch_vertex(m).vid(m));
                return false;
            }
            // check if meet operating threshold
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                if (unscaled_total_error < m.mesh_parameters.m_accuracy_threshold) {
                    wmtk::logger().info("accuracy smaller than threshold");
                    return false;
                }
            } else if (total_error < 4. / 3. * m.mesh_parameters.m_quality_threshold)
                return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kSeq>();
        // used for debugging for early termination
        set_early_termination_number(mesh_parameters.m_early_stopping_number, executor);
        setup_and_execute(executor);
    }
}

bool AdaptiveTessellation::split_edge_before(const Tuple& edge_tuple) // not used anymore
{
    static std::atomic_int cnt = 0;

    // check if the 2 vertices are on the same curve
    if (vertex_attrs[edge_tuple.vid(*this)].curve_id !=
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].curve_id)
        return false;

    cache.local().v1 = edge_tuple.vid(*this);
    cache.local().v2 = edge_tuple.switch_vertex(*this).vid(*this);
    cache.local().partition_id = vertex_attrs[edge_tuple.vid(*this)].partition_id;
    if (is_boundary_vertex(edge_tuple))
        vertex_attrs[cache.local().v1].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v1].boundary_vertex = false;
    if (is_boundary_vertex(edge_tuple.switch_vertex(*this)))
        vertex_attrs[cache.local().v2].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v2].boundary_vertex = false;
    cnt++;
    return true;
}

bool AdaptiveTessellation::split_edge_after(const Tuple& edge_tuple) // not used anymore
{
    // adding heuristic decision. If length2 > 4. / 3. * 4. / 3. * m.m_target_l * m.m_target_l always split
    // transform edge length with displacement
    static std::atomic_int success_cnt = 0;
    const Eigen::Vector2d p =
        (vertex_attrs[cache.local().v1].pos + vertex_attrs[cache.local().v2].pos) / 2.0;
    auto vid = edge_tuple.switch_vertex(*this).vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = cache.local().partition_id;
    vertex_attrs[vid].curve_id = vertex_attrs[cache.local().v1].curve_id;
    // take into account of periodicity
    if (vertex_attrs[cache.local().v1].boundary_vertex &&
        vertex_attrs[cache.local().v2].boundary_vertex) {
        vertex_attrs[vid].boundary_vertex = true;
        vertex_attrs[vid].t = mesh_parameters.m_boundary.uv_to_t(vertex_attrs[vid].pos).second;
    }
    success_cnt++;
    return true;
}
