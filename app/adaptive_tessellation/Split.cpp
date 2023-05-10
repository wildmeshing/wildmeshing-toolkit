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
    if (wmtk::TriMeshSplitEdgeOperation::after(m, ret_data)) {
        const Eigen::Vector2d p =
            (m.vertex_attrs[op_cache.local().v1].pos + m.vertex_attrs[op_cache.local().v2].pos) /
            2.0;
        wmtk::logger().info(p);
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
        // nullify the middle interior edge outdated edge_attrs data
        m.edge_attrs[return_edge_tuple.switch_vertex(m).switch_edge(m).eid(m)].curve_id =
            std::nullopt;
        m.edge_attrs[return_edge_tuple.switch_vertex(m).switch_edge(m).switch_face(m).value().eid(
                         m)]
            .curve_id = std::nullopt;
        // check if it has a neighbore face whose edge_attrs data also need to be nullified
        if (return_edge_tuple.switch_face(m).has_value()) {
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
    return ret_data.success;
}

bool AdaptiveTessellationPairedSplitEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    static std::atomic_int cnt = 0;
    cnt++;
    assert(t.is_valid(m));
    // TODO is this thread safe?
    mirror_edge_tuple = std::nullopt; // reset the mirror edge tuple
    paired_op_cache.local().before_sibling_edges.resize(0); // clear the sibling edge cache
    //// === initiate the paired edge cache  === ////
    paired_op_cache.local().before_sibling_edges.emplace_back(m.get_sibling_edge(t));
    paired_op_cache.local().before_sibling_edges.emplace_back(
        m.get_sibling_edge(t.switch_vertex(m).switch_edge(m)));
    paired_op_cache.local().before_sibling_edges.emplace_back(
        m.get_sibling_edge(t.switch_edge(m).switch_vertex(m)));
    assert(paired_op_cache.local().before_sibling_edges.size() == 3);
    // if it's a seam edge
    if (m.is_seam_edge(t)) {
        mirror_edge_tuple = std::make_optional<wmtk::TriMesh::Tuple>(m.get_oriented_mirror_edge(t));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge(mirror_edge_tuple.value()));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge(mirror_edge_tuple.value().switch_vertex(m).switch_edge(m)));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge(mirror_edge_tuple.value().switch_edge(m).switch_vertex(m)));
        assert(paired_op_cache.local().before_sibling_edges.size() == 6);
    } // else if it is an interior edge
    else if (t.switch_face(m).has_value()) {
        assert(!m.is_seam_edge(t));
        wmtk::TriMesh::Tuple switch_face_oriented = t.switch_face(m).value().switch_vertex(m);
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge(switch_face_oriented));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge(switch_face_oriented.switch_vertex(m).switch_edge(m)));
        paired_op_cache.local().before_sibling_edges.emplace_back(
            m.get_sibling_edge(switch_face_oriented.switch_edge(m).switch_vertex(m)));
        assert(paired_op_cache.local().before_sibling_edges.size() == 6);
    }

    bool split_edge_success = split_edge.before(m, t);
    if (!split_edge_success) return false;
    assert(m.is_seam_edge(t) == mirror_edge_tuple.has_value());
    bool split_mirror_edge_success =
        m.is_seam_edge(t) ? mirror_split_edge.before(m, mirror_edge_tuple.value()) : true;
    // m.write_vtk(m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}.vtu", cnt));
    // m.write_perface_vtk(
    //     m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}_face.vtu", cnt));
    // m.write_displaced_obj(
    //     m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}.obj", cnt),
    //     m.mesh_parameters.m_displacement);
    // m.write_obj(m.mesh_parameters.m_output_folder + fmt::format("/split_{:04d}_2d.obj", cnt));

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
    bool old_t_is_boundary = m.is_boundary_edge(t) & (!old_t_is_seam);
    assert(m.check_mesh_connectivity_validity());
    assert(m.is_seam_edge(t) == mirror_edge_tuple.has_value());
    TriMeshSplitEdgeOperation::ExecuteReturnData ret_data =
        TriMeshSplitEdgeOperation::execute(m, t);
    if (!ret_data.success) return ret_data;
    split_edge.return_edge_tuple = ret_data.tuple;
    assert(split_edge.return_edge_tuple.vid(m) == t.vid(m));
    if (old_t_is_seam) {
        wmtk::logger().info("execute seam");
        assert(t.vid(m) == m.get_oriented_mirror_edge(mirror_edge_tuple.value()).vid(m));
        TriMeshSplitEdgeOperation::ExecuteReturnData mirror_ret_data =
            TriMeshSplitEdgeOperation::execute(m, mirror_edge_tuple.value());
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
        // executor.stopping_criterion_checking_frequency = 100;
        executor.renew_neighbor_tuples = split_renew;
        executor.priority = [&](auto& m, auto _, auto& e) {
            auto error = m.mesh_parameters.m_get_length(e);
            return error;
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            double length = 0.;
            double error1 = 0.;
            double error2 = 0.;
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                if (m.mesh_parameters.m_split_absolute_error_metric) {
                    error1 = m.get_area_accuracy_error_per_face(tup);
                    if (tup.switch_face(m).has_value()) {
                        error2 = m.get_area_accuracy_error_per_face(tup.switch_face(m).value());
                    } else {
                        if (m.is_seam_edge(tup))
                            error2 =
                                m.get_area_accuracy_error_per_face(m.get_oriented_mirror_edge(tup));
                        else
                            error2 = error1;
                    }
                    length = (error1 + error2) * m.get_length2d(tup);
                } else {
                    ///////// !!! TODO this should be deleted
                    error1 = m.get_area_accuracy_error_per_face(tup);
                    if (tup.switch_face(m).has_value()) {
                        error2 = m.get_area_accuracy_error_per_face(tup.switch_face(m).value());
                    } else
                        error2 = error1;
                    double e_before = error1 + error2;
                    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
                    double e_after, error_after_1, error_after_2, error_after_3, error_after_4;

                    triangle.row(0) = m.vertex_attrs[tup.vid(m)].pos;
                    triangle.row(1) = (m.vertex_attrs[tup.vid(m)].pos +
                                       m.vertex_attrs[tup.switch_vertex(m).vid(m)].pos) /
                                      2.;
                    triangle.row(2) =
                        m.vertex_attrs[tup.switch_edge(m).switch_vertex(m).vid(m)].pos;

                    error_after_1 = m.get_area_accuracy_error_per_face_triangle_matrix(triangle);
                    triangle.row(0) = m.vertex_attrs[tup.switch_vertex(m).vid(m)].pos;
                    error_after_2 = m.get_area_accuracy_error_per_face_triangle_matrix(triangle);
                    if (tup.switch_face(m).has_value()) {
                        triangle.row(2) = m.vertex_attrs[(tup.switch_face(m).value())
                                                             .switch_edge(m)
                                                             .switch_vertex(m)
                                                             .vid(m)]
                                              .pos;
                        error_after_3 =
                            m.get_area_accuracy_error_per_face_triangle_matrix(triangle);
                        triangle.row(0) = m.vertex_attrs[tup.vid(m)].pos;
                        error_after_4 =
                            m.get_area_accuracy_error_per_face_triangle_matrix(triangle);
                    } else {
                        error_after_3 = error_after_1;
                        error_after_4 = error_after_2;
                    }
                    e_after = error_after_1 + error_after_2 + error_after_3 + error_after_4;
                    length = e_before - e_after;
                }
            } else
                length = m.mesh_parameters.m_get_length(tup);
            // check if out of date
            if (!is_close(length, weight)) return false;
            // check if meet operating threshold
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::ACCURACY) {
                if (length < m.mesh_parameters.m_accuracy_threshold) return false;
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                if (error1 < m.mesh_parameters.m_accuracy_threshold &&
                    error2 < m.mesh_parameters.m_accuracy_threshold) {
                    // wmtk::logger().info("error1 {} error2 {}", error1, error2);
                    return false;
                }
            } else if (length < 4. / 3. * m.mesh_parameters.m_quality_threshold)
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
    // wmtk::logger().info("new 3d position {}", mesh_parameters.m_displacement->get(p(0),
    // p(1)));
    success_cnt++;
    return true;
}
