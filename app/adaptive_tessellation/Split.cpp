#include "Split.h"
using namespace adaptive_tessellation;
using namespace wmtk;

// every edge is split if it is longer than 4/5 L

auto split_renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationSplitEdgeOperation>());
    e.add_operation(std::make_shared<AdaptiveTessellationPairedSplitEdgeOperation>());
}

wmtk::TriMeshOperation::ExecuteReturnData AdaptiveTessellationSplitEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    return wmtk::TriMeshSplitEdgeOperation::execute(m, t);
}

bool AdaptiveTessellationSplitEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    if (wmtk::TriMeshSplitEdgeOperation::before(m, t)) {
        return m.split_edge_before(t);
    }
    return false;
}

bool AdaptiveTessellationSplitEdgeOperation::after(
    AdaptiveTessellation& m,
    wmtk::TriMeshOperation::ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshSplitEdgeOperation::after(m, ret_data)) {
        ret_data.success &= m.split_edge_after(ret_data.tuple);
    }
    return ret_data;
}

bool AdaptiveTessellationSplitEdgeOperation::invariants(
    AdaptiveTessellation& m,
    wmtk::TriMeshOperation::ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshSplitEdgeOperation::invariants(m, ret_data)) {
        ret_data.success &= m.invariants(ret_data.new_tris);
    }
    return ret_data;
}

wmtk::TriMeshOperation::ExecuteReturnData AdaptiveTessellationPairedSplitEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    auto mirror_edge_tuple = m.face_attrs[t.fid(m)].mirror_edges[t.local_eid(m)];
    wmtk::TriMeshOperation::ExecuteReturnData ret_data =
        wmtk::TriMeshSplitEdgeOperation::execute(m, t);
    assert(ret_data.tuple.local_eid(m) == t.local_eid(m));
    assert(ret_data.tuple.fid(m) == t.fid(m));
    assert(ret_data.tuple.vid(m) == t.vid(m));

    if (mirror_edge_tuple.has_value() && ret_data.success) {
        // TODO need to make sure the mirror_edge_tuple.value().vid(m) corresponds to the t.vid(m)
        wmtk::TriMeshOperation::ExecuteReturnData ret_mirror_data =
            wmtk::TriMeshSplitEdgeOperation::execute(m, mirror_edge_tuple.value());
        ret_data.success &= ret_mirror_data.success;
        for (auto& nt : ret_mirror_data.new_tris) ret_data.new_tris.emplace_back(nt);
        assert(mirror_edge_tuple.vid(m) == ret_mirror_data.tuple.vid(m));
        assert(mirror_edge_tuple.fid(m) == ret_mirror_data.tuple.fid(m));
        // update the mirror edge
        m.face_attrs[ret_data.tuple.fid(m)].mirror_edges[t.local_eid(m)] =
            std::make_optional<wmtk::TriMesh::Tuple>(ret_mirror_data.tuple);

        auto same_side_new_tuple = (ret_data.tuple.switch_vertex(m).switch_edge(m)).switch_face(m);
        assert(same_side_new_tuple.has_value());
        same_side_new_tuple = same_side_new_tuple.value().switch_edge(m).switch_vertex(m);
        // get the other side new fid
        auto other_side_new_tuple =
            ret_mirror_data.tuple.switch_vertex(m).switch_edge(m).switch_face(m);
        assert(other_side_new_tuple.has_value());
        other_side_new_tuple = other_side_new_tuple.value().switch_edge(m).switch_vertex(m);
        assert(other_side_new_tuple.value().fid(m) == same_side_new_tuple.value().fid(m));
        // given the topology changes done by split the local eid is preserved
        m.face_attrs[same_side_new_tuple.value().fid(m)].mirror_edges[t.local_eid(m)] =
            std::make_optional<wmtk::TriMesh::Tuple>(
                m.tuple_from_edge(other_side_new_tuple.value().fid(m), t.local_eid(m)));
    }

    return ret_data;
}

bool AdaptiveTessellationPairedSplitEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    if (!wmtk::TriMeshSplitEdgeOperation::before(m, t)) {
        return false;
    }
    // check for current edge and mirrored edge on the seam
    if (!m.split_edge_before(t)) return false;
    auto mirror_edge_tuple = m.face_attrs[t.fid(m)].mirror_edges[t.local_eid(m)];
    if (mirror_edge_tuple.has_value()) {
        if (!wmtk::TriMeshSplitEdgeOperation::before(m, mirror_edge_tuple.value())) return false;
        if (!m.split_edge_before(mirror_edge_tuple.value())) return false;
    }
    return true;
}

bool AdaptiveTessellationPairedSplitEdgeOperation::after(
    AdaptiveTessellation& m,
    wmtk::TriMeshOperation::ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshSplitEdgeOperation::after(m, ret_data)) {
        ret_data.success &= m.split_edge_after(ret_data.tuple);
    }
    auto mirror_edge_tuple =
        m.face_attrs[ret_data.tuple.fid(m)].mirror_edges[ret_data.tuple.local_eid(m)];
    if (mirror_edge_tuple.has_value()) {
        ret_data.success &= m.split_edge_after(mirror_edge_tuple.value());
    }
    return ret_data.success;
}

bool AdaptiveTessellationPairedSplitEdgeOperation::invariants(
    AdaptiveTessellation& m,
    wmtk::TriMeshOperation::ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshSplitEdgeOperation::invariants(m, ret_data)) {
        ret_data.success &= m.invariants(ret_data.new_tris);
    }
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
        addCustomOps(executor);
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
                error1 = m.get_area_accuracy_error_per_face(tup);
                if (tup.switch_face(m).has_value()) {
                    error2 = m.get_area_accuracy_error_per_face(tup.switch_face(m).value());
                } else
                    error2 = error1;
                if (m.mesh_parameters.m_split_absolute_error_metric) {
                    length = (error1 + error2) * m.get_length2d(tup);
                } else {
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

bool AdaptiveTessellation::split_edge_before(const Tuple& edge_tuple)
{
    static std::atomic_int cnt = 0;
    // write_displaced_obj(
    //     mesh_parameters.m_output_folder + fmt::format("/split_{:02d}.obj", cnt),
    //     mesh_parameters.m_displacement);
    // write_vtk(mesh_parameters.m_output_folder + fmt::format("/split_{:02d}_absolute.vtu", cnt));
    // write_perface_vtk(
    //     mesh_parameters.m_output_folder + fmt::format("/tri_{:02d}_absolute.vtu", cnt));
    // if (cnt % 10000 == 0) {
    //     // write_vtk(mesh_parameters.m_output_folder + fmt::format("/split_{:04d}.vtu", cnt));
    //     write_perface_vtk(mesh_parameters.m_output_folder + fmt::format("/tri_{:06d}.vtu", cnt));
    //     write_displaced_obj(
    //         mesh_parameters.m_output_folder + fmt::format("/split_{:06d}_rel.obj", cnt),
    //         mesh_parameters.m_displacement);
    //     write_obj(mesh_parameters.m_output_folder + fmt::format("/split_{:06d}_rel_2d.obj", cnt));
    // }


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

bool AdaptiveTessellation::split_edge_after(const Tuple& edge_tuple)
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
        vertex_attrs[vid].t = mesh_parameters.m_boundary.uv_to_t(vertex_attrs[vid].pos);
    }
    success_cnt++;
    return true;
}
