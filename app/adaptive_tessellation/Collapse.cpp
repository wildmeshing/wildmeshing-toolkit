#include "Collapse.h"
using namespace wmtk;
using namespace adaptive_tessellation;

// every edge is collapsed, if it is shorter than 3/4 L

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};

template <typename Executor>
void addPairedCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationPairedCollapseEdgeOperation>());
}

template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationCollapseEdgeOperation>());
}

bool AdaptiveTessellationCollapseEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
        // check if the two vertices to be split is of the same curve_id
        if (m.vertex_attrs[t.vid(m)].curve_id != m.vertex_attrs[t.switch_vertex(m).vid(m)].curve_id)
            return false;

        double length3d = m.mesh_parameters.m_get_length(t);
        // enforce heuristic
        assert(length3d < 4. / 5. * m.mesh_parameters.m_quality_threshold);

        // record boundary vertex as boudnary_vertex in vertex attribute for accurate collapse
        // after boundary operations

        // record if the two vertices of the edge is boundary vertex
        m.vertex_attrs[t.vid(m)].boundary_vertex = m.is_boundary_vertex(t);
        m.vertex_attrs[t.switch_vertex(m).vid(m)].boundary_vertex =
            m.is_boundary_vertex(t.switch_vertex(m));

        if (m.mesh_parameters.m_bnd_freeze &&
            (m.vertex_attrs[t.vid(m)].boundary_vertex ||
             m.vertex_attrs[t.switch_vertex(m).vid(m)].boundary_vertex))
            return false;

        // record the two vertices vids to the operation cache
        op_cache.local().v1 = t.vid(m);
        op_cache.local().v2 = t.switch_vertex(m).vid(m);
        op_cache.local().length3d = length3d;
        m.cache.local().partition_id = m.vertex_attrs[t.vid(m)].partition_id;
        return true;
    }
    return false;
}

TriMeshOperation::ExecuteReturnData AdaptiveTessellationCollapseEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    assert(m.check_mesh_connectivity_validity());
    TriMeshOperation::ExecuteReturnData ret_data = TriMeshEdgeCollapseOperation::execute(m, t);
    return_edge_tuple = ret_data.tuple;
    return ret_data;
}
bool AdaptiveTessellationCollapseEdgeOperation::after(
    AdaptiveTessellation& m,
    ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshEdgeCollapseOperation::after(m, ret_data)) {
        ret_data.success &= m.collapse_edge_after(ret_data.tuple);
    }
    return ret_data;

    // check if the both of the 2 vertices are fixed
    // if yes, then collapse is rejected
    if (m.vertex_attrs[op_cache.local().v1].fixed && m.vertex_attrs[op_cache.local().v2].fixed)
        return false;

    // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
    // enforce heuristic
    assert(op_cache.local().length3d < 4. / 5. * m.mesh_parameters.m_quality_threshold);

    Eigen::Vector2d p;
    double t_parameter = 0.;
    double mod_length = 0.;
    if (m.vertex_attrs[op_cache.local().v1].fixed) {
        assert(!m.vertex_attrs[op_cache.local().v2].fixed);
        p = m.vertex_attrs[op_cache.local().v1].pos;
        t_parameter = m.vertex_attrs[op_cache.local().v1].t;
    } else if (m.vertex_attrs[op_cache.local().v2].fixed) {
        assert(!m.vertex_attrs[op_cache.local().v1].fixed);
        p = m.vertex_attrs[op_cache.local().v2].pos;
        t_parameter = m.vertex_attrs[op_cache.local().v2].t;
    } else {
        assert(!m.vertex_attrs[op_cache.local().v1].fixed);
        assert(!m.vertex_attrs[op_cache.local().v2].fixed);
        if (m.vertex_attrs[op_cache.local().v1].boundary_vertex &&
            m.vertex_attrs[op_cache.local().v2].boundary_vertex) {
            m.vertex_attrs[return_edge_tuple.vid(m)].boundary_vertex = true;

            // compare collapse to which one would give lower energy
            m.vertex_attrs[return_edge_tuple.vid(m)].pos = m.vertex_attrs[op_cache.local().v1].pos;
            m.vertex_attrs[return_edge_tuple.vid(m)].t = m.vertex_attrs[op_cache.local().v1].t;
            auto energy1 = m.get_one_ring_energy(return_edge_tuple).first;
            m.vertex_attrs[return_edge_tuple.vid(m)].pos = m.vertex_attrs[op_cache.local().v2].pos;
            m.vertex_attrs[return_edge_tuple.vid(m)].t = m.vertex_attrs[op_cache.local().v2].t;
            auto energy2 = m.get_one_ring_energy(return_edge_tuple).first;
            p = energy1 < energy2 ? m.vertex_attrs[op_cache.local().v1].pos
                                  : m.vertex_attrs[op_cache.local().v2].pos;
            m.vertex_attrs[return_edge_tuple.vid(m)].curve_id =
                energy1 < energy2 ? m.vertex_attrs[op_cache.local().v1].curve_id
                                  : m.vertex_attrs[op_cache.local().v2].curve_id;
            t_parameter = energy1 < energy2 ? m.vertex_attrs[op_cache.local().v1].t
                                            : m.vertex_attrs[op_cache.local().v2].t;
        } else if (m.vertex_attrs[op_cache.local().v1].boundary_vertex) {
            p = m.vertex_attrs[op_cache.local().v1].pos;
            t_parameter = m.vertex_attrs[op_cache.local().v1].t;
            m.vertex_attrs[return_edge_tuple.vid(m)].curve_id =
                m.vertex_attrs[op_cache.local().v1].curve_id;
        } else if (m.vertex_attrs[op_cache.local().v2].boundary_vertex) {
            p = m.vertex_attrs[op_cache.local().v2].pos;
            m.vertex_attrs[return_edge_tuple.vid(m)].curve_id =
                m.vertex_attrs[op_cache.local().v2].curve_id;
            t_parameter = m.vertex_attrs[op_cache.local().v2].t;
        } else {
            assert(!m.vertex_attrs[op_cache.local().v1].boundary_vertex);
            assert(!m.vertex_attrs[op_cache.local().v2].boundary_vertex);
            p = (m.vertex_attrs[op_cache.local().v1].pos +
                 m.vertex_attrs[op_cache.local().v2].pos) /
                2.0;
            // the curve id is the same as the first vertex
            m.vertex_attrs[return_edge_tuple.vid(m)].curve_id =
                m.vertex_attrs[op_cache.local().v1].curve_id;
            t_parameter =
                (m.vertex_attrs[op_cache.local().v1].t + m.vertex_attrs[op_cache.local().v2].t) /
                2.0;

            // !!! update t_parameter and check for periodicity + curretid!!!
        }
    }
    m.vertex_attrs[return_edge_tuple.vid(m)].pos = p;
    m.vertex_attrs[return_edge_tuple.vid(m)].t = t_parameter;
    m.vertex_attrs[return_edge_tuple.vid(m)].partition_id = m.cache.local().partition_id;
    m.vertex_attrs[return_edge_tuple.vid(m)].boundary_vertex =
        (m.vertex_attrs[op_cache.local().v1].boundary_vertex ||
         m.vertex_attrs[op_cache.local().v2].boundary_vertex);
    m.vertex_attrs[return_edge_tuple.vid(m)].fixed =
        (m.vertex_attrs[op_cache.local().v1].fixed || m.vertex_attrs[op_cache.local().v2].fixed);

    auto one_ring = m.get_one_ring_tris_for_vertex(return_edge_tuple);
    // check invariants here since get_area_accuracy_error_per_face requires valid triangle
    if (!m.invariants(one_ring)) return false;

    if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
        for (auto tri : one_ring) {
            auto one_ring_tri_error = m.get_area_accuracy_error_per_face(tri);
            if (one_ring_tri_error > m.mesh_parameters.m_accruacy_safeguard_ratio *
                                         m.mesh_parameters.m_accuracy_threshold)
                return false;
        }
    }
    return true;
}

bool AdaptiveTessellationPairedCollapseEdgeOperation::before(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    bool collapse_edge_success = collapse_edge.before(m, t);
    mirror_edge_tuple = m.face_attrs[t.fid(m)].mirror_edges[t.local_eid(m)];
    bool collapse_mirror_edge_success = true;
    if (mirror_edge_tuple.has_value()) {
        collapse_mirror_edge_success = collapse_mirror_edge.before(m, mirror_edge_tuple.value());
        if (!collapse_mirror_edge_success) return false;
    }
    return collapse_edge_success && collapse_mirror_edge_success;

    // if it is a seam edge and if either of the vertices are fixed
    // and if neither of the mirror edge two vertices are fixed then skip
    if (m.face_attrs[t.fid(m)].mirror_edges[t.local_eid(m)].has_value()) {
        mirror_edge_tuple = m.face_attrs[t.fid(m)].mirror_edges[t.local_eid(m)].value();
    }
    if (!wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
        return false;
    }
}

wmtk::TriMeshOperation::ExecuteReturnData AdaptiveTessellationPairedCollapseEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    size_t mirror_leid = -1;
    Tuple t_copy = t;
    if (mirror_edge_tuple.has_value()) {
        mirror_leid = mirror_edge_tuple.value().local_eid(m);
        if (m.face_attrs[mirror_edge_tuple.value().fid(m)].mirror_edges[mirror_leid].value().vid(
                m) != t.vid(m)) {
            t_copy = t.switch_vertex(m);
            // change the vid stored in op_cache to have the same orientation as t_copy
            collapse_edge.op_cache.local().v1 = t_copy.vid(m);
            collapse_edge.op_cache.local().v2 = t.vid(m);
            assert(collapse_edge.op_cache.local().v2 = t_copy.switch_vertex(m).vid(m));
        }
        assert(
            t_copy.vid(m) ==
            m.face_attrs[mirror_edge_tuple.value().fid(m)].mirror_edges[mirror_leid].value().vid(
                m));
    }

    wmtk::TriMeshOperation::ExecuteReturnData ret_data = collapse_edge.execute(m, t_copy);

    if (mirror_edge_tuple.has_value() && ret_data.success) {
        wmtk::TriMeshOperation::ExecuteReturnData mirror_ret_data =
            collapse_mirror_edge.execute(m, mirror_edge_tuple.value());
        ret_data.success &= mirror_ret_data.success;
        if (!ret_data.success) return ret_data;
        // add the mirror_ret_data.new_tris to the ret_data.new_tris
        for (auto& nt : mirror_ret_data.new_tris) ret_data.new_tris.emplace_back(nt);
    }
    // update influenced seam edges to correspond to the right triangle/vertex
    // TODO add assertions
    // TODO needs some assertions to check the premise
    // i guess one of check is the new_vid replace in-place the old vid in the tri_connectivity for
    // traingles influenced by the collapse

    // update the seam edges mirror_edges info
    if (mirror_edge_tuple.has_value()) {
        auto one_ring_edges = m.get_one_ring_edges_for_vertex(collapse_edge.return_edge_tuple);
        for (auto& e : one_ring_edges) {
            if (m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].has_value()) {
                // this is a seam edge
                // check if the mirror_edge data needs to be updated
                auto mirror_edge_tuple =
                    m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].value();
                if (mirror_edge_tuple.vid(m) == collapse_mirror_edge.op_cache.local().v1 ||
                    mirror_edge_tuple.vid(m) == collapse_mirror_edge.op_cache.local().v2) {
                    m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)] =
                        std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(
                            collapse_mirror_edge.return_edge_tuple.vid(m),
                            mirror_edge_tuple.local_eid(m),
                            mirror_edge_tuple.fid(m),
                            m));
                }
            }
        }

        auto mirror_one_ring_edges =
            m.get_one_ring_edges_for_vertex(collapse_mirror_edge.return_edge_tuple);
        for (auto& e : mirror_one_ring_edges) {
            if (m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].has_value()) {
                // this is a seam edge
                // check if the mirror_edge data needs to be updated
                auto primary_edge_tuple =
                    m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].value();
                if (primary_edge_tuple.vid(m) == collapse_edge.op_cache.local().v1 ||
                    primary_edge_tuple.vid(m) == collapse_edge.op_cache.local().v2) {
                    m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)] =
                        std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(
                            collapse_edge.return_edge_tuple.vid(m),
                            primary_edge_tuple.local_eid(m),
                            primary_edge_tuple.fid(m),
                            m));
                }
            }
        }
        // TODO add assertion for the above
        return ret_data;
    }
}

bool AdaptiveTessellationPairedCollapseEdgeOperation::after(
    AdaptiveTessellation& m,
    ExecuteReturnData& ret_data)
{
    assert(ret_data.success);
    // this is a seam edge. handle it here...
    // if (m.face_attrs[m.cache.local().operation_tuple.fid(m)]
    //         .mirror_edges[m.cache.local().operation_tuple.local_eid(m)]
    //         .has_value()) {
    //     auto mirror_old_edge = m.face_attrs[m.cache.local().operation_tuple.fid(m)]
    //                                .mirror_edges[m.cache.local().operation_tuple.local_eid(m)]
    //                                .value();
    //     // since collapse does not have a definition for mirror edge after the operation
    //     // we use a random traingle that is adjacent to the mirror vertex
    //     wmtk::TriMesh::Tuple mirror_new_vertex_tuple;
    //     for (auto tri : ret_data.new_tris) {
    //         if (tri.vid(m) != ret_data.tuple.vid(m)) {
    //             mirror_new_vertex_tuple = tri;
    //             break;
    //         }
    //     }

    //     // assert this is a seam edge
    //     assert(m.vertex_attrs[m.cache.local().v3].boundary_vertex);
    //     assert(m.vertex_attrs[m.cache.local().v4].boundary_vertex);
    //     // and assert the two vertices can't be fixed
    //     assert(!m.vertex_attrs[m.cache.local().v3].fixed);
    //     assert(!m.vertex_attrs[m.cache.local().v4].fixed);
    //     // and the mirror edge two vertices are also not fixed
    //     assert(m.cache.local().operation_tuple.fid(m) != ret_data.tuple.fid(m));
    //     assert(m.vertex_attrs[m.cache.local().v1].boundary_vertex);
    //     assert(m.vertex_attrs[m.cache.local().v2].boundary_vertex);
    //     assert(!m.vertex_attrs[m.cache.local().v1].fixed);
    //     assert(!m.vertex_attrs[m.cache.local().v2].fixed);

    //     m.vertex_attrs[ret_data.tuple.vid(m)].boundary_vertex = true;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].curve_id =
    //         m.vertex_attrs[m.cache.local().v3].curve_id;
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].curve_id =
    //         m.vertex_attrs[m.cache.local().v1].curve_id;
    //     // compute the mod_length
    //     double mod_length1 = m.mesh_parameters.m_boundary
    //                              .m_arclengths[m.vertex_attrs[ret_data.tuple.vid(m)].curve_id]
    //                              .back();
    //     double mod_length2 =
    //         m.mesh_parameters.m_boundary
    //             .m_arclengths[m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].curve_id]
    //             .back();
    //     assert(
    //         m.vertex_attrs[ret_data.tuple.vid(m)].curve_id ==
    //         m.vertex_attrs[m.cache.local().v3].curve_id);
    //     assert(
    //         m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].curve_id ==
    //         m.vertex_attrs[m.cache.local().v1].curve_id);
    //     // compare collapse to which one would give lower energy
    //     m.vertex_attrs[ret_data.tuple.vid(m)].pos = m.vertex_attrs[m.cache.local().v3].pos;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].t =
    //         std::fmod(m.vertex_attrs[m.cache.local().v3].t, mod_length1);
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].pos =
    //     m.vertex_attrs[m.cache.local().v1].pos;
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].t
    //     =
    //         std::fmod(m.vertex_attrs[m.cache.local().v1].t, mod_length2);
    //     auto energy1 = m.get_one_ring_energy(ret_data.tuple).first;
    //     energy1 += m.get_one_ring_energy(mirror_new_vertex_tuple).first;

    //     m.vertex_attrs[ret_data.tuple.vid(m)].pos = m.vertex_attrs[m.cache.local().v4].pos;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].t =
    //         std::fmod(m.vertex_attrs[m.cache.local().v4].t, mod_length1);
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].pos =
    //     m.vertex_attrs[m.cache.local().v2].pos;
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].t
    //     =
    //         std::fmod(m.vertex_attrs[m.cache.local().v2].t, mod_length2);
    //     auto energy2 = m.get_one_ring_energy(ret_data.tuple).first;
    //     energy2 += m.get_one_ring_energy(mirror_new_vertex_tuple).first;

    //     auto p = energy1 < energy2 ? m.vertex_attrs[m.cache.local().v3].pos
    //                                : m.vertex_attrs[m.cache.local().v4].pos;
    //     auto t_parameter = energy1 < energy2
    //                            ? std::fmod(m.vertex_attrs[m.cache.local().v3].t, mod_length1)
    //                            : std::fmod(m.vertex_attrs[m.cache.local().v4].t,
    //                            mod_length1);
    //     auto mirror_p = energy1 < energy2 ? m.vertex_attrs[m.cache.local().v1].pos
    //                                       : m.vertex_attrs[m.cache.local().v2].pos;
    //     auto mirror_t_parameter =
    //         energy1 < energy2 ? std::fmod(m.vertex_attrs[m.cache.local().v1].t, mod_length2)
    //                           : std::fmod(m.vertex_attrs[m.cache.local().v2].t, mod_length2);

    //     // update current edge
    //     m.vertex_attrs[ret_data.tuple.vid(m)].pos = p;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].t = t_parameter;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].partition_id = m.cache.local().partition_id;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].boundary_vertex = true;
    //     m.vertex_attrs[ret_data.tuple.vid(m)].curve_id =
    //         m.vertex_attrs[m.cache.local().v3].curve_id;
    //     // update mirror edge
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].pos = mirror_p;
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].t = mirror_t_parameter;
    //     // TODO is this thread safe?
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].partition_id =
    //     m.cache.local().partition_id;
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].boundary_vertex = true;
    //     m.vertex_attrs[mirror_new_vertex_tuple.vid(m)].curve_id =
    //         m.vertex_attrs[m.cache.local().v1].curve_id;

    //     // check invariants here since get_area_accuracy_error_per_face requires valid triangle
    //     if (!invariants(ret_data.new_tris)) {
    //         ret_data.success = false;
    //         return ret_data;
    //     }
    //     double current_error = 0.;
    //     for (auto tri : ret_data.new_tris) {
    //         auto one_ring_tri_error = get_area_accuracy_error_per_face(tri);
    //         if (one_ring_tri_error >
    //             mesh_parameters.m_accruacy_safeguard_ratio *
    //             mesh_parameters.m_accuracy_threshold) ret_data.success = false;
    //         return ret_data.success;
    //     }
    //     return ret_data.success;
    // }

    // size_t v1, v2;
    // if (wmtk::TriMeshEdgeCollapseOperation::after(m, ret_data)) {
    //     v1 = m.cache.local().v1;
    //     v2 = m.cache.local().v2;
    //     m.cache.local().v1 = m.cache.local().v3;
    //     m.cache.local().v2 = m.cache.local().v4;
    //     ret_data.success &= m.collapse_edge_after(ret_data.tuple);
    // }
    // auto mirror_edge_tuple =
    //     m.face_attrs[ret_data.tuple.fid(m)].mirror_edges[ret_data.tuple.local_eid(m)];
    // if (mirror_edge_tuple.has_value()) {
    //     m.cache.local().v1 = v1;
    //     m.cache.local().v2 = v2;
    //     ret_data.success &= m.collapse_edge_after(mirror_edge_tuple.value());
    // }
    return ret_data.success;
}

void AdaptiveTessellation::collapse_all_edges()
{
    // collapse is not define for EDGE_QUADRATURE
    // collapse in AREA_QUADRATURE uses 3d edge length

    for (auto f : get_faces()) assert(!is_inverted(f));
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_collapse", t);
    wmtk::logger().info("=======collapse==========");
    wmtk::logger().info("size for edges to be collapse is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            return -m.get_length3d(e); // m.mesh_parameters.m_get_length(e);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            auto length = m.get_length3d(tup);
            if (length != -weight) return false;

            if (length > (4. / 5. * m.mesh_parameters.m_quality_threshold)) return false;

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
bool AdaptiveTessellation::collapse_edge_before(const Tuple& edge_tuple)
{
    // check if the two vertices to be split is of the same curve_id
    if (vertex_attrs[edge_tuple.vid(*this)].curve_id !=
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].curve_id)
        return false;

    double length3d = mesh_parameters.m_get_length(edge_tuple);
    // enforce heuristic
    assert(length3d < 4. / 5. * mesh_parameters.m_quality_threshold);

    // record boundary vertex as boudnary_vertex in vertex attribute for accurate collapse
    // after boundary operations

    if (is_boundary_vertex(edge_tuple))
        vertex_attrs[edge_tuple.vid(*this)].boundary_vertex = true;
    else
        vertex_attrs[edge_tuple.vid(*this)].boundary_vertex = false;
    if (is_boundary_vertex(edge_tuple.switch_vertex(*this)))
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].boundary_vertex = true;
    else
        vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].boundary_vertex = false;
    if (mesh_parameters.m_bnd_freeze &&
        (vertex_attrs[edge_tuple.vid(*this)].boundary_vertex ||
         vertex_attrs[edge_tuple.switch_vertex(*this).vid(*this)].boundary_vertex))
        return false;
    cache.local().v1 = edge_tuple.vid(*this);
    cache.local().v2 = edge_tuple.switch_vertex(*this).vid(*this);
    cache.local().error = length3d;
    cache.local().partition_id = vertex_attrs[edge_tuple.vid(*this)].partition_id;
    // // get max_energy
    // cache.local().max_energy = get_quality(edge_tuple);
    // auto tris = get_one_ring_tris_for_vertex(edge_tuple);
    // for (auto tri : tris) {
    //     cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    // }
    // mesh_parameters.m_max_energy = cache.local().max_energy;
    return true;
}
bool AdaptiveTessellation::collapse_edge_after(const Tuple& edge_tuple)
{
    // check if the both of the 2 vertices are fixed
    // if yes, then collapse is rejected
    if (vertex_attrs[cache.local().v1].fixed && vertex_attrs[cache.local().v2].fixed) return false;

    // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
    double length3d = cache.local().error;
    // enforce heuristic
    if (length3d >= (4. / 5. * mesh_parameters.m_quality_threshold)) {
        return false;
    }

    auto vid = edge_tuple.vid(*this);
    Eigen::Vector2d p;
    double t_parameter;
    double mod_length = 0;
    // mesh_parameters.m_boundary.m_arclengths[vertex_attrs[edge_tuple.vid(*this)].curve_id]
    //    .back();
    if (vertex_attrs[cache.local().v1].fixed) {
        // assert that the old edge is not a seam edge since it would have been prevented by
        // the before
        // assert(!vertex_attrs[cache.local().operation_tuple]
        //             .mirror_edges[cache.local().operation_tuple.local_eid(*this)]
        //             .has_value());
        p = vertex_attrs[cache.local().v1].pos;
        t_parameter = std::fmod(vertex_attrs[cache.local().v1].t, mod_length);
    } else if (vertex_attrs[cache.local().v2].fixed) {
        // assert(!vertex_attrs[cache.local().operation_tuple]
        //             .mirror_edges[cache.local().operation_tuple.local_eid(*this)]
        //             .has_value());
        p = vertex_attrs[cache.local().v2].pos;
        t_parameter = std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
    } else {
        assert(!vertex_attrs[cache.local().v1].fixed);
        assert(!vertex_attrs[cache.local().v2].fixed);
        // this is the same case as both are not boundary

        if (vertex_attrs[cache.local().v1].boundary_vertex &&
            vertex_attrs[cache.local().v2].boundary_vertex) {
            vertex_attrs[vid].boundary_vertex = true;
            vertex_attrs[vid].curve_id = vertex_attrs[cache.local().v1].curve_id;
            // compare collapse to which one would give lower energy
            vertex_attrs[vid].pos = vertex_attrs[cache.local().v1].pos;
            vertex_attrs[vid].t = std::fmod(vertex_attrs[cache.local().v1].t, mod_length);
            auto energy1 = get_one_ring_energy(edge_tuple).first;
            vertex_attrs[vid].pos = vertex_attrs[cache.local().v2].pos;
            vertex_attrs[vid].t = std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
            auto energy2 = get_one_ring_energy(edge_tuple).first;
            p = energy1 < energy2 ? vertex_attrs[cache.local().v1].pos
                                  : vertex_attrs[cache.local().v2].pos;
            t_parameter = energy1 < energy2
                              ? std::fmod(vertex_attrs[cache.local().v1].t, mod_length)
                              : std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
        } else if (vertex_attrs[cache.local().v1].boundary_vertex) {
            p = vertex_attrs[cache.local().v1].pos;
            t_parameter = std::fmod(vertex_attrs[cache.local().v1].t, mod_length);
        } else if (vertex_attrs[cache.local().v2].boundary_vertex) {
            p = vertex_attrs[cache.local().v2].pos;
            t_parameter = std::fmod(vertex_attrs[cache.local().v2].t, mod_length);
        } else {
            assert(!vertex_attrs[cache.local().v1].boundary_vertex);
            assert(!vertex_attrs[cache.local().v2].boundary_vertex);
            p = (vertex_attrs[cache.local().v1].pos + vertex_attrs[cache.local().v2].pos) / 2.0;
            t_parameter = std::fmod(
                (vertex_attrs[cache.local().v1].t + vertex_attrs[cache.local().v2].t) / 2.0,
                mod_length);
            // !!! update t_parameter and check for periodicity + curvid !!!
        }
    }
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].t = t_parameter;
    vertex_attrs[vid].partition_id = cache.local().partition_id;
    vertex_attrs[vid].boundary_vertex =
        (vertex_attrs[cache.local().v1].boundary_vertex ||
         vertex_attrs[cache.local().v2].boundary_vertex);
    vertex_attrs[vid].fixed =
        (vertex_attrs[cache.local().v1].fixed || vertex_attrs[cache.local().v2].fixed);
    vertex_attrs[vid].curve_id = vertex_attrs[cache.local().v1].curve_id;

    auto one_ring = get_one_ring_tris_for_vertex(edge_tuple);
    // check invariants here since get_area_accuracy_error_per_face requires valid triangle
    if (!invariants(one_ring)) return false;
    double current_error = 0.;
    for (auto tri : one_ring) {
        auto one_ring_tri_error = get_area_accuracy_error_per_face(tri);
        if (one_ring_tri_error >
            mesh_parameters.m_accruacy_safeguard_ratio * mesh_parameters.m_accuracy_threshold)
            return false;
    }
    return true;
}
