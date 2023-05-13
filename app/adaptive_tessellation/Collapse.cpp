#include "Collapse.h"
using namespace wmtk;
using namespace adaptive_tessellation;

// every edge is collapsed, if it is shorter than 3/4 L

namespace {
constexpr static size_t dummy = std::numeric_limits<size_t>::max();
}

namespace {
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
} // namespace


bool AdaptiveTessellationCollapseEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    m_op_cache.local() = {};
    if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
        // TODO check link conditions
        OpCache& op_cache = m_op_cache.local();
        // check if the two vertices to be split is of the same curve_id
        const size_t my_vid = t.vid(m);
        const Tuple other_tuple = t.switch_vertex(m);
        const size_t other_vid = other_tuple.vid(m);
        auto& my_vattr = m.vertex_attrs[my_vid];
        auto& other_vattr = m.vertex_attrs[other_vid];


        // check these aren't hte same curve
        if (my_vattr.curve_id != other_vattr.curve_id) return false;


        if (!m.mesh_parameters.m_ignore_embedding) {
            const double& length_3d = op_cache.length3d = m.mesh_parameters.m_get_length(t);
            // enforce heuristic
            assert(length_3d < 4. / 5. * m.mesh_parameters.m_quality_threshold);
        }
        // record boundary vertex as boudnary_vertex in vertex attribute for accurate collapse
        // after boundary operations

        // record if the two vertices of the edge is boundary vertex
        my_vattr.boundary_vertex = m.is_boundary_vertex(t);
        other_vattr.boundary_vertex = m.is_boundary_vertex(other_tuple);

        if (m.mesh_parameters.m_bnd_freeze &&
            (my_vattr.boundary_vertex || other_vattr.boundary_vertex))
            return false;

        // record the two vertices vids to the operation cache
        op_cache.v1 = my_vid;
        op_cache.v2 = other_vid;
        m.cache.local().partition_id = my_vattr.partition_id;
        return true;
    }
    return false;
}


auto AdaptiveTessellationCollapseEdgeOperation::seamed_links_of_vertex(
    AdaptiveTessellation& mesh,
    const Tuple& vertex) -> LinksOfVertex
{
    LinksOfVertex ret;
    std::vector<TriMesh::Tuple> all_mirror_vertices = mesh.get_all_mirror_vertices(vertex);
    for (const TriMesh::Tuple& vtup : all_mirror_vertices) {
        const auto& links = TriMeshEdgeCollapseOperation::links_of_vertex(mesh, vtup);
        ret.vertex.insert(ret.vertex.end(), links.vertex.begin(), links.vertex.end());
        ret.edge.insert(ret.edge.end(), links.edge.begin(), links.edge.end());
    }

    return ret;
}
std::vector<size_t> AdaptiveTessellationCollapseEdgeOperation::seamed_edge_link_of_edge(
    AdaptiveTessellation& mesh,
    const Tuple& edge)
{
    auto get_opposing_vertex_vid = [&mesh](const TriMeshTuple& t) -> size_t {
        return t.switch_edge(mesh).switch_vertex(mesh).vid(mesh);
    };
    std::vector<size_t> lk_edge;
    lk_edge.push_back(get_opposing_vertex_vid(edge));
    const std::optional<Tuple> other_face_opt = mesh.get_sibling_edge_opt(edge);
    if (!other_face_opt.has_value()) {
        lk_edge.push_back(dummy);
    } else {
        lk_edge.push_back(get_opposing_vertex_vid(other_face_opt.value()));
    }
    vector_sort(lk_edge);
    return lk_edge;
}

bool AdaptiveTessellationCollapseEdgeOperation::check_seamed_link_condition(
    AdaptiveTessellation& mesh,
    const Tuple& edge)
{
    assert(edge.is_valid(mesh));
    // the edge initially points at the first of two vertex links we are computing
    const LinksOfVertex v1 = seamed_links_of_vertex(mesh, edge);
    const LinksOfVertex v2 = seamed_links_of_vertex(mesh, edge.switch_vertex(mesh));

    // compute vertex link condition
    auto lk_vid12 = set_intersection(v1.vertex, v2.vertex);
    bool v_link = lk_vid12 == seamed_edge_link_of_edge(mesh, edge);

    // check edge link condition
    // in 2d edge link for an edge is always empty

    std::vector<std::pair<size_t, size_t>> res;
    const auto& lk_e_vid1 = v1.edge;
    const auto& lk_e_vid2 = v2.edge;
    std::set_intersection(
        lk_e_vid1.begin(),
        lk_e_vid1.end(),
        lk_e_vid2.begin(),
        lk_e_vid2.end(),
        std::back_inserter(res));
    const bool e_link = res.empty();
    return v_link && e_link;
}

TriMeshOperation::ExecuteReturnData AdaptiveTessellationCollapseEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    OpCache& op_cache = m_op_cache.local();
    assert(m.check_mesh_connectivity_validity());
    TriMeshOperation::ExecuteReturnData ret_data = TriMeshEdgeCollapseOperation::execute(m, t);
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
    OpCache& op_cache = m_op_cache.local();
    const Tuple& return_edge_tuple = get_return_tuple_opt().value();

    const auto& v1_attr = m.vertex_attrs[op_cache.v1];
    const auto& v2_attr = m.vertex_attrs[op_cache.v2];

    const bool v1_is_fixed = v1_attr.fixed;
    const bool v2_is_fixed = v2_attr.fixed;
    // For now letes reject any condition where the vertices are fixed
    if (v1_is_fixed || v2_is_fixed) return false;

    // check if the both of the 2 vertices are fixed
    // if yes, then collapse is rejected
    if (v1_is_fixed && v2_is_fixed) return false;

    // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
    // enforce heuristic
    assert(op_cache.length3d < 4. / 5. * m.mesh_parameters.m_quality_threshold);

    const size_t return_vid = return_edge_tuple.vid(m);
    auto& return_v_attr = m.vertex_attrs[return_vid];

    auto assign_attr = [&](const auto& attr) {
        return_v_attr.pos = attr.pos;
        return_v_attr.t = attr.t;
    };

    auto assign_v1_attr = [&]() { assign_attr(v1_attr); };
    auto assign_v2_attr = [&]() { assign_attr(v2_attr); };
    if (v1_attr.fixed) {
        assert(!v2_attr.fixed);

        assign_v1_attr();
    } else if (v2_attr.fixed) {
        assert(!v1_attr.fixed);
        assign_v2_attr();
    } else {
        assert(!v1_attr.fixed);
        assert(!v2_attr.fixed);
        if (v1_attr.boundary_vertex && v2_attr.boundary_vertex) {
            auto try_energy = [&](const auto& attr) -> double {
                assign_attr(attr);
                return m.get_one_ring_energy(return_edge_tuple).first;
            };
            // compare collapse to which one would give lower energy
            double energy1 = try_energy(v1_attr);
            double energy2 = try_energy(v2_attr);

            if (energy1 < energy2) {
                assign_v1_attr();
            } else {
                assign_v2_attr();
            }
        } else if (v1_attr.boundary_vertex) {
            assign_v1_attr();
        } else if (v2_attr.boundary_vertex) {
            assign_v2_attr();
        } else {
            assert(!v1_attr.boundary_vertex);
            assert(!v2_attr.boundary_vertex);
            return_v_attr.pos = (v1_attr.pos + v2_attr.pos) / 2.0;
            return_v_attr.t = (v1_attr.t + v2_attr.t) / 2.0;

            // !!! update t_parameter and check for periodicity + curretid!!!
        }
    }
    return_v_attr.partition_id = m.cache.local().partition_id;
    return_v_attr.boundary_vertex = (v1_attr.boundary_vertex || v2_attr.boundary_vertex);
    return_v_attr.fixed = (v1_attr.fixed || v2_attr.fixed);

    auto one_ring = m.get_one_ring_tris_for_vertex(return_edge_tuple);
    // check invariants here since get_area_accuracy_error_per_face requires valid triangle
    if (!m.invariants(one_ring)) return false;

    if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
        for (const Tuple& tri : one_ring) {
            double one_ring_tri_error = m.get_area_accuracy_error_per_face(tri);
            if (one_ring_tri_error > m.mesh_parameters.m_accruacy_safeguard_ratio *
                                         m.mesh_parameters.m_accuracy_threshold)
                return false;
        }
    }
    return true;
}

AdaptiveTessellationPairedCollapseEdgeOperation::operator bool()
{
    auto& op_cache = m_op_cache.local();
    const std::optional<Tuple> normal_ret_opt = collapse_edge.get_return_tuple_opt();

    const bool normal_ok = normal_ret_opt.has_value();


    bool mirror_ok = true;
    if (op_cache.mirror_edge_tuple_opt.has_value()) {
        const std::optional<Tuple> mirror_ret_opt = collapse_mirror_edge.get_return_tuple_opt();
        mirror_ok = mirror_ret_opt.has_value();
    }
    return normal_ok && mirror_ok;
}

bool AdaptiveTessellationPairedCollapseEdgeOperation::before(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    bool collapse_edge_success = collapse_edge.before(m, t);
    auto& op_cache = m_op_cache.local();
    auto& mirror_edge_tuple_opt = op_cache.mirror_edge_tuple_opt;
    mirror_edge_tuple_opt = m.face_attrs[t.fid(m)].mirror_edges[t.local_eid(m)];
    bool collapse_mirror_edge_success = true;
    if (mirror_edge_tuple_opt.has_value()) {
        const Tuple& mirror_edge_tuple = mirror_edge_tuple_opt.value();
        collapse_mirror_edge_success = collapse_mirror_edge.before(m, mirror_edge_tuple);
        if (!collapse_mirror_edge_success) return false;
    }
    return collapse_edge_success && collapse_mirror_edge_success;
}

wmtk::TriMeshOperation::ExecuteReturnData AdaptiveTessellationPairedCollapseEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& collapse_edge_cache = collapse_edge.m_op_cache.local();
    auto& mirror_edge_cache = collapse_mirror_edge.m_op_cache.local();
    auto& op_cache = m_op_cache.local();


    wmtk::TriMeshOperation::ExecuteReturnData ret_data = collapse_edge.execute(m, t);
    if (!ret_data) return ret_data;
    // if we have a mirror edge we need to
    if (op_cache.mirror_edge_tuple_opt.has_value()) {
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        collapse_mirror_edge.execute(m, mirror_edge_tuple.switch_vertex(m));
    }

    // update influenced seam edges to correspond to the right triangle/vertex
    // TODO add assertions
    // TODO needs some assertions to check the premise
    // i guess one of check is the new_vid replace in-place the old vid in the tri_connectivity for
    // traingles influenced by the collapse

    // TODO check if the one_ring_edges is assuming e.vid(m) ==
    // collapse_edge.return_edge_tuple.vid(m) update the seam edges mirror_edges info
    // if (op_cache.mirror_edge_tuple_opt.has_value()) {
    //    auto one_ring_edges =
    //        m.get_one_ring_edges_for_vertex(collapse_edge.return_edge_tuple);
    //    for (auto& e : one_ring_edges) {
    //        if (m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].has_value()) {
    //            // this is a seam edge
    //            // check if the mirror_edge data needs to be updated
    //            // auto mirror_edge_tuple =
    //            //     m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].value();
    //            // if (mirror_edge_tuple.vid(m) == mirror_edge_cache.v1 ||
    //            //     mirror_edge_tuple.vid(m) == mirror_edge_cache.v2) {
    //            //     m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)] =
    //            //         std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(
    //            //             mirror_edge_cache.return_edge_tuple.vid(m),
    //            //             mirror_edge_tuple.local_eid(m),
    //            //             mirror_edge_tuple.fid(m),
    //            //             m));
    //        }
    //    }
    //}

    //// TODO same thing. check the one_ring_edges direction
    // auto mirror_one_ring_edges =
    //     m.get_one_ring_edges_for_vertex(mirror_edge_cache.return_edge_tuple);
    // for (auto& e : mirror_one_ring_edges) {
    //     if (m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].has_value()) {
    //        // this is a seam edge
    //        // check if the mirror_edge data needs to be updated
    //        auto primary_edge_tuple = m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)].value();
    //        if (primary_edge_tuple.vid(m) == collapse_edge_cache.v1 ||
    //            primary_edge_tuple.vid(m) == collapse_edge_cache.v2) {
    //            m.face_attrs[e.fid(m)].mirror_edges[e.local_eid(m)] =
    //                std::make_optional<wmtk::TriMesh::Tuple>(wmtk::TriMesh::Tuple(
    //                    collapse_edge_cache.return_edge_tuple.vid(m),
    //                    primary_edge_tuple.local_eid(m),
    //                    primary_edge_tuple.fid(m),
    //                    m));
    //        }
    //    }
    //}
    //// TODO add assertion for the above
    // return ret_data;
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

    double length3d = get_length3d(edge_tuple);
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
