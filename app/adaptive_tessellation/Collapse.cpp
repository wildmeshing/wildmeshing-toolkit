#include "Collapse.h"
#include "PairUtils.hpp"
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
    return after(m);
}
bool AdaptiveTessellationCollapseEdgeOperation::after(AdaptiveTessellation& m)
{
    const auto ret_tup = get_return_tuple_opt();
    if (!bool(ret_tup)) {
        return false;
    }
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

bool AdaptiveTessellationPairedCollapseEdgeOperation::input_edge_is_mirror() const
{
    auto& op_cache = m_op_cache.local();
    return op_cache.mirror_edge_tuple_opt.has_value();
}

AdaptiveTessellationPairedCollapseEdgeOperation::operator bool() const
{
    return operation_success_T(collapse_edge, collapse_mirror_edge, input_edge_is_mirror());
}
void AdaptiveTessellationPairedCollapseEdgeOperation::set_input_edge_mirror(
    const AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& op_cache = m_op_cache.local();
    if (m.get_mirror_edge_opt(t)) {
        op_cache.mirror_edge_tuple_opt = m.get_mirror_edge_opt(t);
    }
}


bool AdaptiveTessellationPairedCollapseEdgeOperation::before(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& op_cache = m_op_cache.local();
    if (!collapse_edge.before(m, t)) {
        return false;
    }

    set_input_edge_mirror(m, t);

    if (input_edge_is_mirror()) {
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        if (!collapse_mirror_edge.before(m, mirror_edge_tuple)) {
            return false;
        }
    }

    store_boundary_data(m, t);

    return true;
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
    if (input_edge_is_mirror()) {
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        wmtk::TriMeshOperation::ExecuteReturnData ret_data2 =
            collapse_mirror_edge.execute(m, mirror_edge_tuple.switch_vertex(m));
    }

    ret_data.success = bool(*this);
    ret_data.new_tris = modified_tuples(m);

    return ret_data;
}


auto AdaptiveTessellationPairedCollapseEdgeOperation::modified_tuples(
    const AdaptiveTessellation& m) const -> std::vector<Tuple>
{
    modified_tuples_T(m, collapse_edge, collapse_mirror_edge);
}

bool AdaptiveTessellationPairedCollapseEdgeOperation::after(
    AdaptiveTessellation& m,
    ExecuteReturnData& ret_data)
{
    assert(ret_data.success);
    return after(m);
}
bool AdaptiveTessellationPairedCollapseEdgeOperation::after(AdaptiveTessellation& m)
{
    auto& op_cache = m_op_cache.local();
    const Tuple& edge_tuple = collapse_edge.get_return_tuple_opt().value();
    if (!collapse_edge.after(m)) {
        return false;
    }


    if (input_edge_is_mirror()) {
        const Tuple& mirror_edge_tuple = collapse_mirror_edge.get_return_tuple_opt().value();
        if (!collapse_mirror_edge.after(m)) {
            return false;
        }
    }

    rebuild_boundary_data(m);
    return true;
}

void AdaptiveTessellationPairedCollapseEdgeOperation::mark_failed()
{
    collapse_edge.mark_failed();
    collapse_mirror_edge.mark_failed();
}

auto AdaptiveTessellationPairedCollapseEdgeOperation::get_mirror_edge_tuple_opt() const
    -> std::optional<Tuple>
{
    // note that curveid are handled by the individual operations
    auto& op_cache = m_op_cache.local();

    return op_cache.mirror_edge_tuple_opt;
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


void AdaptiveTessellationPairedCollapseEdgeOperation::store_boundary_data(
    const AdaptiveTessellation& m,
    const Tuple& t)
{
    auto& collapse_op_cache = collapse_edge.m_op_cache.local();
    auto& mirror_collapse_op_cache = collapse_mirror_edge.m_op_cache.local();
    OpCache& op_cache = m_op_cache.local();
}
void AdaptiveTessellationPairedCollapseEdgeOperation::rebuild_boundary_data(AdaptiveTessellation& m)
{
    auto& collapse_op_cache = collapse_edge.m_op_cache.local();
    OpCache& op_cache = m_op_cache.local();

    assert(bool(collapse_edge));
    const TriMeshTuple return_tuple = collapse_edge.get_return_tuple_opt().value();

    // stich mirror data
    if (input_edge_is_mirror()) {
        const int ov1 = collapse_op_cache.v1;
        const int ov2 = collapse_op_cache.v2;
        auto& mirror_collapse_op_cache = collapse_mirror_edge.m_op_cache.local();
        const int mv1 = mirror_collapse_op_cache.v1;
        const int mv2 = mirror_collapse_op_cache.v2;


        // ov1 ~ mv2, mv2 ~ ov1
    }
}
