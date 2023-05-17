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

void AdaptiveTessellationCollapseEdgeOperation::store_merged_seam_data(
    const AdaptiveTessellation& m,
    const Tuple& edge_tuple)
{
    OpCache& op_cache = m_op_cache.local();
    auto& seam_data_map = op_cache.new_vertex_seam_data;
    const size_t v1 = op_cache.v1;
    const size_t v2 = op_cache.v2;
    assert(v1 != v2);

    auto store_edge = [&](const Tuple& edge_tuple) {
        if (m.is_seam_edge(edge_tuple)) {
            seam_data_map[edge_tuple.switch_vertex(m).vid(m)] = SeamData{
                .mirror_edge_tuple = m.get_mirror_edge_opt(edge_tuple),
                .curve_id = m.get_edge_attrs(edge_tuple).curve_id.value()};
        }
    };

    for (const Tuple& e : m.get_one_ring_edges_for_vertex(edge_tuple)) {
        store_edge(e);
    }
    for (const Tuple& e : m.get_one_ring_edges_for_vertex(edge_tuple.switch_vertex(m))) {
        store_edge(e);
    }
}

auto AdaptiveTessellationCollapseEdgeOperation::get_constrained_boundary_type(
    const AdaptiveTessellation& m,
    const Tuple& t) const -> ConstrainedBoundaryType
{
    ConstrainedBoundaryType primary = get_constrained_boundary_type_per_face(m, t);
    if (const auto oface_opt = t.switch_face(m); oface_opt.has_value()) {
        ConstrainedBoundaryType secondary =
            get_constrained_boundary_type_per_face(m, oface_opt.value());

        // constraint is implemented as a bitmask so or hte bitmasks to get maximal constraintness
        return static_cast<ConstrainedBoundaryType>(
            static_cast<char>(primary) | static_cast<char>(secondary));
    } else {
        return primary;
    }
}
auto AdaptiveTessellationCollapseEdgeOperation::get_constrained_boundary_type_per_face(
    const AdaptiveTessellation& m,
    const Tuple& t) const -> ConstrainedBoundaryType
{
    const Tuple other_tuple = t.switch_vertex(m);


    auto vertex_is_constrained = [&](const Tuple& v) -> bool {
        return m.get_vertex_attrs(v).fixed;
    };
    auto edge_is_constrained = [&](const Tuple& e) -> bool { return m.is_boundary_edge(e); };

    // t must be an edge tuple  pointing at the desired vertex on a particular face
    auto side_is_constrained = [&](const Tuple& edge) -> bool {
        const bool this_side_constrained =
            edge_is_constrained(edge.switch_edge(m)) || vertex_is_constrained(edge);

        auto edge2_opt = edge.switch_face(m);
        if (edge2_opt) {
            const Tuple& edge2 = edge2_opt.value();
            const bool other_edge_constrained = edge_is_constrained(edge2.switch_edge(m));
            return other_edge_constrained || this_side_constrained;
        } else {
            return this_side_constrained;
        }
    };


    const bool ts = side_is_constrained(t);


    const bool os = side_is_constrained(other_tuple);


    if (ts) {
        if (os) {
            return ConstrainedBoundaryType::BothConstrained;

        } else {
            return ConstrainedBoundaryType::TupleSideConstrained;
        }
    } else {
        if (os) {
            return ConstrainedBoundaryType::OtherSideConstrained;
        } else {
            return ConstrainedBoundaryType::NoConstraints;
        }
    }
    return ConstrainedBoundaryType::NoConstraints;
}

void AdaptiveTessellationCollapseEdgeOperation::fill_cache(
    const AdaptiveTessellation& m,
    const Tuple& t)
{
    OpCache& op_cache = m_op_cache.local();
    // check if the two vertices to be split is of the same curve_id
    const size_t my_vid = t.vid(m);
    const Tuple other_tuple = t.switch_vertex(m);
    const size_t other_vid = other_tuple.vid(m);
    auto& my_vattr = m.vertex_attrs[my_vid];
    auto& other_vattr = m.vertex_attrs[other_vid];


    if (!m.mesh_parameters.m_ignore_embedding) {
        const double& length_3d = op_cache.length3d = m.mesh_parameters.m_get_length(t);
        // adding heuristic decision. If length2 < 4. / 5. * 4. / 5. * m.m_target_l * m.m_target_l always collapse
        // enforce heuristic
        assert(length_3d < 4. / 5. * m.mesh_parameters.m_quality_threshold);
    }

    // record the two vertices vids to the operation cache
    op_cache.v1 = my_vid;
    op_cache.v2 = other_vid;
    op_cache.partition_id = my_vattr.partition_id;

    op_cache.constrained_boundary_type = get_constrained_boundary_type(m, t);

    store_merged_seam_data(m, t);
}
bool AdaptiveTessellationCollapseEdgeOperation::check_edge_mergeability(
    const AdaptiveTessellation& m,
    const Tuple& edge) const
{
    // opposing edges are mergeable if only one of them is boundary
    auto mergeable = [&](const Tuple& e) -> bool {
        const Tuple e0 = edge.switch_edge(m);
        const Tuple e1 = e0.switch_vertex(m).switch_edge(m);

        const bool e0_is_boundary = m.is_boundary_edge(e0);
        const bool e1_is_boundary = m.is_boundary_edge(e1);
        if (e0_is_boundary && e1_is_boundary) {
            return false;
        }
        return true;
    };

    // alternate impl that was used for debug
#if defined(LET_TRIMESH_NONMANIFOLD)
    // lazy code that uses VIDs to check other edges
    auto edge_to_vids = [&](const Tuple& e) -> std::array<size_t, 2> {
        const size_t v0 = e.vid(m);
        const size_t v1 = e.switch_vertex().vid(m);
        if (v0 > v1) {
            std::swap(v0, v1);
        }
        return std::array<size_t, 2>{{v0, v1}};
    };
    const auto my_vids = edge_to_vids(edge);
    for (const auto& tri : tris_bounded_by_edge(edge)) {
        Tuple e = tri;
        size_t attempt = 0;
        for (; edge_to_vids(e) != my_vids && attempt < 3; ++attempt) {
            e.switch_edge(m).switch_vertex(m);
        }
        assert(attempt < 3);
        if (!mergeable(e)) {
            return false;
        }
    }
#else

    // check this edge (and potentially the other face across the boundary)
    if (!mergeable(edge)) {
        return false;
    }

    if (const std::optional<Tuple> other_face_opt = edge.switch_face(m);
        other_face_opt.has_value()) {
        if (!mergeable(other_face_opt.value())) {
            return false;
        }
    }
#endif
    return true;
}
bool AdaptiveTessellationCollapseEdgeOperation::check_vertex_mergeability(
    const AdaptiveTessellation& m,
    const Tuple& t) const
{
    const auto& v1_attr = m.get_vertex_attrs(t);
    const auto& v2_attr = m.get_vertex_attrs(t.switch_vertex(m));


    const bool v1_is_fixed = v1_attr.fixed;
    const bool v2_is_fixed = v2_attr.fixed;
    // For now letes reject any condition where the vertices are fixed
    if (v1_is_fixed || v2_is_fixed) return false;

    // check if the both of the 2 vertices are fixed
    // if yes, then collapse is rejected
    if (v1_is_fixed && v2_is_fixed) return false;
    if (m.mesh_parameters.m_bnd_freeze && (v1_attr.boundary_vertex || v2_attr.boundary_vertex)) {
        return false;
    }
    return true;
}

bool AdaptiveTessellationCollapseEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    m_op_cache.local() = {};
    if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
        check_vertex_mergeability(m, t);
        // TODO: currently edge mergeability just checks for double boundaries
        // which is caught by link condition, is there something else?
        //check_edge_mergeability(m, t);

        // record boundary vertex as boudnary_vertex in vertex attribute for accurate collapse
        // after boundary operations

        const size_t my_vid = t.vid(m);
        const Tuple other_tuple = t.switch_vertex(m);
        const size_t other_vid = other_tuple.vid(m);
        auto& my_vattr = m.vertex_attrs[my_vid];
        auto& other_vattr = m.vertex_attrs[other_vid];
        // record if the two vertices of the edge is boundary vertex
        my_vattr.boundary_vertex = m.is_boundary_vertex(t);
        other_vattr.boundary_vertex = m.is_boundary_vertex(other_tuple);


        fill_cache(m, t);
        return false;
    }
    return true;
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


    return true;
}

auto AdaptiveTessellationCollapseEdgeOperation::assign_new_vertex_attributes(
    AdaptiveTessellation& m,
    const VertexAttributes& attr) const -> const VertexAttributes&
{
    return m.get_vertex_attrs(get_return_tuple_opt().value()) = attr;
}

auto AdaptiveTessellationCollapseEdgeOperation::assign_new_vertex_attributes(
    AdaptiveTessellation& m) const -> const VertexAttributes&
{
    assert(bool(*this));
    const auto return_edge_tuple = get_return_tuple_opt().value();
    const size_t return_vid = return_edge_tuple.vid(m);
    auto& return_v_attr = m.vertex_attrs[return_vid];

    auto assign_attr = [&](const auto& attr) {
        return_v_attr.pos = attr.pos;
        return_v_attr.t = attr.t;
    };
    const OpCache& op_cache = m_op_cache.local();
    const auto& v1_attr = m.vertex_attrs[op_cache.v1];
    const auto& v2_attr = m.vertex_attrs[op_cache.v2];

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
    return_v_attr.partition_id = op_cache.partition_id;
    return_v_attr.boundary_vertex = (v1_attr.boundary_vertex || v2_attr.boundary_vertex);
    return_v_attr.fixed = (v1_attr.fixed || v2_attr.fixed);
    return return_v_attr;
}

void AdaptiveTessellationCollapseEdgeOperation::assign_collapsed_edge_attributes(
    AdaptiveTessellation& m) const
{
    auto& tri_connectivity = this->tri_connectivity(m);
    auto nt_opt = new_vertex(m);
    assert(nt_opt.has_value());
    const Tuple& new_vertex_tuple = nt_opt.value();
    const size_t new_vertex_vid = new_vertex_tuple.vid(m);
    auto& op_cache = m_op_cache.local();
    const std::vector<Tuple> possible_tris = m.get_one_ring_tris_for_vertex(new_vertex_tuple);
    // TODO: cache tris that have already been used
    for (const auto& [other_vertex, seam_data] : op_cache.new_vertex_seam_data) {
        for (const Tuple t : possible_tris) {
            const size_t fid = t.fid(m);
            int other_index = tri_connectivity[fid].find(other_vertex);
            if (other_index == -1) {
                continue;
            }
            // this tuple is the triangle of this vertex's vid
            int new_vid_index = tri_connectivity[fid].find(new_vertex_vid);
            assert(new_vid_index != -1);
            assert(new_vid_index != other_index);
            assert(new_vid_index >= 0);
            assert(new_vid_index < 3);
            assert(other_index >= 0);
            assert(other_index < 3);
            // 0 + 1 + 2 == 3
            int edge_index = 3 - other_index - new_vid_index;
            // TODO: do this in a more tuple-navigation like way
            Tuple edge_tuple(t.fid(m), edge_index, new_vid_index, m);

            assert(edge_tuple.switch_vertex(m).vid(m) == other_index);

            m.edge_attrs[t.eid(m)].curve_id = seam_data.curve_id;

            m.face_attrs[t.fid(m)].mirror_edges[edge_tuple.local_eid(m)] =
                seam_data.mirror_edge_tuple;
        }
    }
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
    auto one_ring = m.get_one_ring_tris_for_vertex(edge_tuple);
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
