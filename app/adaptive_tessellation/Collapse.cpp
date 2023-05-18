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
    auto& vertex_seam_data_map = op_cache.new_vertex_seam_data;
    auto& edge_seam_data_map = op_cache.opposing_edge_seam_data;

    const size_t v1 = op_cache.v1;
    const size_t v2 = op_cache.v2;
    assert(v1 != v2);

    // store edges that will be radial from the new vertex
    // assumes the vid is not one of the input edge vids
    // edge tuple pointing from the radial vertex to the center vertex
    auto store_new_vertex_edge = [&](const Tuple& edge_tuple) {
        assert(edge_tuple.vid(m) != v1 && edge_tuple.vid(m) != v2);
        ;
        if (!m.is_boundary_edge(edge_tuple)) {
            return;
        }
        std::optional<std::array<size_t, 2>> mirror_edge_vids_opt;
        if (m.is_seam_edge(edge_tuple)) {
            const Tuple mt = m.get_oriented_mirror_edge(edge_tuple);
            const size_t mirror_vid = mt.switch_vertex(m).vid(m);
            mirror_edge_vids_opt = std::array<size_t, 2>{{mirror_vid, mirror_vid}};
        }

        const auto curveid_opt = m.get_edge_attrs(edge_tuple).curve_id;
        assert(curveid_opt.has_value());

        vertex_seam_data_map[edge_tuple.vid(m)] =
            SeamData{.mirror_edge_vids = mirror_edge_vids_opt, .curve_id = curveid_opt.value()};
    };
    // takes in a edge that does not includ either input edge vids
    auto store_opposing_edge = [&](const Tuple& edge_tuple) {
        if (!m.is_boundary_edge(edge_tuple)) {
            return;
        }
        std::optional<std::array<size_t, 2>> mirror_edge_vids_opt;
        if (m.is_seam_edge(edge_tuple)) {
            const auto meopt = m.get_mirror_edge_opt(edge_tuple);
            assert(meopt.has_value());
            const Tuple& mt = meopt.value();
            const size_t mirror_vid0 = mt.vid(m);
            const size_t mirror_vid1 = mt.switch_vertex(m).vid(m);
            mirror_edge_vids_opt = std::array<size_t, 2>{{mirror_vid0, mirror_vid1}};
        }
        size_t v0 = edge_tuple.vid(m);
        size_t v1 = edge_tuple.switch_vertex(m).vid(m);

        const auto curveid_opt = m.get_edge_attrs(edge_tuple).curve_id;
        assert(curveid_opt.has_value());
        edge_seam_data_map[std::array<size_t, 2>{{v0, v1}}] =
            SeamData{.mirror_edge_vids = mirror_edge_vids_opt, .curve_id = curveid_opt.value()};
    };

    // store the 1 ring tris of a
    for (const Tuple& tri : m.get_one_ring_tris_for_vertex(edge_tuple)) {
        const Tuple e = tri.switch_vertex(m).switch_edge(m);
        size_t a = e.vid(m);
        size_t b = e.switch_vertex(m).vid(m);
        if (a != v2 && b != v2) store_opposing_edge(e);
    }
    for (const Tuple& tri : m.get_one_ring_tris_for_vertex(edge_tuple.switch_vertex(m))) {
        const Tuple e = tri.switch_vertex(m).switch_edge(m);
        size_t a = e.vid(m);
        size_t b = e.switch_vertex(m).vid(m);
        if (a != v1 && b != v1) store_opposing_edge(e);
    }

    for (const Tuple& edge : m.get_one_ring_edges_for_vertex(edge_tuple)) {
        if (edge.vid(m) != v2) store_new_vertex_edge(edge);
    }

    for (const Tuple& edge : m.get_one_ring_edges_for_vertex(edge_tuple.switch_vertex(m))) {
        if (edge.vid(m) != v1) store_new_vertex_edge(edge);
    }

    // store the 1 ring tris of b

    /*
    // store A B according to previous diagram
    {
        const Tuple e1 = edge_tuple.switch_edge(m).switch_vertex(m);
        store_edge(e1);
        const Tuple e2 = e1.switch_edge(m);
        store_edge(e2);
    }
    {
        // Store C D according to previous
        if (auto face_opt = edge_tuple.switch_face(m); face_opt.has_value()) {
            const Tuple e1 = face_opt.value().switch_edge(m).switch_vertex(m);
            store_edge(e1);
            const Tuple e2 = e1.switch_edge(m);
            store_edge(e2);
        }
    }
    */
}
auto AdaptiveTessellationCollapseEdgeOperation::merge(
    ConstrainedBoundaryType a,
    ConstrainedBoundaryType b) -> ConstrainedBoundaryType
{
    // constraint is implemented as a bitmask so or hte bitmasks to get maximal constraintness
    return static_cast<ConstrainedBoundaryType>(static_cast<char>(a) | static_cast<char>(b));
}

auto AdaptiveTessellationCollapseEdgeOperation::get_constrained_boundary_type(
    const AdaptiveTessellation& m,
    const Tuple& t) const -> ConstrainedBoundaryType
{
    ConstrainedBoundaryType primary = get_constrained_boundary_type_per_face(m, t);
    if (const auto oface_opt = t.switch_face(m); oface_opt.has_value()) {
        ConstrainedBoundaryType secondary =
            get_constrained_boundary_type_per_face(m, oface_opt.value());

        return merge(primary, secondary);
    } else {
        return primary;
    }
}
auto AdaptiveTessellationCollapseEdgeOperation::get_constrained_boundary_type_per_face(
    const AdaptiveTessellation& m,
    const Tuple& t) const -> ConstrainedBoundaryType
{
    const Tuple other_tuple = t.switch_vertex(m);
    const auto& t_vattr = m.get_vertex_attrs(t);
    const auto& o_vattr = m.get_vertex_attrs(other_tuple);

    const auto& edge_curveid_opt = m.get_edge_attrs(t).curve_id;

    const bool edge_is_boundary = m.is_boundary_edge(t);
    // const bool vertex_same_vattrs = t_vattr.curve_id == o_vattr.curve_id;

    auto vertex_is_constrained = [&](const Tuple& v) -> bool {
        const auto& vattr = m.get_vertex_attrs(v);
        // fixed vertices can't move
        if (vattr.fixed) {
            return true;
        }

        // if a vertex is on boundary then the other one cannot be on a boundary unless it's on an
        // edge
        // TODO: make sure boundary vertices are properly handled
        if (edge_is_boundary) {
            return false;

        } else {
            if (m.is_boundary_vertex(v)) {
                return true;
            }
        }

        return false;
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

    if (m.mesh_parameters.m_bnd_freeze && (v1_attr.boundary_vertex || v2_attr.boundary_vertex)) {
        return false;
    }
    return true;
}

bool AdaptiveTessellationCollapseEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    m_op_cache.local() = {};
    if (wmtk::TriMeshEdgeCollapseOperation::before(m, t)) {
        wmtk::logger().info("pass in before");
        if (!check_vertex_mergeability(m, t)) {
            wmtk::logger().info("fail in mergeability check");
            return false;
        }
        // TODO: currently edge mergeability just checks for double boundaries
        // which is caught by link condition, is there something else?
        // if(!check_edge_mergeability(m, t)) { return false; }

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

        // make sure that we'll be able to put a vertex in the right positoin
        const ConstrainedBoundaryType cbt = m_op_cache.local().constrained_boundary_type;
        if (cbt == ConstrainedBoundaryType::BothConstrained) {
            wmtk::logger().info("fail in constrained boundary type");
            return false;
        }
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
    const VertexAttributes& attr) const -> VertexAttributes&
{
    return m.get_vertex_attrs(get_return_tuple_opt().value()) = attr;
}

auto AdaptiveTessellationCollapseEdgeOperation::assign_new_vertex_attributes(
    AdaptiveTessellation& m) const -> VertexAttributes&
{
    assert(bool(*this));
    const auto return_edge_tuple = get_return_tuple_opt().value();
    const size_t return_vid = return_edge_tuple.vid(m);
    auto& return_v_attr = m.vertex_attrs[return_vid];

    const OpCache& op_cache = m_op_cache.local();
    const auto& v1_attr = m.vertex_attrs[op_cache.v1];
    const auto& v2_attr = m.vertex_attrs[op_cache.v2];
    switch (op_cache.constrained_boundary_type) {
    case ConstrainedBoundaryType::NoConstraints: {
        return_v_attr.pos = (v1_attr.pos + v2_attr.pos) / 2.0;
        return_v_attr.t = (v1_attr.t + v2_attr.t) / 2.0;
        return_v_attr.partition_id = op_cache.partition_id;
    } break;
    case ConstrainedBoundaryType::TupleSideConstrained: {
        return_v_attr = v1_attr;
    } break;
    case ConstrainedBoundaryType::OtherSideConstrained: {
        return_v_attr = v2_attr;
    } break;
    case ConstrainedBoundaryType::BothConstrained: {
        assert(false); // before should have caught this?
    } break;
    }


    // TODO: figure out if any of htese are redundant / necessary
    return_v_attr.partition_id = op_cache.partition_id;
    // TODO: boundary_vertex is always overwritten / is a cache varible?
    return_v_attr.boundary_vertex = (v1_attr.boundary_vertex || v2_attr.boundary_vertex);
    return_v_attr.fixed = (v1_attr.fixed || v2_attr.fixed);
    return return_v_attr;
}

void AdaptiveTessellationCollapseEdgeOperation::assign_collapsed_edge_attributes(
    AdaptiveTessellation& m,
    const std::optional<Tuple>& new_mirror_vertex_opt) const
{
    auto& tri_connectivity = this->tri_connectivity(m);
    auto nt_opt = new_vertex(m);
    assert(nt_opt.has_value());
    const Tuple& new_vertex_tuple = nt_opt.value();
    const size_t new_vertex_vid = new_vertex_tuple.vid(m);
    auto& op_cache = m_op_cache.local();

    size_t mirror_index = 0;
    if (new_mirror_vertex_opt.has_value()) {
        mirror_index = new_mirror_vertex_opt.value().vid(m);
    }
    //==========
    // for every triangle in the seam-ful one ring nbd lets look for boundary edges
    //==========


    // we cache the edges that do not involve the new vertex lets
    // handle opposing edge pairs update
    auto try_latching_seam_data = [&](size_t v0, size_t v1, const SeamData& data) {
        auto edge_opt = m.tuple_from_edge_vids_opt(v0, v1);

        assert(edge_opt.has_value());
        // edge points to new_vertex
        const Tuple edge = edge_opt.value();

        m.edge_attrs[edge.eid(m)].curve_id = data.curve_id;
        const auto& mopt = data.mirror_edge_vids;
        if (mopt) {
            const auto& mirror_vids = mopt.value();
            auto [m0, m1] = mirror_vids;
            if (m0 == m1) {
                assert(new_mirror_vertex_opt.has_value());
                m0 = mirror_index;
            }
            auto eopt = m.tuple_from_edge_vids_opt(m0, m1);
            assert(eopt.has_value());
            const Tuple mirror = eopt.value();
            m.set_mirror_edge_data(edge, mirror);
            m.set_mirror_edge_data(mirror, edge);
        }
    };


    // TODO: cache tris that have already been used
    for (const auto& [other_vertex, seam_data] : op_cache.new_vertex_seam_data) {
        try_latching_seam_data(new_vertex_vid, other_vertex, seam_data);
    }
    for (const auto& [edge_vids, seam_data] : op_cache.opposing_edge_seam_data) {
        const auto& [a, b] = edge_vids;
        try_latching_seam_data(a, b, seam_data);
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
    spdlog::info("PCStart");
    auto& op_cache = m_op_cache.local();

    spdlog::info("Seamed link");
    if (!check_seamed_link_condition(m, t)) {
        return false;
    }
    spdlog::info("Collapse before");
    if (!collapse_edge.before(m, t)) {
        return false;
    }

    set_input_edge_mirror(m, t);


    if (input_edge_is_mirror()) {
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        spdlog::info("has mirror, doing before");
        if (!collapse_mirror_edge.before(m, mirror_edge_tuple)) {
            return false;
        }

        // check that if both are constrained they aren't overconstrained
        auto& collapse_edge_cache = collapse_edge.m_op_cache.local();
        auto& mirror_edge_cache = collapse_mirror_edge.m_op_cache.local();
        auto& a = collapse_edge_cache.constrained_boundary_type;
        auto& b = mirror_edge_cache.constrained_boundary_type;

        spdlog::info("PC merging constarintness");
        const auto merged = AdaptiveTessellationCollapseEdgeOperation::merge(a, b);
        if (merged ==
            AdaptiveTessellationCollapseEdgeOperation::ConstrainedBoundaryType::BothConstrained) {
            spdlog::info("PC both constrained");
            return false;
        }
        a = merged;
        b = merged;
    }

    store_boundary_data(m, t);

    return true;
}

wmtk::TriMeshOperation::ExecuteReturnData AdaptiveTessellationPairedCollapseEdgeOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    spdlog::info("PC exec");
    auto& collapse_edge_cache = collapse_edge.m_op_cache.local();
    auto& mirror_edge_cache = collapse_mirror_edge.m_op_cache.local();
    auto& op_cache = m_op_cache.local();


    spdlog::info("PC exec primary");
    wmtk::TriMeshOperation::ExecuteReturnData ret_data = collapse_edge.execute(m, t);

    if (!ret_data) return ret_data;

    // if we have a mirror edge we need to
    if (input_edge_is_mirror()) {
        spdlog::info("PC exec secondary");
        const Tuple& mirror_edge_tuple = op_cache.mirror_edge_tuple_opt.value();
        wmtk::TriMeshOperation::ExecuteReturnData ret_data2 =
            collapse_mirror_edge.execute(m, mirror_edge_tuple.switch_vertex(m));
    }

    ret_data.success = bool(*this);
    ret_data.new_tris = modified_tuples(m);
    spdlog::info("PC exec done");

    return ret_data;
}


auto AdaptiveTessellationPairedCollapseEdgeOperation::modified_tuples(
    const AdaptiveTessellation& m) const -> std::vector<Tuple>
{
    const Tuple& return_tuple = collapse_edge.get_return_tuple_opt().value();
    std::vector<size_t> all_mirrors = m.get_all_mirror_vids(return_tuple);

    std::vector<Tuple> one_ring;
    for (const size_t ret_vid : all_mirrors) {
        auto a = m.get_one_ring_tris_for_vertex(m.tuple_from_vertex(ret_vid));
        one_ring.insert(one_ring.end(), a.begin(), a.end());
    }
    return one_ring;
    // modified_tuples_T(m, collapse_edge, collapse_mirror_edge);
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
    const Tuple& return_tuple = collapse_edge.get_return_tuple_opt().value();
    if (!collapse_edge.after(m)) {
        return false;
    }

    const auto& new_vertex_attr = collapse_edge.assign_new_vertex_attributes(m);

    collapse_edge.assign_collapsed_edge_attributes(m, collapse_mirror_edge.new_vertex(m));
    if (input_edge_is_mirror()) {
        if (!collapse_mirror_edge.after(m)) {
            return false;
        }
        collapse_mirror_edge.assign_collapsed_edge_attributes(m, collapse_edge.new_vertex(m));
        // const Tuple& mirror_edge_tuple = collapse_mirror_edge.get_return_tuple_opt().value();
        auto& mirror_vertex_attr = collapse_mirror_edge.assign_new_vertex_attributes(m);

        if (collapse_edge.m_op_cache.local().constrained_boundary_type ==
            AdaptiveTessellationCollapseEdgeOperation::ConstrainedBoundaryType::NoConstraints) {
            mirror_vertex_attr.pos_world = new_vertex_attr.pos_world;
        }
    }

    std::vector<size_t> all_mirrors = m.get_all_mirror_vids(return_tuple);

    std::vector<Tuple> one_ring;
    for (const size_t ret_vid : all_mirrors) {
        auto a = m.get_one_ring_tris_for_vertex(m.tuple_from_vertex(ret_vid));
        one_ring.insert(one_ring.end(), a.begin(), a.end());
    }


    // check invariants here since get_area_accuracy_error_per_face requires valid triangle
    if (!m.invariants(one_ring)) return false;

    if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
        for (const Tuple& tri : one_ring) {
            double one_ring_tri_error = m.get_area_accuracy_error_per_face(tri);
            if (one_ring_tri_error > m.mesh_parameters.m_accuracy_safeguard_ratio *
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
