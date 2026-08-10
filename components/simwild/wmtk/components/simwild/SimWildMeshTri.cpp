#include "SimWildMeshTri.hpp"

#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/predicates/predicates.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <map>
#include <paraviewo/VTUWriter.hpp>
#include <set>
#include <unordered_map>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/DirichletEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>

#include <wmtk/utils/partition_utils.hpp>
#include "expression_parser/Parser.hpp"

namespace wmtk::components::simwild::tri {

auto renew = [](const SimWildMeshTri& m, auto op, auto& tris) {
    using Tuple = TriMesh::Tuple;
    std::vector<Tuple> edges;
    for (const auto& t : tris) {
        for (auto j = 0; j < 3; j++) {
            edges.push_back(m.tuple_from_edge(t.fid(m), j));
        }
    }
    wmtk::unique_edge_tuples(m, edges);

    std::vector<std::pair<std::string, Tuple>> optup;
    optup.reserve(edges.size());
    for (const Tuple& e : edges) {
        optup.emplace_back(op, e);
    }
    return optup;
};


auto edge_locker = [](auto& m, const auto& e, int task_id) {
    // TODO: this should not be here
    return m.try_set_edge_mutex_two_ring(e, task_id);
};

// TODO: this should not be here
void SimWildMeshTri::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        m_vertex_attribute[i].partition_id = m_vertex_partition_id[i];
}

// TODO: morton should not be here, but inside wmtk
void SimWildMeshTri::partition_mesh_morton()
{
    if (NUM_THREADS == 0) {
        return;
    }
    logger().info("Number of parts: {} by morton", NUM_THREADS);

    // The shared partitioner is 3D; a zero z leaves the bounding box, the scale and the
    // Morton code exactly where a 2D-specific version would put them.
    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) {
            const Vector2d& p = m_vertex_attribute[i].m_posf;
            return Eigen::Vector3d(p[0], p[1], 0);
        },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = partition_id[i];
    }
}

double SimWildMeshTri::get_length2(const Tuple& l) const
{
    const auto vs = get_edge_vids(l);
    const Vector2d& p0 = m_vertex_attribute.at(vs[0]).m_posf;
    const Vector2d& p1 = m_vertex_attribute.at(vs[1]).m_posf;

    return (p1 - p0).squaredNorm();
}

void SimWildMeshTri::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixSi& T_tags,
    const std::vector<std::string>& tag_names)
{
    assert(V.cols() == 2);
    assert(T.cols() == 3);
    assert(T_tags.rows() == T.rows());
    assert(T_tags.cols() == tag_names.size());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.resize(V.rows());
    m_face_attribute.resize(T.rows());
    m_edge_attribute.resize(T.rows() * 3);

    // A float input is exactly representable by construction, so every vertex starts rounded.
    // Only a split can un-round one after this.
    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_posf = V.row(i);
        m_vertex_attribute[i].m_pos = to_rational(m_vertex_attribute[i].m_posf);
        m_vertex_attribute[i].m_is_rounded = true;
    }

    // init quality and check for inverted mesh
    bool is_mesh_inverted = false;
    for (const Tuple& t : get_faces()) {
        if (is_mesh_inverted ^ is_inverted(t)) {
            if (!is_mesh_inverted) {
                is_mesh_inverted = true;
            } else {
                log_and_throw_error("Tets with different orientations in the input!");
            }
        }
        m_face_attribute[t.fid(*this)].m_quality = get_quality(t);
    }

    if (is_mesh_inverted) {
        log_and_throw_error(
            "Input mesh is fully inverted! This should not happen... Might be a bug.");
    }


    // add tags
    for (size_t i = 0; i < (size_t)T_tags.rows(); ++i) {
        // m_face_attribute[i].tags.resize(m_tags_count);
        for (size_t j = 0; j < m_tags_count; ++j) {
            if (T_tags.coeff(i, j) != 0) {
                m_face_attribute[i].tags.insert(j);
            }
        }
    }

    // add tag names
    for (size_t i = 0; i < tag_names.size(); ++i) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }

    init_surfaces_and_boundaries();
}

void SimWildMeshTri::init_from_image(
    const MatrixXr& V,
    const MatrixXi& T,
    const MatrixSi& T_tags,
    const std::vector<std::string>& tag_names)
{
    assert(V.cols() == 2);
    assert(T.cols() == 3);
    assert(T_tags.rows() == T.rows());
    assert(T_tags.cols() == tag_names.size());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.resize(V.rows());
    m_face_attribute.resize(T.rows());
    m_edge_attribute.resize(T.rows() * 3);

    // A vertex is "direct" when its exact coordinate IS a double -- the round trip through
    // to_double/to_rational returns it unchanged. Those start rounded; the rest are the
    // arrangement's crossing vertices, which generally have no double representation at all.
    size_t n_indirect = 0;
    for (int i = 0; i < vert_capacity(); i++) {
        VertexAttributes& va = m_vertex_attribute[i];
        va.m_pos = V.row(i);
        va.m_posf = to_double(va.m_pos);
        va.m_is_rounded = (to_rational(va.m_posf) == va.m_pos);
        if (!va.m_is_rounded) {
            ++n_indirect;
        }
    }
    if (n_indirect > 0) {
        m_all_rounded.store(false, std::memory_order_relaxed);
    }

    // Orientation is checked in EXACT arithmetic: an un-rounded vertex sends is_inverted down
    // the rational path, which is the whole point of keeping V. Rounding first and checking
    // after would reject valid input, because two exactly-distinct arrangement vertices can
    // round onto the same double and make a valid triangle look degenerate.
    bool is_mesh_inverted = false;
    for (const Tuple& t : get_faces()) {
        if (is_mesh_inverted ^ is_inverted(t)) {
            if (!is_mesh_inverted) {
                is_mesh_inverted = true;
            } else {
                log_and_throw_error("Faces with different orientations in the input!");
            }
        }
    }
    if (is_mesh_inverted) {
        log_and_throw_error(
            "Input mesh is fully inverted! This should not happen... Might be a bug.");
    }

    // Reclaim what can be rounded without inverting an incident face. round() takes the
    // vertices one at a time, so a vertex that only becomes roundable once a neighbour has
    // committed is still picked up -- the sweep is serial for exactly that reason.
    const size_t reclaimed = round_all_vertices();
    logger().info(
        "init: {} of {} vertices had no exact double representation, {} reclaimed by rounding",
        n_indirect,
        vert_capacity(),
        reclaimed);

    for (const Tuple& t : get_faces()) {
        m_face_attribute[t.fid(*this)].m_quality = get_quality(t);
    }

    // add tags
    for (size_t i = 0; i < (size_t)T_tags.rows(); ++i) {
        for (size_t j = 0; j < m_tags_count; ++j) {
            if (T_tags.coeff(i, j) != 0) {
                m_face_attribute[i].tags.insert(j);
            }
        }
    }

    // add tag names
    for (size_t i = 0; i < tag_names.size(); ++i) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }

    init_surfaces_and_boundaries();
}

void SimWildMeshTri::init_surfaces_and_boundaries()
{
    const auto edges = get_edges();
    logger().info("#E = {}", edges.size());

    // tag surface edges and vertices
    std::vector<Vector2i> tempE;
    for (const Tuple& e : edges) {
        SmartTuple ee(*this, e);

        const auto t_opp = ee.switch_face();
        if (!t_opp) {
            continue;
        }

        {
            const auto& tag0 = m_face_attribute[ee.fid()].tags;
            const auto& tag1 = m_face_attribute[t_opp.value().fid()].tags;
            if (tag0 == tag1) {
                continue;
            }
        }

        m_edge_attribute[ee.eid()].m_is_surface_fs = 1;

        const size_t v1 = ee.vid();
        const size_t v2 = ee.switch_vertex().vid();
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;

        tempE.emplace_back(v1, v2);
    }

    if (!m_envelope) {
        logger().info("Init envelope from face tags");
        // build envelopes
        std::vector<Vector2d> tempV(vert_capacity());
        for (int i = 0; i < vert_capacity(); i++) {
            tempV[i] = m_vertex_attribute[i].m_posf;
        }

        m_V_envelope = tempV;
        m_E_envelope = tempE;
        m_envelope = std::make_shared<SampleEnvelope>(!m_params.use_sample_envelope);
        m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
        logger().info(
            "Envelope: {} (eps {:.6})",
            m_envelope->use_exact ? "EXACT" : "sampled",
            m_envelope_eps);
        m_envelope_orig = m_envelope;
    } else if (m_params.operation == "remeshing" && m_params.check_envelope_at_init) {
        // All surface edges must be inside the envelope. Opt-in: see
        // Parameters::check_envelope_at_init for why it is not worth its cost by default.
        logger().info("Envelope sanity check");
        const auto surf_edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : surf_edges) {
            std::array<Vector2d, 2> pp = {
                {m_vertex_attribute[verts[0]].m_posf, m_vertex_attribute[verts[1]].m_posf}};
            if (m_envelope->is_outside(pp)) {
                log_and_throw_error("Edge {} is outside!", verts);
            }
        }
        logger().info("Envelope sanity check done");
    }

    // track bounding box
    for (size_t i = 0; i < edges.size(); i++) {
        const auto vids = get_edge_vids(edges[i]);
        int on_bbox = -1;
        // Read the EXACT coordinate, as triwild does. An == against a domain bound is
        // precisely where rounding decides membership: two vertices that are exactly on the
        // boundary can round off it, or off it can round onto it, and the bbox tag is what
        // keeps the domain from collapsing.
        for (int k = 0; k < 2; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_params.box_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        assert(!edges[i].switch_face(*this)); // face must be on boundary

        const size_t eid = edges[i].eid(*this);
        m_edge_attribute[eid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });
}

void SimWildMeshTri::init_envelope(const MatrixXd& V, const MatrixXi& E)
{
    if (m_envelope) {
        log_and_throw_error("Envelope was already initialized once.");
    }
    assert(m_V_envelope.empty() && m_E_envelope.empty());
    assert(V.size() != 0 && E.size() != 0);
    assert(V.cols() == 2); // vertices must be in 2D
    assert(E.cols() == 2); // envelope must be edges


    m_V_envelope.resize(V.rows());
    for (size_t i = 0; i < m_V_envelope.size(); ++i) {
        m_V_envelope[i] = V.row(i);
    }
    m_E_envelope.resize(E.rows());
    for (size_t i = 0; i < m_E_envelope.size(); ++i) {
        m_E_envelope[i] = E.row(i);
    }

    m_envelope = std::make_shared<SampleEnvelope>(!m_params.use_sample_envelope);
    m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
    logger().info(
        "Envelope: {} (eps {:.6})",
        m_envelope->use_exact ? "EXACT" : "sampled",
        m_envelope_eps);

    if (!m_envelope_orig) {
        m_envelope_orig = m_envelope;
    }
}

CellTag SimWildMeshTri::string_set_to_cell_tag(const std::set<std::string>& str_set)
{
    CellTag cell_tag;
    for (const auto& str : str_set) {
        const auto it = m_tag_name_to_id.find(str);
        if (it != m_tag_name_to_id.end()) {
            cell_tag.insert(it->second);
        } else {
            logger().warn("Tag name {} does not exist! Adding new tag.", str);
            int64_t new_id = m_tags_count++;
            m_tag_name_to_id[str] = new_id;
            m_tag_id_to_name[new_id] = str;
            cell_tag.insert(new_id);
        }
    }
    return cell_tag;
}

void SimWildMeshTri::set_sizing_field(const nlohmann::json& sizing_field_json)
{
    if (!sizing_field_json.is_array()) {
        log_and_throw_error(
            "sizing_field should be an array of objects, each defining a region and its target "
            "length.");
    }

    for (const auto& region_json : sizing_field_json) {
        if (!region_json.contains("tags")) {
            log_and_throw_error("Each sizing_field entry must contain a 'tags' field.");
        }
        const std::string tags_str_set = region_json["tags"];
        auto& [expr, length] = m_sizing_field.emplace_back();
        expr = expression_parser::parse(tags_str_set, m_tag_name_to_id);

        length = region_json["length"];
        double length_rel = region_json["length_rel"];
        if (length < 0 && length_rel < 0) {
            log_and_throw_error(
                "Each sizing_field entry must specify at least one of 'length' or 'length_rel'.");
        }

        if (length < 0) {
            length = length_rel * m_params.diag_l;
        }

        logger().info("Added sizing field: expr = {}, length = {}", expr->to_string(), length);
    }

    // apply sizing fields to vertices
    for (const Tuple& t : get_faces()) {
        const auto tid = t.fid(*this);
        for (const auto& [expr, length] : m_sizing_field) {
            if (!expr->eval(m_face_attribute[tid].tags)) {
                continue;
            }
            const auto vs = oriented_tri_vids(tid);
            for (const size_t& vid : vs) {
                auto& s = m_vertex_attribute[vid].m_sizing_scalar;
                s = length / m_params.l; // overwrite previous value
            }
        }
    }
    for (const Tuple& t : get_faces()) {
        const auto tid = t.fid(*this);
        double sizing = 1.0; // default
        for (const auto& [expr, length] : m_sizing_field) {
            if (expr->eval(m_face_attribute[tid].tags)) {
                sizing = length / m_params.l;
            }
        }
        const auto vs = oriented_tri_vids(tid);
        for (const size_t& vid : vs) {
            auto& s = m_vertex_attribute[vid].m_sizing_scalar;
            s = std::min(s, sizing);
        }
    }
}

void SimWildMeshTri::set_quality_field(const nlohmann::json& quality_field_json)
{
    if (!quality_field_json.is_array()) {
        log_and_throw_error(
            "quality_field should be an array of objects, each defining a region and its "
            "target "
            "quality.");
    }

    for (const auto& region_json : quality_field_json) {
        if (!region_json.contains("tags")) {
            log_and_throw_error("Each quality_field entry must contain a 'tags' field.");
        }
        if (!region_json.contains("quality")) {
            log_and_throw_error("Each quality_field entry must contain a 'quality' field.");
        }
        const std::string tags_str_set = region_json["tags"];
        auto& [expr, quality] = m_quality_field.emplace_back();
        expr = expression_parser::parse(tags_str_set, m_tag_name_to_id);

        quality = region_json["quality"];

        logger().info("Added quality field: expr = {}, quality = {}", expr->to_string(), quality);
    }
}

double SimWildMeshTri::target_quality(const size_t tid) const
{
    double quality = m_params.stop_energy; // default
    for (const auto& [expr, q] : m_quality_field) {
        if (expr->eval(m_face_attribute[tid].tags)) {
            quality = q;
        }
    }
    return quality;
}

double SimWildMeshTri::target_quality(const Tuple& t) const
{
    const auto tid = t.fid(*this);
    return target_quality(tid);
}

double SimWildMeshTri::quality_rel(const size_t tid) const
{
    return m_face_attribute[tid].m_quality / target_quality(tid);
}

double SimWildMeshTri::quality_rel(const Tuple& t) const
{
    return quality_rel(t.fid(*this));
}

bool SimWildMeshTri::check_mesh_quality(double& max_rel_quality, const bool verbose) const
{
    bool all_good = true;
    size_t num_bad = 0;
    size_t num_total = 0;
    max_rel_quality = 0;
    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple tup = tuple_from_tri(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        num_total++;
        double rel_quality = quality_rel(i);
        max_rel_quality = std::max(max_rel_quality, rel_quality);
        if (rel_quality > 1.0) {
            all_good = false;
            num_bad++;
        }
    }
    if (verbose && num_bad > 0) {
        logger().info(
            "Bad elements: {} of {}, max relative quality: {:.6}",
            num_bad,
            num_total,
            max_rel_quality);
    }
    return all_good;
}

size_t SimWildMeshTri::refine_sizing_around_worst()
{
    const int n_rings = std::max(0, m_params.stuck_refine_rings);

    // Relative quality against the per-cell target, filtered at 1.0 -- the same selection the
    // hand-rolled version made with `if (q < target_quality(tid)) continue;`, and the same
    // shape the 3D mesh uses. target_quality defaults to m_params.stop_energy and is only
    // overridden by the per-tag quality_field, so this is simwild's relative-quality model,
    // not tetwild/triwild's absolute filter_energy.
    //
    // m_quality is the AMIPS2D energy itself here, so no cube root (unlike the 3D mesh).
    const auto worst = utils::select_worst_cells(
        tri_capacity(),
        [this](size_t fid) { return tuple_from_tri(fid).is_valid(*this); },
        [this](size_t fid) {
            return m_face_attribute[fid].m_quality / target_quality(fid); // relative quality
        },
        1.0,
        m_params.stuck_refine_num_worst);

    if (worst.empty()) {
        return 0;
    }

    // Seed the region with the worst triangles' vertices, then BFS n_rings hops.
    std::vector<size_t> seeds;
    seeds.reserve(3 * worst.size());
    for (const auto& [_, fid] : worst) {
        for (const size_t v : oriented_tri_vids(fid)) {
            seeds.push_back(v);
        }
    }
    const auto region = utils::grow_vertex_region(seeds, n_rings, [this](size_t v) {
        return get_one_ring_vids_for_vertex_duplicate(v);
    });

    // Apply the multiplicative refinement, clamped at the floor.
    const auto refined = utils::apply_sizing_refinement(
        region,
        m_params.stuck_refine_factor,
        m_params.stuck_refine_min_scalar,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; });

    // Grade the refined region into its surroundings (monotone, only lowers).
    gradation_smooth_sizing(m_params.stuck_refine_gradation, refined);

    // restrict sizing scalar according to sizing field
    for (const Tuple& t : get_faces()) {
        const auto tid = t.fid(*this);
        double sizing = std::numeric_limits<double>::max();
        for (const auto& [expr, length] : m_sizing_field) {
            if (expr->eval(m_face_attribute[tid].tags)) {
                sizing = std::min(sizing, length / m_params.l);
            }
        }
        const auto vs = oriented_tri_vids(tid);
        for (const size_t& vid : vs) {
            auto& s = m_vertex_attribute[vid].m_sizing_scalar;
            s = std::min(s, sizing);
        }
    }

    logger().info(
        "[stuck-refine] worst {} tris (relE {:.4}), refined {} of {} region vertices",
        worst.size(),
        worst.back().first,
        refined.size(),
        region.size());
    return refined.size();
}

void SimWildMeshTri::gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds)
{
    utils::gradation_smooth_sizing(
        grade,
        seeds,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex_duplicate(v); });
}

void SimWildMeshTri::write_msh(std::string file, const bool write_envelope)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_face_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        Vector2d p2 = m_vertex_attribute[i].m_posf;
        return Vector3d(p2[0], p2[1], 0);
    });

    const auto& faces = get_faces();

    int64_t max_tag = -1;
    for (const Tuple& t : faces) {
        const size_t fid = t.fid(*this);
        const auto& tags = m_face_attribute[fid].tags;
        if (tags.size() == 0) {
            continue;
        }
        int64_t mt = *tags.rbegin();
        max_tag = std::max(max_tag, mt);
    }

    if (m_tags_count < max_tag + 1) {
        logger().warn(
            "Max tag is {} but m_tags_count is {}. Adjusting m_tags_count.",
            max_tag,
            m_tags_count);
        m_tags_count = max_tag + 1;
    }

    std::vector<Tuple> faces_with_tag;
    faces_with_tag.reserve(faces.size());

    auto msh_add_faces = [&]() {
        msh.add_faces(faces_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tri_vids(faces_with_tag[k]);
            std::array<size_t, 3> data;
            for (int j = 0; j < 3; j++) {
                data[j] = vs[j];
            }
            return data;
        });
    };

    // ambient mesh (no non-zero tags)
    for (const Tuple& t : faces) {
        const size_t fid = t.fid(*this);
        if (m_face_attribute[fid].tags.empty()) {
            faces_with_tag.push_back(t);
        }
    }
    msh_add_faces();

    msh.add_physical_group("ambient");


    // add a group for each tag
    for (size_t tag_img = 0; tag_img < m_tags_count; ++tag_img) {
        faces_with_tag.clear();
        for (const Tuple& t : faces) {
            const size_t fid = t.fid(*this);
            if (m_face_attribute[fid].tags.count(tag_img)) {
                faces_with_tag.push_back(t);
            }
        }

        if (faces_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(2);
        msh_add_faces();

        std::string group_name;
        if (m_tag_id_to_name.count(tag_img)) {
            group_name = m_tag_id_to_name[tag_img];
        } else {
            group_name = fmt::format("tag_{}", tag_img);
            while (m_tag_name_to_id.count(group_name)) {
                group_name += "_";
            }
            m_tag_name_to_id[group_name] = tag_img;
            m_tag_id_to_name[tag_img] = group_name;
            logger().warn(
                "Tag {} does not have a name. Assigning the name {}.",
                tag_img,
                group_name);
        }
        msh.add_physical_group(group_name);
    }

    if (m_envelope && write_envelope) {
        msh.add_edge_vertices(m_V_envelope.size(), [this](size_t k) {
            return Vector3d(m_V_envelope[k][0], m_V_envelope[k][1], 0);
        });
        msh.add_edges(m_E_envelope.size(), [this](size_t k) { return m_E_envelope[k]; });
        msh.add_physical_group("EnvelopeSurface");
    }

    logger().info("Write {}", file);
    msh.save(file, true);
}

void SimWildMeshTri::write_vtu(const std::string& path) const
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& faces = get_faces();
    const auto edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });

    Eigen::MatrixXd V(vert_capacity(), 2);
    Eigen::MatrixXi F(tri_capacity(), 3);
    Eigen::MatrixXi E(edges.size(), 2);

    V.setZero();
    F.setZero();
    E.setZero();

    Eigen::VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tri_capacity(), 1));
    VectorXd amips(tri_capacity());
    VectorXd amips_target(tri_capacity());
    VectorXd amips_rel(tri_capacity());

    int index = 0;
    for (const Tuple& t : faces) {
        size_t tid = t.fid(*this);
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_face_attribute[tid].tags.count(j) ? 1 : 0;
        }
        amips[index] = m_face_attribute[tid].m_quality;
        amips_target[index] = target_quality(tid);
        amips_rel[index] = quality_rel(tid);
        const auto tv = oriented_tri_vids(t);
        for (size_t j = 0; j < 3; j++) {
            F(index, j) = (int)tv[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = (int)edges[i][j];
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    for (size_t j = 0; j < m_tags_count; ++j) {
        writer->add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer->add_cell_field("quality", amips);
    writer->add_cell_field("quality_target", amips_target);
    writer->add_cell_field("quality_rel", amips_rel);
    writer->add_field("sizing_field", v_sizing_field);
    writer->write_mesh(path + ".vtu", V, F, paraviewo::CellType::Triangle);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        surf_writer->add_field("sizing_field", v_sizing_field);

        logger().info("Write {}", surf_out_path);
        surf_writer->write_mesh(surf_out_path, V, E, paraviewo::CellType::Line);
    }
}

void SimWildMeshTri::write_vtu_with_energies(const std::string& path) const
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write with energies {}", out_path);
    const auto& vs = get_vertices();
    const auto& faces = get_faces();
    const auto edges = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });

    MatrixXd V(vert_capacity(), 2);
    MatrixXi F(tri_capacity(), 3);
    MatrixXi E(edges.size(), 2);

    V.setZero();
    F.setZero();
    E.setZero();

    VectorXd v_sizing_field(vert_capacity());
    v_sizing_field.setZero();
    MatrixXd v_energy_grad_amips(vert_capacity(), 2);
    v_energy_grad_amips.setZero();
    MatrixXd v_energy_grad_envelope(vert_capacity(), 2);
    v_energy_grad_envelope.setZero();
    MatrixXd v_energy_grad_sum(vert_capacity(), 2);
    v_energy_grad_sum.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tri_capacity(), 1));
    MatrixXd amips(tri_capacity(), 1);

    int index = 0;
    for (const Tuple& t : faces) {
        size_t tid = t.fid(*this);
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_face_attribute[tid].tags.count(j) ? 1 : 0;
        }
        amips(index, 0) = m_face_attribute[tid].m_quality;

        const auto tv = oriented_tri_vids(t);
        for (size_t j = 0; j < 3; j++) {
            F(index, j) = (int)tv[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = (int)edges[i][j];
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        const Vector2d& x = m_vertex_attribute.at(vid).m_posf;
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;

        if (m_vertex_attribute.at(vid).m_is_on_surface) {
            auto energy_sum = std::make_shared<optimization::EnergySum>();

            auto amips_energy = get_amips_energy(v);
            auto envelope_energy = get_envelope_energy(v);

            energy_sum->add_energy(amips_energy);
            energy_sum->add_energy(envelope_energy);

            VectorXd g;
            amips_energy->gradient(x, g);
            v_energy_grad_amips.row(vid) = g;
            envelope_energy->gradient(x, g);
            v_energy_grad_envelope.row(vid) = g;
            energy_sum->gradient(x, g);
            v_energy_grad_sum.row(vid) = g;
        }
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    for (size_t j = 0; j < m_tags_count; ++j) {
        writer->add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer->add_cell_field("quality", amips);
    writer->add_field("sizing_field", v_sizing_field);
    writer->write_mesh(path + ".vtu", V, F, paraviewo::CellType::Triangle);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        surf_writer->add_field("sizing_field", v_sizing_field);
        surf_writer->add_field("amips_grad", v_energy_grad_amips);
        surf_writer->add_field("envelope_grad", v_energy_grad_envelope);
        surf_writer->add_field("sum_grad", v_energy_grad_sum);


        logger().info("Write {}", surf_out_path);
        surf_writer->write_mesh(surf_out_path, V, E, paraviewo::CellType::Line);
    }
}

std::vector<std::array<size_t, 2>> SimWildMeshTri::get_edges_by_condition(
    std::function<bool(const EdgeAttributes&)> cond) const
{
    std::vector<std::array<size_t, 2>> res;
    for (const Tuple& e : get_edges()) {
        size_t eid = e.eid(*this);
        if (cond(m_edge_attribute[eid])) {
            res.push_back({{e.vid(*this), e.switch_vertex(*this).vid(*this)}});
        }
    }
    return res;
}

void SimWildMeshTri::split_all_edges()
{
    igl::Timer timer;
    double time;
    m_exact_split_count = 0;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_split", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("edge split prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples =
            [](const SimWildMeshTri& m, std::string op, const auto& newts) {
                std::vector<std::pair<std::string, TriMesh::Tuple>> op_tups;
                for (const auto& t : newts) {
                    op_tups.emplace_back(op, t);
                    op_tups.emplace_back(op, t.switch_edge(m));
                    op_tups.emplace_back(op, t.switch_vertex(m).switch_edge(m));
                }
                return op_tups;
            };

        executor.priority = [&](const SimWildMeshTri& m, std::string op, const Tuple& t) {
            return m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const SimWildMeshTri& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) {
                return false;
            }
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            const auto& VA = m_vertex_attribute;
            double sizing_ratio = 0.5 * (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar);
            if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = ExecutePass<SimWildMeshTri>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = ExecutePass<SimWildMeshTri>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time serial: {:.4}s", time);
    }
    if (m_exact_split_count > 0) {
        wmtk::logger().info(
            "{} splits fell back to the exact rational midpoint",
            m_exact_split_count);
    }
}

bool SimWildMeshTri::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_edges.clear();
    cache.faces.clear();

    cache.v1_id = loc0.vid(*this);
    cache.v2_id = loc0.switch_vertex(*this).vid(*this);

    cache.old_e_attrs = m_edge_attribute[loc0.eid(*this)];

    cache.max_quality_before = 0.;
    for (const size_t fid : get_incident_fids_for_edge(loc0)) {
        cache.max_quality_before =
            std::max(cache.max_quality_before, m_face_attribute[fid].m_quality);
    }

    const simplex::Edge edge(cache.v1_id, cache.v2_id);

    auto faces = get_incident_fids_for_edge(loc0);
    for (const size_t fid : faces) {
        auto vs = oriented_tri_vids(fid);
        for (int j = 0; j < 3; j++) {
            const simplex::Edge e(vs[j], vs[(j + 1) % 3]);
            if (e == edge) {
                continue;
            }
            if (cache.changed_edges.count(e) != 0) {
                continue;
            }
            auto [_, eid] = tuple_from_edge(e.vertices());
            cache.changed_edges[e] = m_edge_attribute[eid];
        }
    }

    // store tet attributes
    for (const size_t fid : faces) {
        const simplex::Face face = simplex_from_face(fid);
        const size_t opp = face.opposite_vertex(edge).id();
        // if (m_face_attribute.at(fid).tags.size() == 0) {
        //    log_and_throw_error("No tags in face {}", fid); // for debugging
        //}
        cache.faces[opp] = m_face_attribute.at(fid);
    }

    return true;
}

bool SimWildMeshTri::split_edge_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TriMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    const std::vector<Tuple> locs = get_one_ring_tris_for_vertex(loc.switch_vertex(*this));
    const size_t v_id = loc.switch_vertex(*this).vid(*this);

    auto& cache = split_cache.local();
    cache.v_new = v_id;

    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    auto& p = m_vertex_attribute[v_id].m_posf;
    p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    // The new vertex was default-constructed, so it starts un-rounded with no exact
    // position. Both have to be set before the first is_inverted, which dispatches on them.
    m_vertex_attribute[v_id].m_pos = to_rational(p);
    m_vertex_attribute[v_id].m_is_rounded = true;

    for (const Tuple& t : locs) {
        if (is_inverted(t)) {
            m_vertex_attribute[v_id].m_is_rounded = false;
            break;
        }
    }
    if (!m_vertex_attribute[v_id].m_is_rounded) {
        // The rounded (double) midpoint inverts an incident triangle, so place the new vertex
        // at the EXACT rational midpoint of the two endpoints instead. That midpoint lies on
        // the shared edge, so it can never invert a previously-valid incident triangle: the
        // split always succeeds and a stuck region can keep being refined. The vertex stays
        // un-rounded (m_pos exact, m_is_rounded = false) until a later round() reclaims it.
        //
        // This used to return false instead -- refusing the split outright exactly where the
        // mesh is degenerate, which is where refinement is needed. triwild ran the refusing
        // version and diagnosed it as a stall cause: the stuck-refine machinery then hammers
        // the region from outside, driving the max energy up by orders of magnitude per pass.
        // On Thingi10K 509315 it cost 5 of 8 triwild runs, which diverged to 1e16..1e20 and
        // hit the sweep's one-hour timeout; without it 8 of 8 converge, in 2-5 iterations.
        //
        // Exact coordinates reaching the output is prevented by the iteration, not here: a
        // split is the only operation that can un-round a vertex (collapse, swap and smoothing
        // never do), the post-optimization pass is collapse-only, and mesh_improvement does
        // not stop until every vertex is rounded as well as the quality target being met.
        std::atomic_ref<size_t>(m_exact_split_count).fetch_add(1, std::memory_order_relaxed);
        m_vertex_attribute[v_id].m_pos =
            (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
        // Keep m_posf in step with the exact position rather than with the two endpoint
        // approximations: when an endpoint is itself un-rounded, rounding the exact midpoint
        // once is the better approximation. Same as triwild.
        p = to_double(m_vertex_attribute[v_id].m_pos);
        // Guard against a pre-existing inverted incident triangle: re-check in exact
        // arithmetic (un-rounded v_id => is_inverted uses the rational path).
        for (const Tuple& t : locs) {
            if (is_inverted(t)) {
                return false;
            }
        }
        // This split keeps an un-rounded vertex, so the sweep must not skip the next pass.
        // Set after the rollback checks above, which leave the mesh unchanged.
        m_all_rounded.store(false, std::memory_order_relaxed);
    }

    // If a Voronoi split function is set, binary-search vmid onto its zero-crossing.
    // p0 stays on the negative side, p1 on the positive side.
    //
    // Skipped for an un-rounded vertex: the exact midpoint is then the only position known to
    // keep every incident triangle valid, and this search only considers doubles -- including
    // the plain double midpoint it reverts to, which is the position that just inverted.
    if (m_voronoi_split_fn && m_vertex_attribute[v_id].m_is_rounded) {
        Vector2d p0 = m_vertex_attribute[v1_id].m_posf;
        Vector2d p1 = m_vertex_attribute[v2_id].m_posf;
        if (m_voronoi_split_fn(p0) >= 0) {
            std::swap(p0, p1); // ensure p0 is negative side
        }
        for (int i = 0; i < 20; ++i) {
            p = 0.5 * (p0 + p1);
            m_vertex_attribute[v_id].m_pos = to_rational(p);
            bool inv = false;
            for (const Tuple& t : locs) {
                if (is_inverted(t)) {
                    inv = true;
                    break;
                }
            }
            if (inv || (p1 - p0).squaredNorm() < 1e-20) {
                break;
            }
            if (m_voronoi_split_fn(p) < 0) {
                p0 = p;
            } else {
                p1 = p;
            }
        }
        // final inversion guard: revert to midpoint if needed
        bool inv = false;
        for (const Tuple& t : locs) {
            if (is_inverted(t)) {
                inv = true;
                logger().warn(
                    "Voronoi split resulted in inversion, reverting to midpoint. Iteration: {}",
                    m_debug_print_counter++);
                break;
            }
        }
        if (inv) {
            p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
            m_vertex_attribute[v_id].m_pos = to_rational(p);
        }
    }

    // update face attributes
    {
        // v1 - v_new
        const auto faces1 = get_incident_fids_for_edge(v1_id, v_id);
        const simplex::Edge edge1(v1_id, v_id);
        for (const size_t fid : faces1) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge1).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        // v2 - v_new
        const auto faces2 = get_incident_fids_for_edge(v2_id, v_id);
        const simplex::Edge edge2(v2_id, v_id);
        for (const size_t fid : faces2) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge2).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        assert(faces1.size() + faces2.size() == locs.size());

        const auto [_1, eid1] = tuple_from_edge(edge1.vertices());
        const auto [_2, eid2] = tuple_from_edge(edge2.vertices());

        m_edge_attribute[eid1] = cache.old_e_attrs;
        m_edge_attribute[eid2] = cache.old_e_attrs;
        for (const auto& [vid, _] : cache.faces) {
            const auto [_tup, eid] = tuple_from_edge({{v_id, vid}});
            m_edge_attribute[eid].reset();
        }
    }

    /// update quality
    //
    // A split is otherwise unconditional: it checks orientation and the envelope but never
    // quality. That is right for a length-driven split of a long, well-behaved edge and
    // wrong for the force-split of a stalled sliver's longest edge, where the midpoint can
    // land essentially on the opposite edge and leave a correctly-oriented element whose
    // area/volume is too small for AMIPS -- get_quality then returns MAX_ENERGY, and every
    // control decision that divides by the max energy is meaningless from then on. Refuse
    // to be the operation that creates one; subdividing an already-degenerate region is
    // still allowed, so a stuck region can keep being refined. See tetwild's EdgeSplitting
    // for the measurement this comes from.
    double max_quality_after = 0.;
    for (const Tuple& loc : locs) {
        max_quality_after = std::max(max_quality_after, get_quality(loc));
    }
    if (max_quality_after >= MAX_ENERGY && cache.max_quality_before < MAX_ENERGY) {
        return false;
    }
    for (const Tuple& loc : locs) {
        m_face_attribute[loc.fid(*this)].m_quality = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.old_e_attrs.m_is_surface_fs;

    /// update edge attribute
    for (const auto& [e, e_attr] : cache.changed_edges) {
        auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attr;
    }

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    return true;
}

void SimWildMeshTri::collapse_all_edges(bool is_limit_length)
{
    m_collapse_limit_length = is_limit_length;
    std::vector<std::pair<std::string, Tuple>> all_ops;

    auto setup_and_execute = [&](auto& executor) {
        executor.priority = [](const SimWildMeshTri& m, Op op, const Tuple& t) {
            return -m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const SimWildMeshTri& m,
                                            const std::tuple<double, Op, Tuple>& ele) {
            const auto& VA = m_vertex_attribute;
            auto& [weight, op, tup] = ele;
            const double length = m.get_length2(tup);
            if (length != -weight) {
                return false;
            }
            // Deliberately NOT filtered on length here. An over-length edge stays a
            // candidate and collapse_edge_before decides it on quality instead: it is kept
            // only if it STRICTLY improves the worst element of the ring. See there.
            return true;
        };

        // Execute!!
        do {
            all_ops.clear();
            const auto all_edges = get_edges();
            logger().info("#E = {}", all_edges.size());
            for (const Tuple& loc : all_edges) {
                // collect all edges. Filtering too long edges happens in `is_weight_up_to_date`
                all_ops.emplace_back("edge_collapse", loc);
                all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
            }
            executor(*this, all_ops);
            logger().info(
                "success: {}, failed: {}",
                executor.get_cnt_success(),
                executor.get_cnt_fail());
        } while (executor.get_cnt_success() > 0);
    };

    igl::Timer timer;
    timer.start();
    if (NUM_THREADS > 0) {
        auto executor = ExecutePass<SimWildMeshTri>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [](SimWildMeshTri& m, const Tuple& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        wmtk::logger().info("edge collapse time parallel: {:.4}s", timer.getElapsedTimeInSec());
    } else {
        auto executor = ExecutePass<SimWildMeshTri>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
        wmtk::logger().info("edge collapse time serial: {:.4}s", timer.getElapsedTimeInSec());
    }
}

bool SimWildMeshTri::collapse_edge_before(const Tuple& loc)
// input is an edge
{
    const auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();

    cache.changed_edges.clear();
    cache.changed_fids.clear();
    cache.changed_energies.clear();
    cache.surface_edges.clear();

    size_t v1_id = loc.vid(*this);
    auto loc1 = switch_vertex(loc);
    size_t v2_id = loc1.vid(*this);

    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    cache.edge_length = (VA[v1_id].m_posf - VA[v2_id].m_posf).norm();

    ///check if on bbox/surface/boundary
    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) {
            return false;
        }
        for (int on_bbox : VA[v1_id].on_bbox_faces)
            if (std::find(
                    VA[v2_id].on_bbox_faces.begin(),
                    VA[v2_id].on_bbox_faces.end(),
                    on_bbox) == VA[v2_id].on_bbox_faces.end()) {
                return false;
            }
    }

    // surface
    //
    // Both rules are gated on v1 being rounded, as triwild's are. An un-rounded vertex is
    // stuck: the optimizer could not place it at any double, and a collapse that removes it
    // also removes the exact coordinate. Refusing that collapse for the sake of a preference
    // -- staying on the surface, not moving a singular vertex -- keeps the rational
    // coordinate in the mesh, which is the more expensive outcome.
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_surface) {
        // if (!VA[v2_id].m_is_on_surface && m_envelope->is_outside(VA[v2_id].m_posf)) {
        //     return false;
        // }
        if (VA[v1_id].m_is_rounded && !VA[v2_id].m_is_on_surface) {
            return false; // do not collapse away from surface
        }

        if (VA[v1_id].m_is_rounded && get_order_of_vertex(v1_id) > 1) {
            return false; // do not move singular vertices
        }
    }

    const auto& n1_locs = get_one_ring_fids_for_vertex(loc);

    cache.changed_fids.reserve(n1_locs.size());
    cache.max_energy = 0;
    for (const size_t& tid : n1_locs) {
        const double q = m_face_attribute.at(tid).m_quality;
        cache.max_energy = std::max(cache.max_energy, q);
        const auto vs = oriented_tri_vids(tid);
        if (vs[0] != v2_id && vs[1] != v2_id && vs[2] != v2_id) {
            cache.changed_fids.emplace_back(tid);
        }
    }

    // pre-compute after-collapse energies
    cache.changed_energies.reserve(cache.changed_fids.size());
    for (const size_t tid : cache.changed_fids) {
        std::array<size_t, 3> vs = oriented_tri_vids(tid);
        for (size_t i = 0; i < 3; ++i) {
            if (vs[i] == v1_id) {
                vs[i] = v2_id;
                break;
            }
        }

        if (is_inverted(vs)) {
            return false;
        }
        double q = get_quality(vs);
        // Quality check only when v1 is rounded -- same reasoning as the surface rules above.
        if (VA[v1_id].m_is_rounded && q > target_quality(tid) && q > cache.max_energy) {
            return false;
        }
        cache.changed_energies.emplace_back(q);
    }
    assert(cache.changed_energies.size() == cache.changed_fids.size());

    // Length gate, applied here rather than when the candidate list is built.
    //
    // Coarsening a well-shaped mesh is what the length limit exists to prevent, so a short
    // edge keeps the old behaviour. But applying it to the CANDIDATE LIST made any element
    // whose edges are all longer than 0.8 * l invisible to this pass: it was never offered,
    // so it produced no rejection record anywhere and looked untouched rather than refused.
    //
    // Measured in triwild on triwild20k 189017 at eps_rel 1e-4, where a collinear triangle
    // with edges 55 / 165 / 220 against a gate of 38.7 survived every pass while stuck-refine
    // split it -- halving its short edge and DOUBLING its energy each round, 6.7e16 -> 1.5e17
    // -> 3.1e17 -> inverted -- and the mesh grew from 17k to 6.6M elements in ten iterations.
    // Refinement cannot repair a shape defect (AMIPS is scale-invariant, so splitting a
    // collinear element leaves it collinear); collapse is the only operation that can remove
    // one, so it has to be allowed to see it. With this, that model converges in 21
    // iterations, and the eps_rel 1e-3 run it already handled is unchanged at 12.
    //
    // The condition is strict improvement of the ring's worst element, which needs no
    // threshold and is self-limiting: in a healthy mesh almost no long-edge collapse strictly
    // improves anything. Note changed_fids excludes the faces the collapse deletes, so when
    // the ring's worst is one of those the test passes by construction -- the rule admits
    // precisely the collapses that remove a bad element.
    if (m_collapse_limit_length && VA[v1_id].m_is_rounded) {
        const double sizing_ratio = (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2;
        const double len2 = cache.edge_length * cache.edge_length;
        if (len2 > m_params.collapsing_l2 * sizing_ratio * sizing_ratio) {
            double max_after = 0.;
            for (const double q : cache.changed_energies) {
                max_after = std::max(max_after, q);
            }
            if (max_after >= cache.max_energy) {
                return false;
            }
        }
    }


    //
    const auto& n12_locs = get_incident_fids_for_edge(loc);
    for (const size_t& tid : n12_locs) {
        auto vs = oriented_tri_vids(tid);
        std::array<size_t, 2> e_vids = {{v1_id, 0}};
        int cnt = 1;
        // get the vertex that is not v1/v2, i.e., the edge-link vertices.
        for (int j = 0; j < 3; j++) {
            if (vs[j] != v1_id && vs[j] != v2_id) {
                e_vids[cnt] = vs[j];
                cnt++;
            }
        }
        auto [_1, global_eid1] = tuple_from_edge(e_vids);
        auto [_2, global_eid2] = tuple_from_edge({{v2_id, e_vids[1]}});
        auto e_attr = m_edge_attribute.at(global_eid1);
        e_attr.merge(m_edge_attribute.at(global_eid2));
        cache.changed_edges.push_back(std::make_pair(e_attr, e_vids));
    }

    if (VA[v1_id].m_is_on_surface) {
        // this code must check if a face is tagged as surface face
        // only checking the vertices is not enough
        std::vector<std::array<size_t, 2>> fs;
        for (const size_t& tid : n1_locs) {
            const auto vs = oriented_tri_vids(tid);

            int j_v1 = -1;
            auto skip = [&]() {
                for (int j = 0; j < 3; j++) {
                    const size_t vid = vs[j];
                    if (vid == v2_id) {
                        // ignore tets incident to the edge (v1,v2)
                        return true; // v1-v2 definitely not on surface.
                    }
                    if (vid == v1_id) j_v1 = j;
                }
                return false;
            };
            if (skip()) continue;

            for (int k = 0; k < 2; k++) {
                auto va = vs[(j_v1 + 1 + k) % 3];
                if (VA[va].m_is_on_surface) {
                    std::array<size_t, 2> f = {{v1_id, va}};
                    const auto [f_tuple, fid] = tuple_from_edge(f);
                    if (!m_edge_attribute.at(fid).m_is_surface_fs) {
                        // check if this face is actually on the surface
                        continue;
                    }
                    std::sort(f.begin(), f.end());
                    fs.push_back(f);
                }
            }
        }
        wmtk::vector_unique(fs);

        cache.surface_edges.reserve(fs.size());
        for (auto& f : fs) {
            std::replace(f.begin(), f.end(), v1_id, v2_id);
            cache.surface_edges.push_back(f);
        }
    }

    if (m_params.preserve_topology) {
        const bool v1_surf = VA[v1_id].m_is_on_surface;
        const bool v2_surf = VA[v2_id].m_is_on_surface;
        const bool v1_bbox = !VA[v1_id].on_bbox_faces.empty();
        const bool v2_bbox = !VA[v2_id].on_bbox_faces.empty();

        if ((v1_surf || v1_bbox) && (v2_surf || v2_bbox)) {
            if (!substructure_link_condition(loc)) {
                return false;
            }
        }
    }

    return true;
}

bool SimWildMeshTri::collapse_edge_after(const Tuple& loc)
{
    auto& VA = m_vertex_attribute;
    auto& cache = collapse_cache.local();
    size_t v1_id = cache.v1_id;
    size_t v2_id = cache.v2_id;

    if (!TriMesh::collapse_edge_after(loc)) {
        return false;
    }

    // surface
    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_edges) {
            const Vector2d a = VA.at(vids[0]).m_posf;
            const Vector2d b = VA.at(vids[1]).m_posf;
            // surface envelope
            bool is_out = m_envelope->is_outside(std::array<Vector2d, 2>{{a, b}});
            if (is_out) {
                return false;
            }
        }
    }

    //// update attrs
    // tet attr
    for (int i = 0; i < cache.changed_fids.size(); i++) {
        m_face_attribute[cache.changed_fids[i]].m_quality = cache.changed_energies[i];
    }
    // vertex attr
    VA[v2_id].m_is_on_surface = VA.at(v1_id).m_is_on_surface || VA.at(v2_id).m_is_on_surface;

    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_edges) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        auto [_, global_fid] = tuple_from_edge({{v2_id, old_vids[1]}});
        if (global_fid == -1) {
            return false;
        }
        m_edge_attribute[global_fid] = f_attr;
    }

    return true;
}

size_t SimWildMeshTri::swap_all_edges()
{
    igl::Timer timer;
    timer.start();
    auto collect_all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](SimWildMeshTri&, const Tuple& e, auto& out) { out.emplace_back("edge_swap", e); });
    logger().info("#E = {}", collect_all_ops.size());
    logger().info("edge swap prepare time: {:.4}s", timer.getElapsedTimeInSec());

    size_t total_success = 0;
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = renew;
        executor.num_threads = NUM_THREADS;
        executor.priority = [](const SimWildMeshTri& m, std::string op, const Tuple& e) {
            return m.swap_weight(e);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](const SimWildMeshTri& m, auto& ele) {
            auto& [val, _, e] = ele;
            const double w = m.swap_weight(e);
            return (w > 1e-5) && ((w - val) * (w - val) < 1e-8);
        };
        // Retry a failed swap only where the mesh actually changed this round
        // (dirty-epoch localized retry).
        total_success = wmtk::run_localized_to_convergence(*this, executor, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = ExecutePass<SimWildMeshTri>(ExecutionPolicy::kPartition);
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = ExecutePass<SimWildMeshTri>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }

    // The caller (local_operations) stops the swap loop once a pass changes nothing, so this
    // has to be the real count -- it used to return `true`, i.e. always 1, which meant the
    // early exit could never fire and every requested swap round ran in full. triwild carries
    // the same fix and the same note.
    return total_success;
}

double SimWildMeshTri::swap_weight(const Tuple& t) const
{
    const SmartTuple tt(*this, t);
    const auto t_opp = tt.switch_face();
    if (!t_opp) {
        return std::numeric_limits<double>::lowest();
    }

    if (is_edge_on_surface(t)) {
        return std::numeric_limits<double>::lowest();
    }

    const size_t v0 = tt.vid();
    const size_t v1 = tt.switch_vertex().vid();
    const size_t v2 = tt.switch_edge().switch_vertex().vid();
    const size_t v3 = t_opp.value().switch_edge().switch_vertex().vid();

    // before swap
    const double q012 = get_quality({{v0, v1, v2}});
    const double q031 = get_quality({{v0, v3, v1}});
    // after swap
    const double q032 = get_quality({{v0, v3, v2}});
    const double q231 = get_quality({{v2, v3, v1}});

    const double q_before = std::max(q012, q031);
    const double q_after = std::max(q032, q231);

    return q_before - q_after;
}

bool SimWildMeshTri::swap_edge_before(const Tuple& t)
{
    if (is_edge_on_surface(t)) {
        return false;
    }

    const auto& FA = m_face_attribute;
    auto& cache = swap_cache.local();
    cache.changed_edges.clear();

    const auto incident_faces = get_incident_fids_for_edge(t);

    cache.face_tags = FA[incident_faces[0]].tags;

    double max_energy = -1.0;
    for (const size_t fid : incident_faces) {
        max_energy = std::max(FA[fid].m_quality, max_energy);
    }
    cache.max_energy = max_energy;

    // cache edges
    simplex::Edge edge = simplex_from_edge(t);
    for (const size_t fid : incident_faces) {
        for (int j = 0; j < 3; j++) {
            const Tuple tup = tuple_from_edge(fid, j);
            simplex::Edge e = simplex_from_edge(tup);
            if (e == edge) {
                continue;
            }
            cache.changed_edges.try_emplace(e, m_edge_attribute[tup.eid(*this)]);
        }
    }

    return true;
}

bool SimWildMeshTri::swap_edge_after(const Tuple& t)
{
    auto& cache = swap_cache.local();
    const auto incident_faces = get_incident_fids_for_edge(t);

    auto& FA = m_face_attribute;

    double max_energy = -1.0;
    for (const size_t fid : incident_faces) {
        if (is_inverted(fid)) {
            return false;
        }
        double q = get_quality(fid);
        FA[fid].m_quality = q;
        max_energy = std::max(q, max_energy);

        FA[fid].tags = cache.face_tags;
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    // cached edges
    for (const auto& [e, e_attrs] : cache.changed_edges) {
        const auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attrs;
    }
    m_edge_attribute[t.eid(*this)].reset();

    return true;
}

void SimWildMeshTri::smooth_all_vertices(const size_t n_iters)
{
    for (size_t i = 0; i < n_iters; ++i) {
        // log_total_surface_energy();
        igl::Timer timer;
        timer.start();
        m_smooth_rejects.reset();
        std::vector<std::pair<std::string, Tuple>> collect_all_ops;
        for (const Tuple& t : get_vertices()) {
            collect_all_ops.emplace_back("vertex_smooth", t);
        }
        logger().info("vertex smoothing prepare time: {:.4}s", timer.getElapsedTimeInSec());
        logger().info("#V = {}", collect_all_ops.size());
        if (NUM_THREADS > 0) {
            timer.start();
            ExecutePass<SimWildMeshTri> executor(ExecutionPolicy::kPartition);
            executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
                return m.try_set_vertex_mutex_one_ring(e, task_id);
            };
            executor.num_threads = NUM_THREADS;
            executor(*this, collect_all_ops);
            logger().info("vertex smoothing time parallel: {:.4}s", timer.getElapsedTimeInSec());
        } else {
            timer.start();
            ExecutePass<SimWildMeshTri> executor(ExecutionPolicy::kSeq);
            executor(*this, collect_all_ops);
            logger().info("vertex smoothing time serial: {:.4}s", timer.getElapsedTimeInSec());
        }
        logger().info("\tsmooth: {}", m_smooth_rejects.to_string());
        if (m_params.debug_output) {
            write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
        }
    }
}

bool SimWildMeshTri::smooth_before(const Tuple& t)
{
    // Try to round first: smoothing is the operation that frees a vertex a split had to leave
    // rational, and set_smoothing_position writes a double, so a vertex that will not round
    // must not be smoothed -- its exact position is the only one keeping its ring valid.
    const bool r = round(t);

    const size_t vid = t.vid(*this);
    if (!m_vertex_attribute.at(vid).on_bbox_faces.empty()) {
        return false;
    }

    if (m_vertex_attribute[vid].m_is_rounded) {
        return true;
    }
    // Note: no need to roll back.
    return r;
}

bool SimWildMeshTri::smooth_after(const Tuple& t)
{
    // The body lives in wmtk::optimization::smooth_vertex_2d, shared with triwild.
    optimization::SmoothVertexOptions opts;
    opts.w_amips = m_params.w_amips;
    opts.w_envelope = m_params.w_envelope;
    opts.s_amips = m_s_amips;
    opts.s_envelope = m_s_envelope;
    opts.two_stage = true;
    opts.quality_veto_on_surface = false;

    return optimization::smooth_vertex_2d(*this, t, opts, m_solver.local(), &m_smooth_rejects);
}

Vector2d SimWildMeshTri::smoothing_position(const size_t vid) const
{
    return m_vertex_attribute[vid].m_posf;
}

void SimWildMeshTri::set_smoothing_position(const size_t vid, const Vector2d& p)
{
    m_vertex_attribute[vid].m_posf = p;
    m_vertex_attribute[vid].m_pos = to_rational(p);
}

bool SimWildMeshTri::is_inverted_f(const size_t fid) const
{
    const auto vs = oriented_tri_vids(fid);

    igl::predicates::exactinit();
    const auto res = igl::predicates::orient2d(
        m_vertex_attribute[vs[0]].m_posf,
        m_vertex_attribute[vs[1]].m_posf,
        m_vertex_attribute[vs[2]].m_posf);
    if (res == igl::predicates::Orientation::POSITIVE) {
        return false;
    }
    return true;
}

std::shared_ptr<SampleEnvelope> SimWildMeshTri::smoothing_energy_envelope(const size_t) const
{
    return m_envelope_orig;
}

std::shared_ptr<SampleEnvelope> SimWildMeshTri::smoothing_containment_envelope(const size_t) const
{
    // Not m_envelope_orig: the pull target and the containment test are different objects,
    // the same split the 3D mesh has.
    return m_envelope;
}

std::shared_ptr<polysolve::nonlinear::Problem> SimWildMeshTri::get_envelope_energy(
    const Tuple& t) const
{
    const double w = m_s_envelope * m_params.w_envelope;

    auto envelope_energy = std::make_shared<optimization::EnvelopeEnergy2D>(m_envelope_orig, w);
    return envelope_energy;
}

std::vector<std::array<double, 6>> SimWildMeshTri::get_amips_assembles(const Tuple& t) const
{
    const size_t vid = t.vid(*this);
    const auto& locs = get_one_ring_fids_for_vertex(t);

    const auto& VA = m_vertex_attribute;

    std::vector<std::array<double, 6>> assembles;
    assembles.reserve(locs.size());

    for (const size_t fid : locs) {
        if (is_inverted(fid)) {
            log_and_throw_error("Inverted face in amips assemble!");
        }
        std::array<size_t, 3> local_verts = oriented_tri_vids(fid);
        {
            size_t v_loc = 0;
            for (size_t i = 0; i < 3; ++i) {
                if (local_verts[i] == vid) {
                    v_loc = i;
                    break;
                }
            }
            std::array<size_t, 3> buf = local_verts;
            local_verts[0] = buf[v_loc];
            local_verts[1] = buf[(v_loc + 1) % 3];
            local_verts[2] = buf[(v_loc + 2) % 3];
        }

        std::array<double, 6> T;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 2; j++) {
                T[i * 2 + j] = VA[local_verts[i]].m_posf[j];
            }
        }
        assembles.push_back(T);
    }

    return assembles;
}

std::shared_ptr<polysolve::nonlinear::Problem> SimWildMeshTri::get_amips_energy(
    const Tuple& t) const
{
    const double w = m_params.w_amips > 0 ? m_s_amips * m_params.w_amips : 1;

    const auto assembles = get_amips_assembles(t);
    auto amips_energy = std::make_shared<optimization::AMIPSEnergy2D>(assembles, w);
    assert(amips_energy->initial_position() == m_vertex_attribute.at(t.vid(*this)).m_posf);
    return amips_energy;
}


void SimWildMeshTri::log_total_surface_energy()
{
    double e_sum = 0;
    double e_amips = 0;
    double e_smooth = 0;
    double e_envelope = 0;
    size_t n_pts = 0;

    const auto& VA = m_vertex_attribute;
    for (const Tuple& t : get_vertices()) {
        const size_t vid = t.vid(*this);
        if (!VA[vid].m_is_on_surface) {
            continue;
        }

        const Vector2d old_pos = VA[vid].m_posf;

        auto amips_energy = get_amips_energy(t);

        auto envelope_energy = get_envelope_energy(t);

        auto energy_sum = std::make_shared<optimization::EnergySum>();
        energy_sum->add_energy(amips_energy);
        energy_sum->add_energy(envelope_energy);

        e_sum += energy_sum->value(old_pos);
        e_amips += amips_energy->value(old_pos);
        e_envelope += envelope_energy->value(old_pos);

        ++n_pts;
    }

    logger().warn(
        ">>>>> Energies <<<<<\nSUM = {}\n  AMIPS = {}\n  Envelope = {}\n  #V = {}",
        e_sum,
        e_amips,
        e_envelope,
        n_pts);
}


bool SimWildMeshTri::is_inverted(const std::array<size_t, 3>& vs) const
{
    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded) {
        igl::predicates::exactinit();
        const auto res = igl::predicates::orient2d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf);
        if (res == igl::predicates::Orientation::POSITIVE) {
            return false;
        }
        return true;
    } else {
        const Vector2r& v0 = m_vertex_attribute[vs[0]].m_pos;
        const Vector2r& v1 = m_vertex_attribute[vs[1]].m_pos;
        const Vector2r& v2 = m_vertex_attribute[vs[2]].m_pos;
        const Vector2r a = v1 - v0;
        const Vector2r b = v2 - v0;
        const Rational res = a.x() * b.y() - a.y() * b.x();
        return !(res > 0);
    }
}

bool SimWildMeshTri::is_inverted(const Tuple& loc) const
{
    return is_inverted(oriented_tri_vids(loc));
}

bool SimWildMeshTri::is_inverted(const size_t fid) const
{
    return is_inverted(oriented_tri_vids(fid));
}

double SimWildMeshTri::get_quality(const std::array<size_t, 3>& vs) const
{
    std::array<Vector2d, 3> ps;
    for (size_t k = 0; k < 3; k++) {
        ps[k] = m_vertex_attribute[vs[k]].m_posf;
    }
    double energy = -1.;
    {
        std::array<double, 6> T;
        for (size_t k = 0; k < 3; k++)
            for (size_t j = 0; j < 2; j++) {
                T[k * 2 + j] = ps[k][j];
            }
        energy = AMIPS2D_energy(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 2 - 1e-3) {
        return MAX_ENERGY;
    }
    return energy;
}

double SimWildMeshTri::get_quality(const Tuple& loc) const
{
    return get_quality(oriented_tri_vids(loc));
}

double SimWildMeshTri::get_quality(const size_t fid) const
{
    return get_quality(oriented_tri_vids(fid));
}

bool SimWildMeshTri::round(const Tuple& v)
{
    const size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) {
        return true;
    }

    const Vector2r old_pos = m_vertex_attribute[i].m_pos;
    m_vertex_attribute[i].m_pos = to_rational(m_vertex_attribute[i].m_posf);
    // Set before the loop so is_inverted takes the float path: the question being asked is
    // exactly whether the ROUNDED position keeps every incident face valid.
    m_vertex_attribute[i].m_is_rounded = true;
    for (const Tuple& f : get_one_ring_tris_for_vertex(v)) {
        if (is_inverted(f)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_pos = old_pos;
            return false;
        }
    }

    return true;
}

size_t SimWildMeshTri::round_all_vertices()
{
    if (m_all_rounded.load(std::memory_order_relaxed)) {
        return 0;
    }

    size_t reclaimed = 0, still_unrounded = 0;
    for (const Tuple& v : get_vertices()) {
        if (m_vertex_attribute[v.vid(*this)].m_is_rounded) {
            continue;
        }
        if (round(v)) {
            ++reclaimed;
        } else {
            ++still_unrounded;
        }
    }

    if (still_unrounded == 0) {
        m_all_rounded.store(true, std::memory_order_relaxed);
    }
    if (reclaimed > 0 || still_unrounded > 0) {
        logger().info(
            "rounding sweep: reclaimed {}, still unrounded {}",
            reclaimed,
            still_unrounded);
    }
    return reclaimed;
}

bool SimWildMeshTri::round_and_check_all_rounded()
{
    round_all_vertices();
    return m_all_rounded.load(std::memory_order_relaxed);
}

double SimWildMeshTri::triangle_area(const size_t fid) const
{
    const auto vs = oriented_tri_vids(fid);
    const Vector2d& p0 = m_vertex_attribute[vs[0]].m_posf;
    const Vector2d& p1 = m_vertex_attribute[vs[1]].m_posf;
    const Vector2d& p2 = m_vertex_attribute[vs[2]].m_posf;
    const double area =
        0.5 * std::abs((p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]));
    return area;
}

bool SimWildMeshTri::is_edge_on_surface(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (!m_vertex_attribute.at(vs[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vs[1]).m_is_on_surface) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool SimWildMeshTri::is_edge_on_surface(const std::array<size_t, 2>& vids) const
{
    if (!m_vertex_attribute.at(vids[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vids[1]).m_is_on_surface) {
        return false;
    }

    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool SimWildMeshTri::is_edge_on_bbox(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (m_vertex_attribute.at(vs[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vs[1]).on_bbox_faces.empty()) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

bool SimWildMeshTri::is_edge_on_bbox(const std::array<size_t, 2>& vids) const
{
    if (m_vertex_attribute.at(vids[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vids[1]).on_bbox_faces.empty()) {
        return false;
    }
    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

void SimWildMeshTri::mesh_improvement(int max_its)
{
    ////preprocessing
    partition_mesh_morton();

    // write_vtu(fmt::format("debug_{}", m_m_debug_print_counter++));

    wmtk::logger().info("========it pre========");
    const double stop_energy = m_params.stop_energy;
    local_operations({{0, 1, 0, 0}}, true);

    ////operation loops
    double pre_quality_rel = 0.;
    check_mesh_quality(pre_quality_rel, true);

    int refine_cooldown =
        m_params.stuck_refine_cooldown; // iterations left before stuck-refine may fire again
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);
        double quality_rel = local_operations({{1, 1, 1, 1}});

        ///energy check
        logger().info("max rel quality {}", quality_rel);
        if (check_mesh_quality(quality_rel, true)) {
            // Quality alone is not a sufficient termination condition -- see
            // round_and_check_all_rounded. Keep iterating while anything is un-rounded, and
            // let max_its bound the run as before.
            if (round_and_check_all_rounded()) {
                break;
            }
            logger().info("quality target reached, but some vertices are un-rounded; continuing");
        }
        consolidate_mesh();

        wmtk::logger().info("#V = {}, #F = {}", vert_capacity(), tri_capacity());

        /// sizing field: when the max energy stalls, refine around the worst
        /// elements to escape stuck configurations (replaces the old global
        /// adjust_sizing_field mechanism). After a refinement, wait
        /// stuck_refine_cooldown iterations so the operations get full passes on
        /// the new sizing field before more refinement is added.
        logger().info("pre_quality_rel = {:.6}, quality_rel = {:.6}", pre_quality_rel, quality_rel);
        if (refine_cooldown > 0) {
            --refine_cooldown;
        } else if (
            it > 0 && quality_rel > 1.0 &&
            (pre_quality_rel - quality_rel) <=
                m_params.stuck_refine_stall_eps * (quality_rel - 1.0)) {
            logger().info(">>>>stuck-refine (maxE {:.6} stalled)...", quality_rel);
            refine_sizing_around_worst();
            logger().info(">>>>stuck-refine finished...");
            refine_cooldown = m_params.stuck_refine_cooldown;
        }
        pre_quality_rel = std::min(pre_quality_rel, quality_rel);
    }

    wmtk::logger().info("========it post========");
    local_operations({{0, 1, 0, 0}});

    // The post pass is collapse-only, so it cannot un-round anything; this is the last chance
    // to reclaim a vertex the loop left rational because it ran out of iterations.
    round_all_vertices();
}

double SimWildMeshTri::local_operations(const std::array<int, 4>& ops, bool collapse_limit_length)
{
    igl::Timer timer;

    double quality_rel = 0;

    auto sanity_checks = [this]() {
        if (!m_params.perform_sanity_checks) {
            return;
        }
        logger().info("Perform sanity checks...");
        const auto faces = get_edges_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : faces) {
            const auto& p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto& p1 = m_vertex_attribute[verts[1]].m_posf;
            if (m_envelope->is_outside(std::array<Vector2d, 2>{{p0, p1}})) {
                logger().error("Edge {} is outside!", verts);
            }
        }

        // check for inverted faces
        for (const Tuple& t : get_faces()) {
            if (!is_inverted(t)) {
                continue;
            }
            const auto vs = oriented_tri_vids(t);
            logger().error("Face {} is inverted! Vertices = {}", t.fid(*this), vs);
        }
        logger().info("Sanity checks done.");
    };

    sanity_checks();

    timer.start();
    for (int i = 0; i < ops.size(); i++) {
        if (i == 0) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==splitting {}==", n);
                split_all_edges();
                logger().info(
                    "#V = {}, #F = {} after split",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            check_mesh_quality(quality_rel, true);
            sanity_checks();
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                logger().info(
                    "#V = {}, #F = {} after collapse",
                    get_vertices().size(),
                    get_faces().size());
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            check_mesh_quality(quality_rel, true);
            sanity_checks();
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                logger().info("==swapping {}==", n);
                size_t cnt_success = swap_all_edges();
                if (cnt_success == 0) {
                    break;
                }
            }
            if (m_params.debug_output) {
                write_vtu(fmt::format("debug_{}", m_debug_print_counter++));
            }
            check_mesh_quality(quality_rel, true);
            sanity_checks();
        } else if (i == 3) {
            logger().info("==smoothing ==");
            smooth_all_vertices(ops[i]);
            // Reclaim whatever smoothing just made roundable: once per iteration, after all of
            // its passes. Here rather than at the end of the run because smoothing is what
            // frees a stuck vertex, and because a vertex left rational makes every incident
            // face take the exact-arithmetic path in is_inverted -- so rounding early keeps
            // the following passes on doubles.
            //
            // Guarded on ops[i] so it really is once per iteration: this branch is also
            // entered by the collapse-only pre and post passes, which pass ops[3] == 0 and
            // have no smoothing for the sweep to follow.
            if (ops[i] > 0) {
                round_all_vertices();
            }
            check_mesh_quality(quality_rel, true);
            sanity_checks();
        }
    }
    check_mesh_quality(quality_rel, true);
    logger().info("time = {:.4}s", timer.getElapsedTimeInSec());


    return quality_rel;
}

std::tuple<double, double> SimWildMeshTri::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple tup = tuple_from_tri(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        const double q = m_face_attribute[tup.fid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += q;
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(max_energy, avg_energy);
}

} // namespace wmtk::components::simwild::tri