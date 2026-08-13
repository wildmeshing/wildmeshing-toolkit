#include "SimWildMeshTri.hpp"

#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
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
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/EnergySum.hpp>
#include <wmtk/optimization/EnvelopeEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/predicates.hpp>

#include <wmtk/utils/partition_utils.hpp>
#include "expression_parser/Parser.hpp"

namespace wmtk::components::simwild::tri {

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
        m_envelope = std::make_shared<SampleEnvelope>(!m_sim_params.use_sample_envelope);
        m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
        logger().info(
            "Envelope: {} (eps {:.6})",
            m_envelope->use_exact ? "EXACT" : "sampled",
            m_envelope_eps);
    } else if (m_sim_params.operation == "remeshing" && m_sim_params.check_envelope_at_init) {
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
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_sim_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_sim_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_sim_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_sim_params.box_max[k]) {
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

    m_envelope = std::make_shared<SampleEnvelope>(!m_sim_params.use_sample_envelope);
    m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
    logger().info(
        "Envelope: {} (eps {:.6})",
        m_envelope->use_exact ? "EXACT" : "sampled",
        m_envelope_eps);
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

std::tuple<double, double> SimWildMeshTri::optimization_quality_stats()
{
    double max_quality = -1.;
    double avg_quality = 0.;
    size_t count = 0;
    for (size_t fid = 0; fid < tri_capacity(); ++fid) {
        if (!tuple_from_tri(fid).is_valid(*this)) continue;
        const double quality = quality_rel(fid);
        max_quality = std::max(max_quality, quality);
        avg_quality += quality;
        ++count;
    }
    if (count > 0) avg_quality /= count;
    return {max_quality, avg_quality};
}

std::vector<size_t> SimWildMeshTri::active_vertices() const
{
    // Normalize by the tag-dependent target before applying the shared activity margin.
    return utils::active_vertices(
        vert_capacity(),
        tri_capacity(),
        [this](size_t fid) { return tuple_from_tri(fid).is_valid(*this); },
        [this](size_t fid) { return quality_rel(fid); },
        [this](size_t fid) { return oriented_tri_vids(fid); },
        m_params.skip_good_regions_margin,
        [this](size_t vid) { return m_vertex_attribute[vid].m_is_on_surface; });
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

size_t SimWildMeshTri::refine_sizing_around_worst(double)
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

    // Match TriWild: the next split pass must take the longest edge of each selected worst
    // triangle even when its ordinary length gate says no.
    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [_, fid] : worst) {
            m_force_split_edges.insert(
                utils::longest_edge(oriented_tri_vids(fid), [this](size_t vid) -> const Vector2d& {
                    return m_vertex_attribute[vid].m_posf;
                }));
        }
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

bool SimWildMeshTri::split_adjust_position(const size_t v_new, const std::vector<Tuple>& children)
{
    if (!m_voronoi_split_fn || !m_vertex_attribute[v_new].m_is_rounded) return true;

    const auto& cache = split_cache.local();
    const size_t v1 = cache.v1_id;
    const size_t v2 = cache.v2_id;
    auto& p = m_vertex_attribute[v_new].m_posf;

    Vector2d p0 = m_vertex_attribute[v1].m_posf;
    Vector2d p1 = m_vertex_attribute[v2].m_posf;
    if (m_voronoi_split_fn(p0) >= 0) std::swap(p0, p1);

    for (int i = 0; i < 20; ++i) {
        p = 0.5 * (p0 + p1);
        m_vertex_attribute[v_new].m_pos = to_rational(p);
        bool inverted = false;
        for (const Tuple& child : children) {
            if (is_inverted(child)) {
                inverted = true;
                break;
            }
        }
        if (inverted || (p1 - p0).squaredNorm() < 1e-20) break;
        if (m_voronoi_split_fn(p) < 0) {
            p0 = p;
        } else {
            p1 = p;
        }
    }

    for (const Tuple& child : children) {
        if (!is_inverted(child)) continue;
        logger().warn("Voronoi split inverted a face; reverting to the TriWild midpoint");
        p = 0.5 * (m_vertex_attribute[v1].m_posf + m_vertex_attribute[v2].m_posf);
        m_vertex_attribute[v_new].m_pos = to_rational(p);
        break;
    }
    return true;
}
bool SimWildMeshTri::collapse_quality_allowed(
    size_t v1,
    size_t fid,
    double quality,
    double ring_max) const
{
    return !m_vertex_attribute.at(v1).m_is_rounded || quality <= target_quality(fid) ||
           quality <= ring_max;
}

void SimWildMeshTri::collapse_after_vertex(size_t, size_t v2)
{
    // SimWild's tracked edges are derived from neighboring cell tags. Rebuild the local
    // interface instead of retaining an OR-merge of the two old edge flags.
    std::set<size_t> affected_vertices;
    affected_vertices.insert(v2);

    for (const size_t fid : get_one_ring_fids_for_vertex(v2)) {
        if (!tuple_from_tri(fid).is_valid(*this)) continue;
        for (int j = 0; j < 3; ++j) {
            const Tuple edge = tuple_from_edge(fid, j);
            const auto opposite = edge.switch_face(*this);
            const bool is_interface =
                opposite.has_value() &&
                m_face_attribute[fid].tags != m_face_attribute[opposite->fid(*this)].tags;
            m_edge_attribute[edge.eid(*this)].m_is_surface_fs = is_interface;
            const auto evs = get_edge_vids(edge);
            affected_vertices.insert(evs.begin(), evs.end());
        }
    }

    for (const size_t vid : affected_vertices) {
        bool on_interface = false;
        for (const Tuple& edge : get_one_ring_edges_for_vertex(vid)) {
            if (m_edge_attribute[edge.eid(*this)].m_is_surface_fs) {
                on_interface = true;
                break;
            }
        }
        m_vertex_attribute[vid].m_is_on_surface = on_interface;
    }
}

std::shared_ptr<polysolve::nonlinear::Problem> SimWildMeshTri::get_envelope_energy(
    const Tuple& t) const
{
    const double w = m_s_envelope * m_params.w_envelope;

    auto envelope_energy = std::make_shared<optimization::ExactDistanceEnergy2D>(m_envelope, w);
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

} // namespace wmtk::components::simwild::tri
