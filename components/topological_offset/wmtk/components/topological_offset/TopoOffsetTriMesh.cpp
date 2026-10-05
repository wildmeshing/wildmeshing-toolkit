#include "TopoOffsetTriMesh.h"
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <paraviewo/VTUWriter.hpp>
#include <queue>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/predicates.hpp>


namespace wmtk::components::topological_offset {


void TopoOffsetTriMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& F,
    const MatrixSi& F_tags,
    const MatrixXd& V_env,
    const MatrixXi& F_env,
    const std::vector<std::string>& tag_names,
    const std::string& curve_name)
{
    // assert dimensions
    assert(V.cols() == 2);
    assert(F.cols() == 3);
    assert(F.rows() == F_tags.rows());
    assert((V_env.rows() == 0) || (V_env.cols() == 2));
    assert((F_env.rows() == 0) || (F_env.cols() == 2)); // edges, envelope
    m_tags_count = F_tags.cols() + 1; // + 1 for ambient

    // initialize connectivity
    init(F);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.resize(V.rows());
    m_edge_attribute.resize(3 * F.rows());
    m_face_attribute.resize(F.rows());

    // set envelope data
    if (V_env.rows() > 0) {
        logger().info(
            "Envelope ({} vertices, {} edges) found. will be retained in output.",
            V_env.rows(),
            F_env.rows());
        m_has_envelope = true;
        m_V_envelope = V_env;
        m_F_envelope = F_env;
    }

    // set tag string/id maps. Internally, ambient is explicit
    m_tag_id_to_name[0] = "ambient";
    m_tag_name_to_id["ambient"] = 0;
    for (int64_t i = 0; i < tag_names.size(); i++) {
        m_tag_id_to_name[i + 1] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i + 1;
    }

    // add any new tags to map
    for (const std::string& tag : m_offset_params.offset_output_tag) {
        if (std::find(tag_names.begin(), tag_names.end(), tag) == tag_names.end()) {
            // INFO: creating the requested output tag is the normal path, not a defect.
            logger().info("Tag '{}' does not exist. Adding to mesh.", tag);
            int64_t new_id = m_tag_id_to_name.size();
            m_tag_id_to_name[new_id] = tag;
            m_tag_name_to_id[tag] = new_id;
            m_tags_count++;
        }
    }

    // The curve group is registered as a tag so an OPEN curve can be selected as the input
    // complex; offset_selection is otherwise an expression over face tags. Membership is decided
    // geometrically below: triwild writes its curves with their own vertices, nothing matches by
    // index, so an edge inside the polyline's tube (the eps the tag envelopes use) carries it.
    if (F_env.rows() > 0) {
        m_curve_V = V_env;
        m_curve_E = F_env;
        std::string name = curve_name.empty() ? "curve" : curve_name;
        while (m_tag_name_to_id.count(name)) name += "_";
        m_curve_tag = int64_t(m_tag_id_to_name.size());
        m_tag_id_to_name[m_curve_tag] = name;
        m_tag_name_to_id[name] = m_curve_tag;
        m_tags_count++;
    }
    // collect int ids for offset output
    for (const std::string& name : m_offset_params.offset_output_tag) {
        m_offset_output_tag_ids.insert(m_tag_name_to_id[name]);
    }

    // One mask bit per input tag, ambient included, in id order. Assigned here, once the maps are
    // complete and before init_surfaces_and_boundaries() seeds the vertex masks. Tags introduced
    // later (the band's output tag) get no bit: boundary membership is a property of the input
    // partition, which is why the masks are propagated rather than recomputed from current tags.
    if (m_tag_id_to_name.size() > 62) {
        log_and_throw_error(
            "Per-tag boundary envelopes support at most 62 input tags (two mask bits are "
            "reserved for the domain wall and the input complex boundary), got {}",
            m_tag_id_to_name.size());
    }
    m_tag_bit.clear();
    for (const auto& [tag_id, name] : m_tag_id_to_name) {
        const int bit = int(m_tag_bit.size());
        m_tag_bit[tag_id] = bit;
    }
    // The two reserved bits of the WallComplex setup, see EnvelopeSetup.
    m_tag_bit[m_wall_tag] = int(m_tag_bit.size());
    m_tag_bit[m_complex_tag] = int(m_tag_bit.size());

    // propagate tags to faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        for (int j = 0; j < F_tags.cols(); j++) {
            if (F_tags.coeff(f_id, j) == 1) {
                m_face_attribute[f_id].tags.insert(j + 1);
            }
        }
        if (m_face_attribute[f_id].tags.size() == 0) { // tri is ambient
            m_face_attribute[f_id].tags.insert(0);
        }
    }

    // check for no ambient overlap
    assert(ambient_assert());

    // Set position of verts. Through set_vertex_position(), not by assigning m_posf alone: m_pos
    // and m_is_rounded must be filled too, or is_inverted() takes the rational path against a
    // default (0,0), reports every face incident to an input vertex inverted, and smooth_before()
    // then silently refuses every original input vertex.
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        set_vertex_position(v_id, Vector2d(V.row(v_id)));
    }
    // Once at load too, before the masks are seeded: init_surfaces_and_boundaries() reads on_curve
    // to put the curve group's edges in its tube, and a curve that bounds no region is held by
    // nothing otherwise. label_input_complex() re-derives it afterwards.
    classify_curve_edges();
    init_surfaces_and_boundaries();
}


void TopoOffsetTriMesh::init_surfaces_and_boundaries()
{
    const auto edges = get_edges();
    logger().info("E = {}", edges.size());

    // Which edges are tracked region boundaries: an interior edge whose two faces carry different
    // tag sets, an edge on the curve group, or a wall edge (no opposite face: the ambient region
    // against the unmeshed outside). These flags are topology the operations maintain. Which
    // tube holds an edge, and the vertex masks, are build_boundary_envelopes()'s.
    size_t n_edges_tracked = 0;
    for (const Tuple& e : edges) {
        const size_t eid = e.eid(*this);
        const bool on_curve = m_edge_extra[eid].on_curve;
        const std::optional<Tuple> f_opp = e.switch_face(*this);
        if (f_opp) {
            const auto& tag0 = m_face_attribute[e.fid(*this)].tags;
            const auto& tag1 = m_face_attribute[f_opp->fid(*this)].tags;
            if (tag0 == tag1 && !on_curve) continue;
        }
        m_edge_attribute[eid].m_is_surface_fs = true;
        ++n_edges_tracked;
        const size_t v1 = e.vid(*this);
        const size_t v2 = e.switch_vertex(*this).vid(*this);
        if (f_opp) {
            for (const size_t v : {v1, v2}) m_vertex_extra[v].m_is_on_region = true;
        }
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
    }
    if (n_edges_tracked > 0) build_boundary_envelopes("load", EnvelopeSetup::PerTag);

    // track bounding box. box_min/box_max are only set by Parameters::init(), which a mesh built
    // from a default-constructed Parameters never calls -- skip rather than index out of bounds.
    //
    // on_bbox_faces carries which wall (2k / 2k+1 for the min / max side of axis k), not merely
    // that there is one, so the split's set_intersection of its endpoints can tell a corner vertex
    // from an edge one, exactly as in 3D.
    if (m_offset_params.box_min.size() >= 2 && m_offset_params.box_max.size() >= 2) {
        for (const Tuple& e : edges) {
            if (e.switch_face(*this)) continue; // interior: not on the wall
            const size_t v1 = e.vid(*this);
            const size_t v2 = e.switch_vertex(*this).vid(*this);
            int on_bbox = -1;
            for (int k = 0; k < 2; ++k) {
                if (m_vertex_attribute[v1].m_posf[k] == m_offset_params.box_min[k] &&
                    m_vertex_attribute[v2].m_posf[k] == m_offset_params.box_min[k]) {
                    on_bbox = k * 2;
                    break;
                }
                if (m_vertex_attribute[v1].m_posf[k] == m_offset_params.box_max[k] &&
                    m_vertex_attribute[v2].m_posf[k] == m_offset_params.box_max[k]) {
                    on_bbox = k * 2 + 1;
                    break;
                }
            }
            if (on_bbox < 0) {
                continue;
            }
            m_edge_attribute[e.eid(*this)].m_is_bbox_fs = on_bbox;
            for (const size_t vid : {v1, v2}) {
                m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
            }
        }
        for (const Tuple& v : get_vertices()) {
            wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces);
        }
    }
}


std::string TopoOffsetTriMesh::envelope_key_name(const int64_t tag) const
{
    if (tag == m_wall_tag) return "wall";
    if (tag == m_complex_tag) return "input_complex";
    const auto it = m_tag_id_to_name.find(tag);
    return it == m_tag_id_to_name.end() ? fmt::format("tag#{}", tag) : it->second;
}

bool TopoOffsetTriMesh::edge_is_complex_boundary(const Tuple& e) const
{
    const bool a = m_face_extra[e.fid(*this)].label == 1;
    const std::optional<Tuple> opp = e.switch_face(*this);
    const bool b = opp ? (m_face_extra[opp->fid(*this)].label == 1) : false;
    if (a != b) return true; // a face of the complex on exactly one side
    if (a && b) return false; // interior to the complex
    // No complex face on either side: the edge is itself a piece of the complex (a curve
    // selection, or an edge the selection expression labelled), or it is not on it at all.
    return m_edge_extra[e.eid(*this)].label == 1;
}

void TopoOffsetTriMesh::build_boundary_envelopes(const char* when, const EnvelopeSetup setup)
{
    m_envelope_eps = m_offset_params.envelope_size;

    // Fresh: every mask from the tracked edges as they stand, nothing carried over.
    for (const Tuple& v : get_vertices()) m_vertex_extra[v.vid(*this)].m_boundary_mask = 0;

    std::map<int64_t, std::vector<Eigen::Vector2i>> buckets;
    size_t n_tracked = 0, n_wall = 0, n_complex = 0, n_free = 0;
    for (const Tuple& e : get_edges()) {
        const size_t eid = e.eid(*this);
        // Region-class tracked edges only: the front has its own tube.
        if (!m_edge_attribute[eid].m_is_surface_fs || edge_is_offset(eid)) continue;
        ++n_tracked;
        const std::optional<Tuple> f_opp = e.switch_face(*this);
        CellTag keys;
        if (setup == EnvelopeSetup::WallComplex) {
            if (!f_opp) {
                keys.insert(m_wall_tag);
                ++n_wall;
            }
            if (edge_is_complex_boundary(e)) {
                keys.insert(m_complex_tag);
                ++n_complex;
            }
            if (keys.empty()) ++n_free;
        } else {
            // Whose boundary this edge is. Interior edge: every tag on exactly one side (the
            // symmetric difference; a tag present on both sides has no boundary here). Wall
            // edge: every tag of its one face, which is how ambient's tube comes to hold the
            // box. The curve group's edges are in its tube too.
            if (m_edge_extra[eid].on_curve) keys.insert(m_curve_tag);
            if (f_opp) {
                const auto& tag0 = m_face_attribute[e.fid(*this)].tags;
                const auto& tag1 = m_face_attribute[f_opp->fid(*this)].tags;
                std::set_symmetric_difference(
                    tag0.begin(),
                    tag0.end(),
                    tag1.begin(),
                    tag1.end(),
                    std::inserter(keys, keys.begin()));
            } else {
                keys = m_face_attribute[e.fid(*this)].tags;
                ++n_wall;
            }
            // The band's output tags are not region boundaries: the front is the offset tube's.
            for (const int64_t t : m_offset_output_tag_ids) keys.erase(t);
        }
        const size_t v1 = e.vid(*this);
        const size_t v2 = e.switch_vertex(*this).vid(*this);
        const uint64_t bits = tag_bits(keys);
        for (const size_t v : {v1, v2}) m_vertex_extra[v].m_boundary_mask |= bits;
        for (const int64_t t : keys) buckets[t].emplace_back(int(v1), int(v2));
    }

    std::vector<Eigen::Vector2d> tempV(vert_capacity());
    for (size_t i = 0; i < vert_capacity(); ++i) tempV[i] = m_vertex_attribute[i].m_posf;

    m_tag_envelopes.clear();
    {
        std::lock_guard<std::mutex> lock(m_isect_mutex);
        m_isect_cache.clear();
        m_offset_isect_cache.clear();
    }
    const bool exact_ok = std::isfinite(m_envelope_eps) && m_envelope_eps > 0.;
    std::vector<std::shared_ptr<SampleEnvelope>> members;
    std::string per_tag_log;
    for (const auto& [tag, bucket] : buckets) {
        if (bucket.empty()) continue;
        auto env = std::make_shared<SampleEnvelope>(/*exact=*/exact_ok);
        env->init(tempV, bucket, m_envelope_eps);
        m_tag_envelopes[tag] = env;
        members.push_back(env);
        per_tag_log += fmt::format(" {}:{}", envelope_key_name(tag), bucket.size());
    }
    // The base's pointer survives as the union of the members -- inside any tube -- because
    // the shared engine's direct uses of it ask exactly that question. Everything else
    // dispatches per simplex through envelope_for_mask().
    m_envelope = members.empty() ? nullptr : std::make_shared<UnionEnvelope>(std::move(members));

    logger().info(
        "\t[envelopes @ {}] {}: {} region-boundary segments tracked ({} on the wall), eps {:.6g}, "
        "{} |{}{}",
        when,
        setup == EnvelopeSetup::WallComplex ? "wall + input complex" : "per tag",
        n_tracked,
        n_wall,
        m_envelope_eps,
        exact_ok ? "EXACT" : "sampled (no valid eps)",
        per_tag_log,
        setup == EnvelopeSetup::WallComplex
            ? fmt::format(
                  " | {} on the input complex boundary, {} held by nothing (plastic)",
                  n_complex,
                  n_free)
            : std::string());
}

void TopoOffsetTriMesh::mark_input_complex_vertices()
{
    // The only place the input complex is known: this runs after label_input_complex() has
    // evaluated the selection, whereas init_surfaces_and_boundaries() runs earlier and can only
    // see region boundaries -- which is why m_is_on_region, set there, is not this. Keys on the
    // vertex label, so a filled complex, a curve and an isolated point are all covered.
    size_t n = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const bool on_input = m_vertex_extra[vid].label == 1;
        m_vertex_extra[vid].m_is_on_input = on_input;
        n += on_input ? 1 : 0;
    }
    logger().info("\tInput-complex vertices: {}", n);
}


bool TopoOffsetTriMesh::ambient_assert()
{
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        bool has_ambient = (m_face_attribute[f_id].tags.count(0) != 0);
        if (has_ambient && (m_face_attribute[f_id].tags.size() != 1)) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTriMesh::classify_curve_edges()
{
    // See the declaration. Idempotent: every edge is answered, so a stale true is cleared.
    if (m_curve_tag < 0 || m_curve_E.rows() == 0) return;
    const double eps = m_offset_params.envelope_size;
    if (!(std::isfinite(eps) && eps > 0.)) {
        logger().warn(
            "\tCurve group '{}': no valid envelope_size to classify edges by; no edge carries it",
            m_tag_id_to_name[m_curve_tag]);
        return;
    }
    std::vector<Eigen::Vector2d> cv(m_curve_V.rows());
    for (int i = 0; i < m_curve_V.rows(); ++i) cv[i] = m_curve_V.row(i).head<2>();
    std::vector<Eigen::Vector2i> ce(m_curve_E.rows());
    for (int i = 0; i < m_curve_E.rows(); ++i)
        ce[i] = Eigen::Vector2i(m_curve_E(i, 0), m_curve_E(i, 1));
    SampleEnvelope curve(/*exact=*/true);
    curve.init(cv, ce, eps);
    size_t n_on = 0;
    for (const Tuple& e : get_edges()) {
        const Vector2d p = m_vertex_attribute[e.vid(*this)].m_posf;
        const Vector2d q = m_vertex_attribute[e.switch_vertex(*this).vid(*this)].m_posf;
        const bool on = !curve.is_outside(std::array<Vector2d, 2>{{p, q}});
        m_edge_extra[e.eid(*this)].on_curve = on;
        n_on += on ? 1 : 0;
    }
    logger().info(
        "\tCurve group '{}' (tag {}): {} polyline segments; {} of the mesh's edges lie inside its "
        "tube (eps {:.6g}) and carry the tag",
        m_tag_id_to_name[m_curve_tag],
        m_curve_tag,
        m_curve_E.rows(),
        n_on,
        eps);
}

void TopoOffsetTriMesh::label_input_complex()
{
    classify_curve_edges(); // the mesh may have been refined since the last call
    // ensure all tags exist in map
    const ExpressionPtr& expr = m_offset_params.offset_selection;
    const CellTag tags_involved = expr->tags_involved();
    for (const int64_t& tag : tags_involved) {
        auto it = m_tag_id_to_name.find(tag);
        if (it == m_tag_id_to_name.end()) {
            log_and_throw_error("Unknown tag given in offset_selection (id {})", tag);
        }
    }

    // only true if exactly one tag exists, ie "tag_0" (no &, |, !)
    bool single_body = (tags_involved.size() == 1) && expr->contains_only_or() &&
                       expr->contains_only_and() && expr->contains_only_not();

    if (single_body) { // single body mode
        m_singlebody = true;
        m_single_tag = *tags_involved.begin();
        if (m_single_tag == m_curve_tag) {
            // The curve is the complex: its edges and their vertices, no faces. The band then
            // grows on both sides of it, which is the offset of a curve, open or closed.
            // offset_in / offset_out have no meaning for it and are not consulted.
            size_t n = 0;
            for (const Tuple& e : get_edges()) {
                const size_t e_id = e.eid(*this);
                if (!m_edge_extra[e_id].on_curve) continue;
                m_edge_extra[e_id].label = 1;
                m_vertex_extra[e.vid(*this)].label = 1;
                m_vertex_extra[e.switch_vertex(*this).vid(*this)].label = 1;
                ++n;
            }
            logger().info(
                "Using the curve group '{}' as the complex: {} edges",
                m_tag_id_to_name[m_single_tag],
                n);
            if (n == 0)
                log_and_throw_error(
                    "offset_selection names the curve group but no mesh edge lies on it");
            mark_input_complex_vertices();
            return;
        }
        if (!(m_offset_params.offset_in || m_offset_params.offset_out)) {
            log_and_throw_error(
                "At least one of offset_in and offset_out must be true for singlebody mode.");
        }
        logger().info("Using single body mode for '{}'", m_tag_id_to_name[m_single_tag]);

        if (m_offset_params.offset_in &&
            m_offset_params.offset_out) { // input complex is boundary simplices
            auto faces = get_faces();
            for (const Tuple& f : faces) {
                size_t f_id = f.fid(*this);
                if (m_face_attribute[f_id].tags.count(m_single_tag) != 0) {
                    Tuple ftup = tuple_from_tri(f_id);
                    auto vs = oriented_tri_vids(f_id);
                    for (int i = 0; i < 3; i++) {
                        Tuple etup = tuple_from_edge(vs[i], vs[(i + 1) % 3], f_id);
                        auto other = etup.switch_face(*this);
                        if (!other || (m_face_attribute[other.value().fid(*this)].tags.count(
                                           m_single_tag) == 0)) {
                            size_t e_id = etup.eid(*this);
                            m_edge_extra[e_id].label = 1;
                            m_vertex_extra[vs[i]].label = 1;
                            m_vertex_extra[vs[(i + 1) % 3]].label = 1;
                        }
                    }
                }
            }
        } else if (m_offset_params.offset_in) { // input complex is everything outside body plus
                                                // boundary
            // faces (hacky but works)
            auto faces = get_faces();
            for (const Tuple& f : faces) {
                size_t f_id = f.fid(*this);
                if (m_face_attribute[f_id].tags.count(m_single_tag) == 0) {
                    m_face_extra[f_id].label = 1;
                    // propagate to edges and verts in tri
                    m_edge_extra[f.eid(*this)].label = 1;
                    m_edge_extra[f.switch_edge(*this).eid(*this)].label = 1;
                    m_edge_extra[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                    m_vertex_extra[f.vid(*this)].label = 1;
                    m_vertex_extra[f.switch_vertex(*this).vid(*this)].label = 1;
                    m_vertex_extra[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
                } else { // face is in body, check for boundary edges
                    auto vs = oriented_tri_vids(f_id);
                    for (int i = 0; i < 3; i++) {
                        Tuple etup = tuple_from_edge(vs[i], vs[(i + 1) % 3], f_id);
                        auto other = etup.switch_face(*this);
                        if (!other) {
                            m_edge_extra[etup.eid(*this)].label = 1;
                            m_vertex_extra[etup.vid(*this)].label = 1;
                            m_vertex_extra[etup.switch_vertex(*this).vid(*this)].label = 1;
                        }
                    }
                }
            }
        } else { // input complex is body itself
            auto faces = get_faces();
            for (const Tuple& f : faces) {
                size_t f_id = f.fid(*this);
                if (m_face_attribute[f_id].tags.count(m_single_tag) != 0) {
                    m_face_extra[f_id].label = 1;
                    // propagate to edges and verts in tri
                    m_edge_extra[f.eid(*this)].label = 1;
                    m_edge_extra[f.switch_edge(*this).eid(*this)].label = 1;
                    m_edge_extra[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                    m_vertex_extra[f.vid(*this)].label = 1;
                    m_vertex_extra[f.switch_vertex(*this).vid(*this)].label = 1;
                    m_vertex_extra[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
                }
            }
        }
    } else { // not single body mode. must evaluate expression

        // label faces
        auto faces = get_faces();
        for (const Tuple& f : faces) {
            size_t f_id = f.fid(*this);
            if (expr->eval(m_face_attribute[f_id].tags)) {
                m_face_extra[f_id].label = 1;
                // propagate to edges and verts in tri
                m_edge_extra[f.eid(*this)].label = 1;
                m_edge_extra[f.switch_edge(*this).eid(*this)].label = 1;
                m_edge_extra[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                m_vertex_extra[f.vid(*this)].label = 1;
                m_vertex_extra[f.switch_vertex(*this).vid(*this)].label = 1;
                m_vertex_extra[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        // label edges
        auto edges = get_edges();
        for (const Tuple& e : edges) {
            size_t e_id = e.eid(*this);
            if (m_edge_extra[e_id].label == 1) {
                continue;
            }

            CellTag adj_tags = m_face_attribute[e.fid(*this)].tags;
            auto other = e.switch_face(*this);
            if (other) {
                for (const int64_t& tag : m_face_attribute[other.value().fid(*this)].tags) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_edge_extra[e_id].label = 1;
                // propagate to vertices
                m_vertex_extra[e.vid(*this)].label = 1;
                m_vertex_extra[e.switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        // label vertices
        auto verts = get_vertices();
        for (const Tuple& v : verts) {
            size_t v_id = v.vid(*this);
            if (m_vertex_extra[v_id].label == 1) {
                continue;
            }

            CellTag adj_tags;
            auto one_ring_fids = get_one_ring_fids_for_vertex(v_id);
            for (const size_t& f_id : one_ring_fids) {
                for (const int64_t& tag : m_face_attribute[f_id].tags) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_vertex_extra[v_id].label = 1;
            }
        }
    }

    mark_input_complex_vertices();
}


bool TopoOffsetTriMesh::empty_input_complex()
{
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (m_vertex_extra[v_id].label == 1) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTriMesh::init_input_complex_bvh()
{
    // used a few times. just collect once
    auto faces = get_faces();
    auto edges = get_edges();
    auto verts = get_vertices();

    // to check if an edge is in closure of input complex faces
    std::map<simplex::Edge, bool> edge_in_closure;
    for (const Tuple& e : edges) {
        edge_in_closure[simplex_from_edge(e)] = false;
    }

    // to check if vertex is in closure of input complex faces and edges
    std::map<size_t, bool> vertex_in_closure;
    for (const Tuple& v : verts) {
        vertex_in_closure[v.vid(*this)] = false;
    }

    // Which input region each complex primitive belongs to is decided further down, once the
    // complex has been collected: a region is one connected piece of the input complex, not one
    // tag. See m_n_regions.
    // collect faces in input complex
    std::vector<simplex::Face> complex_faces;
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_extra[f_id].label == 1) {
            size_t v0 = f.vid(*this);
            size_t v1 = f.switch_vertex(*this).vid(*this);
            size_t v2 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
            complex_faces.emplace_back(v0, v1, v2);
            edge_in_closure[simplex::Edge(v0, v1)] = true;
            edge_in_closure[simplex::Edge(v1, v2)] = true;
            edge_in_closure[simplex::Edge(v0, v2)] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
            vertex_in_closure[v2] = true;
        }
    }

    // collect edges in input complex that are not contained in a face
    std::vector<simplex::Edge> complex_edges;
    for (const Tuple& e : edges) {
        simplex::Edge e_simp = simplex_from_edge(e);
        if (!edge_in_closure[e_simp] && m_edge_extra[e.eid(*this)].label == 1) {
            size_t v0 = e.vid(*this);
            size_t v1 = e.switch_vertex(*this).vid(*this);
            complex_edges.emplace_back(v0, v1);
            edge_in_closure[e_simp] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
        }
    }

    // collect vertices in input complex not contained in an edge or face
    std::vector<size_t> complex_verts;
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (!vertex_in_closure[v_id] && m_vertex_extra[v_id].label == 1) {
            complex_verts.push_back(v_id);
            vertex_in_closure[v_id] = true;
        }
    }

    // extract vertices included in simplicial complex
    std::vector<Vector2d> V_vec;
    std::map<size_t, size_t> v_index_map; // new = map[old]
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (vertex_in_closure[v_id]) {
            v_index_map[v_id] = V_vec.size();
            V_vec.push_back(m_vertex_attribute[v_id].m_posf);
        }
    }
    MatrixXd V(V_vec.size(), 2);
    for (int i = 0; i < V_vec.size(); i++) {
        V.row(i) = V_vec[i];
    }

    MatrixXi T(0, 4); // no tets

    MatrixXi F(complex_faces.size(), 3); // faces
    int index = 0;
    for (const simplex::Face& f_simp : complex_faces) {
        auto vs = f_simp.vertices();
        // NOTE: does id order matter here (i.e., in BVH class?)
        F.row(index) << v_index_map[vs[0]], v_index_map[vs[1]], v_index_map[vs[2]];
        index++;
    }

    MatrixXi E(complex_edges.size(), 2); // isolated edges
    index = 0;
    for (const simplex::Edge& e_simp : complex_edges) {
        auto vs = e_simp.vertices();
        E.row(index) << v_index_map[vs[0]], v_index_map[vs[1]];
        index++;
    }

    MatrixXi P(complex_verts.size(), 1); // isolated vertices
    index = 0;
    for (const size_t& v_id : complex_verts) {
        P(index, 0) = v_index_map[v_id];
        index++;
    }

    // One region per connected piece of the input complex -- see m_n_regions and
    // m_region_potentials. A region must not be one tag: one tag covering two pieces that never
    // touch makes them share a field, and the smooth potential's barriers then add across the
    // gap, so the level set bridges it and there is none to place a front on.
    //
    // A piece is a connected component under vertex connectivity: pieces meeting at a single point
    // share one offset there, so they must share one field. Read off the captured complex, not the
    // live mesh, so the numbering is fixed for the whole run.
    std::vector<int> comp_of(size_t(V.rows()), -1);
    {
        std::vector<int> parent(size_t(V.rows()));
        for (size_t i = 0; i < parent.size(); ++i) parent[i] = int(i);
        const std::function<int(int)> find = [&](int x) {
            while (parent[size_t(x)] != x) {
                parent[size_t(x)] = parent[size_t(parent[size_t(x)])];
                x = parent[size_t(x)];
            }
            return x;
        };
        const auto unite = [&](const int a, const int b) {
            const int ra = find(a), rb = find(b);
            if (ra != rb) parent[size_t(ra)] = rb;
        };
        for (int i = 0; i < F.rows(); ++i) {
            unite(F(i, 0), F(i, 1));
            unite(F(i, 1), F(i, 2));
        }
        for (int i = 0; i < E.rows(); ++i) unite(E(i, 0), E(i, 1));
        std::map<int, int> root_to_region;
        for (int i = 0; i < V.rows(); ++i) {
            const int r = find(i);
            const auto it = root_to_region.find(r);
            if (it == root_to_region.end()) {
                comp_of[size_t(i)] = int(root_to_region.size());
                root_to_region[r] = comp_of[size_t(i)];
            } else {
                comp_of[size_t(i)] = it->second;
            }
        }
        m_n_regions = int(root_to_region.size());
    }
    m_phi_vert_region.assign(comp_of.begin(), comp_of.end());
    logger().info(
        "\tInput complex: {} connected piece(s), one offset field each ({} complex vertices)",
        m_n_regions,
        V.rows());

    // The boundary curve, derived before the BVH so the one retained structure carries it. Phi's
    // 2D primitives are segments and points, so a solid input region enters as its boundary -- the
    // complex triangles' edges with exactly one incident complex triangle. Outside the region, the
    // only place an offset exists, distance to the region and to its boundary are the same number.
    std::map<simplex::Edge, int> boundary_count;
    for (size_t fi = 0; fi < complex_faces.size(); ++fi) {
        const auto vs = complex_faces[fi].vertices();
        for (const simplex::Edge e :
             {simplex::Edge(vs[0], vs[1]),
              simplex::Edge(vs[1], vs[2]),
              simplex::Edge(vs[0], vs[2])}) {
            ++boundary_count[e];
        }
    }

    // Every primitive's region is its own vertices' region: a primitive cannot span two pieces.
    std::vector<Eigen::Vector2i> phi_segs;
    std::vector<int64_t> phi_seg_region;
    for (const auto& [e_simp, count] : boundary_count) {
        if (count != 1) continue; // interior to the complex: carries no boundary geometry
        const auto vs = e_simp.vertices();
        phi_segs.emplace_back(v_index_map[vs[0]], v_index_map[vs[1]]);
        phi_seg_region.push_back(comp_of[size_t(phi_segs.back()[0])]);
    }
    for (int i = 0; i < E.rows(); ++i) { // isolated edges of the complex
        phi_segs.emplace_back(E(i, 0), E(i, 1));
        phi_seg_region.push_back(comp_of[size_t(E(i, 0))]);
    }

    MatrixXi E_phi(phi_segs.size(), 2);
    for (size_t i = 0; i < phi_segs.size(); ++i) {
        E_phi.row(i) = phi_segs[i];
    }

    // Isolated points: those of the complex, plus any vertex the boundary extraction left with
    // no segment at all (a complex that is a single triangle contributes its three edges, so
    // this only fires for genuinely isolated input vertices).
    std::vector<int> P_phi;
    for (int i = 0; i < P.rows(); ++i) {
        P_phi.push_back(P(i, 0));
    }

    // set BVH -- a fresh object rather than clear+reinit, so anything still holding the old one
    // keeps a coherent view.
    //
    // The edge set is the curve E_phi, not just the isolated edges E: the euclidean potential's
    // nearest_point_feature() runs on the BVH's edges and must see the boundary of a solid
    // complex, which the face set alone cannot answer. Indexing them leaves squared_dist() -- the
    // distance to the solid complex -- unchanged, since every boundary segment lies on a face.
    m_input_complex_bvh = std::make_shared<SimplicialComplexBVH>();
    m_input_complex_bvh->init(V, T, F, E_phi, P);

    // Kept, not built. The extraction must not diverge from the BVH's, so it is done here and
    // once; the potential itself needs target_distance and offset_dhat_factor, which a caller
    // wanting only the distance field has no reason to have set.
    m_phi_V = V;
    m_phi_E = E_phi;
    m_phi_F = F;
    m_phi_P = P_phi;
    m_phi_seg_region = phi_seg_region;
    m_phi_face_region.clear();
    for (int i = 0; i < F.rows(); ++i) m_phi_face_region.push_back(comp_of[size_t(F(i, 0))]);
    m_phi_point_region.clear();
    for (const int p : P_phi) m_phi_point_region.push_back(comp_of[size_t(p)]);
}


void TopoOffsetTriMesh::init_offset_potential()
{
    if (m_phi_V.rows() == 0 || !m_input_complex_bvh) {
        log_and_throw_error("init_offset_potential() called before init_input_complex_bvh()");
    }
    // Which field defines the offset; see OffsetPotential.hpp and the offset_field parameter.
    // Both are built from the same extraction (m_phi_V/E/P), so whichever is chosen measures the
    // same geometry the diagnostics do. The euclidean one queries m_input_complex_bvh, the only
    // input-complex structure this mesh keeps.
    const size_t n_input_segments = size_t(m_phi_E.rows()) + m_phi_P.size();

    if (m_offset_params.offset_field == "euclidean") {
        m_offset_potential = std::make_shared<EuclideanOffsetPotential2D>(
            m_input_complex_bvh,
            m_offset_params.target_distance);
        logger().info(
            "\tOffset field: EUCLIDEAN (exact distance), level d = {}, {} segments ({} of them "
            "isolated points)",
            m_offset_params.target_distance,
            n_input_segments,
            m_phi_P.size());
        init_region_potentials(m_offset_params.target_distance, 0.);
        return;
    }

    // dhat is sized to the offset it has to hold, not to target_distance alone: construction puts
    // the offset on the input triangulation's own cell boundaries, so how far out it lands is a
    // property of the input mesh, not a multiple of delta, and a fixed factor x delta fails when
    // delta is small relative to the background triangles.
    //
    // The floor keeps the configured factor authoritative whenever construction was good: dhat is
    // not a neutral scaling -- it selects the level c, so a purely data-driven dhat would give the
    // same geometry a different offset depending on how the input was meshed. Same rule as 3D.
    const double delta = m_offset_params.target_distance;
    const double reach = max_band_vertex_distance();
    const double dhat = std::max(m_offset_params.offset_dhat_factor * delta, 2. * reach);
    const double effective_factor = dhat / delta;
    if (reach > 0.) {
        logger().info(
            "\tdhat sized from the constructed offset: furthest offset vertex {:.6g} = {:.4g}x "
            "delta, so dhat = max({}x delta, 2x that) = {:.6g} = {:.4g}x delta",
            reach,
            reach / delta,
            m_offset_params.offset_dhat_factor,
            dhat,
            effective_factor);
    }
    m_offset_potential = std::make_shared<SmoothOffsetPotential2D>(
        m_phi_V,
        m_phi_E,
        MatrixXi(0, 3), // no triangle primitive in 2D
        m_phi_P,
        delta,
        effective_factor);
    init_region_potentials(delta, effective_factor);
}

void TopoOffsetTriMesh::init_region_potentials(const double delta, const double effective_factor)
{
    // See m_region_potentials. A connected input complex is one region, and that region's field
    // is the union field itself -- nothing to build, and every lookup falls through to it.
    m_region_potentials.clear();
    if (m_n_regions <= 1) {
        assign_band_regions();
        return;
    }
    const bool euclidean = m_offset_params.offset_field == "euclidean";
    for (size_t r = 0; r < size_t(m_n_regions); ++r) {
        // A primitive whose region is unknown (-1) is given to every field, conservatively.
        std::vector<int> erows, frows, pidx;
        for (int i = 0; i < m_phi_E.rows(); ++i) {
            if (m_phi_seg_region[size_t(i)] < 0 || m_phi_seg_region[size_t(i)] == int64_t(r))
                erows.push_back(i);
        }
        for (int i = 0; i < m_phi_F.rows(); ++i) {
            if (m_phi_face_region[size_t(i)] < 0 || m_phi_face_region[size_t(i)] == int64_t(r))
                frows.push_back(i);
        }
        for (size_t i = 0; i < m_phi_P.size(); ++i) {
            if (m_phi_point_region[i] < 0 || m_phi_point_region[i] == int64_t(r))
                pidx.push_back(int(i));
        }
        MatrixXi E_r(erows.size(), 2);
        for (size_t i = 0; i < erows.size(); ++i) E_r.row(i) = m_phi_E.row(erows[i]);
        MatrixXi F_r(frows.size(), 3);
        for (size_t i = 0; i < frows.size(); ++i) F_r.row(i) = m_phi_F.row(frows[i]);
        std::vector<int> P_r;
        for (const int i : pidx) P_r.push_back(m_phi_P[size_t(i)]);
        if (euclidean) {
            MatrixXi P_m(P_r.size(), 1);
            for (size_t i = 0; i < P_r.size(); ++i) P_m(i, 0) = P_r[i];
            auto bvh = std::make_shared<SimplicialComplexBVH>();
            bvh->init(m_phi_V, MatrixXi(0, 4), F_r, E_r, P_m);
            m_region_potentials.push_back(std::make_shared<EuclideanOffsetPotential2D>(bvh, delta));
        } else {
            m_region_potentials.push_back(
                std::make_shared<SmoothOffsetPotential2D>(
                    m_phi_V,
                    E_r,
                    MatrixXi(0, 3),
                    P_r,
                    delta,
                    effective_factor));
        }
        logger().info(
            "\tOffset field for region {} (one connected piece of the input complex): {} "
            "segments, {} faces, {} points -- the band grown from this piece is placed on this "
            "field alone",
            r,
            E_r.rows(),
            F_r.rows(),
            P_r.size());
    }
    assign_band_regions();
}


double TopoOffsetTriMesh::max_band_vertex_distance() const
{
    // How far the offset boundary actually ended up from the input complex, as a length. Exact
    // (BVH nearest point), not the straddle-edge upper bound: dhat selects the level c, so an
    // overestimate changes which curve the run solves for. Returns 0 when there is no offset
    // boundary yet, which is the signal to fall back to the configured factor.
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& e : get_edges()) {
        if (!edge_is_offset_surface_live(e)) continue;
        on_band[e.vid(*this)] = true;
        on_band[e.switch_vertex(*this).vid(*this)] = true;
    }
    double worst = 0.;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue;
        const Vector2d p = m_vertex_attribute[vid].m_posf;
        const Vector3d near3 = m_input_complex_bvh->nearest_point(VectorXd(p));
        worst = std::max(worst, (p - Vector2d(near3[0], near3[1])).norm());
    }
    return worst;
}


void TopoOffsetTriMesh::construct_offset(const std::filesystem::path& output_file)
{
    // make embedding simplicial
    logger().info("Creating simplicial embedding...");
    m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Midpoint;
    if (!is_simplicially_embedded()) {
        simplicial_embedding();
        bool dummy = is_simplicially_embedded();
    }
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_debug_frame("simplicial_embedding");
    }

    // repulsion_smoothing_passes: push the marched edges' outer ends out before the march.
    repulsion_smoothing();

    // initialize offset
    logger().info("Initializing offset...");
    // marching_tris() chooses the placement itself (construction_mode) and leaves the split mode
    // at Midpoint.
    marching_tris();
    m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Midpoint;
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_debug_frame("marching");
    }

    // Must stay outside the branch above and unconditional: consolidating renumbers, which
    // changes the order later passes enumerate operations in, which changes the run.
    consolidate_mesh();
    set_offset_tri_tags();
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_debug_frame("offset_tagged");
    }

    assert(ambient_assert());
}

size_t TopoOffsetTriMesh::flood_fill()
{
    size_t current_id = 0;
    std::vector<char> visited(vert_capacity(), 0);
    for (const Tuple& v : get_vertices()) {
        const size_t v_id = v.vid(*this);
        if (m_vertex_extra[v_id].label == 0) continue; // vertex not in complex
        if (visited[v_id]) continue; // vertex already visited

        visited[v_id] = 1;
        std::queue<size_t> bfs_queue;
        for (const size_t other_v_id : connected_components_helper(v_id)) {
            if (!visited[other_v_id]) bfs_queue.push(other_v_id);
        }
        while (!bfs_queue.empty()) {
            const size_t curr_vid = bfs_queue.front();
            bfs_queue.pop();
            if (visited[curr_vid]) continue;
            visited[curr_vid] = 1;
            for (const size_t other_v_id : connected_components_helper(curr_vid)) {
                if (!visited[other_v_id]) bfs_queue.push(other_v_id);
            }
        }
        current_id++;
    }
    return current_id;
}

void TopoOffsetTriMesh::relabel_input_complex()
{
    // label_input_complex() only ever sets labels to 1; every label goes back to 0 first, so a
    // simplex an operation moved out of the complex is not left marked. As in 3D.
    for (const Tuple& v : get_vertices()) m_vertex_extra[v.vid(*this)].label = 0;
    for (const Tuple& e : get_edges()) m_edge_extra[e.eid(*this)].label = 0;
    for (const Tuple& f : get_faces()) m_face_extra[f.fid(*this)].label = 0;
    label_input_complex();
}

void TopoOffsetTriMesh::repulsion_smoothing()
{
    // The march traces every marched edge to target_distance only if every outer end is farther
    // than it (the maximum marchable distance, see marching_tris()); otherwise it falls back to
    // half that distance and the loop has to carry the front out. These passes push the outer
    // ends out first, under one per-tri energy: w AMIPS plus, for each corner v of the face that
    // is an outer end within it (repulsion_cell_term()), O(v) = (max(0, 2 delta - d(v)) /
    // front_conv)^2 -- one-sided, so an outer end already beyond 2 delta is left to AMIPS, and
    // 2 delta so that the march at delta splits each edge with room to spare (the fallback's "half
    // the maximum marchable distance" run backwards). The smoother minimises its sum over a
    // vertex's ring, the vetoes and the rounds' collapses and swaps compare its max, as in the
    // loop. An outer end that an envelope holds carries no term. First up to
    // repulsion_smoothing_passes smoothing passes, then up to repulsion_rounds rounds of the
    // loop's operations (see below); both stop once every outer end is beyond delta + front_conv,
    // outside the tolerance band. As in 3D.
    const int n_passes = m_offset_params.repulsion_smoothing_passes;
    const int n_rounds = m_offset_params.repulsion_rounds;
    if (n_passes <= 0 && n_rounds <= 0) return;
    const double delta = m_offset_params.target_distance;
    const double stop = delta + m_offset_params.front_conv;

    // The outer ends exactly as marching_tris() picks its edges: the two ends differ in "label
    // 0". Recomputed at every report: the rounds' operations change the set. n_held: those an
    // envelope holds, which is_repulsion_vertex() leaves to TriWild's rule.
    std::vector<size_t> outer;
    size_t n_held = 0;
    const auto collect_outer = [&]() {
        outer.clear();
        n_held = 0;
        std::vector<char> seen(vert_capacity(), 0);
        for (const Tuple& e : get_edges()) {
            const size_t v1 = e.vid(*this);
            const size_t v2 = e.switch_vertex(*this).vid(*this);
            if (!is_marched_edge(v1, v2)) continue;
            const size_t v_out = m_vertex_extra[v1].label == 0 ? v1 : v2;
            if (!seen[v_out]) {
                seen[v_out] = 1;
                outer.push_back(v_out);
                if (vertex_boundary_mask(v_out) != 0) ++n_held;
            }
        }
    };
    collect_outer();
    if (outer.empty()) {
        logger().info("\t[repulsion] no edge to march, no pass");
        return;
    }
    // The march's own distance (the input-complex BVH it traces with), at level 2 delta.
    // Non-null switches the repulsion on (is_repulsion_vertex()).
    m_repulsion_potential =
        std::make_shared<EuclideanOffsetPotential2D>(m_input_complex_bvh, 2. * delta);
    m_repulsion_term = std::make_shared<OffsetEnergy2D>(
        m_repulsion_potential,
        4. * offset_term_weight(),
        true,
        true,
        /*one_sided=*/true);

    // Logs the state and says whether every outer end is beyond delta + front_conv.
    const auto report = [&](const std::string& when) {
        collect_outer();
        double d_min = std::numeric_limits<double>::infinity();
        size_t n_stop = 0, n_delta = 0, n_2delta = 0;
        double ring_max = 0.;
        for (const size_t v : outer) {
            const double d = m_input_complex_bvh->dist(m_vertex_attribute[v].m_posf);
            d_min = std::min(d_min, d);
            if (d <= stop) ++n_stop;
            if (d <= delta) ++n_delta;
            if (d < 2. * delta) ++n_2delta;
            for (const size_t fid : get_one_ring_fids_for_vertex(v)) {
                ring_max = std::max(ring_max, get_quality(fid));
            }
        }
        double mesh_max = 0.;
        for (const Tuple& f : get_faces()) mesh_max = std::max(mesh_max, get_quality(f));
        logger().info(
            "\t[repulsion] {}: maximum marchable distance {:.6g} ({:.4g}x target_distance) | "
            "outer ends {} ({} held by an envelope): within delta + front_conv {}, within delta "
            "{}, below 2 delta {} | max AMIPS around them {:.6g}, whole mesh {:.6g}",
            when,
            d_min,
            d_min / delta,
            outer.size(),
            n_held,
            n_stop,
            n_delta,
            n_2delta,
            ring_max,
            mesh_max);
        return n_stop == 0;
    };
    const auto log_newton = [&](const std::string& what) {
        const size_t refused = m_repulsion_embed_refused.exchange(0);
        logger().info(
            "\t[repulsion] {}: newton, repulsion: {} | veto: fired {} of {}{}",
            what,
            m_newton_repulsion.to_string(),
            m_repulsion_veto_fired.exchange(0),
            m_repulsion_veto_asked.exchange(0),
            n_rounds > 0
                ? fmt::format(" | operations refused for breaking the embedding: {}", refused)
                : std::string());
        m_newton_repulsion.reset();
    };

    logger().info(
        "\t[repulsion] {} repulsion vertices pushed toward 2 x target_distance = {:.6g}; the "
        "passes stop once every outer end is beyond target_distance + front_conv = {:.6g}, at "
        "most {} pass(es)",
        outer.size() - n_held,
        2. * delta,
        stop,
        n_passes);
    bool done = report("before");
    // The engine's per-pass frames are the loop's numbered series; these passes write their own.
    // m_params and m_offset_params are one object, so the flag is read before it is switched off.
    const bool frames = m_params.debug_output;
    m_params.debug_output = false;
    int k = 0;
    while (!done && k < n_passes) {
        ++k;
        smooth_all_vertices(1);
        log_newton(fmt::format("pass {}", k));
        done = report(fmt::format("after pass {}", k));
        if (frames) write_debug_frame(fmt::format("repulsion_{}", k));
    }
    if (n_passes > 0) {
        if (done) {
            logger().info(
                "\t[repulsion] every outer end is beyond target_distance + front_conv after {} "
                "pass(es)",
                k);
        } else {
            logger().info(
                "\t[repulsion] {} pass(es), the cap: some outer end is still within "
                "target_distance + front_conv",
                k);
        }
    }

    // repulsion_rounds: the loop's turn -- split, collapse and swap, each followed by its
    // smoothing passes (interleaved_smoothing, as optimize_offset_loop() shapes it; adaptive
    // smoothing is not used here) -- with no refinement (the sizing field is never lowered, so
    // every operation aims at the base edge length) and no split of a marched edge
    // (split_edge_before()): its midpoint would be a new outer end nearer the input. The
    // repulsion vertices follow every operation, since is_repulsion_vertex() asks the mesh.
    int r = 0;
    if (!done && n_rounds > 0) {
        // simplex_in_input_complex() reads the complex off the faces' labels, which is exact
        // when the complex is a body's faces (offset_out) or the faces outside a body (offset_in).
        // The curve group is 2D's surface group: the complex is edges, not faces.
        if (!m_singlebody || m_single_tag == m_curve_tag ||
            m_offset_params.offset_in == m_offset_params.offset_out) {
            log_and_throw_error(
                "repulsion_rounds: supported for a single body offset inward or outward only "
                "(not a curve group, an expression, or both directions)");
        }
        m_edge_split_mode = EdgeSplitMode::Optimization;
        partition_mesh_morton(); // optimize_offset() recomputes it
        const bool interleaved = m_params.interleaved_smoothing;
        const int ks = std::max(
            1,
            interleaved ? m_params.interleaved_smoothing_passes : m_params.num_smoothing_passes);
        const std::vector<std::array<int, 4>> groups =
            interleaved
                ? std::vector<std::array<int, 4>>{{{1, 0, 0, ks}}, {{0, 1, 0, ks}}, {{0, 0, 1, ks}}}
                : std::vector<std::array<int, 4>>{{{1, 1, 1, ks}}};
        logger().info(
            "\t[repulsion] up to {} round(s): split, collapse, swap{}; no refinement, no "
            "marched-edge split",
            n_rounds,
            interleaved ? fmt::format(", each followed by {} smoothing pass(es)", ks)
                        : fmt::format(" back to back, then {} smoothing pass(es)", ks));
        while (!done && r < n_rounds) {
            ++r;
            for (const auto& g : groups) {
                // The operations, then the labels from the tags, then the smoothing: the loop's
                // operations carry the faces' tags and labels but not the construction labels of
                // vertices and edges, which say what the input complex is and so which vertices
                // the smoothing pushes.
                local_operations({{g[0], g[1], g[2], 0}});
                relabel_input_complex();
                // A collapse or a swap may have joined two complex vertices by an edge off the
                // complex. The march needs the complex simplicially embedded, so it is embedded
                // here, before the smoothing: the embedding's midpoints are new outer ends, and
                // the smoothing and the stop test must see the mesh the march will get.
                if (!is_simplicially_embedded()) {
                    m_edge_split_mode = EdgeSplitMode::Midpoint;
                    simplicial_embedding();
                    bool dummy = is_simplicially_embedded();
                    m_edge_split_mode = EdgeSplitMode::Optimization;
                    partition_mesh_morton();
                }
                local_operations({{0, 0, 0, g[3]}});
            }
            // The 2D engine has no [ops accounting] / [swap reject] counters; the embedding
            // guard's refusals are on the newton line.
            log_newton(fmt::format("round {}", r));
            done = report(fmt::format("after round {}", r));
            if (frames) write_debug_frame(fmt::format("repulsion_round_{}", r));
        }
        // Every group above leaves the complex embedded and smoothing changes no topology, so
        // this finds nothing; it stays as the guard the march relies on.
        m_edge_split_mode = EdgeSplitMode::Midpoint;
        if (!is_simplicially_embedded()) {
            logger().warn(
                "\t[repulsion] the complex is not simplicially embedded after the rounds; "
                "embedding it before the march");
            simplicial_embedding();
            bool dummy = is_simplicially_embedded();
        }
        consolidate_mesh();
        if (done) {
            logger().info(
                "\t[repulsion] every outer end is beyond target_distance + front_conv after {} "
                "round(s)",
                r);
        } else {
            logger().info(
                "\t[repulsion] {} round(s), the cap: some outer end is still within "
                "target_distance + front_conv",
                r);
        }
    }
    m_params.debug_output = frames;
    m_repulsion_potential.reset();
    m_repulsion_term.reset();
}

bool TopoOffsetTriMesh::simplex_in_input_complex(const size_t a, const size_t b) const
{
    // An edge of a face of the complex (label 1: a body face for offset_out, a face outside the
    // body for offset_in), as label_input_complex() labels them; for offset_in also a
    // domain-boundary edge of a body face, which label_input_complex() adds to the complex.
    const std::vector<size_t> faces = get_incident_fids_for_edge(a, b);
    for (const size_t fid : faces) {
        if (m_face_extra[fid].label != 0) return true;
    }
    if (!m_offset_params.offset_in || faces.size() != 1) return false;
    return m_face_attribute[faces[0]].tags.count(m_single_tag) != 0;
}

bool TopoOffsetTriMesh::repulsion_embedding_kept(const std::vector<size_t>& fids) const
{
    // tri_is_simp_emb() on each face, with the spanned edge judged by
    // simplex_in_input_complex(): vertex and face labels are exact during an operation pass, the
    // edge labels of new edges are not. As in 3D.
    for (const size_t fid : fids) {
        if (m_face_extra[fid].label != 0) continue;
        std::vector<size_t> in;
        for (const size_t v : oriented_tri_vids(fid)) {
            if (m_vertex_extra[v].label != 0) in.push_back(v);
        }
        if (in.size() <= 1) continue;
        if (in.size() == 3 || !simplex_in_input_complex(in[0], in[1])) return false;
    }
    return true;
}


bool TopoOffsetTriMesh::is_simplicially_embedded() const
{
    int bad_tris = 0;
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        bad_tris += (!tri_is_simp_emb(f));
    }
    if (bad_tris == 0) {
        logger().info("\tInput complex/offset simplicially embedded: TRUE");
        return true;
    } else {
        logger().info(
            "\tInput complex/offset simplicially embedded: FALSE ({} bad tris)",
            bad_tris);
        return false;
    }
}


bool TopoOffsetTriMesh::tri_is_simp_emb(const Tuple& t) const
{
    size_t f_id = t.fid(*this);
    if (m_face_extra[f_id].label != 0) { // entire tri in input
        return true;
    }

    auto vs = oriented_tri_vids(f_id);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 3; i++) {
        if (m_vertex_extra[vs[i]].label != 0) {
            vs_in.push_back(vs[i]);
        }
    }

    if (vs_in.size() <= 1) { // nothing or just one vert
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t e_id = edge_id_from_simplex(simplex::Edge(vs_in[0], vs_in[1]));
        return (m_edge_extra[e_id].label != 0);
    } else { // all 3 verts in input but tri isnt, cant be simplicially embedded
        return false;
    }
}


void TopoOffsetTriMesh::simplicial_embedding()
{
    // identify tris to split
    std::vector<simplex::Face> tris_to_split;
    auto tris = get_faces();
    for (const Tuple& f : tris) {
        size_t f_id = f.fid(*this);
        auto vs = oriented_tri_vids(f_id);
        if (m_face_extra[f_id].label == 0) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
                size_t e_id = edge_id_from_simplex(simplex::Edge(v1, v2));
                if (m_edge_extra[e_id].label == 0) {
                    to_split = false;
                    break;
                }
            }

            if (to_split) { // tri not in input but all edges are
                tris_to_split.push_back(simplex::Face(vs[0], vs[1], vs[2]));
            }
        }
    }

    // actually split tris
    for (const simplex::Face& f : tris_to_split) {
        const auto& vs = f.vertices();
        Tuple t = tuple_from_vids(vs[0], vs[1], vs[2]);
        std::vector<Tuple> garbage;
        if (!split_face(t, garbage)) {
            log_and_throw_error("face split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tTris split: {}", tris_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t e_id = e.eid(*this);
        if (m_edge_extra[e_id].label == 0) {
            size_t v1_id = e.vid(*this);
            size_t v2_id = e.switch_vertex(*this).vid(*this);
            if ((m_vertex_extra[v1_id].label != 0) && (m_vertex_extra[v2_id].label != 0)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = get_tuple_from_edge(e);
        std::vector<Tuple> garbage;
        if (!split_edge(t, garbage)) {
            log_and_throw_error("edge split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tEdges split: {}", edges_to_split.size());
}


void TopoOffsetTriMesh::marching_tris()
{
    m_marching_root_splits = 0;
    m_marching_midpoint_splits = 0;
    m_marching_trace_steps = 0;
    m_marching_trace_steps_max = 0;
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);

        // if one background and the other input/offset
        if ((m_vertex_extra[v1].label == 0) != (m_vertex_extra[v2].label == 0)) {
            e_to_split.emplace_back(v1, v2);
        }
    }

    // sort edges by length
    if (m_offset_params.sorted_marching) {
        logger().info("\tSorting edges by length...");
        sort_edges_by_length(e_to_split);
    }

    // Construction (construction_mode). The maximum marchable distance is the smallest
    // d(outer end) over the marched edges: an edge's inner end is on the complex (d = 0), so by
    // continuity the level set d = D crosses every marched edge for every D up to it, and the
    // sphere trace (edge_split_sphere_trace()) finds it. A target below it is traced to.
    // Otherwise "max_marchable_fallback" traces to half of it, where every edge holds the level
    // set with room to spare, and "midpoint_fallback" splits every marched edge at its midpoint.
    // target_distance itself is untouched: the optimization carries the surface out to it.
    // Identical to TopoOffsetTetMesh::marching_tets().
    const double target = m_offset_params.target_distance;
    double d_max = std::numeric_limits<double>::infinity();
    for (const simplex::Edge& e : e_to_split) {
        const size_t va = e.vertices()[0];
        const size_t vb = e.vertices()[1];
        // Exactly one end carries a zero label: that is how e_to_split was built.
        const size_t v_out = m_vertex_extra[va].label != 0 ? vb : va;
        d_max = std::min(d_max, m_input_complex_bvh->dist(m_vertex_attribute[v_out].m_posf));
    }
    const bool reachable = target < d_max;
    const std::string& mode = m_offset_params.construction_mode;
    m_construction_distance = target;
    m_edge_split_mode = EdgeSplitMode::SphereTrace;
    if (e_to_split.empty()) {
        logger().info("\t[construction] construction_mode {}: no edge to march", mode);
    } else if (reachable) {
        logger().info(
            "\t[construction] construction_mode {}: maximum marchable distance {:.6g} over {} "
            "marched edges; target_distance {:.6g} is below it, so the march traces to the target",
            mode,
            d_max,
            e_to_split.size(),
            target);
    } else if (mode == "max_marchable_fallback" && d_max > 0.) {
        m_construction_distance = 0.5 * d_max;
        logger().info(
            "\t[construction] construction_mode {}: maximum marchable distance {:.6g} over {} "
            "marched edges; target_distance {:.6g} is not below it, so the march traces to half "
            "of it, {:.6g} ({:.4g}x target_distance)",
            mode,
            d_max,
            e_to_split.size(),
            target,
            m_construction_distance,
            m_construction_distance / target);
    } else {
        m_edge_split_mode = EdgeSplitMode::Midpoint;
        if (mode == "max_marchable_fallback") {
            logger().warn(
                "\t[construction] construction_mode {}: maximum marchable distance {} -- a "
                "marched edge ends on the complex -- so there is nothing to trace to and every "
                "marched edge is split at its midpoint",
                mode,
                d_max);
        } else {
            logger().info(
                "\t[construction] construction_mode {}: maximum marchable distance {:.6g} over "
                "{} marched edges; target_distance {:.6g} is not below it, so every marched edge "
                "is split at its midpoint",
                mode,
                d_max,
                e_to_split.size(),
                target);
        }
    }

    // init_optimize, only when the target is not below the maximum marchable distance:
    // optimize_offset() opens with a loop without refinement. A target below it is marched to and
    // the run is the ordinary one. As in 3D.
    if (m_offset_params.init_optimize && !e_to_split.empty()) {
        if (!reachable) {
            m_init_optimize = true;
            logger().info(
                "\t[init_optimize] target_distance is not below the maximum marchable distance, so "
                "the loop opens with a stencil_order {} loop without refinement, then the same "
                "stencil with refinement",
                m_offset_params.stencil_order);
        } else {
            logger().info(
                "\t[init_optimize] not done: target_distance is below the maximum marchable "
                "distance; the offset is marched to the target and the loop runs with refinement "
                "as usual");
        }
    }

    // actually split edges
    std::vector<Tuple> garbage;
    std::vector<size_t> frontier_verts; // the one-ring of these verts must be labelled offset
    for (const simplex::Edge& e : e_to_split) {
        // get vert of edge in offset
        size_t v_in = e.vertices()[0];
        if (m_vertex_extra[v_in].label == 0) {
            v_in = e.vertices()[1];
        }

        // split edge
        garbage.clear();
        Tuple t = get_tuple_from_edge(e);
        if (split_edge(t, garbage)) { // this should never fail
            frontier_verts.push_back(v_in);
        } else {
            log_and_throw_error("edge split failed! (marching_tris)");
        }
    }
    if (m_edge_split_mode == EdgeSplitMode::SphereTrace) {
        logger().info(
            "\t[construction] sphere trace: {} of {} marched edges placed where |d(x) - D| <= {} "
            "x D (D = {:.6g}), {} at the midpoint (the trace left the edge) | trace steps: {} "
            "total, {} max, {:.1f} per edge",
            m_marching_root_splits,
            e_to_split.size(),
            m_offset_params.sphere_trace_target_rel_tol,
            m_construction_distance,
            m_marching_midpoint_splits,
            m_marching_trace_steps,
            m_marching_trace_steps_max,
            e_to_split.empty() ? 0. : double(m_marching_trace_steps) / double(e_to_split.size()));
    } else {
        logger().info("\t[construction] {} marched edges split at the midpoint", e_to_split.size());
    }
    // The march's placement decision is its own; every later split is a midpoint one until the
    // optimization sets its own mode.
    m_edge_split_mode = EdgeSplitMode::Midpoint;

    // mark all offset tris (incident to any vert with label 1 or 2)
    for (const size_t v_id : frontier_verts) {
        auto tris = get_one_ring_tris_for_vertex(tuple_from_vertex(v_id));
        for (const Tuple& t : tris) {
            size_t f_id = t.fid(*this);
            if (m_face_extra[f_id].label == 0) { // dont want to overwrite if in input
                m_face_extra[f_id].label = 2;
                // propagate to children
                auto vs = oriented_tri_vids(f_id);
                for (int i = 0; i < 3; i++) {
                    if (m_vertex_extra[vs[i]].label != 1) {
                        m_vertex_extra[vs[i]].label = 2;
                    }
                    size_t e_id = tuple_from_edge(f_id, i).eid(*this);
                    if (m_edge_extra[e_id].label != 1) {
                        m_edge_extra[e_id].label = 2;
                    }
                }
            }
        }
    }
}


void TopoOffsetTriMesh::set_offset_tri_tags()
{
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_extra[f_id].label == 2) {
            CellTag new_tag;

            // add existing protected tags
            for (const int64_t& existing_tag : m_face_attribute[f_id].tags) {
                if (std::find(
                        m_offset_params.protected_tags.begin(),
                        m_offset_params.protected_tags.end(),
                        m_tag_id_to_name[existing_tag]) != m_offset_params.protected_tags.end()) {
                    new_tag.insert(existing_tag);
                }
            }

            // add actual offset tags
            if (m_offset_output_tag_ids.size() == 0) {
                if (new_tag.size() == 0) { // no protected tags, should be ambient
                    new_tag.insert(0);
                }
            } else {
                for (const int64_t& tag : m_offset_output_tag_ids) {
                    new_tag.insert(tag);
                }
            }

            m_face_attribute[f_id].tags = new_tag;
        }
    }
}


bool TopoOffsetTriMesh::offset_is_manifold()
{
    // vertex map
    std::map<size_t, bool> included_vids;
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        included_vids[v.vid(*this)] = false;
    }

    // collect faces in closed offset region (labelled 1 or 2)
    auto tris = get_faces();
    std::vector<Vector3i> offset_tris;
    for (const Tuple& t : tris) {
        size_t t_id = t.fid(*this);
        // Region membership from the tags, which every operation propagates -- not from the
        // label derived alongside them. See face_in_region().
        if (face_in_region(t_id)) {
            auto vs = oriented_tri_vids(t_id);
            offset_tris.emplace_back(vs[0], vs[1], vs[2]);
            included_vids[vs[0]] = true;
            included_vids[vs[1]] = true;
            included_vids[vs[2]] = true;
        }
    }

    // create consolidated v_id map
    int vert_count = 0;
    std::map<size_t, size_t> v_id_map;
    for (const auto& pair : included_vids) {
        if (pair.second) {
            v_id_map[pair.first] = vert_count++;
        }
    }

    // form matrix
    MatrixXi F(offset_tris.size(), 3);
    for (int i = 0; i < offset_tris.size(); i++) {
        for (int j = 0; j < 3; j++) {
            F(i, j) = v_id_map[offset_tris[i](j)];
        }
    }

    // check manifoldness
    bool is_edge_man = igl::is_edge_manifold(F);
    VectorXi B;
    bool is_vert_man = igl::is_vertex_manifold(F, B);
    return (is_edge_man && is_vert_man);
}


bool TopoOffsetTriMesh::invariants(const std::vector<Tuple>& tris)
{
    wmtk::utils::predicates::exactinit();
    for (const Tuple& t : tris) {
        auto vs = oriented_tri_vids(t);

        auto res = wmtk::utils::predicates::orient2d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf);
        if (res != wmtk::utils::predicates::Orientation::POSITIVE) {
            return false;
        }
    }
    return true;
}


void TopoOffsetTriMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(
        get_vertices().size(),
        -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<paraviewo::CellElement> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset;
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_extra[i].label == 1) {
            Eigen::Vector2d p = m_vertex_attribute[i].m_posf;
            verts_to_offset.emplace_back(p(0), p(1), 0.0);
            vid_map[i] = verts_to_offset.size() - 1;
        }
    }
    Eigen::MatrixXd V(verts_to_offset.size(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V.row(i) = verts_to_offset[i];
    }

    // get all offset input edges
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_extra[e.eid(*this)].label == 1) {
            paraviewo::CellElement curr_e;
            curr_e.ctype = paraviewo::CellType::Line;
            curr_e.vertices.push_back(vid_map[e.vid(*this)]);
            curr_e.vertices.push_back(vid_map[e.switch_vertex(*this).vid(*this)]);
            cells.push_back(curr_e);
        }
    }

    // get all offset input triangles
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_extra[f_id].label == 1) {
            auto v_ids = oriented_tri_vids(f_id);
            std::vector<int> curr_f;
            for (const size_t v_id : v_ids) {
                curr_f.push_back(vid_map[v_id]);
            }
            paraviewo::CellElement curr_f_elem;
            curr_f_elem.vertices = curr_f;
            curr_f_elem.ctype = paraviewo::CellType::Triangle;
            cells.push_back(curr_f_elem);
        }
    }

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells);
}


void TopoOffsetTriMesh::write_vtu(const std::string& path)
{
    logger().info("Write {}.vtu (tag for offset is included)", path);

    // Writing debug output must not change the mesh: consolidate_mesh() compacts the slot arrays
    // and renumbers every vertex and cell, and under kPartition threading get_partition_id() is
    // keyed on vertex id, so that changes which thread owns which vertex and with it the order
    // operations are applied in.
    //
    // Only the output is compacted, locally below. 2D sizes its arrays by the live count while
    // indexing them by slot id, so it needs a slot -> packed remap where 3D uses capacity-sized
    // point arrays.
    const auto& vs = get_vertices();
    const auto& tris = get_faces();

    std::vector<int> packed(vert_capacity(), -1);
    for (size_t k = 0; k < vs.size(); ++k) packed[vs[k].vid(*this)] = int(k);

    Eigen::MatrixXd V(vs.size(), 2);
    Eigen::MatrixXi F(tris.size(), 3);

    V.setZero();
    F.setZero();

    // last matrix is for offset
    std::vector<MatrixXd> tags(m_tags_count + 1, MatrixXd(tris.size(), 1));
    VectorXd amips(tris.size());

    for (size_t k = 0; k < tris.size(); ++k) {
        const size_t f_id = tris[k].fid(*this);

        // set tri tags -- row k, the packed index, not the slot
        for (int j = 0; j < m_tags_count; j++) {
            tags[j](k, 0) = (m_face_attribute[f_id].tags.count(j) == 1) ? 1 : 0;
        }
        tags[m_tags_count](k, 0) = (m_face_extra[f_id].label == 2) ? 1 : 0;
        amips[k] = m_face_attribute[f_id].m_quality;
    }

    for (size_t k = 0; k < tris.size(); ++k) {
        // set tri verts, remapped through `packed`
        const auto& loc_vs = oriented_tri_vertices(tris[k]);
        for (int j = 0; j < 3; j++) {
            F(k, j) = packed[loc_vs[j].vid(*this)];
        }
    }

    // The sizing field, as point data: it drives every split and collapse gate, and a
    // discontinuity in it is invisible in the geometry until the elements it produces are already
    // degenerate. Two forms: the raw scalar, and the target edge length l * scalar it means.
    Eigen::MatrixXd S(vs.size(), 1), Ltgt(vs.size(), 1), LAB(vs.size(), 1), VID(vs.size(), 1);
    for (size_t k = 0; k < vs.size(); ++k) {
        const size_t vid = vs[k].vid(*this);
        V.row(k) = m_vertex_attribute[vid].m_posf;
        S(k, 0) = m_vertex_attribute[vid].m_sizing_scalar;
        Ltgt(k, 0) = m_params.l * S(k, 0);
        LAB(k, 0) = m_vertex_extra[vid].label;
        VID(k, 0) = double(vid);
    }

    // Front convergence diagnostics, as point data: the vertex measure the loop reports next to
    // what it does not, so the two can be compared at the same vertex.
    //
    //   front_conv_ratio      front_vertex_conv_ratio(): the vertex's own relative error
    //                         |relative_residual()| over the relative bar -- the chord term's
    //                         measure at the vertex alone, its distance to the level set along
    //                         the field over front_conv. <= 1 reads as "placed". The loop
    //                         reports it and does not exit on it (EnergyCriterion::converged()).
    //   front_residual_rel    residual_length() over front_conv: the vertex's actual distance to
    //                         the level set, as a MULTIPLE OF THE BAR, so < 1 is converged. Never
    //                         tested; the same number as front_conv_ratio up to rounding.
    //   front_grad_norm       |grad Phi| at the vertex. The objective's pull is built from this,
    //                         so where it collapses the Newton step collapses with it.
    //   front_complex_distance the plain Euclidean distance from the vertex to the WHOLE input
    //                         complex, straight off the BVH. Not a field measure: it does not go
    //                         through potential_for(), so it is the same number whatever
    //                         offset_field is and whichever region the vertex belongs to, and
    //                         target_distance is what it should equal. For the smooth field it is
    //                         the only Euclidean number on the frame -- residual_length() there is
    //                         the length to the smooth level set, not to the complex. -2 before the
    //                         BVH exists (the construction frames written ahead of it).
    //
    // Together they separate "placed" from "stationary but wrong": where the field gives the
    // objective no gradient to move along (front_grad_norm near zero, as on the medial axis of
    // the smooth field), the vertex stops while front_conv_ratio stays at whatever error the
    // geometry left. -1 marks a vertex that is not on the front, -2 a value that is not finite.
    // Debug output only. Same fields as 3D.
    // MEASURED AGAINST A FRESHLY DERIVED BAND-REGION MAP. Every one of these reads the vertex's
    // own region's field through potential_for(vid) -> m_vertex_region, and that member is
    // rebuilt only once a turn. Between rebuilds it is stale two different ways: a split appends
    // vertices it does not cover (harmless -- vertex_region() bounds-checks and they fall back to
    // the union field), and a base pass consolidates and renumbers every vid mid-pass, after
    // which its entries name the WRONG vertices and the measures come out as residuals of tens
    // of delta and ratios in the thousands. Testing the map's size caught the second case only
    // by also catching the first, and refused 240 of this model's 281 frames.
    //
    // So the map is re-derived here and put back exactly as it was afterwards: the frame is
    // measured against the regions the mesh has AT THIS MOMENT -- which is what
    // energy_criterion() sees, since the loop rebuilds the map before it measures -- and the run
    // reads the same (possibly stale) map after the frame as before it. Nothing about the
    // optimization changes; only the frame stops being unmeasurable. -1 marks a vertex that is
    // not on the front, -2 a value that is not finite; there is no longer a "not measured" case.
    Eigen::MatrixXd CR(vs.size(), 1), RL(vs.size(), 1), GN(vs.size(), 1), MA(vs.size(), 1),
        CD(vs.size(), 1);
    std::vector<int> saved_face_region, saved_vertex_region;
    const bool region_map_refreshed = !m_region_potentials.empty();
    if (region_map_refreshed) {
        saved_face_region = m_face_region;
        saved_vertex_region = m_vertex_region;
        assign_band_regions(/*log=*/false);
    }
    {
        const auto finite_or = [](const double x) { return std::isfinite(x) ? x : -2.; };
        for (size_t k = 0; k < vs.size(); ++k) {
            CR(k, 0) = RL(k, 0) = GN(k, 0) = MA(k, 0) = CD(k, 0) = -1.;
            const size_t vid = vs[k].vid(*this);
            if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_attribute[vid].m_is_rounded) {
                continue;
            }
            const Vector2d p = m_vertex_attribute[vid].m_posf;
            const auto& pot = potential_for(vid);
            CR(k, 0) = finite_or(front_vertex_conv_ratio(vid));
            // RELATIVE to the one bar, so < 1 reads as placed at a glance. As in 3D.
            RL(k, 0) =
                finite_or(pot.residual_length(p) / std::max(m_offset_params.front_conv, 1e-300));
            GN(k, 0) = finite_or(pot.gradient(p).norm());
            MA(k, 0) = finite_or(front_move_alignment(vid));
            CD(k, 0) =
                m_input_complex_bvh ? finite_or(m_input_complex_bvh->dist(VectorXd(p))) : -2.;
        }
    }

    // The front solves since the last frame (m_front_solve_log), as point data:
    //   front_newton_iters   the Newton iterations the vertex's solve took; 10 is the cap.
    //   front_newton_status  polysolve's stop status + 1, NewtonCounters::status_name()'s
    //                        numbering: 2 IterationLimit, 6 GradNormTolerance,
    //                        7 RelGradNormTolerance, 12 LineSearchFailed.
    // -1 where the vertex was not solved since the last frame: not on the front, refused before
    // its solve, or the frame closes an operation pass. DEBUG ONLY, as the log is. As in 3D.
    Eigen::MatrixXd NIT(vs.size(), 1), NST(vs.size(), 1);
    NIT.setConstant(-1.);
    NST.setConstant(-1.);
    for (const FrontSolveRecord& r : m_front_solve_log) {
        if (r.vid >= packed.size() || packed[r.vid] < 0) continue;
        NIT(packed[r.vid], 0) = r.iterations;
        NST(packed[r.vid], 0) = r.status;
    }

    // Collapsed-foldover flag, as point data: 1 where the offset curve has folded back on
    // itself at this vertex (outer angle over the threshold), 0 elsewhere. DEBUG ONLY --
    // computed only under debug_output, so a plain save_vtu run writes it as all 0 rather than
    // paying for the curve walk. Packed like every other point field here, through `packed`.
    // See offset_surface_foldover_labels() for what the angle is and how its side is decided.
    Eigen::MatrixXd FOLD(vs.size(), 1);
    FOLD.setZero();
    if (m_offset_params.debug_output) {
        const std::vector<char> fold = offset_surface_foldover_labels();
        for (size_t k = 0; k < vs.size(); ++k) {
            const size_t vid = vs[k].vid(*this);
            if (vid < fold.size() && fold[vid]) FOLD(k, 0) = 1.;
        }
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->add_cell_field("amips", amips);
    for (int64_t i = 0; i < m_tags_count; i++) {
        writer->add_cell_field(m_tag_id_to_name[i], tags[i]);
    }
    writer->add_cell_field("offset_tag", tags[m_tags_count]); // also hacky but it works.
    writer->add_field("labels", LAB);
    writer->add_field("vid", VID);
    writer->add_field("sizing_scalar", S);
    writer->add_field("target_edge_length", Ltgt);
    writer->add_field("front_conv_ratio", CR);
    writer->add_field("front_residual_rel", RL);
    writer->add_field("front_grad_norm", GN);
    writer->add_field("front_move_align", MA);
    writer->add_field("front_complex_distance", CD);
    writer->add_field("offset_foldover", FOLD);
    writer->add_field("front_newton_iters", NIT);
    writer->add_field("front_newton_status", NST);
    writer->write_mesh(path + ".vtu", V, F, paraviewo::CellType::Triangle);

    // The front's per-CHORD convergence measure, as a companion line mesh `<path>_front.vtu`: a
    // triangle .vtu has nowhere to put an edge quantity. Same packed vertex indexing as the
    // frame above, so a viewer can key the field onto the offset curve it derives from the
    // triangles, by vertex pair. The 3D twin writes the same fields on its `_off.vtu`.
    //
    //   front_err_ratio   the root of edge_offset_term(): the RMS over the chord's
    //                     stencil_order stencil of relative_residual() -- the distance to the
    //                     level set along the field over target_distance -- over the one bar as
    //                     a fraction of it. > 1 is what makes a chord refinable, and the same
    //                     number at 1 point is what makes a vertex placed. -1 unmeasurable,
    //                     including a chord with an end that is not a front vertex. Measured
    //                     under the same re-derived region map as the vertex fields above.
    //                     REPLACES front_sag_ratio, the midpoint sag against a separate bar; a
    //                     series mixing the two compares two quantities under one name, so the
    //                     field was renamed rather than redefined in place.
    //   chord_length      |b - a|, so the error can be read against the chord that produced it.
    //   front_ring_ratio  point data, in both front_measure modes: the ring measure at each front
    //                     vertex, sqrt(mean of front_err_ratio^2) over its incident chords with
    //                     both ends front vertices, every chord weighted equally -- the exit test
    //                     under front_measure "vertex_ring" (EnergyCriterion::ring_exit), with
    //                     energy_criterion()'s rules: a vertex with an unmeasurable incident
    //                     chord has none. NaN where there is no ring measure.
    {
        const auto front = [&](const size_t vid) {
            return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
        };
        std::vector<std::array<int, 2>> fe;
        std::vector<double> fe_term, fe_len;
        std::vector<char> fe_front;
        for (const Tuple& e : get_edges()) {
            if (!edge_is_offset_surface_live(e)) continue;
            const size_t va = e.vid(*this), vb = e.switch_vertex(*this).vid(*this);
            if (packed[va] < 0 || packed[vb] < 0) continue;
            fe.push_back({packed[va], packed[vb]});
            fe_front.push_back(front(va) && front(vb) ? 1 : 0);
            fe_term.push_back(fe_front.back() ? edge_offset_term(va, vb) : -1.);
            fe_len.push_back(
                (m_vertex_attribute[va].m_posf - m_vertex_attribute[vb].m_posf).norm());
        }
        if (!fe.empty()) {
            Eigen::MatrixXi FE(fe.size(), 2);
            Eigen::MatrixXd ERR(fe.size(), 1), LEN(fe.size(), 1);
            std::vector<double> ring_sum(vs.size(), 0.), ring_n(vs.size(), 0.);
            std::vector<char> ring_bad(vs.size(), 0);
            for (size_t k = 0; k < fe.size(); ++k) {
                FE(k, 0) = fe[k][0];
                FE(k, 1) = fe[k][1];
                ERR(k, 0) = fe_term[k] < 0. ? -1. : std::sqrt(fe_term[k]);
                LEN(k, 0) = fe_len[k];
                if (!fe_front[k]) continue;
                for (const int u : fe[k]) {
                    if (fe_term[k] < 0.) {
                        ring_bad[size_t(u)] = 1;
                    } else {
                        ring_sum[size_t(u)] += fe_term[k];
                        ring_n[size_t(u)] += 1.;
                    }
                }
            }
            Eigen::MatrixXd RING(vs.size(), 1);
            for (size_t k = 0; k < vs.size(); ++k) {
                RING(k, 0) = !ring_bad[k] && ring_n[k] > 0.
                                 ? std::sqrt(ring_sum[k] / ring_n[k])
                                 : std::numeric_limits<double>::quiet_NaN();
            }
            const std::string front_path = path + "_front.vtu";
            std::shared_ptr<paraviewo::ParaviewWriter> front_writer =
                std::make_shared<paraviewo::VTUWriter>();
            front_writer->add_cell_field("front_err_ratio", ERR);
            front_writer->add_cell_field("chord_length", LEN);
            front_writer->add_field("vid", VID);
            front_writer->add_field("sizing_scalar", S);
            front_writer->add_field("front_conv_ratio", CR);
            front_writer->add_field("front_ring_ratio", RING);
            front_writer->add_field("front_residual_rel", RL);
            front_writer->add_field("front_grad_norm", GN);
            front_writer->add_field("front_move_align", MA);
            front_writer->add_field("front_complex_distance", CD);
            front_writer->add_field("offset_foldover", FOLD);
            front_writer->add_field("front_newton_iters", NIT);
            front_writer->add_field("front_newton_status", NST);
            front_writer->write_mesh(front_path, V, FE, paraviewo::CellType::Line);
        }
    }

    // The band-region map back exactly as the run left it (see the frame diagnostics above).
    if (region_map_refreshed) {
        m_face_region.swap(saved_face_region);
        m_vertex_region.swap(saved_vertex_region);
    }

    // surface output
    if (m_has_envelope) {
        const auto out_surf_path = path + "_surf.vtu";
        std::shared_ptr<paraviewo::ParaviewWriter> surf_writer;
        surf_writer = std::make_shared<paraviewo::VTUWriter>();
        logger().info("Write {}", out_surf_path);
        surf_writer
            ->write_mesh(out_surf_path, m_V_envelope, m_F_envelope, paraviewo::CellType::Line);
    }
}


void TopoOffsetTriMesh::write_phi_grid(const std::string& path, const int n) const
{
    // A dense triangulated grid over the bounding box carrying Phi as a vertex field, so a viewer
    // can draw the level set Phi = c as an isoline. The offset is a level set of a field that
    // exists everywhere, and the mesh only ever samples it along one curve.
    if (n < 2 || !m_offset_potential) return;

    const Vector2d lo = m_offset_params.box_min.head<2>();
    const Vector2d hi = m_offset_params.box_max.head<2>();

    MatrixXd V(n * n, 2);
    MatrixXi F(2 * (n - 1) * (n - 1), 3);
    MatrixXd phi(n * n, 1), residual(n * n, 1), euclid(n * n, 1);

    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const int k = j * n + i;
            const Vector2d p(
                lo[0] + (hi[0] - lo[0]) * i / (n - 1),
                lo[1] + (hi[1] - lo[1]) * j / (n - 1));
            V.row(k) = p.transpose();
            // Phi diverges on the input complex, which would flatten the colour map everywhere
            // else; clamped to a few times the level value, which is the range that matters.
            phi(k, 0) =
                std::min(m_offset_potential->value(p), 8. * m_offset_potential->target_level());
            residual(k, 0) = std::min(
                m_offset_potential->residual_length(p),
                8. * m_offset_params.target_distance);
            euclid(k, 0) = m_input_complex_bvh->dist(VectorXd(p));
        }
    }
    int f = 0;
    for (int j = 0; j + 1 < n; ++j) {
        for (int i = 0; i + 1 < n; ++i) {
            const int a = j * n + i, b = a + 1, c = a + n, d = c + 1;
            F.row(f++) << a, b, d;
            F.row(f++) << a, d, c;
        }
    }

    logger().info(
        "Write {}_phi.vtu ({}x{} samples of the smooth offset potential; the offset is the "
        "isoline phi = {})",
        path,
        n,
        n,
        m_offset_potential->target_level());
    auto writer = std::make_shared<paraviewo::VTUWriter>();
    writer->add_field("phi", phi);
    writer->add_field("phi_residual_length", residual);
    writer->add_field("euclidean_distance", euclid);
    writer->write_mesh(path + "_phi.vtu", V, F, paraviewo::CellType::Triangle);
}


void TopoOffsetTriMesh::write_msh_groups(const std::string& file)
{
    logger().info("Write {}.msh", file);
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& faces = get_faces();

    // set vertices
    const auto& verts = get_vertices();
    msh.add_face_vertices(verts.size(), [&](size_t k) {
        auto i = verts[k].vid(*this);
        Vector2d p2 = m_vertex_attribute[i].m_posf;
        return Vector3d(p2(0), p2(1), 0);
    });

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

    // add ambient (tag id=0). assumed that ambient does not overlap with anything
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        if (m_face_attribute[f_id].tags.count(0) != 0) {
            faces_with_tag.push_back(f);
        }
    }
    msh_add_faces();
    msh.add_physical_group("ambient");

    // group for each tag
    for (int64_t tag_img = 1; tag_img < m_tags_count; tag_img++) {
        faces_with_tag.clear();
        for (const Tuple& f : faces) {
            size_t f_id = f.fid(*this);
            if (m_face_attribute[f_id].tags.count(tag_img) != 0) {
                faces_with_tag.push_back(f);
            }
        }

        if (faces_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(2);
        msh_add_faces();

        const std::string group_name = m_tag_id_to_name[tag_img];
        msh.add_physical_group(group_name);
    }

    if (m_has_envelope) {
        msh.add_edge_vertices(m_V_envelope.rows(), [this](size_t k) {
            return Vector3d(m_V_envelope(k, 0), m_V_envelope(k, 1), 0);
        });
        msh.add_edges(m_F_envelope.rows(), [this](size_t k) { return m_F_envelope.row(k); });
        msh.add_physical_group("EnvelopeSurface");
    }

    msh.save(file + ".msh", true);
}


} // namespace wmtk::components::topological_offset
