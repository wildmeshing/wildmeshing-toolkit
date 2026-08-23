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
    const std::vector<std::string>& tag_names)
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
            logger().warn("Tag '{}' does not exist. Adding to mesh.", tag);
            int64_t new_id = m_tag_id_to_name.size();
            m_tag_id_to_name[new_id] = tag;
            m_tag_name_to_id[tag] = new_id;
            m_tags_count++;
        }
    }

    // collect int ids for offset output
    for (const std::string& name : m_offset_params.offset_output_tag) {
        m_offset_output_tag_ids.insert(m_tag_name_to_id[name]);
    }

    // One mask bit per input tag, ambient included, in id order. Assigned HERE, once the maps
    // are complete and before init_surfaces_and_boundaries() seeds the vertex masks from the
    // boundary edges. Tags introduced later (the band's offset tag) get no bit -- boundary
    // membership is a property of the INPUT partition, which is also why the masks are
    // propagated rather than ever recomputed from current face tags.
    if (m_tag_id_to_name.size() > 64) {
        log_and_throw_error(
            "Per-tag boundary envelopes support at most 64 input tags, got {}",
            m_tag_id_to_name.size());
    }
    m_tag_bit.clear();
    for (const auto& [tag_id, name] : m_tag_id_to_name) {
        const int bit = int(m_tag_bit.size());
        m_tag_bit[tag_id] = bit;
    }

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

    // Set position of verts. Through set_vertex_position(), NOT by assigning m_posf alone: the
    // exact position m_pos and the m_is_rounded flag have to be filled too. is_inverted() falls
    // back to the RATIONAL path as soon as any vertex of a face is not rounded, and reads
    // m_pos -- so leaving m_pos at its default (0,0) makes every face incident to an input
    // vertex report itself inverted. That makes round() fail, which makes smooth_before()
    // refuse the vertex, which silently excluded every original input vertex from smoothing:
    // on the dragon, 9557 of 10684 vertices per pass.
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        set_vertex_position(v_id, Vector2d(V.row(v_id)));
    }

    init_surfaces_and_boundaries();
}


void TopoOffsetTriMesh::init_surfaces_and_boundaries()
{
    const auto edges = get_edges();
    logger().info("E = {}", edges.size());

    // THE DOMAIN WALL IS A REGION BOUNDARY. An edge with no opposite face is the outer boundary
    // of the mesh -- the bounding box -- and it is the boundary between the ambient region and
    // the space outside the domain; the only reason it has no second tag to compare against is
    // that the outside is not meshed. Held in ambient's envelope, it may be refined and its
    // vertices may move within eps of where they started, which is the same contract every other
    // region boundary gets. See the 3D twin for the degeneracy this replaced.
    size_t n_edges_tracked = 0;
    std::map<int64_t, std::vector<Eigen::Vector2i>> tag_edges; // per-tag boundary buckets
    for (const Tuple& e : edges) {
        const size_t eid = e.eid(*this);

        // WHOSE boundary this edge is. Interior edge: every tag on exactly one side (the
        // symmetric difference -- multi-tag faces exist, and a tag present on both sides has no
        // boundary here). Wall edge: every tag of its single face, the boundary against the
        // unmeshed outside -- which is how ambient's envelope comes to hold the wall.
        CellTag edge_tags;
        const std::optional<Tuple> f_opp = e.switch_face(*this);
        if (f_opp) {
            const auto& tag0 = m_face_attribute[e.fid(*this)].tags;
            const auto& tag1 = m_face_attribute[f_opp->fid(*this)].tags;
            if (tag0 == tag1) {
                continue;
            }
            std::set_symmetric_difference(
                tag0.begin(),
                tag0.end(),
                tag1.begin(),
                tag1.end(),
                std::inserter(edge_tags, edge_tags.begin()));
        } else {
            edge_tags = m_face_attribute[e.fid(*this)].tags;
        }

        m_edge_attribute[eid].m_is_surface_fs = true;
        ++n_edges_tracked;

        const size_t v1 = e.vid(*this);
        const size_t v2 = e.switch_vertex(*this).vid(*this);

        // Seed the per-vertex boundary masks and the per-tag edge buckets from the same
        // classification, so the dispatch (a segment is constrained by the AND of its ends'
        // masks) and the envelopes it dispatches to can never disagree about what is where.
        const uint64_t bits = tag_bits(edge_tags);
        for (const size_t v : {v1, v2}) m_vertex_extra[v].m_boundary_mask |= bits;
        for (const int64_t t : edge_tags) {
            tag_edges[t].emplace_back(int(v1), int(v2));
        }
        // The region flag, for interior boundaries only: the wall carries none because
        // vertex_is_on_region() reads it off on_bbox_faces. Note this is NOT "on the input
        // complex" -- label_input_complex() has not run yet and cannot; that is
        // mark_input_complex_vertices()'s job, from the labels, once it has.
        if (f_opp) {
            for (const size_t v : {v1, v2}) m_vertex_extra[v].m_is_on_region = true;
        }
        // The base's own flag, a DIFFERENT field from the ones above: those say which tracked
        // surface a vertex belongs to, this says that it belongs to one at all. Every
        // surface-aware path in the shared engine gates on it.
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
    }

    if (!m_envelope && n_edges_tracked > 0) {
        logger().info("Init per-tag envelopes from face tags");
        std::vector<Eigen::Vector2d> tempV(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) {
            tempV[i] = m_vertex_attribute[i].m_posf;
        }

        // FROM THE PARAMETERS. Without it m_envelope_eps keeps its -1 sentinel and the envelopes
        // are built with a NEGATIVE half-width, so is_outside() answers true for every segment,
        // including the ones they were just constructed from -- freezing every boundary solid.
        //
        // params.init() runs in topological_offset() before init_from_image(), so envelope_size
        // is already resolved from envelope_size_rel x the bbox diagonal by the time we read it.
        // Unit tests construct without params.init(), which is why this whole block stays behind
        // the n_edges_tracked guard rather than throwing on a nonpositive eps.
        m_envelope_eps = m_offset_params.envelope_size;

        // ONE ENVELOPE PER TAG, from that tag's boundary bucket -- the input partition as it
        // stands BEFORE offset construction rewrites tags, which is what makes E_t the tube
        // around the as-loaded geometry the offset potential also measures against. A boundary
        // edge between two regions enters both regions' envelopes; the wall enters its face's
        // tags' (ambient's, mostly). See the m_tag_envelopes doc for the intersection semantics.
        m_tag_envelopes.clear();
        {
            std::lock_guard<std::mutex> lock(m_isect_mutex);
            m_isect_cache.clear();
        }
        std::vector<std::shared_ptr<SampleEnvelope>> members;
        std::string per_tag_log;
        for (const auto& [tag, bucket] : tag_edges) {
            if (bucket.empty()) continue; // offset_output_tag ids with no faces yet
            auto env = std::make_shared<SampleEnvelope>();
            env->init(tempV, bucket, m_envelope_eps);
            m_tag_envelopes[tag] = env;
            members.push_back(env);
            per_tag_log += fmt::format(" {}:{}", m_tag_id_to_name.at(tag), bucket.size());
        }

        // The base's pointer survives as the UNION of the members -- inside any tube -- because
        // the shared engine's direct uses of it ask exactly that question. Everything else
        // dispatches per simplex through envelope_for_mask().
        m_envelope = std::make_shared<UnionEnvelope>(std::move(members));
        logger().info(
            "\tPer-tag boundary envelopes: {} segments total (tag boundaries + domain wall), "
            "eps {:.6g} |{}",
            n_edges_tracked,
            m_envelope_eps,
            per_tag_log);
    }

    // track bounding box. box_min/box_max are only set by Parameters::init(), which callers that
    // go through the full offset pipeline call before init_from_image() -- but plenty of unit
    // tests construct a mesh from a default-constructed Parameters and never call it, leaving
    // these as empty (size 0) VectorXd. Skip rather than index out of bounds.
    //
    // WHICH wall, not merely "a wall": on_bbox_faces carries the wall index (2k / 2k+1 for the
    // min / max side of axis k), so the split's set_intersection of its endpoints can tell a
    // corner vertex from an edge one, exactly as in 3D. It used to push a constant 0 for every
    // boundary vertex, which made every wall the same wall.
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


void TopoOffsetTriMesh::mark_input_complex_vertices()
{
    // THE ONLY PLACE THE INPUT COMPLEX IS KNOWN. This runs at the end of label_input_complex(),
    // which has just evaluated the selection expression; init_surfaces_and_boundaries() runs
    // earlier, out of init_from_image(), and can only see region boundaries -- which is exactly
    // why m_is_on_region, set there, is not this.
    //
    // The label is the input complex at every dimension, so this covers a filled complex, a
    // curve and an isolated point alike -- the sub-manifold cases have no input FACE to read the
    // flag off, which is why this keys on the vertex label rather than on incident faces.
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


void TopoOffsetTriMesh::label_input_complex()
{
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
        } else if (m_offset_params
                       .offset_in) { // input complex is everything outside body plus boundary
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

    // THE BOUNDARY CURVE, derived before the BVH so the ONE retained structure can carry it.
    //
    // Phi's 2D primitives are segments and points; there is no 2D area primitive. A solid input
    // region therefore enters as its BOUNDARY -- the edges of the complex's triangles that have
    // exactly one incident complex triangle. Outside the region, which is the only place an
    // offset exists, "distance to the region" and "distance to its boundary" are the same
    // number, so nothing is lost. (Inside the region they differ, and Phi has a second, mirrored
    // level set in there; it is unreachable, because the band is grown outward and the runaway
    // guard would catch a vertex that crossed the complex.)
    std::map<simplex::Edge, int> boundary_count;
    for (const simplex::Face& f_simp : complex_faces) {
        const auto vs = f_simp.vertices();
        ++boundary_count[simplex::Edge(vs[0], vs[1])];
        ++boundary_count[simplex::Edge(vs[1], vs[2])];
        ++boundary_count[simplex::Edge(vs[0], vs[2])];
    }

    std::vector<Eigen::Vector2i> phi_segs;
    for (const auto& [e_simp, count] : boundary_count) {
        if (count != 1) continue; // interior to the complex: carries no boundary geometry
        const auto vs = e_simp.vertices();
        phi_segs.emplace_back(v_index_map[vs[0]], v_index_map[vs[1]]);
    }
    for (int i = 0; i < E.rows(); ++i) { // isolated edges of the complex
        phi_segs.emplace_back(E(i, 0), E(i, 1));
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

    // set BVH -- a fresh object rather than clear+reinit, so any potential still holding the
    // old one (there should be none; this runs once, before construction) keeps a coherent view.
    //
    // THE EDGE SET IS THE CURVE, E_phi, NOT just the isolated edges E: the euclidean
    // potential's feature query (nearest_point_feature) runs on the BVH's edges, and it must
    // see the boundary of a solid complex, which the face set alone cannot answer -- measured
    // the hard way: on two_circles the complex is the two solid disks, E was empty, and the
    // first feature query walked an uninitialized tree. E_phi already contains the isolated
    // edges, and every boundary segment lies ON a complex face, so squared_dist() -- the
    // distance to the SOLID complex -- is unchanged by indexing them too.
    m_input_complex_bvh = std::make_shared<SimplicialComplexBVH>();
    m_input_complex_bvh->init(V, T, F, E_phi, P);

    // KEPT, not built. The extraction is what must not diverge from the BVH's, so it is done
    // here and once; the potential itself needs target_distance and offset_dhat_factor, which a
    // caller that only wants the distance field (the unit tests build a TopoOffsetTriMesh from a
    // default-constructed Parameters) has no reason to have set.
    m_phi_V = V;
    m_phi_E = E_phi;
    m_phi_P = P_phi;
}


void TopoOffsetTriMesh::init_offset_potential()
{
    if (m_phi_V.rows() == 0 || !m_input_complex_bvh) {
        log_and_throw_error("init_offset_potential() called before init_input_complex_bvh()");
    }
    // WHICH FIELD DEFINES THE OFFSET; see OffsetPotential.hpp and the offset_field parameter.
    // Both are built from the SAME extraction -- m_phi_V/E/P, which init_input_complex_bvh()
    // produced -- so whichever is chosen measures the same geometry the diagnostics do.
    // NO SECOND STRUCTURE. The euclidean potential queries m_input_complex_bvh -- the one
    // input-complex structure this mesh keeps, built when the object was initialized and
    // retained through the whole run. An exact-kind SampleEnvelope used to be built here over
    // the same segments as its private query engine; it duplicated the geometry, and its other
    // consumer (the convergence criterion's projection normal) is gone. Containment of the
    // complex was never this object's job either -- the per-tag region envelopes hold it.
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
        return;
    }

    // DHAT IS SIZED TO THE OFFSET IT HAS TO HOLD, not to target_distance alone.
    //
    // Construction places the offset at the input triangulation's own cell boundaries -- midpoint
    // marching, no target_distance in it at all -- so how far out it lands is an ABSOLUTE
    // property of the input mesh, not a multiple of delta. A fixed factor x delta therefore fails
    // exactly when delta is small relative to the background triangles. Measured on
    // topo_annots_groups at target_distance 0.25 the moment the growth pass went: the constructed
    // offset reached 2.83x delta and the factor-2 support rejected 436 vertices at construction.
    //
    // The floor keeps the configured factor authoritative whenever construction was good. That
    // matters because dhat is NOT a neutral scaling: c = Phi at delta from flat input grows
    // roughly as 1.14 ln(factor) - 0.74, so a purely data-driven dhat would give the same
    // geometry a different offset near corners and gaps depending only on how the input was
    // meshed. Same rule as 3D.
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
}


double TopoOffsetTriMesh::max_band_vertex_distance() const
{
    // How far the offset boundary actually ended up from the input complex, as a LENGTH. Exact
    // (BVH nearest point), not the straddle-edge upper bound: an input-to-offset edge can be long
    // and nearly tangential, and dhat is not a free parameter to inflate -- it selects the level
    // c, so an overestimate changes which curve the run solves for.
    //
    // Returns 0 when there is no offset boundary yet -- the band loop finds no edges and never
    // touches the BVH -- which is the signal to fall back to the configured factor.
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


void TopoOffsetTriMesh::execute_offset(const std::filesystem::path& output_file)
{
    // BEFORE ANY OF THE CONSTRUCTION, optionally improve the mesh the construction runs on.
    // The marching puts the offset on this triangulation's own cell boundaries, so its quality
    // decides how far the constructed offset lands from the complex and therefore how large
    // dhat has to be. See pre_optimize_input_mesh().
    if (m_offset_params.pre_optimize_input) {
        pre_optimize_input_mesh();
        if (m_offset_params.debug_output) {
            write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
        }
    }

    // make embedding simplicial
    m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Midpoint;
    logger().info("Creating simplicial embedding...");
    if (!is_simplicially_embedded()) {
        simplicial_embedding();
        bool dummy = is_simplicially_embedded();
    }
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }


    // initialize offset
    logger().info("Initializing offset...");
    // The inserted vertex is the plain edge midpoint -- target_distance does not enter the
    // placement at all. Carrying the boundary out to target_distance is the optimization phase's
    // job; see the note above set_offset_tri_tags().
    m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Midpoint;
    marching_tris();
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // NO GROWTH PASS. The band is exactly the one layer of background triangles marching_tris()
    // labelled from the frontier one-rings, so the offset boundary sits on the input
    // triangulation's own cell boundaries and nothing has widened it toward target_distance.
    // Closing that gap is the optimization phase's job.

    // simplicially embed again, if needed
    m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Midpoint;
    if (!is_simplicially_embedded()) {
        simplicial_embedding();
        bool dummy = is_simplicially_embedded();
    }
    // Outside the branch above, and unconditional. write_vtu() consolidates, so leaving this
    // inside the `if` meant that on a mesh already simplicially embedded the ONLY consolidate
    // here was the debug one -- and consolidating renumbers, which changes the order later passes
    // enumerate operations in, which changes the run. Same reason as the one before the
    // optimization loop.
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    // The pass that used to run here root-found every band-boundary edge onto d(x) = delta, i.e.
    // it did the optimizer's work with a mechanism that has no error feedback and no envelope or
    // inversion guards. It is gone, as it is in 3D: the paper puts inserted vertices at the
    // midpoint (Sec. 5.2) and leaves the distance to Step 3.
    //
    // Consequence, and the reason optimize_offset() is unconditional: construction leaves the
    // band boundary on BACKGROUND-CELL boundaries, so its distance to the input complex is only
    // accurate to the local cell size until the optimization moves it.
    set_offset_tri_tags();
    consolidate_mesh();
    if (m_offset_params.debug_output) {
        write_vtu(output_file.string() + fmt::format("_{}", m_vtu_counter++));
    }

    assert(ambient_assert());
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

    // WRITING DEBUG OUTPUT MUST NOT CHANGE THE MESH. This used to call consolidate_mesh()
    // first, which compacts the slot arrays and RENUMBERS every vertex and cell. That makes
    // debug output non-observational: turning DEBUG_output on changes the run it is supposed to
    // be showing you. The renumbering is not cosmetic either -- under kPartition threading
    // get_partition_id() is keyed on vertex id, so which thread owns which vertex changes, and
    // with it the order operations are applied in.
    //
    // Nothing here needs the mesh compacted; it only needed the OUTPUT compacted, which is done
    // locally below instead. 2D sized its arrays by the LIVE count while
    // indexing them by slot id, which is only equivalent after a consolidate -- so it needs a
    // slot -> packed remap rather than 3D's capacity-sized point arrays.
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

    for (size_t k = 0; k < tris.size(); ++k) {
        const size_t f_id = tris[k].fid(*this);

        // set tri tags -- row k, the PACKED index, not the slot
        for (int j = 0; j < m_tags_count; j++) {
            tags[j](k, 0) = (m_face_attribute[f_id].tags.count(j) == 1) ? 1 : 0;
        }
        tags[m_tags_count](k, 0) = (m_face_extra[f_id].label == 2) ? 1 : 0;
    }

    for (size_t k = 0; k < tris.size(); ++k) {
        // set tri verts, remapped through `packed`
        const auto& loc_vs = oriented_tri_vertices(tris[k]);
        for (int j = 0; j < 3; j++) {
            F(k, j) = packed[loc_vs[j].vid(*this)];
        }
    }

    // THE SIZING FIELD, as point data. It is what drives every split and collapse gate, it is
    // the one thing the debug output could not show, and a discontinuity in it is invisible in
    // the geometry until the elements it produces are already degenerate. Two forms: the raw
    // scalar, and the TARGET EDGE LENGTH l * scalar it actually means, which is directly
    // comparable to the edge lengths in the same picture.
    Eigen::MatrixXd S(vs.size(), 1), Ltgt(vs.size(), 1);
    for (size_t k = 0; k < vs.size(); ++k) {
        V.row(k) = m_vertex_attribute[vs[k].vid(*this)].m_posf;
        S(k, 0) = m_vertex_attribute[vs[k].vid(*this)].m_sizing_scalar;
        Ltgt(k, 0) = m_params.l * S(k, 0);
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    for (int64_t i = 0; i < m_tags_count; i++) {
        writer->add_cell_field(m_tag_id_to_name[i], tags[i]);
    }
    writer->add_cell_field("offset_tag", tags[m_tags_count]); // also hacky but it works.
    writer->add_field("sizing_scalar", S);
    writer->add_field("target_edge_length", Ltgt);
    writer->write_mesh(path + ".vtu", V, F, paraviewo::CellType::Triangle);

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
    // A dense triangulated grid over the bounding box carrying Phi as a VERTEX field, so that a
    // viewer can draw the level set Phi = c as an isoline rather than a cloud of coloured dots.
    // This is the only way to SEE what the optimization is actually minimising: the offset is a
    // level set of a field that exists everywhere, and the mesh only ever samples it along one
    // curve.
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
