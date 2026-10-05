
#include "TopoOffsetTetMesh.h"
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/bundled/format.h>
#include <wmtk/utils/predicates.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/boundary_facets.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_unreferenced.h>
#include <paraviewo/VTUWriter.hpp>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <queue>
#include <unordered_map>


namespace wmtk::components::topological_offset {

/**
 * Construction and I/O -- the twin of TopoOffsetTriMesh.cpp. The optimization phase lives in
 * Optimize3d.cpp, FrontSmooth3d.cpp and EdgeSplittingTet.cpp.
 */

namespace {
/// The exact-kind envelope Phi's euclidean twin queries: triangles where there are any,
/// segments (isolated input points as the degenerate segment (i, i)) otherwise.
std::shared_ptr<SampleEnvelope> euclidean_query_envelope(
    const MatrixXd& V,
    const MatrixXi& E,
    const MatrixXi& F,
    const std::vector<int>& P,
    const double eps)
{
    std::vector<Eigen::Vector3d> verts(size_t(V.rows()));
    for (int i = 0; i < V.rows(); ++i) {
        verts[size_t(i)] = V.row(i).head<3>();
    }
    auto env = std::make_shared<SampleEnvelope>();
    env->use_exact = true;
    if (F.rows() > 0) {
        std::vector<Eigen::Vector3i> tris(size_t(F.rows()));
        for (int i = 0; i < F.rows(); ++i) {
            tris[size_t(i)] = Eigen::Vector3i(F(i, 0), F(i, 1), F(i, 2));
        }
        env->init(verts, tris, eps);
    } else {
        std::vector<Eigen::Vector2i> segs;
        segs.reserve(size_t(E.rows()) + P.size());
        for (int i = 0; i < E.rows(); ++i) {
            segs.emplace_back(E(i, 0), E(i, 1));
        }
        for (const int i : P) {
            segs.emplace_back(i, i);
        }
        env->init(verts, segs, eps);
    }
    return env;
}
} // namespace

// assumes tag has been found. won't be called otherwise
void TopoOffsetTetMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixSi& T_tags,
    const MatrixXd& V_env,
    const MatrixXi F_env,
    const std::vector<std::string>& tag_names,
    const std::string& sheet_name)
{
    // assert dimensions
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T.rows() == T_tags.rows());
    assert((V_env.rows() == 0) || (V_env.cols() == 3));
    assert((F_env.rows() == 0) || (F_env.cols() == 3));
    m_tags_count = T_tags.cols() + 1; // + 1 is for ambient

    // initialize connectivity
    init(T);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.resize(V.rows());
    m_edge_attribute.resize(6 * T.rows());
    m_face_attribute.resize(4 * T.rows());
    m_tet_attribute.resize(T.rows());

    // set envelope data
    if (V_env.rows() > 0) {
        logger().info(
            "Envelope ({} vertices, {} faces) found. will be retained in output.",
            V_env.rows(),
            F_env.rows());
        m_has_envelope = true;
        m_V_envelope = V_env;
        m_F_envelope = F_env;
    }

    // set tag string/id maps
    m_tag_id_to_name[0] = "ambient";
    m_tag_name_to_id["ambient"] = 0;
    for (int64_t i = 0; i < tag_names.size(); i++) {
        m_tag_id_to_name[i + 1] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i + 1;
    }

    // add potential new tags to maps
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

    // The envelope surface group is registered as a tag so an OPEN sheet can be selected as the
    // input complex; offset_selection is otherwise an expression over tet tags. Membership is
    // decided geometrically below: the .msh carries the sheet with its own vertices, nothing
    // matches by index, so a face inside the sheet's tube (the eps the tag envelopes use)
    // carries it. The 2D twin is the curve group.
    if (F_env.rows() > 0) {
        m_sheet_V = V_env;
        m_sheet_F = F_env;
        std::string name = sheet_name.empty() ? "surface" : sheet_name;
        while (m_tag_name_to_id.count(name)) name += "_";
        m_sheet_tag = int64_t(m_tag_id_to_name.size());
        m_tag_id_to_name[m_sheet_tag] = name;
        m_tag_name_to_id[name] = m_sheet_tag;
        m_tags_count++;
    }

    // collect int ids for offset output
    for (const std::string& name : m_offset_params.offset_output_tag) {
        m_offset_output_tag_ids.insert(m_tag_name_to_id[name]);
    }

    // propagate labels to tets
    const auto& tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        for (int j = 0; j < T_tags.cols(); j++) {
            if (T_tags.coeff(t_id, j) == 1) {
                m_tet_attribute[t_id].tag.insert(j + 1);
            }
        }
        if (m_tet_attribute[t_id].tag.size() == 0) { // tet is ambient
            m_tet_attribute[t_id].tag.insert(0);
        }
    }

    assert(ambient_assert());

    // set vertex positions
    const auto& verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        set_vertex_position(v_id, V.row(v_id));
    }

    // Once at load too: init_surfaces_and_boundaries() reads on_sheet to track the sheet group's
    // faces. label_input_complex() re-derives it afterwards.
    classify_sheet_faces();
    init_surfaces_and_boundaries();
}

void TopoOffsetTetMesh::init_surfaces_and_boundaries()
{
    const auto faces = get_faces();
    logger().info("F = {}", faces.size());

    // Which faces are tracked region boundaries: an interior face whose two tets carry different
    // tag sets, a face on the sheet group, or a wall face (no opposite tet: the ambient region
    // against the unmeshed outside). These flags are topology the operations maintain. Which of
    // them an envelope holds is face_is_held()'s, and the envelope is build_envelopes()'s.
    size_t n_faces_tracked = 0;
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);
        const size_t fid = ff.fid();
        const bool on_sheet = m_face_extra[fid].on_sheet;
        const auto t_opp = ff.switch_tetrahedron();
        if (t_opp) {
            const auto& tag0 = m_tet_attribute[ff.tid()].tag;
            const auto& tag1 = m_tet_attribute[t_opp.value().tid()].tag;
            if (tag0 == tag1 && !on_sheet) continue;
        }
        m_face_attribute[fid].m_is_surface_fs = true;
        ++n_faces_tracked;
        const size_t v1 = ff.vid();
        const size_t v2 = ff.switch_vertex().vid();
        const size_t v3 = ff.switch_edge().switch_vertex().vid();
        if (t_opp) {
            for (const size_t v : {v1, v2, v3}) m_vertex_extra[v].m_is_on_region = true;
        }
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
        m_vertex_attribute[v3].m_is_on_surface = true;
    }
    logger().info("\tTracked faces (region boundaries, sheet, domain wall): {}", n_faces_tracked);

    // track bounding box. box_min/box_max are only set by Parameters::init(), which plenty of
    // unit tests never call -- skip rather than index out of bounds.
    if (m_offset_params.box_min.size() >= 3 && m_offset_params.box_max.size() >= 3) {
        for (size_t i = 0; i < faces.size(); i++) {
            const auto vs = get_face_vertices(faces[i]);
            std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
            int on_bbox = -1;
            for (int k = 0; k < 3; k++) {
                if (m_vertex_attribute[vids[0]].m_posf[k] == m_offset_params.box_min[k] &&
                    m_vertex_attribute[vids[1]].m_posf[k] == m_offset_params.box_min[k] &&
                    m_vertex_attribute[vids[2]].m_posf[k] == m_offset_params.box_min[k]) {
                    on_bbox = k * 2;
                    break;
                }
                if (m_vertex_attribute[vids[0]].m_posf[k] == m_offset_params.box_max[k] &&
                    m_vertex_attribute[vids[1]].m_posf[k] == m_offset_params.box_max[k] &&
                    m_vertex_attribute[vids[2]].m_posf[k] == m_offset_params.box_max[k]) {
                    on_bbox = k * 2 + 1;
                    break;
                }
            }
            if (on_bbox < 0) {
                continue;
            }
            assert(!faces[i].switch_tetrahedron(*this)); // face must be on boundary

            const size_t fid = faces[i].fid(*this);
            m_face_attribute[fid].m_is_bbox_fs = on_bbox;

            for (const size_t vid : vids) {
                m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
            }
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });
}

bool TopoOffsetTetMesh::face_is_complex_boundary(const Tuple& f) const
{
    const bool a = m_tet_attribute[f.tid(*this)].label == 1;
    const std::optional<Tuple> opp = f.switch_tetrahedron(*this);
    const bool b = opp ? (m_tet_attribute[opp->tid(*this)].label == 1) : false;
    if (a != b) return true; // a cell of the complex on exactly one side
    if (a && b) return false; // interior to the complex
    // No complex cell on either side: the face is itself a piece of the complex (a sheet
    // selection, or a face the selection expression labelled), or it is not on it at all.
    return m_face_extra[f.fid(*this)].label == 1;
}

bool TopoOffsetTetMesh::face_is_held(const Tuple& f) const
{
    // See the declaration. The caller has a tracked face (m_is_surface_fs); the offset surface
    // is surface_envelope_for_face()'s to answer, and is excluded here for vertex_is_held().
    if (!f.switch_tetrahedron(*this)) return true; // the domain wall
    if (face_is_offset_surface_live(f)) return false;
    return !m_offset_params.deform_others || face_is_complex_boundary(f);
}

bool TopoOffsetTetMesh::vertex_is_held(const size_t vid) const
{
    // Most vertices are on no tracked face at all; the flag is a superset (it is never cleared),
    // so it only short-cuts the walk.
    if (!m_vertex_attribute[vid].m_is_on_surface) return false;
    for (const size_t tid : get_one_ring_tids_for_vertex(vid)) {
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            const size_t fid = f.fid(*this);
            if (!m_face_attribute[fid].m_is_surface_fs) continue;
            const auto vs = get_face_vids(f);
            if (vs[0] != vid && vs[1] != vid && vs[2] != vid) continue;
            if (face_is_held(f)) return true;
        }
    }
    return false;
}

void TopoOffsetTetMesh::build_envelopes()
{
    // See m_envelope's declaration: one envelope, around every held face, built once.
    m_envelope_eps = m_offset_params.envelope_size;
    std::vector<Eigen::Vector3i> tris;
    size_t n_tracked = 0, n_wall = 0;
    for (const Tuple& f : get_faces()) {
        if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
        ++n_tracked;
        if (!face_is_held(f)) continue;
        if (!f.switch_tetrahedron(*this)) ++n_wall;
        const auto vs = get_face_vids(f);
        tris.emplace_back(int(vs[0]), int(vs[1]), int(vs[2]));
    }
    const bool exact_ok = std::isfinite(m_envelope_eps) && m_envelope_eps > 0.;
    if (tris.empty()) {
        m_envelope = nullptr;
    } else {
        std::vector<Eigen::Vector3d> verts(vert_capacity());
        for (size_t i = 0; i < vert_capacity(); ++i) verts[i] = m_vertex_attribute[i].m_posf;
        m_envelope = std::make_shared<SampleEnvelope>(/*exact=*/exact_ok);
        m_envelope->init(verts, tris, m_envelope_eps);
    }
    logger().info(
        "\t[envelope] {} of {} tracked faces held ({} on the domain wall; deform_others {}: {}), "
        "eps {:.6g}, {}",
        tris.size(),
        n_tracked,
        n_wall,
        m_offset_params.deform_others,
        m_offset_params.deform_others ? "the input complex boundary and the wall"
                                      : "every region boundary and the wall",
        m_envelope_eps,
        exact_ok ? "EXACT" : "sampled (no valid eps)");
}

void TopoOffsetTetMesh::mark_input_complex_vertices()
{
    // The earliest point at which the input complex is known: label_input_complex() has just
    // evaluated the selection expression. Keys on the vertex label, so a solid complex, a sheet,
    // a wire and an isolated point are all covered. From here the operations maintain it.
    size_t n = 0;
    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        const bool on_input = m_vertex_extra[vid].label == 1;
        m_vertex_extra[vid].m_is_on_input = on_input;
        n += on_input ? 1 : 0;
    }
    logger().info("\tInput-complex vertices: {}", n);
}

bool TopoOffsetTetMesh::ambient_assert()
{
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        bool has_ambient = (m_tet_attribute[t_id].tag.count(0) != 0);
        if (has_ambient && (m_tet_attribute[t_id].tag.size() != 1)) {
            std::string err_str = "";
            for (int64_t tag : m_tet_attribute[t_id].tag) {
                err_str = err_str + " " + std::to_string(tag);
            }
            logger().info(err_str);
            return false;
        }
    }
    return true;
}

void TopoOffsetTetMesh::classify_sheet_faces()
{
    // See the declaration. Idempotent: every face is answered, so a stale true is cleared.
    if (m_sheet_tag < 0 || m_sheet_F.rows() == 0) return;
    const double eps = m_offset_params.envelope_size;
    if (!(std::isfinite(eps) && eps > 0.)) {
        logger().warn(
            "\tSurface group '{}': no valid envelope_size to classify faces by; no face carries it",
            m_tag_id_to_name[m_sheet_tag]);
        return;
    }
    std::vector<Eigen::Vector3d> sv(m_sheet_V.rows());
    for (int i = 0; i < m_sheet_V.rows(); ++i) sv[i] = m_sheet_V.row(i).head<3>();
    std::vector<Eigen::Vector3i> sf(m_sheet_F.rows());
    for (int i = 0; i < m_sheet_F.rows(); ++i)
        sf[i] = Eigen::Vector3i(m_sheet_F(i, 0), m_sheet_F(i, 1), m_sheet_F(i, 2));
    SampleEnvelope sheet(/*exact=*/true);
    sheet.init(sv, sf, eps);
    size_t n_on = 0;
    for (const Tuple& f : get_faces()) {
        const auto vs = get_face_vids(f);
        const std::array<Vector3d, 3> tri = {
            {m_vertex_attribute[vs[0]].m_posf,
             m_vertex_attribute[vs[1]].m_posf,
             m_vertex_attribute[vs[2]].m_posf}};
        const bool on = !sheet.is_outside(tri);
        m_face_extra[f.fid(*this)].on_sheet = on;
        n_on += on ? 1 : 0;
    }
    logger().info(
        "\tSurface group '{}' (tag {}): {} triangles; {} of the mesh's faces lie inside its tube "
        "(eps {:.6g}) and carry the tag",
        m_tag_id_to_name[m_sheet_tag],
        m_sheet_tag,
        m_sheet_F.rows(),
        n_on,
        eps);
}

void TopoOffsetTetMesh::label_input_complex()
{
    classify_sheet_faces(); // the mesh may have been refined since the last call
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

    if (single_body) {
        m_singlebody = true;
        m_single_tag = *tags_involved.begin();
        if (m_single_tag == m_sheet_tag) {
            // The sheet is the complex: its faces, their edges and vertices, no tets. The band
            // then grows on both sides of it. offset_in / offset_out have no meaning for it.
            size_t n = 0;
            for (const Tuple& f : get_faces()) {
                const size_t f_id = f.fid(*this);
                if (!m_face_extra[f_id].on_sheet) continue;
                m_face_extra[f_id].label = 1;
                m_edge_attribute[f.eid(*this)].label = 1;
                m_edge_attribute[f.switch_edge(*this).eid(*this)].label = 1;
                m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                for (const size_t v : get_face_vids(f)) m_vertex_extra[v].label = 1;
                ++n;
            }
            logger().info(
                "Using the surface group '{}' as the complex: {} faces",
                m_tag_id_to_name[m_single_tag],
                n);
            if (n == 0)
                log_and_throw_error(
                    "offset_selection names the surface group but no mesh face lies on it");
            mark_input_complex_vertices();
            return;
        }
        if (!(m_offset_params.offset_in || m_offset_params.offset_out)) {
            log_and_throw_error(
                "At least one of offset_in and offset_out must be true for singlebody mode.");
        }
        logger().info("Using single body mode for '{}'", m_tag_id_to_name[m_single_tag]);

        if (m_offset_params.offset_in &&
            m_offset_params.offset_out) { // input complex is body boundary
            auto tets = get_tets();
            for (const Tuple& t : tets) {
                size_t t_id = t.tid(*this);
                if (m_tet_attribute[t_id].tag.count(m_single_tag) == 0) {
                    continue;
                }

                auto check_and_set = [this](const Tuple& tup) {
                    auto other = tup.switch_tetrahedron(*this);
                    if (!other ||
                        (m_tet_attribute[other.value().tid(*this)].tag.count(m_single_tag) == 0)) {
                        m_face_extra[tup.fid(*this)].label = 1;
                        // propagate to children simplices
                        m_edge_attribute[tup.eid(*this)].label = 1;
                        m_edge_attribute[tup.switch_edge(*this).eid(*this)].label = 1;
                        m_edge_attribute[tup.switch_vertex(*this).switch_edge(*this).eid(*this)]
                            .label = 1;
                        m_vertex_extra[tup.vid(*this)].label = 1;
                        m_vertex_extra[tup.switch_vertex(*this).vid(*this)].label = 1;
                        m_vertex_extra[tup.switch_edge(*this).switch_vertex(*this).vid(*this)]
                            .label = 1;
                    }
                };

                // for each face, check if boundary and if so, set as input
                check_and_set(t);
                check_and_set(t.switch_face(*this));
                check_and_set(t.switch_edge(*this).switch_face(*this));
                check_and_set(t.switch_vertex(*this).switch_edge(*this).switch_face(*this));
            }
        } else if (m_offset_params.offset_in) { // input complex is everything outside body
            auto tets = get_tets();
            for (const Tuple& t : tets) {
                size_t t_id = t.tid(*this);
                if (m_tet_attribute[t_id].tag.count(m_single_tag) == 0) {
                    m_tet_attribute[t_id].label = 1;
                    // propagate to faces, edges, verts
                    auto v_ids = oriented_tet_vids(t_id);
                    for (const size_t& v_id : v_ids) {
                        m_vertex_extra[v_id].label = 1;
                    }
                    for (int i = 0; i < 6; i++) {
                        m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                    }
                    for (int i = 0; i < 4; i++) {
                        m_face_extra[tuple_from_face(t_id, i).fid(*this)].label = 1;
                    }
                } else {
                    auto check_and_set = [this](const Tuple& tup) {
                        auto other = tup.switch_tetrahedron(*this);
                        if (!other) {
                            m_face_extra[tup.fid(*this)].label = 1;
                            // propagate to children simplices
                            m_edge_attribute[tup.eid(*this)].label = 1;
                            m_edge_attribute[tup.switch_edge(*this).eid(*this)].label = 1;
                            m_edge_attribute[tup.switch_vertex(*this).switch_edge(*this).eid(*this)]
                                .label = 1;
                            m_vertex_extra[tup.vid(*this)].label = 1;
                            m_vertex_extra[tup.switch_vertex(*this).vid(*this)].label = 1;
                            m_vertex_extra[tup.switch_edge(*this).switch_vertex(*this).vid(*this)]
                                .label = 1;
                        }
                    };

                    check_and_set(t);
                    check_and_set(t.switch_face(*this));
                    check_and_set(t.switch_edge(*this).switch_face(*this));
                    check_and_set(t.switch_vertex(*this).switch_edge(*this).switch_face(*this));
                }
            }
        } else if (m_offset_params.offset_out) {
            auto tets = get_tets();
            for (const Tuple& t : tets) {
                size_t t_id = t.tid(*this);
                if (m_tet_attribute[t_id].tag.count(m_single_tag) != 0) {
                    m_tet_attribute[t_id].label = 1;
                    // propagate to faces, edges, verts
                    auto v_ids = oriented_tet_vids(t_id);
                    for (const size_t& v_id : v_ids) {
                        m_vertex_extra[v_id].label = 1;
                    }
                    for (int i = 0; i < 6; i++) {
                        m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                    }
                    for (int i = 0; i < 4; i++) {
                        m_face_extra[tuple_from_face(t_id, i).fid(*this)].label = 1;
                    }
                }
            }
        }
    } else { // not single body mode. must evaluate expression

        // label tets
        auto tets = get_tets();
        for (const Tuple& t : tets) {
            size_t t_id = t.tid(*this);
            if (expr->eval(m_tet_attribute[t_id].tag)) {
                m_tet_attribute[t_id].label = 1;
                // propagate to faces, edges, verts
                auto v_ids = oriented_tet_vids(t_id);
                for (const size_t& v_id : v_ids) {
                    m_vertex_extra[v_id].label = 1;
                }
                for (int i = 0; i < 6; i++) {
                    m_edge_attribute[tuple_from_edge(t_id, i).eid(*this)].label = 1;
                }
                for (int i = 0; i < 4; i++) {
                    m_face_extra[tuple_from_face(t_id, i).fid(*this)].label = 1;
                }
            }
        }

        // label faces
        auto faces = get_faces();
        for (const Tuple& f : faces) {
            size_t f_id = f.fid(*this);
            if (m_face_extra[f_id].label == 1) {
                continue;
            }

            // collect incident tags
            CellTag adj_tags = m_tet_attribute[f.tid(*this)].tag;
            auto other = f.switch_tetrahedron(*this);
            if (other) {
                for (const int64_t& tag : m_tet_attribute[other.value().tid(*this)].tag) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_face_extra[f_id].label = 1;
                // propagate to children simplices
                m_edge_attribute[f.eid(*this)].label = 1;
                m_edge_attribute[f.switch_edge(*this).eid(*this)].label = 1;
                m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
                m_vertex_extra[f.vid(*this)].label = 1;
                m_vertex_extra[f.switch_vertex(*this).vid(*this)].label = 1;
                m_vertex_extra[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
            }
        }

        // label edges
        auto edges = get_edges();
        for (const Tuple& e : edges) {
            size_t e_id = e.eid(*this);
            if (m_edge_attribute[e_id].label == 1) {
                continue;
            }

            // collect adjacent tags
            CellTag adj_tags;
            auto adj_tets = get_incident_tids_for_edge(e);
            for (const size_t& inc_t_id : adj_tets) {
                for (const int64_t& tag : m_tet_attribute[inc_t_id].tag) {
                    adj_tags.insert(tag);
                }
            }

            if (expr->eval(adj_tags)) {
                m_edge_attribute[e_id].label = 1;
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

            // collect one ring tags
            CellTag adj_tags;
            auto one_ring_t_ids = get_one_ring_tids_for_vertex(v_id);
            for (const size_t& t_id : one_ring_t_ids) {
                for (const int64_t& tag : m_tet_attribute[t_id].tag) {
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

bool TopoOffsetTetMesh::empty_input_complex()
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

void TopoOffsetTetMesh::init_input_complex_bvh()
{
    // used a few times, just collect once
    auto tets = get_tets();
    auto faces = get_faces();
    auto edges = get_edges();
    auto verts = get_vertices();

    // to check if a face is in the closure of input complex tets
    std::map<simplex::Face, bool> face_in_closure;
    for (const Tuple& f : faces) {
        size_t v0 = f.vid(*this);
        size_t v1 = f.switch_vertex(*this).vid(*this);
        size_t v2 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
        face_in_closure[simplex::Face(v0, v1, v2)] = false;
    }

    // to check if an edge is in the closure of input complex tets and faces
    std::map<simplex::Edge, bool> edge_in_closure;
    for (const Tuple& e : edges) {
        size_t v0 = e.vid(*this);
        size_t v1 = e.switch_vertex(*this).vid(*this);
        edge_in_closure[simplex::Edge(v0, v1)] = false;
    }

    // to check if a vertex is in the closure of input complex tets, faces, and edges
    std::map<size_t, bool> vertex_in_closure;
    for (const Tuple& v : verts) {
        vertex_in_closure[v.vid(*this)] = false;
    }

    // collect tets in input complex
    std::vector<simplex::Tet> complex_tets;
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].label == 1) {
            auto vs = oriented_tet_vids(t_id);
            complex_tets.emplace_back(vs[0], vs[1], vs[2], vs[3]);
            face_in_closure[simplex::Face(vs[0], vs[1], vs[2])] = true;
            face_in_closure[simplex::Face(vs[1], vs[2], vs[3])] = true;
            face_in_closure[simplex::Face(vs[2], vs[3], vs[0])] = true;
            face_in_closure[simplex::Face(vs[3], vs[0], vs[1])] = true;
            edge_in_closure[simplex::Edge(vs[0], vs[1])] = true;
            edge_in_closure[simplex::Edge(vs[0], vs[2])] = true;
            edge_in_closure[simplex::Edge(vs[0], vs[3])] = true;
            edge_in_closure[simplex::Edge(vs[1], vs[2])] = true;
            edge_in_closure[simplex::Edge(vs[1], vs[3])] = true;
            edge_in_closure[simplex::Edge(vs[2], vs[3])] = true;
            vertex_in_closure[vs[0]] = true;
            vertex_in_closure[vs[1]] = true;
            vertex_in_closure[vs[2]] = true;
            vertex_in_closure[vs[3]] = true;
        }
    }

    // collect isolated faces in input complex
    std::vector<simplex::Face> complex_faces;
    for (const Tuple& f : faces) {
        size_t f_id = f.fid(*this);
        size_t v0 = f.vid(*this);
        size_t v1 = f.switch_vertex(*this).vid(*this);
        size_t v2 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
        simplex::Face f_simp(v0, v1, v2);
        if (!face_in_closure[f_simp] && m_face_extra[f_id].label == 1) {
            complex_faces.emplace_back(v0, v1, v2);
            face_in_closure[f_simp] = true;
            edge_in_closure[simplex::Edge(v0, v1)] = true;
            edge_in_closure[simplex::Edge(v1, v2)] = true;
            edge_in_closure[simplex::Edge(v2, v0)] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
            vertex_in_closure[v2] = true;
        }
    }

    // collect isolated edges in input complex
    std::vector<simplex::Edge> complex_edges;
    for (const Tuple& e : edges) {
        size_t e_id = e.eid(*this);
        size_t v0 = e.vid(*this);
        size_t v1 = e.switch_vertex(*this).vid(*this);
        simplex::Edge e_simp(v0, v1);
        if (!edge_in_closure[e_simp] && m_edge_attribute[e_id].label == 1) {
            complex_edges.emplace_back(v0, v1);
            edge_in_closure[e_simp] = true;
            vertex_in_closure[v0] = true;
            vertex_in_closure[v1] = true;
        }
    }

    // collect isolated vertices
    std::vector<size_t> complex_verts;
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (!vertex_in_closure[v_id] && m_vertex_extra[v_id].label == 1) {
            complex_verts.push_back(v_id);
            vertex_in_closure[v_id] = true;
        }
    }

    // extract vertices included in simplicial complex
    std::vector<Vector3d> V_vec;
    std::map<size_t, size_t> v_id_map;
    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (vertex_in_closure[v_id]) {
            v_id_map[v_id] = V_vec.size();
            V_vec.push_back(m_vertex_attribute[v_id].m_posf);
        }
    }
    MatrixXd V(V_vec.size(), 3);
    for (int i = 0; i < V_vec.size(); i++) {
        V.row(i) = V_vec[i];
    }

    MatrixXi T(complex_tets.size(), 4); // tets
    int index = 0;
    for (const simplex::Tet& t_simp : complex_tets) {
        auto vs = t_simp.vertices();
        T.row(index) << v_id_map[vs[0]], v_id_map[vs[1]], v_id_map[vs[2]], v_id_map[vs[3]];
        index++;
    }

    MatrixXi F(complex_faces.size(), 3); // faces
    index = 0;
    for (const simplex::Face& f_simp : complex_faces) {
        auto vs = f_simp.vertices();
        F.row(index) << v_id_map[vs[0]], v_id_map[vs[1]], v_id_map[vs[2]];
        index++;
    }

    MatrixXi E(complex_edges.size(), 2); // edges
    index = 0;
    for (const simplex::Edge& e_simp : complex_edges) {
        auto vs = e_simp.vertices();
        E.row(index) << v_id_map[vs[0]], v_id_map[vs[1]];
        index++;
    }

    MatrixXi P(complex_verts.size(), 1); // isolated vertices
    index = 0;
    for (const size_t& v_id : complex_verts) {
        P(index, 0) = v_id_map[v_id];
        index++;
    }

    // One region per connected piece of the input complex -- see m_n_regions and
    // m_region_potentials. A piece is a connected component under vertex connectivity, read off
    // the captured complex so the numbering is fixed for the whole run. As in 2D.
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
        for (int i = 0; i < T.rows(); ++i) {
            unite(T(i, 0), T(i, 1));
            unite(T(i, 1), T(i, 2));
            unite(T(i, 2), T(i, 3));
        }
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

    // set BVH -- a fresh object rather than clear+reinit, so anything still holding the old one
    // keeps a coherent view.
    m_input_complex_bvh = std::make_shared<SimplicialComplexBVH>();
    m_input_complex_bvh->init(V, T, F, E, P);

    // The smooth offset potential's primitives, from the same extraction. Phi's 3D primitives
    // are triangles, segments and points; there is no volume primitive, so a solid input region
    // enters as its boundary -- the faces of the complex's tets with exactly one incident complex
    // tet. Outside the region, the only place an offset exists, distance to the region and
    // distance to its boundary are the same number.
    std::map<simplex::Face, int> boundary_count;
    for (const simplex::Tet& t_simp : complex_tets) {
        for (const simplex::Face& f : t_simp.faces()) {
            ++boundary_count[f];
        }
    }

    std::vector<Vector3i> phi_tris;
    std::vector<int64_t> phi_face_region;
    for (const auto& [f_simp, count] : boundary_count) {
        if (count != 1) continue; // interior to the complex: carries no boundary geometry
        const auto vs = f_simp.vertices();
        phi_tris.emplace_back(v_id_map[vs[0]], v_id_map[vs[1]], v_id_map[vs[2]]);
        phi_face_region.push_back(comp_of[size_t(phi_tris.back()[0])]);
    }
    for (int i = 0; i < F.rows(); ++i) { // isolated triangles of the complex
        phi_tris.emplace_back(F(i, 0), F(i, 1), F(i, 2));
        phi_face_region.push_back(comp_of[size_t(F(i, 0))]);
    }

    // The edge list must be complete: ipc derives faces_to_edges from it and throws if a
    // triangle edge is missing, and an edge's ESP weight counts the triangles on it.
    std::set<std::pair<int, int>> phi_edge_set;
    const auto add_edge = [&](const int a, const int b) {
        phi_edge_set.emplace(std::min(a, b), std::max(a, b));
    };
    for (const Vector3i& t : phi_tris) {
        add_edge(t[0], t[1]);
        add_edge(t[1], t[2]);
        add_edge(t[2], t[0]);
    }
    for (int i = 0; i < E.rows(); ++i) { // isolated edges (wires) of the complex
        add_edge(E(i, 0), E(i, 1));
    }

    MatrixXi F_phi(phi_tris.size(), 3);
    for (size_t i = 0; i < phi_tris.size(); ++i) {
        F_phi.row(i) = phi_tris[i].transpose();
    }
    MatrixXi E_phi(phi_edge_set.size(), 2);
    std::vector<int64_t> phi_seg_region;
    {
        int i = 0;
        for (const auto& [a, b] : phi_edge_set) {
            E_phi(i, 0) = a;
            E_phi(i, 1) = b;
            phi_seg_region.push_back(comp_of[size_t(a)]);
            ++i;
        }
    }

    std::vector<int> P_phi;
    for (int i = 0; i < P.rows(); ++i) {
        P_phi.push_back(P(i, 0));
    }

    // Kept, not built. The extraction must not diverge from the BVH's, so it is done here and
    // once; the potential itself needs target_distance and offset_dhat_factor.
    m_phi_V = V;
    m_phi_E = E_phi;
    m_phi_F = F_phi;
    m_phi_P = P_phi;
    m_phi_seg_region = phi_seg_region;
    m_phi_face_region = phi_face_region;
    m_phi_point_region.clear();
    for (const int p : P_phi) m_phi_point_region.push_back(comp_of[size_t(p)]);

    // The input complex needs no containment envelopes of its own: every simplex of it lies on
    // tag-region boundaries, so the per-tag envelopes already hold all of it.
}

void TopoOffsetTetMesh::init_offset_potential()
{
    if (m_phi_V.rows() == 0 || !m_input_complex_bvh) {
        log_and_throw_error("init_offset_potential() called before init_input_complex_bvh()");
    }
    // Which field defines the offset; see OffsetPotential.hpp and the offset_field parameter.
    // Both are built from the same extraction (m_phi_V/E/F/P), so whichever is chosen measures
    // the same geometry the diagnostics do.
    if (m_offset_params.offset_field == "euclidean") {
        // A query engine, not a tolerance: nearest_point_feature() supplies the foot point and
        // the feature kind the exact derivatives case on, and only the exact kind answers it.
        m_input_complex_envelope = euclidean_query_envelope(
            m_phi_V,
            m_phi_E,
            m_phi_F,
            m_phi_P,
            m_offset_params.target_distance);
        if (m_phi_F.rows() > 0) {
            logger().info(
                "\tOffset field: EUCLIDEAN (exact distance), level d = {}, {} triangles",
                m_offset_params.target_distance,
                m_phi_F.rows());
        } else {
            logger().info(
                "\tOffset field: EUCLIDEAN (exact distance), level d = {}, {} segments (no "
                "triangles in the input complex)",
                m_offset_params.target_distance,
                size_t(m_phi_E.rows()) + m_phi_P.size());
        }
        m_offset_potential = std::make_shared<EuclideanOffsetPotential3D>(
            m_input_complex_envelope,
            m_offset_params.target_distance);
        init_region_potentials(m_offset_params.target_distance, 0.);
        return;
    }

    // dhat is sized to the offset it has to hold, not to target_distance alone: construction
    // places the offset at the input tetrahedralization's own cell boundaries. The floor keeps
    // the configured factor authoritative whenever construction was good. As in 2D.
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
    m_offset_potential = std::make_shared<SmoothOffsetPotential3D>(
        m_phi_V,
        m_phi_E,
        m_phi_F,
        m_phi_P,
        delta,
        effective_factor);
    init_region_potentials(delta, effective_factor);
}

void TopoOffsetTetMesh::init_region_potentials(const double delta, const double effective_factor)
{
    // See m_region_potentials. A connected input complex is one region, and that region's field
    // is the union field itself -- nothing to build.
    m_region_potentials.clear();
    m_region_bvhs.clear();
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
        // The piece's own BVH, over its boundary primitives: what assign_band_regions() reads a
        // seed's piece off.
        {
            MatrixXi P_m(P_r.size(), 1);
            for (size_t i = 0; i < P_r.size(); ++i) P_m(i, 0) = P_r[i];
            auto bvh = std::make_shared<SimplicialComplexBVH>();
            bvh->init(m_phi_V, MatrixXi(0, 4), F_r, E_r, P_m);
            m_region_bvhs.push_back(bvh);
        }
        if (euclidean) {
            m_region_potentials.push_back(
                std::make_shared<EuclideanOffsetPotential3D>(
                    euclidean_query_envelope(m_phi_V, E_r, F_r, P_r, delta),
                    delta));
        } else {
            m_region_potentials.push_back(
                std::make_shared<
                    SmoothOffsetPotential3D>(m_phi_V, E_r, F_r, P_r, delta, effective_factor));
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

size_t TopoOffsetTetMesh::flood_fill()
{
    size_t current_id = 0;
    auto verts = get_vertices();
    std::map<size_t, bool> visited_verts;

    for (const Tuple& v : verts) {
        size_t v_id = v.vid(*this);
        if (m_vertex_extra[v_id].label == 0) continue; // vertex not in complex
        if (visited_verts.find(v_id) != visited_verts.end()) continue; // vertex already visited

        visited_verts[v_id] = true;
        std::queue<size_t> bfs_queue;

        // initial propagation
        auto onering_verts = connected_components_helper(v_id);
        for (const size_t& other_v_id : onering_verts) {
            if (visited_verts.find(other_v_id) == visited_verts.end()) { // other vertex not visited
                bfs_queue.push(other_v_id);
            }
        }

        while (!bfs_queue.empty()) {
            size_t curr_vid = bfs_queue.front();
            bfs_queue.pop();
            if (visited_verts.find(curr_vid) != visited_verts.end()) continue; // already visited
            visited_verts[curr_vid] = true;

            // propagate
            auto onering_verts_tmp = connected_components_helper(curr_vid);
            for (const size_t& other_v_id : onering_verts_tmp) {
                if (visited_verts.find(other_v_id) == visited_verts.end()) {
                    bfs_queue.push(other_v_id);
                }
            }
        }
        current_id++;
    }

    return current_id;
}

bool TopoOffsetTetMesh::is_order_2_edge(const std::array<size_t, 2>& e) const
{
    return get_order_of_edge(e) == 2;
}

bool TopoOffsetTetMesh::is_order_2_edge(const Tuple& e) const
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    return is_order_2_edge({{v1, v2}});
}

size_t TopoOffsetTetMesh::get_order_of_vertex(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_order;
}

void TopoOffsetTetMesh::init_vertex_order()
{
    std::array<size_t, 4> count{{0, 0, 0, 0}};

    for (const Tuple& t : get_vertices()) {
        const size_t vid = t.vid(*this);
        const size_t order = compute_vertex_order(vid);
        m_vertex_attribute[vid].m_order = order;
        count[order]++;
    }

    logger().info("Vertex order count (0,1,2,3): {}", count);
}

std::vector<std::array<size_t, 3>> TopoOffsetTetMesh::get_faces_by_condition(
    std::function<bool(const FaceAttributes&)> cond) const
{
    auto res = std::vector<std::array<size_t, 3>>();
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
        if (cond(m_face_attribute[fid])) {
            auto verts = get_face_vertices(f);
            res.emplace_back( //
                std::array<size_t, 3>{
                    {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
        }
    }
    return res;
}

double TopoOffsetTetMesh::max_band_vertex_distance() const
{
    // How far the offset surface ended up from the input complex, as a length. Exact (BVH
    // nearest point): dhat selects the level c, so an overestimate changes which surface the run
    // solves for. Returns 0 when there is no offset surface yet.
    std::vector<bool> on_band(vert_capacity(), false);
    for (const Tuple& f : get_faces()) {
        if (!face_is_offset_surface_live(f)) continue;
        for (const size_t vid : get_face_vids(f)) on_band[vid] = true;
    }
    double worst = 0.;
    for (size_t vid = 0; vid < vert_capacity(); ++vid) {
        if (!on_band[vid]) continue;
        if (!m_vertex_attribute[vid].m_is_rounded) continue;
        const Vector3d p = m_vertex_attribute[vid].m_posf;
        worst = std::max(worst, (p - m_input_complex_bvh->nearest_point(p)).norm());
    }
    return worst;
}

void TopoOffsetTetMesh::construct_offset(const std::filesystem::path& output_file)
{
    // make embedding simplicial (split components per Alg 1)
    logger().info("Creating simplicial embedding...");
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    if (!is_simplicially_embedded()) { // internally prints to console
        simplicial_embedding();
        bool dummy = is_simplicially_embedded(); // internally prints to console
    }
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_debug_frame("simplicial_embedding");
    }

    // THE envelope, once: the complex is labelled and embedded, and nothing has moved yet.
    build_envelopes();

    // repulsion_smoothing_passes: push the marched edges' outer ends out before the march.
    repulsion_smoothing();

    // initialize offset
    logger().info("Initializing offset...");
    // marching_tets() chooses the placement itself (construction_mode) and leaves the split mode
    // at Midpoint.
    marching_tets();
    m_edge_split_mode = EdgeSplitMode::Midpoint;
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_debug_frame("marching");
    }

    // Must stay outside the branch above and unconditional: consolidating renumbers, which
    // changes the order later passes enumerate operations in, which changes the run.
    consolidate_mesh();
    set_offset_tet_tags();
    consolidate_mesh();
    if (m_offset_params.debug_output) { // intermediate output
        write_debug_frame("offset_tagged");
    }

    assert(ambient_assert());
}

bool TopoOffsetTetMesh::is_simplicially_embedded() const
{
    int bad_tets = 0;
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        bad_tets += (!tet_is_simp_emb(t));
    }
    if (bad_tets == 0) {
        logger().info("\tInput complex/offset simplicially embedded: TRUE");
        return true;
    } else {
        logger().info(
            "\tInput complex/offset simplicially embedded: FALSE ({} bad tets)",
            bad_tets);
        return false;
    }
}

bool TopoOffsetTetMesh::tet_is_simp_emb(const Tuple& t) const
{
    size_t t_id = t.tid(*this);
    if (m_tet_attribute[t_id].label != 0) { // entire tet in input/offset
        return true;
    }

    auto vs = oriented_tet_vids(t);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_extra[vs[i]].label != 0) {
            vs_in.push_back(vs[i]);
        }
    }
    if (vs_in.size() <= 1) { // nothing or just one vertex in input
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t glob_eid = tuple_from_edge({{vs_in[0], vs_in[1]}}).eid(*this);
        return (m_edge_attribute[glob_eid].label != 0);
    } else if (vs_in.size() == 3) { // potentially one face in input
        auto [_, glob_fid] = tuple_from_face({{vs_in[0], vs_in[1], vs_in[2]}});
        return (m_face_extra[glob_fid].label != 0);
    } else { // all four verts in complex but tet isn't, can't be simplicially embedded
        return false;
    }
}

void TopoOffsetTetMesh::simplicial_embedding()
{
    // identify necessary tets to split (by vertices)
    std::vector<simplex::Tet> tets_to_split;
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);
        if (m_tet_attribute[tet.tid(*this)].label == 0) {
            bool to_split = true;
            for (int i = 0; i < 4; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 4];
                size_t v3 = vs[(i + 2) % 4];
                auto [_, glob_fid] = tuple_from_face({{v1, v2, v3}});
                if (m_face_extra[glob_fid].label == 0) {
                    to_split = false;
                    break;
                }
            }

            if (to_split) { // tet not in input but all faces are
                tets_to_split.push_back(simplex::Tet(vs[0], vs[1], vs[2], vs[3]));
            }
        }
    }

    // actually split tets
    for (const simplex::Tet& tet : tets_to_split) {
        const auto& vs = tet.vertices();
        Tuple t = tuple_from_vids(vs[0], vs[1], vs[2], vs[3]);
        std::vector<Tuple> garbage;
        if (!split_tet(t, garbage)) {
            log_and_throw_error("tet split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tTets split: {}", tets_to_split.size());

    // identify necessary faces to split
    std::vector<simplex::Face> faces_to_split;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto vs = get_face_vids(f);
        if (m_face_extra[f.fid(*this)].label == 0) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
                size_t glob_eid = tuple_from_edge({{v1, v2}}).eid(*this);
                if (m_edge_attribute[glob_eid].label == 0) {
                    to_split = false;
                }
            }

            if (to_split) {
                faces_to_split.push_back(simplex::Face(vs[0], vs[1], vs[2]));
            }
        }
    }

    // actually split faces
    for (const simplex::Face& f : faces_to_split) {
        const auto& vs = f.vertices();
        auto [t, _] = tuple_from_face({{vs[0], vs[1], vs[2]}});
        std::vector<Tuple> garbage;
        if (!split_face(t, garbage)) {
            log_and_throw_error("face split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tFaces split: {}", faces_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 0) { // edge not in input
            size_t v1_id = e.vid(*this);
            size_t v2_id = switch_vertex(e).vid(*this);
            if ((m_vertex_extra[v1_id].label != 0) && (m_vertex_extra[v2_id].label != 0)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = tuple_from_edge(e.vertices());
        std::vector<Tuple> garbage;
        if (!split_edge(t, garbage)) {
            log_and_throw_error("edge split failed! (simplicial_embedding)");
        }
    }
    logger().info("\tEdges split: {}", edges_to_split.size());
}

void TopoOffsetTetMesh::relabel_input_complex()
{
    // label_input_complex() only ever sets labels to 1; every label goes back to 0 first, so a
    // simplex an operation moved out of the complex is not left marked.
    for (const Tuple& v : get_vertices()) m_vertex_extra[v.vid(*this)].label = 0;
    for (const Tuple& e : get_edges()) m_edge_attribute[e.eid(*this)].label = 0;
    for (const Tuple& f : get_faces()) m_face_extra[f.fid(*this)].label = 0;
    for (const Tuple& t : get_tets()) m_tet_attribute[t.tid(*this)].label = 0;
    label_input_complex();
}

void TopoOffsetTetMesh::repulsion_smoothing()
{
    // The march traces every marched edge to target_distance only if every outer end is farther
    // than it (the maximum marchable distance, see marching_tets()); otherwise it falls back to
    // half that distance and the loop has to carry the front out. These passes push the outer
    // ends out first, under one per-tet energy: w AMIPS^3 plus, for each corner v of the cell that
    // is an outer end within it (repulsion_cell_term()), O(v) = (max(0, 2 delta - d(v)) /
    // front_conv)^2 -- one-sided, so an outer end already beyond 2 delta is left to AMIPS, and
    // 2 delta so that the march at delta splits each edge with room to spare (the fallback's "half
    // the maximum marchable distance" run backwards). The smoother minimises its sum over a
    // vertex's ring, the vetoes and the rounds' collapses and swaps compare its max, as in the
    // loop. An outer end that an envelope holds carries no term. First up to
    // repulsion_smoothing_passes smoothing passes, then up to repulsion_rounds rounds of the
    // loop's operations (see below); both stop once every outer end is beyond delta + front_conv,
    // outside the tolerance band.
    const int n_passes = m_offset_params.repulsion_smoothing_passes;
    const int n_rounds = m_offset_params.repulsion_rounds;
    if (n_passes <= 0 && n_rounds <= 0) return;
    const double delta = m_offset_params.target_distance;
    const double stop = delta + m_offset_params.front_conv;

    // The outer ends exactly as marching_tets() picks its edges: the two ends differ in "label
    // 0". Recomputed at every report: the rounds' operations change the set. n_held: those an
    // envelope holds, which is_repulsion_vertex() leaves to TetWild's rule.
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
                if (vertex_is_held(v_out)) ++n_held;
            }
        }
    };
    collect_outer();
    if (outer.empty()) {
        logger().info("\t[repulsion] no edge to march, no pass");
        return;
    }
    // The march's own distance (the same input-complex arrays as m_input_complex_bvh), at level
    // 2 delta. Non-null switches the repulsion on (is_repulsion_vertex()).
    m_repulsion_potential = std::make_shared<EuclideanOffsetPotential3D>(
        euclidean_query_envelope(m_phi_V, m_phi_E, m_phi_F, m_phi_P, 2. * delta),
        2. * delta);
    m_repulsion_term = std::make_shared<OffsetEnergy3D>(
        m_repulsion_potential,
        4. * offset_term_weight(),
        true,
        true,
        /*one_sided=*/true);

    // Logs the state and says whether every outer end is beyond delta + front_conv. AMIPS is
    // the cube root of the stored AMIPS^3.
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
            for (const size_t tid : get_one_ring_tids_for_vertex(v)) {
                ring_max = std::max(ring_max, get_quality(tuple_from_tet(tid)));
            }
        }
        double mesh_max = 0.;
        for (const Tuple& t : get_tets()) mesh_max = std::max(mesh_max, get_quality(t));
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
            std::cbrt(ring_max),
            std::cbrt(mesh_max));
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
        // simplex_in_input_complex() reads the complex off the cells' labels, which is exact
        // when the complex is a body's cells (offset_out) or the cells outside a body (offset_in).
        if (!m_singlebody || m_single_tag == m_sheet_tag ||
            m_offset_params.offset_in == m_offset_params.offset_out) {
            log_and_throw_error(
                "repulsion_rounds: supported for a single body offset inward or outward only "
                "(not a surface group, an expression, or both directions)");
        }
        m_edge_split_mode = EdgeSplitMode::Optimization;
        init_vertex_order(); // the collapse rules read it; optimize_offset() recomputes it
        compute_vertex_partition_morton();
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
        op_accounting_reset(); // the [ops accounting] lines below are per round
        swap_counters_reset();
        while (!done && r < n_rounds) {
            ++r;
            for (const auto& g : groups) {
                // The operations, then the labels from the tags, then the smoothing: the loop's
                // operations carry the tets' tags and labels but not the construction labels of
                // vertices, edges and faces, which say what the input complex is and so which
                // vertices the smoothing pushes.
                local_operations({{g[0], g[1], g[2], 0}});
                relabel_input_complex();
                // A collapse or a swap may have joined two complex vertices by an edge off the
                // complex. The march needs the complex simplicially embedded, so it is embedded
                // here, before the smoothing: the embedding's midpoints are new outer ends, and
                // the smoothing and the stop test must see the mesh the march will get. (Measured
                // without this, on the cylinder inward at 1e-1: one round left a single outer end
                // at 1.37 delta, the embedding after the rounds then split 249 tets' chords and
                // the march fell back to 0.027 delta.)
                if (!is_simplicially_embedded()) {
                    m_edge_split_mode = EdgeSplitMode::Midpoint;
                    simplicial_embedding();
                    bool dummy = is_simplicially_embedded();
                    m_edge_split_mode = EdgeSplitMode::Optimization;
                    init_vertex_order();
                    compute_vertex_partition_morton();
                }
                local_operations({{0, 0, 0, g[3]}});
            }
            // How every split, collapse and swap of the round ended, as the loop logs it per turn.
            // The embedding guard's refusals (repulsion_embedding_kept()) have no reason name in
            // the engine's counters, so these lines count them as unexplained; the guard's own
            // count is in the newton line below. Logged at info for that reason.
            logger().info("\t[repulsion] round {} [swap reject]: {}", r, swap_reject_report());
            swap_counters_reset();
            for (const OpKind k :
                 {OpKind::split,
                  OpKind::collapse,
                  OpKind::swap_32,
                  OpKind::swap_44,
                  OpKind::swap_56,
                  OpKind::swap_face}) {
                logger().info(
                    "\t[repulsion] round {} [ops accounting] {}",
                    r,
                    op_accounting_report(k));
            }
            op_accounting_reset();
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

bool TopoOffsetTetMesh::simplex_in_input_complex(const std::vector<size_t>& c) const
{
    // In a cell of the complex (label 1: a body cell for offset_out, a cell outside the body for
    // offset_in), as label_input_complex() labels them; for offset_in also on a domain-boundary
    // face of a body cell, which label_input_complex() adds to the complex.
    std::vector<Tuple> cells;
    if (c.size() == 2) {
        cells = get_incident_tets_for_edge(c[0], c[1]);
    } else if (c.size() == 3) {
        const auto [ft, fid] = tuple_from_face({{c[0], c[1], c[2]}});
        cells.push_back(ft);
        if (const auto o = ft.switch_tetrahedron(*this)) cells.push_back(*o);
    } else {
        return false;
    }
    for (const Tuple& t : cells) {
        if (m_tet_attribute[t.tid(*this)].label != 0) return true;
    }
    if (!m_offset_params.offset_in) return false;
    for (const Tuple& t : cells) {
        const size_t tid = t.tid(*this);
        if (m_tet_attribute[tid].tag.count(m_single_tag) == 0) continue;
        for (int j = 0; j < 4; ++j) {
            const Tuple f = tuple_from_face(tid, j);
            if (f.switch_tetrahedron(*this)) continue; // not on the domain boundary
            const auto fv = get_face_vids(f);
            bool contains = true;
            for (const size_t v : c) {
                contains = contains && std::find(fv.begin(), fv.end(), v) != fv.end();
            }
            if (contains) return true;
        }
    }
    return false;
}

bool TopoOffsetTetMesh::repulsion_embedding_kept(const std::vector<size_t>& tids) const
{
    // tet_is_simp_emb() on each cell, with the spanned simplex judged by
    // simplex_in_input_complex(): vertex and cell labels are exact during an operation pass, the
    // edge and face labels of new simplices are not.
    for (const size_t tid : tids) {
        if (m_tet_attribute[tid].label != 0) continue;
        std::vector<size_t> in;
        for (const size_t v : oriented_tet_vids(tid)) {
            if (m_vertex_extra[v].label != 0) in.push_back(v);
        }
        if (in.size() <= 1) continue;
        if (in.size() == 4 || !simplex_in_input_complex(in)) return false;
    }
    return true;
}

void TopoOffsetTetMesh::marching_tets()
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

        // if one background and other in input/offset
        if ((m_vertex_extra[v1].label == 0) != (m_vertex_extra[v2].label == 0)) {
            e_to_split.emplace_back(v1, v2);
        }
    }

    // sort edges (split longest first), should give better output mesh quality
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

    // init_optimize, only when the target is not below the maximum
    // marchable distance: optimize_offset() opens with a loop without refinement. A target below
    // it is marched to and the run is the ordinary one.
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
    std::vector<size_t> frontier_verts; // one ring of these verts are in offset
    for (const simplex::Edge& e : e_to_split) {
        // determine which vertex in input/offset
        size_t v_in = e.vertices()[0];
        if (m_vertex_extra[v_in].label == 0) {
            v_in = e.vertices()[1];
        }

        // split edge
        garbage.clear();
        Tuple t = tuple_from_edge(e.vertices());
        if (split_edge(t, garbage)) { // should never fail
            frontier_verts.push_back(v_in);
        } else {
            log_and_throw_error("edge split failed! (marching_tets)");
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

    // mark all offset tets and children
    for (const size_t& v_id : frontier_verts) {
        auto t_ids = get_one_ring_tids_for_vertex(v_id);
        for (const size_t& t_id : t_ids) {
            if (m_tet_attribute[t_id].label == 0) {
                m_tet_attribute[t_id].label = 2;
                // propagate to children
                for (int i = 0; i < 4; i++) {
                    size_t f_id = tuple_from_face(t_id, i).fid(*this);
                    if (m_face_extra[f_id].label != 1) {
                        m_face_extra[f_id].label = 2;
                    }
                }
                for (int i = 0; i < 6; i++) {
                    size_t e_id = tuple_from_edge(t_id, i).eid(*this);
                    if (m_edge_attribute[e_id].label != 1) {
                        m_edge_attribute[e_id].label = 2;
                    }
                }
                auto vs = oriented_tet_vids(t_id);
                for (const size_t& v_id : vs) {
                    if (m_vertex_extra[v_id].label != 1) {
                        m_vertex_extra[v_id].label = 2;
                    }
                }
            }
        }
    }
}

void TopoOffsetTetMesh::set_offset_tet_tags()
{
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].label == 2) {
            CellTag new_tag;

            // add existing protected tags
            for (const int64_t& existing_tag : m_tet_attribute[t_id].tag) {
                if (std::find(
                        m_offset_params.protected_tags.begin(),
                        m_offset_params.protected_tags.end(),
                        m_tag_id_to_name[existing_tag]) != m_offset_params.protected_tags.end()) {
                    new_tag.insert(existing_tag);
                }
            }

            // add actual offset tags
            if (m_offset_output_tag_ids.size() == 0) {
                if (new_tag.size() == 0) { // no protected tags, write ambient
                    new_tag.insert(0);
                }
            } else {
                for (const int64_t& tag : m_offset_output_tag_ids) {
                    new_tag.insert(tag);
                }
            }

            m_tet_attribute[t_id].tag = new_tag;
        }
    }
}

bool TopoOffsetTetMesh::offset_is_manifold()
{
    // collect tets in closed offset (labelled 1 or 2)
    auto tets = get_tets();
    std::vector<Vector4i> offset_tets;
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (cell_in_region(t_id)) {
            auto vs = oriented_tet_vids(t_id);
            offset_tets.emplace_back(vs[0], vs[1], vs[2], vs[3]);
        }
    }

    // construct tet matrix
    MatrixXi T(offset_tets.size(), 4);
    for (int i = 0; i < offset_tets.size(); i++) {
        T.row(i) = offset_tets[i];
    }

    // extract boundary
    MatrixXi F;
    igl::boundary_facets(T, F);

    // remove unreferenced
    VectorXi I;
    VectorXi J;
    igl::remove_unreferenced(F.maxCoeff() + 1, F, I, J);
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < F.cols(); j++) {
            F(i, j) = I(F(i, j));
        }
    }

    // check manifoldness
    bool is_edge_man = igl::is_edge_manifold(F);
    VectorXi B;
    bool is_vert_man = igl::is_vertex_manifold(F, B);
    return (is_edge_man && is_vert_man);
}

bool TopoOffsetTetMesh::invariants(const std::vector<Tuple>& tets)
{
    wmtk::utils::predicates::exactinit();
    for (const Tuple& t : tets) {
        auto vs = oriented_tet_vids(t);
        auto res = wmtk::utils::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);

        if (res != wmtk::utils::predicates::Orientation::NEGATIVE) {
            return false;
        }
    }
    return true;
}

void TopoOffsetTetMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(vertex_size(), -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<paraviewo::CellElement> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset; // vertices to offset
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_extra[i].label == 1) {
            verts_to_offset.push_back(m_vertex_attribute[i].m_posf);
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
        if (m_edge_attribute[e.eid(*this)].label == 1) {
            std::vector<int> curr_e;
            curr_e.push_back(vid_map[e.vid(*this)]);
            curr_e.push_back(vid_map[switch_vertex(e).vid(*this)]);
            paraviewo::CellElement curr_e_elem;
            curr_e_elem.vertices = curr_e;
            curr_e_elem.ctype = paraviewo::CellType::Line;
            cells.push_back(curr_e_elem);
        }
    }

    // get all offset input faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        if (m_face_extra[f.fid(*this)].label == 1) {
            std::vector<int> curr_f;
            curr_f.push_back(vid_map[f.vid(*this)]);
            Tuple f1 = switch_vertex(f);
            curr_f.push_back(vid_map[f1.vid(*this)]);
            Tuple f2 = switch_vertex(switch_edge(f1));
            curr_f.push_back(vid_map[f2.vid(*this)]);
            paraviewo::CellElement curr_f_elem;
            curr_f_elem.vertices = curr_f;
            curr_f_elem.ctype = paraviewo::CellType::Triangle;
            cells.push_back(curr_f_elem);
        }
    }

    // get all offset input tets
    int num_tets = tet_size();
    for (size_t i = 0; i < num_tets; i++) {
        if (m_tet_attribute[i].label == 1) {
            auto vids = oriented_tet_vids(i);
            std::vector<int> curr_t;
            for (const size_t vid : vids) {
                curr_t.push_back(vid_map[vid]);
            }
            paraviewo::CellElement curr_t_elem;
            curr_t_elem.vertices = curr_t;
            curr_t_elem.ctype = paraviewo::CellType::Tetrahedron;
            cells.push_back(curr_t_elem);
        }
    }

    // output
    paraviewo::VTUWriter writer;
    writer.write_mesh(path + ".vtu", V, cells);
}

void TopoOffsetTetMesh::write_vtu(const std::string& path)
{
    logger().info("Write {}.vtu (tag for offset is included)", path);

    // Writing debug output must not change the mesh, so never consolidate here. Only the output
    // is compacted, locally: point arrays stay capacity-sized and slot-indexed, so every vid
    // stays valid and dead slots are unreferenced points; the cell arrays are packed.
    const auto& vs = get_vertices();
    const auto& tets = get_tets();
    const auto faces_in = get_faces_by_condition(
        [](auto& f) { return f.m_is_surface_fs && f.m_surface_class != OFFSET_SURFACE_CLASS; });
    const auto faces_off = get_faces_by_condition(
        [](auto& f) { return f.m_is_surface_fs && f.m_surface_class == OFFSET_SURFACE_CLASS; });
    std::vector<simplex::Edge> edges;
    for (const Tuple& t : get_edges()) {
        simplex::Edge e = simplex_from_edge(t);
        if (is_order_2_edge(e.vertices())) {
            edges.push_back(e);
        }
    }

    MatrixXd V(vert_capacity(), 3);
    MatrixXi T(tets.size(), 4);
    MatrixXi F_in(faces_in.size(), 3);
    MatrixXi F_off(faces_off.size(), 3);
    MatrixXi E(edges.size(), 2);

    V.setZero();
    T.setZero();
    F_in.setZero();
    F_off.setZero();
    E.setZero();

    // last matrix is offset
    std::vector<MatrixXd> tags(m_tags_count + 1, MatrixXd(tets.size(), 1));
    VectorXd amips(tets.size());
    VectorXd labels(vert_capacity());
    labels.setZero();
    VectorXd v_order(vert_capacity());
    v_order.setZero();
    VectorXd v_id(vert_capacity());
    v_id.setZero();
    // The sizing field, as point data: it drives every split and collapse gate. Two forms, as
    // in 2D: the raw scalar, and the target edge length l * scalar it means.
    VectorXd v_sizing(vert_capacity());
    v_sizing.setZero();
    VectorXd v_target(vert_capacity());
    v_target.setZero();
    // Front convergence diagnostics, as point data: the vertex measure the loop reports next to
    // what it does not, so the two can be compared at the same vertex. Same three fields as 2D.
    //
    //   front_conv_ratio      front_vertex_conv_ratio(): the vertex's own relative error
    //                         |relative_residual()| over the relative bar -- the face term's
    //                         measure at the vertex alone, its distance to the level set along
    //                         the field over front_conv (for the euclidean field |d -
    //                         target_distance| / front_conv). <= 1 reads as "placed". The loop reports it and no
    //                         longer exits on it: since 2026-09-25 the exit tests the face
    //                         measure alone (EnergyCriterion::converged()).
    //   front_residual_rel    residual_length() over front_conv: the vertex's actual distance to
    //                         the level set, as a MULTIPLE OF THE BAR, so < 1 is converged. Never
    //                         tested by the loop -- front_conv_ratio is the per-vertex measure it
    //                         reports -- and the two are the same number under both fields, up to
    //                         rounding: the distance to the level set along the field over the
    //                         bar. Until 2026-09-27 they parted company under "smooth", where
    //                         front_conv_ratio was the relative FIELD error (Phi - c)/c and read
    //                         3.44x this one (see SmoothOffsetPotential::relative_residual()).
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
    // geometry left. -1 marks a vertex that is not on the front, -2 a
    // value that is not finite. Costs one objective build per front vertex per frame, which is
    // the same work energy_criterion() does once a turn; debug output only.
    VectorXd v_conv(vert_capacity()), v_resid(vert_capacity()), v_grad(vert_capacity()),
        v_align(vert_capacity()), v_cdist(vert_capacity());
    v_conv.setConstant(-1.);
    v_resid.setConstant(-1.);
    v_grad.setConstant(-1.);
    v_align.setConstant(-1.);
    v_cdist.setConstant(-1.);

    // Collapsed-foldover flag, as point data: 1 where the vertex is an endpoint of an offset
    // surface edge whose two faces have folded back onto each other (outer angle over the
    // threshold), 0 elsewhere. DEBUG ONLY -- computed only under debug_output, so a plain
    // save_vtu run writes it as all 0 rather than paying for the surface walk.
    // See offset_surface_foldover_labels() for what the angle is and how its side is decided.
    VectorXd v_fold(vert_capacity());
    v_fold.setZero();
    if (m_offset_params.debug_output) {
        const std::vector<char> fold = offset_surface_foldover_labels();
        for (size_t vid = 0; vid < fold.size() && vid < size_t(v_fold.size()); ++vid) {
            v_fold[int(vid)] = fold[vid] ? 1. : 0.;
        }
    }

    // The front solves since the last frame (m_front_solve_log), as point data:
    //   front_newton_iters   the Newton iterations the vertex's solve took; 10 is the cap.
    //   front_newton_status  polysolve's stop status + 1, NewtonCounters::status_name()'s
    //                        numbering: 2 IterationLimit, 6 GradNormTolerance,
    //                        7 RelGradNormTolerance, 12 LineSearchFailed.
    // -1 where the vertex was not solved since the last frame: not on the front, refused before
    // its solve, or the frame closes an operation pass. DEBUG ONLY, as the log is.
    VectorXd v_newton_it(vert_capacity()), v_newton_st(vert_capacity());
    v_newton_it.setConstant(-1.);
    v_newton_st.setConstant(-1.);
    for (const FrontSolveRecord& r : m_front_solve_log) {
        if (r.vid >= vert_capacity()) continue;
        v_newton_it[int(r.vid)] = r.iterations;
        v_newton_st[int(r.vid)] = r.status;
    }

    for (size_t k = 0; k < tets.size(); ++k) {
        const size_t t_id = tets[k].tid(*this);
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](k, 0) = (m_tet_attribute[t_id].tag.count(i) == 1) ? 1 : 0;
        }
        tags[m_tags_count](k, 0) = (m_tet_attribute[t_id].label == 2) ? 1 : 0;
        amips[k] = m_tet_attribute[t_id].m_quality;
    }

    for (size_t i = 0; i < faces_in.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_in(i, j) = faces_in[i][j];
        }
    }

    for (size_t i = 0; i < faces_off.size(); ++i) {
        for (size_t j = 0; j < 3; ++j) {
            F_off(i, j) = faces_off[i][j];
        }
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        E(i, 0) = edges[i].vertices()[0];
        E(i, 1) = edges[i].vertices()[1];
    }

    for (const Tuple& v : vs) {
        size_t vid = v.vid(*this);
        labels[vid] = m_vertex_extra[vid].label;
        v_order[vid] = m_vertex_attribute[vid].m_order;
        v_id[vid] = vid;
        v_sizing[vid] = m_vertex_attribute[vid].m_sizing_scalar;
        v_target[vid] = m_params.l * v_sizing[vid];
    }

    // MEASURED AGAINST A FRESHLY DERIVED BAND-REGION MAP, as in 2D. Every one of these reads the
    // vertex's own region's field through potential_for(vid) -> m_vertex_region, and that member
    // is rebuilt only once a turn. Between rebuilds it is stale two different ways: a split
    // appends vertices it does not cover (harmless -- vertex_region() bounds-checks and they fall
    // back to the union field), and a base pass consolidates and renumbers every vid mid-pass,
    // after which its entries name the WRONG vertices and the measures come out as residuals of
    // tens of delta and ratios in the thousands. Testing the map's size caught the second case
    // only by also catching the first, and refused the great majority of a run's frames.
    //
    // So the map is re-derived here and put back exactly as it was afterwards: the frame is
    // measured against the regions the mesh has AT THIS MOMENT -- which is what
    // energy_criterion() sees, since the loop rebuilds the map before it measures -- and the run
    // reads the same (possibly stale) map after the frame as before it. -1 marks a vertex that is
    // not on the front, -2 a value that is not finite; there is no "not measured" case.
    std::vector<int> saved_cell_region, saved_vertex_region;
    const bool region_map_refreshed = !m_region_potentials.empty();
    if (region_map_refreshed) {
        saved_cell_region = m_cell_region;
        saved_vertex_region = m_vertex_region;
        assign_band_regions(/*log=*/false);
    }
    {
        const auto finite_or = [](const double x) { return std::isfinite(x) ? x : -2.; };
        for (const Tuple& v : vs) {
            const size_t vid = v.vid(*this);
            if (!m_vertex_extra[vid].m_is_on_offset || !m_vertex_attribute[vid].m_is_rounded) {
                continue;
            }
            const Vector3d p = m_vertex_attribute[vid].m_posf;
            const auto& pot = potential_for(vid);
            v_conv[vid] = finite_or(front_vertex_conv_ratio(vid));
            // RELATIVE to the one bar, so < 1 reads as placed at a glance. front_conv is
            // a length and residual_length() is a length, so the quotient is the same
            // test the criterion makes, in the criterion's own units.
            v_resid[vid] =
                finite_or(pot.residual_length(p) / std::max(m_offset_params.front_conv, 1e-300));
            v_grad[vid] = finite_or(pot.gradient(p).norm());
            v_align[vid] = finite_or(front_move_alignment(vid));
            v_cdist[vid] =
                m_input_complex_bvh ? finite_or(m_input_complex_bvh->dist(VectorXd(p))) : -2.;
        }
    }

    for (size_t k = 0; k < tets.size(); ++k) {
        const auto& loc_vs = oriented_tet_vertices(tets[k]);
        for (int j = 0; j < 4; j++) {
            T(k, j) = loc_vs[j].vid(*this);
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    paraviewo::VTUWriter writer;
    writer.add_cell_field("amips", amips);
    for (int64_t i = 0; i < m_tags_count; i++) {
        writer.add_cell_field(m_tag_id_to_name[i], tags[i]);
    }
    writer.add_cell_field("offset_tag", tags[m_tags_count]);
    writer.add_field("labels", labels);
    writer.add_field("order", v_order);
    writer.add_field("vid", v_id);
    writer.add_field("sizing_scalar", v_sizing);
    writer.add_field("target_edge_length", v_target);
    writer.add_field("front_conv_ratio", v_conv);
    writer.add_field("front_residual_rel", v_resid);
    writer.add_field("front_grad_norm", v_grad);
    writer.add_field("front_move_align", v_align);
    writer.add_field("front_complex_distance", v_cdist);
    writer.add_field("offset_foldover", v_fold);
    writer.add_field("front_newton_iters", v_newton_it);
    writer.add_field("front_newton_status", v_newton_st);
    writer.write_mesh(path + ".vtu", V, T, paraviewo::CellType::Tetrahedron);

    // surface
    const std::string surf_out_path = path + "_surf.vtu";
    {
        paraviewo::VTUWriter surf_writer;
        surf_writer.add_field("order", v_order);
        surf_writer.add_field("vid", v_id);
        surf_writer.add_field("sizing_scalar", v_sizing);
        logger().info("Write {}", surf_out_path);
        surf_writer.write_mesh(surf_out_path, V, F_in, paraviewo::CellType::Triangle);
    }

    // offset faces
    const std::string off_out_path = path + "_off.vtu";
    {
        // The front's per-FACE convergence measure, which since 2026-09-24 is THE convergence
        // test rather than the resolution half of it, and which has nowhere to live on the tet
        // frame above. The 2D twin writes the same pair on its `_front.vtu` line mesh.
        //
        //   front_err_ratio  the root of face_offset_term(): the RMS over the
        //                    face's stencil_order stencil of relative_residual() -- the distance
        //                    to the level set along the field over target_distance -- over the
        //                    one bar as a fraction of it.
        //                    > 1 is what makes a face refinable, and the same number at 1 point
        //                    is what makes a vertex placed. -1 unmeasurable, including a face
        //                    with a corner that is not a front vertex. Measured under the same
        //                    re-derived region map as the vertex fields above. REPLACES
        //                    front_sag_ratio, which reported an interpolation error against a
        //                    separate sag bar; a series mixing the two is comparing two
        //                    different quantities under one name, so the field was renamed
        //                    rather than redefined in place.
        //   chord_length     the face's longest edge, so the error can be read against the
        //                    geometry that produced it.
        //   front_ring_ratio point data, in both front_measure modes: the ring measure at each
        //                    front vertex, sqrt(mean of front_err_ratio^2) over its incident
        //                    offset faces with three front corners, every face weighted equally
        //                    -- the exit test under front_measure "vertex_ring"
        //                    (EnergyCriterion::ring_exit), with energy_criterion()'s rules: a
        //                    vertex with an unmeasurable incident face has none. NaN where there
        //                    is no ring measure.
        VectorXd f_err(faces_off.size()), f_len(faces_off.size());
        VectorXd v_ring_sum(vert_capacity()), v_ring_n(vert_capacity());
        v_ring_sum.setZero();
        v_ring_n.setZero();
        std::vector<char> ring_bad(vert_capacity(), 0);
        const auto front = [&](const size_t vid) {
            return m_vertex_extra[vid].m_is_on_offset && m_vertex_attribute[vid].m_is_rounded;
        };
        for (size_t i = 0; i < faces_off.size(); ++i) {
            const size_t a = faces_off[i][0], b = faces_off[i][1], c = faces_off[i][2];
            const bool all_front = front(a) && front(b) && front(c);
            const double term = all_front ? face_offset_term(a, b, c) : -1.;
            f_err[i] = term < 0. ? -1. : std::sqrt(term);
            const Vector3d pa = m_vertex_attribute[a].m_posf, pb = m_vertex_attribute[b].m_posf,
                           pc = m_vertex_attribute[c].m_posf;
            f_len[i] = std::max({(pb - pa).norm(), (pc - pb).norm(), (pa - pc).norm()});
            if (all_front) {
                for (const size_t u : {a, b, c}) {
                    if (term < 0.) {
                        ring_bad[u] = 1;
                    } else {
                        v_ring_sum[int(u)] += term;
                        v_ring_n[int(u)] += 1.;
                    }
                }
            }
        }
        VectorXd v_ring(vert_capacity());
        v_ring.setConstant(std::numeric_limits<double>::quiet_NaN());
        for (size_t vid = 0; vid < vert_capacity(); ++vid) {
            if (ring_bad[vid] || !(v_ring_n[int(vid)] > 0.)) continue;
            v_ring[int(vid)] = std::sqrt(v_ring_sum[int(vid)] / v_ring_n[int(vid)]);
        }
        paraviewo::VTUWriter off_writer;
        off_writer.add_cell_field("front_err_ratio", f_err);
        off_writer.add_cell_field("chord_length", f_len);
        off_writer.add_field("order", v_order);
        off_writer.add_field("vid", v_id);
        off_writer.add_field("sizing_scalar", v_sizing);
        off_writer.add_field("front_conv_ratio", v_conv);
        off_writer.add_field("front_ring_ratio", v_ring);
        off_writer.add_field("front_residual_rel", v_resid);
        off_writer.add_field("front_grad_norm", v_grad);
        off_writer.add_field("front_move_align", v_align);
        off_writer.add_field("front_complex_distance", v_cdist);
        off_writer.add_field("offset_foldover", v_fold);
        off_writer.add_field("front_newton_iters", v_newton_it);
        off_writer.add_field("front_newton_status", v_newton_st);
        logger().info("Write {}", off_out_path);
        off_writer.write_mesh(off_out_path, V, F_off, paraviewo::CellType::Triangle);
    }

    // The band-region map back exactly as the run left it (see the frame diagnostics above).
    if (region_map_refreshed) {
        m_cell_region.swap(saved_cell_region);
        m_vertex_region.swap(saved_vertex_region);
    }
    // edges
    const std::string edge_out_path = path + "_edge.vtu";
    {
        paraviewo::VTUWriter edge_writer;
        edge_writer.add_field("order", v_order);
        edge_writer.add_field("vid", v_id);
        edge_writer.add_field("sizing_scalar", v_sizing);
        logger().info("Write {}", edge_out_path);
        edge_writer.write_mesh(edge_out_path, V, E, paraviewo::CellType::Line);
    }
}

void TopoOffsetTetMesh::write_phi_grid(const std::string& path, const int n) const
{
    // A plane slice of the offset potential, carrying Phi per grid vertex. A dense volume is
    // unaffordable in 3D, so the slice is the plane through the box centre with the shortest
    // extent removed.
    if (n < 2 || !m_offset_potential) return;

    const Vector3d lo = m_offset_params.box_min, hi = m_offset_params.box_max;
    const Vector3d ext = hi - lo;
    int normal = 0;
    for (int d = 1; d < 3; ++d)
        if (ext[d] < ext[normal]) normal = d;
    const int a0 = (normal + 1) % 3, a1 = (normal + 2) % 3;
    const double at = 0.5 * (lo[normal] + hi[normal]);

    MatrixXd V(n * n, 3);
    MatrixXi F(2 * (n - 1) * (n - 1), 3);
    MatrixXd phi(n * n, 1), residual(n * n, 1), euclid(n * n, 1);
    const double level = m_offset_potential->target_level();

    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const int k = j * n + i;
            Vector3d p;
            p[normal] = at;
            p[a0] = lo[a0] + ext[a0] * i / (n - 1);
            p[a1] = lo[a1] + ext[a1] * j / (n - 1);
            V.row(k) = p.transpose();
            phi(k, 0) = std::min(m_offset_potential->value(p), 8. * level);
            residual(k, 0) = std::min(
                m_offset_potential->residual_length(p),
                8. * m_offset_params.target_distance);
            euclid(k, 0) = (p - m_input_complex_bvh->nearest_point(p)).norm();
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
        "Write {}_phi.vtu ({}x{} samples of the smooth offset potential on the {} = {:.6g} plane; "
        "the offset is the isosurface phi = {})",
        path,
        n,
        n,
        "xyz"[normal],
        at,
        level);
    auto writer = std::make_shared<paraviewo::VTUWriter>();
    writer->add_field("phi", phi);
    writer->add_field("phi_residual_length", residual);
    writer->add_field("euclidean_distance", euclid);
    writer->write_mesh(path + "_phi.vtu", V, F, paraviewo::CellType::Triangle);
}

void TopoOffsetTetMesh::write_msh_groups(const std::string& file)
{
    logger().info("Write {}.msh, {} tags", file, m_tags_count);
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& tets = get_tets();

    std::vector<Tuple> tets_with_tag;
    tets_with_tag.reserve(tets.size());

    // add vertices
    const auto& verts = get_vertices();
    msh.add_tet_vertices(verts.size(), [&](size_t k) {
        auto i = verts[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    auto msh_add_tets = [&]() {
        msh.add_tets(tets_with_tag.size(), [&](size_t k) {
            auto vs = oriented_tet_vids(tets_with_tag[k]);
            std::array<size_t, 4> data;
            for (int j = 0; j < 4; j++) {
                data[j] = vs[j];
            }
            return data;
        });
    };

    // add ambient
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        if (m_tet_attribute[t_id].tag.count(0) != 0) {
            tets_with_tag.push_back(t);
        }
    }
    msh_add_tets();
    msh.add_physical_group("ambient");

    // group for each tag
    for (int64_t tag_img = 1; tag_img < m_tags_count; tag_img++) {
        tets_with_tag.clear();
        for (const Tuple& t : tets) {
            size_t t_id = t.tid(*this);
            if (m_tet_attribute[t_id].tag.count(tag_img) != 0) {
                tets_with_tag.push_back(t);
            }
        }

        if (tets_with_tag.empty()) {
            continue;
        }

        msh.add_empty_vertices(3);
        msh_add_tets();

        const std::string group_name = m_tag_id_to_name[tag_img];
        msh.add_physical_group(group_name);
    }

    if (m_has_envelope) {
        msh.add_face_vertices(m_V_envelope.rows(), [this](size_t k) {
            return Vector3d(m_V_envelope.row(k));
        });
        msh.add_faces(m_F_envelope.rows(), [this](size_t k) {
            const auto r = m_F_envelope.row(k);
            return std::array<size_t, 3>{{size_t(r(0)), size_t(r(1)), size_t(r(2))}};
        });
        msh.add_physical_group("EnvelopeSurface");
    }

    // No node data: the file has the same shape as simwild's output (node and element blocks per
    // physical group, plus the retained envelope) and nothing else.
    msh.save(file + ".msh", true);
}

} // namespace wmtk::components::topological_offset
