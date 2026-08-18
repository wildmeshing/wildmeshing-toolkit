#include <fstream>
#include <set>
#include <wmtk/Types.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/EmbedTriangles.hpp>
#include <wmtk/utils/predicates.hpp>

#include "TetWildMesh.h"

#include <atomic>
#include <mutex>

namespace wmtk::components::tetwild {

// ---------------------------------------------------------------------------
// insertion_by_volumeremesher
//
// Conformally insert the input surface into a background tet mesh via
// vol_rem::embed_tri_in_poly_mesh:
//
//   1. Build a Delaunay background mesh over the input vertices plus a padded
//      voxel lattice (init_from_delaunay_box_mesh).
//   2. Flatten the input surface and the background tets into the flat coordinate
//      and index arrays the remesher expects.
//   3. Call embed_tri_in_poly_mesh, which computes the exact arrangement and
//      returns a conforming tetrahedralization with surface-tracking metadata.
//   4. Convert the remesher's bigrational coordinates to Vector3r, decode the
//      facets (already triangles, fixed stride of 4: one size prefix + 3 vertex
//      ids) and recover which output faces lie on the input surface, using
//      final_tets_parent (which polygonal cell each tet came from),
//      final_tets_parent_faces (which polygon faces bound it) and
//      cells_with_faces_on_input (a fast skip for cells touching no input face).
//   5. Compact away vertices left unreferenced by out_tets and remap all indices.
//
// Tets come straight from the remesher, so no centroid Steiner points are added
// and the output has no extra interior vertices. Orientation is passed through:
// the remesher already emits WMTK-positively oriented tets, so out_tets is copied
// into tets_after unchanged and only the 4 per-tet face flags are reordered into
// WMTK's local face order.
// ---------------------------------------------------------------------------
void TetWildMesh::insertion_by_volumeremesher(
    const std::vector<Vector3d>& vertices, // input surface vertices (double)
    const std::vector<std::array<size_t, 3>>& faces, // input surface triangles
    std::vector<Vector3r>& v_rational, // out: arrangement vertices (rational)
    std::vector<std::array<size_t, 3>>& polygon_faces, // out: triangular facets
    std::vector<bool>& is_v_on_input, // out: vertex-on-input-surface flags
    std::vector<std::array<size_t, 4>>& tets_after, // out: output tets
    std::vector<bool>& tet_face_on_input_surface, // out: 4 face-on-surface flags per tet
    const std::vector<Vector3d>& feature_edge_vertices, // optional feature-edge vertices
    const std::vector<std::array<size_t, 2>>& feature_edges, // optional feature edges
    const std::vector<Vector3d>& feature_points, // optional feature points
    utils::EmbedFeaturesResult* features_out) // out: where the features ended up
{
    logger().info("Insertion Surface: #V = {}, #F = {}", vertices.size(), faces.size());
    if (!feature_edges.empty() || !feature_points.empty()) {
        logger().info(
            "Insertion features: #E = {} ({} vertices), #P = {}",
            feature_edges.size(),
            feature_edge_vertices.size(),
            feature_points.size());
    }

    // Step 1: build the Delaunay background mesh.
    // generate background mesh
    init_from_delaunay_box_mesh(vertices);

    // Sanity check: the freshly built Delaunay tets should all be well oriented.
    if (m_params.perform_sanity_checks) {
        logger().info("Check inverted tets after init from delaunay...");
        for (const Tuple& t : get_tets()) {
            if (is_inverted(t)) {
                log_and_throw_error("Tet {} is inverted after init from delaunay!", t.tid(*this));
            }
        }
        logger().info("done");
    }

    // Step 2: flatten the background tet mesh into plain double / uint32 arrays
    // (xyz-interleaved coords, 4 vertex ids per tet).
    // prepare tet vertices and tet index info


    std::vector<double> tet_vrt_coord;
    std::vector<uint32_t> tet_indices;

    {
        const auto tet_vers = get_vertices();
        const auto tets = get_tets();
        tet_vrt_coord.resize(3 * tet_vers.size());
        tet_indices.resize(4 * tets.size());
        logger().info("After delaunay: #V = {}, #T = {}", tet_vers.size(), tets.size());

        for (int i = 0; i < tet_vers.size(); ++i) {
            tet_vrt_coord[3 * i] = m_vertex_attribute[i].m_posf[0];
            tet_vrt_coord[3 * i + 1] = m_vertex_attribute[i].m_posf[1];
            tet_vrt_coord[3 * i + 2] = m_vertex_attribute[i].m_posf[2];
        }

        for (int i = 0; i < tets.size(); ++i) {
            const auto tet_vids = oriented_tet_vids(tets[i]);
            tet_indices[4 * i] = (int)tet_vids[0];
            tet_indices[4 * i + 1] = (int)tet_vids[1];
            tet_indices[4 * i + 2] = (int)tet_vids[2];
            tet_indices[4 * i + 3] = (int)tet_vids[3];
        }
    }

    // Step 2 (cont.): flatten the input surface the same way.
    // prepare input surfaces info
    std::vector<double> tri_vrt_coord(3 * vertices.size());
    std::vector<uint32_t> triangle_indices(3 * faces.size());

    for (int i = 0; i < vertices.size(); ++i) {
        tri_vrt_coord[3 * i] = vertices[i][0];
        tri_vrt_coord[3 * i + 1] = vertices[i][1];
        tri_vrt_coord[3 * i + 2] = vertices[i][2];
    }

    for (int i = 0; i < faces.size(); ++i) {
        triangle_indices[3 * i] = (int)faces[i][0];
        triangle_indices[3 * i + 1] = (int)faces[i][1];
        triangle_indices[3 * i + 2] = (int)faces[i][2];
    }
    // Steps 3-5: the exact arrangement and everything that has to happen to its output --
    // bigrational conversion, facet decoding, surface tracking, vertex compaction and the
    // per-tet face-flag reorder. Shared with simwild, which does the same from matrices.
    std::vector<bool> polygon_faces_on_input;
    utils::EmbedTrianglesOptions opts;
    opts.check_collinear_input = m_params.perform_sanity_checks;
    opts.check_orientation = m_params.perform_sanity_checks;
    opts.check_surface_provenance = m_params.perform_sanity_checks;

    // Marshal the feature inputs into the remesher's flat arrays, and check the domain: the
    // features AND their forcing-triangle apexes -- which stick out by up to 0.1x the edge
    // length along an axis -- must lie inside the background box, or the remesher's
    // non-Delaunay insertion cannot locate them. The caller grows the input bbox over the
    // features, so a violation here means only one thing: a feature edge so long that a
    // tenth of it exceeds the box padding (diag/15). Throw with that diagnosis rather than
    // letting the remesher assert.
    const bool has_features = !feature_edges.empty() || !feature_points.empty();
    utils::EmbedFeaturesInput vr_features;
    if (has_features) {
        const auto& lo = m_tet_params.box_min;
        const auto& hi = m_tet_params.box_max;
        const auto in_box = [&](const Vector3d& p, double margin) {
            for (int d = 0; d < 3; ++d) {
                if (p[d] - margin < lo[d] || p[d] + margin > hi[d]) {
                    return false;
                }
            }
            return true;
        };
        vr_features.edge_vrt_coord.reserve(3 * feature_edge_vertices.size());
        for (const Vector3d& p : feature_edge_vertices) {
            vr_features.edge_vrt_coord.insert(vr_features.edge_vrt_coord.end(), p.data(), p.data() + 3);
        }
        vr_features.edge_indices.reserve(2 * feature_edges.size());
        for (const auto& e : feature_edges) {
            const Vector3d& a = feature_edge_vertices.at(e[0]);
            const Vector3d& b = feature_edge_vertices.at(e[1]);
            const double apex_margin = 0.1 * (a - b).norm();
            if (!in_box(a, apex_margin) || !in_box(b, apex_margin)) {
                log_and_throw_error(
                    "Feature edge ({}, {}) or its forcing-triangle apexes (offset 0.1 x its "
                    "length {}) leave the background box [{}, {}]; the edge is too long "
                    "relative to the box padding",
                    a.transpose(),
                    b.transpose(),
                    (a - b).norm(),
                    lo.transpose(),
                    hi.transpose());
            }
            vr_features.edge_indices.push_back(uint32_t(e[0]));
            vr_features.edge_indices.push_back(uint32_t(e[1]));
        }
        // Point apexes are offset by 0.1x the estimated point spacing, itself bounded by the
        // point set's own bbox diagonal -- small against the box padding; checked with the
        // conservative margin of the padding itself would over-reject, so points are checked
        // with no margin and the remesher's own debug assert stays as the backstop.
        vr_features.point_coord.reserve(3 * feature_points.size());
        for (const Vector3d& p : feature_points) {
            if (!in_box(p, 0)) {
                log_and_throw_error(
                    "Feature point ({}) lies outside the background box [{}, {}]",
                    p.transpose(),
                    lo.transpose(),
                    hi.transpose());
            }
            vr_features.point_coord.insert(vr_features.point_coord.end(), p.data(), p.data() + 3);
        }
    }

    utils::embed_triangles_in_tets(
        tri_vrt_coord,
        triangle_indices,
        tet_vrt_coord,
        tet_indices,
        v_rational,
        polygon_faces,
        polygon_faces_on_input,
        is_v_on_input,
        tets_after,
        tet_face_on_input_surface,
        opts,
        nullptr,
        has_features ? &vr_features : nullptr,
        has_features ? features_out : nullptr);


    // TODO this is a sanity check, but it is checked all the time for now, until insertion is
    // stable.
    // check for consistent orientation between tets
    std::set<std::array<size_t, 3>> face_set; // each face must be unique
    for (int i = 0; i < tets_after.size(); ++i) {
        const auto& t = tets_after[i];
        std::array<std::array<size_t, 3>, 4> faces = {{
            {{t[1], t[3], t[2]}}, // opposite t[0]
            {{t[0], t[2], t[3]}}, // opposite t[1]
            {{t[0], t[3], t[1]}}, // opposite t[2]
            {{t[0], t[1], t[2]}}, // opposite t[3]
        }};
        // Rotate vertex IDs in faces to start with the lowest vertex ID. This ensures that the
        // same face is represented by the same set of vertex IDs, regardless of the order in
        // which they appear in the tet.
        for (auto& f : faces) {
            std::rotate(f.begin(), std::min_element(f.begin(), f.end()), f.end());
        }

        for (const auto& f : faces) {
            if (face_set.count(f) > 0) {
                log_and_throw_error("Face {} appears more than once in the tet list", f);
            }
            face_set.insert(f);
        }
    }
}

void TetWildMesh::init_from_Volumeremesher(
    const std::vector<Vector3r>& v_rational,
    const std::vector<std::array<size_t, 3>>& facets,
    const std::vector<bool>& is_v_on_input,
    const std::vector<std::array<size_t, 4>>& tets,
    const std::vector<bool>& tet_face_on_input_surface,
    const utils::EmbedFeaturesResult* features,
    const std::vector<Vector3d>* feature_anchors)
{
    // Turn on feature tracking BEFORE init so the connectivity init sizes the edge
    // attributes along with everything else. Only when features exist: with the flag off the
    // shared operations skip every edge-tag branch and nothing is allocated.
    if (features != nullptr) {
        enable_feature_edge_tracking();
    }

    init_with_isolated_vertices(v_rational.size(), tets);
    assert(check_mesh_connectivity_validity());

    auto& VA = m_vertex_attribute;

    m_vertex_attribute.resize(v_rational.size());
    m_tet_attribute.resize(tets.size());
    m_face_attribute.resize(tets.size() * 4);

    // A VolumeRemesher output vertex is either a direct (explicit) point -- whose
    // exact coordinate is already a double -- or an indirect (implicit) point that
    // is genuinely rational. A direct point rounds for free: its coordinate already
    // equals its double, so snapping it cannot invert any tet. Mark those rounded
    // here and remember which are direct, so the rounding pass below only does real
    // work (and can do it in parallel) for the indirect points.
    std::vector<char> is_direct_point(vert_capacity(), 0);
    {
        // Per-vertex, independent: each i writes only its own m_pos/m_posf/
        // m_is_rounded/is_direct_point; the rational->double conversion and the
        // exact comparison read only local data.
        threading::parallel_for(
            threading::range(0, vert_capacity()),
            [&](const threading::range& range) {
                for (size_t i = range.begin(); i < range.end(); ++i) {
                    m_vertex_attribute[i].m_pos = v_rational[i];
                    m_vertex_attribute[i].m_posf = to_double(v_rational[i]);
                    const Vector3r& rp = m_vertex_attribute[i].m_pos;
                    const Vector3d& d = m_vertex_attribute[i].m_posf;
                    const bool direct = (wmtk::Rational(d[0]) == rp[0]) &&
                                        (wmtk::Rational(d[1]) == rp[1]) &&
                                        (wmtk::Rational(d[2]) == rp[2]);
                    is_direct_point[i] = direct ? 1 : 0;
                    m_vertex_attribute[i].m_is_rounded = direct;
                }
            },
            NUM_THREADS);
    }

    for (size_t i = 0; i < tet_face_on_input_surface.size(); i++) {
        m_face_attribute[i].m_is_surface_fs = tet_face_on_input_surface[i];
    }

    const auto faces = get_faces();
    logger().info("#faces = {}", faces.size());

    // mark surface vertices (parallel). Different faces write `true` to the shared
    // m_is_on_surface of a common vertex; atomic_ref makes those same-value writes
    // well-defined instead of a data race.
    threading::parallel_for(
        threading::range(0, faces.size()),
        [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                const Tuple& f = faces[i];
                if (m_face_attribute[f.fid(*this)].m_is_surface_fs != 1) continue;
                const size_t v1 = f.vid(*this);
                const size_t v2 = f.switch_vertex(*this).vid(*this);
                const size_t v3 = f.switch_edge(*this).switch_vertex(*this).vid(*this);
                std::atomic_ref<bool>(m_vertex_attribute[v1].m_is_on_surface).store(true);
                std::atomic_ref<bool>(m_vertex_attribute[v2].m_is_on_surface).store(true);
                std::atomic_ref<bool>(m_vertex_attribute[v3].m_is_on_surface).store(true);
            }
        },
        NUM_THREADS);

    // Feature tags: mark each tiling edge with its input file id and flag its vertices.
    // The tiling pairs come out of the same arrangement as the connectivity, so a missing
    // edge is a contract violation, not an input problem -- throw, don't warn.
    if (features != nullptr) {
        size_t n_tagged = 0;
        for (size_t e = 0; e < features->edge_tiling.size(); ++e) {
            for (const auto& seg : features->edge_tiling[e]) {
                const Tuple t = tuple_from_edge({{seg[0], seg[1]}});
                if (!t.is_valid(*this)) {
                    log_and_throw_error(
                        "Feature tiling edge ({}, {}) is not an edge of the initial mesh",
                        seg[0],
                        seg[1]);
                }
                m_feature_edge_attribute[t.eid(*this)].m_is_feature_edge = true;
                m_vertex_extra[seg[0]].m_is_on_feature_curve = true;
                m_vertex_extra[seg[1]].m_is_on_feature_curve = true;
                ++n_tagged;
            }
        }
        logger().info(
            "feature tags: {} tet edges tagged across {} input edges",
            n_tagged,
            features->edge_tiling.size());

        // Anchor the 0-dimensional features. Anchor positions are input coordinates, and
        // every one of them exists in the output exactly -- input points via the point
        // provenance, curve endpoints as tiling vertices -- and input coordinates are
        // explicit points, so their rationals ARE doubles and an exact double lookup finds
        // them. A vertex already carrying an id keeps it: one anchor per position is
        // enough, the retention audit is geometric.
        if (feature_anchors != nullptr && !feature_anchors->empty()) {
            std::map<std::array<double, 3>, size_t> vid_of;
            const auto key = [](const Vector3d& p) {
                return std::array<double, 3>{{p[0], p[1], p[2]}};
            };
            for (const int64_t v : features->point_vertex) {
                if (v >= 0) {
                    vid_of.emplace(key(m_vertex_attribute[size_t(v)].m_posf), size_t(v));
                }
            }
            for (const auto& tiling : features->edge_tiling) {
                for (const auto& seg : tiling) {
                    for (const size_t v : {seg[0], seg[1]}) {
                        vid_of.emplace(key(m_vertex_attribute[v].m_posf), v);
                    }
                }
            }
            size_t n_anchored = 0;
            for (const Vector3d& p : *feature_anchors) {
                const auto it = vid_of.find(key(p));
                if (it == vid_of.end()) {
                    log_and_throw_error(
                        "Feature anchor ({}) is not a vertex of the initial mesh",
                        p.transpose());
                }
                if (m_vertex_extra[it->second].m_feature_point_id != NO_FEATURE) {
                    continue;
                }
                m_vertex_extra[it->second].m_feature_point_id = m_feature_points.size();
                m_feature_points.push_back(p);
                ++n_anchored;
            }
            logger().info(
                "feature anchors: {} points anchored within {:.6} of their input positions",
                n_anchored,
                m_feature_eps);
        }
    }

    // track bounding box (parallel). The per-face exact-rational corner test is the
    // cost; on-bbox faces are rare, so each chunk collects (vertex, bbox-side) pairs
    // locally and merges once, and the per-vertex vectors are appended serially.
    {
        std::vector<std::pair<size_t, int>> bbox_vert_faces;
        std::mutex bbox_mutex;
        threading::parallel_for(
            threading::range(0, faces.size()),
            [&](const threading::range& r) {
                std::vector<std::pair<size_t, int>> local;
                for (size_t i = r.begin(); i < r.end(); i++) {
                    auto vs = get_face_vertices(faces[i]);
                    std::array<size_t, 3> vids = {
                        {vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
                    int on_bbox = -1;
                    for (int k = 0; k < 3; k++) {
                        if (m_vertex_attribute[vids[0]].m_pos[k] == m_tet_params.box_min[k] &&
                            m_vertex_attribute[vids[1]].m_pos[k] == m_tet_params.box_min[k] &&
                            m_vertex_attribute[vids[2]].m_pos[k] == m_tet_params.box_min[k]) {
                            on_bbox = k * 2;
                            break;
                        }
                        if (m_vertex_attribute[vids[0]].m_pos[k] == m_tet_params.box_max[k] &&
                            m_vertex_attribute[vids[1]].m_pos[k] == m_tet_params.box_max[k] &&
                            m_vertex_attribute[vids[2]].m_pos[k] == m_tet_params.box_max[k]) {
                            on_bbox = k * 2 + 1;
                            break;
                        }
                    }
                    if (on_bbox < 0) continue;
                    m_face_attribute[faces[i].fid(*this)].m_is_bbox_fs = on_bbox;
                    for (size_t vid : vids) local.emplace_back(vid, on_bbox);
                }
                if (local.empty()) return;
                std::lock_guard<std::mutex> lk(bbox_mutex);
                bbox_vert_faces.insert(bbox_vert_faces.end(), local.begin(), local.end());
            },
            NUM_THREADS);
        for (const auto& [vid, on_bbox] : bbox_vert_faces)
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
    }

    for_each_vertex([&](const Tuple& v) { vector_unique(VA[v.vid(*this)].on_bbox_faces); });

    // track open boundaries
    find_open_boundary();

    init_vertex_order();
    {
        std::array<size_t, 4> vo{{0, 0, 0, 0}};
        const auto vs = get_vertices();
        for (const Tuple& t : vs) {
            const size_t o = VA[t.vid(*this)].m_order;
            assert(o < 4);
            ++vo[o];
        }
        assert(vo[0] + vo[1] + vo[2] + vo[3] == vs.size());
        logger().info("{} vertices with order counts (0,1,2,3): {}", vs.size(), vo);
    }

    // Round the indirect vertices in parallel. Rounding snaps m_pos to its double
    // and may not invert any incident tet, so it is not independent across adjacent
    // vertices and the old code did it serially (round() per vertex, an exact-
    // rational inversion check over each vertex's incident tets). Instead, snap all
    // indirect vertices at once and then revalidate/revert the few conflicts:
    //   1. snap every not-yet-rounded (indirect) vertex to its double (parallel);
    //   2. scan all tets (parallel) -- any inverted tet must contain an indirect
    //      vertex (a direct vertex never moves, so cannot cause an inversion), so
    //      revert those indirect vertices to their exact rational coordinate;
    //   3. repeat until no tet is inverted.
    // This terminates: every non-empty pass un-rounds at least one vertex, and the
    // all-rational mesh is valid. Direct points are skipped throughout.
    for_each_vertex([&](const Tuple& v) {
        const size_t i = v.vid(*this);
        if (!VA[i].m_is_rounded) {
            VA[i].m_pos = to_rational(VA[i].m_posf);
            VA[i].m_is_rounded = true;
        }
    });

    while (true) {
        // A flag per vertex rather than a shared appended list: a vertex is reachable
        // from several tets, so the old version took a lock per push and then pushed the
        // same vid repeatedly. Every write here stores the same value to a slot indexed
        // by vid, so the threads cannot disagree -- relaxed is enough, and the duplicates
        // collapse for free.
        std::vector<std::atomic<uint8_t>> to_revert(vert_capacity());
        for (auto& f : to_revert) {
            f.store(0, std::memory_order_relaxed);
        }

        std::atomic<bool> any{false};
        for_each_tetra([&](const Tuple& t) {
            if (is_inverted(t)) {
                for (const size_t vid : oriented_tet_vids(t)) {
                    if (!is_direct_point[vid] && m_vertex_attribute[vid].m_is_rounded) {
                        to_revert[vid].store(1, std::memory_order_relaxed);
                        any.store(true, std::memory_order_relaxed);
                    }
                }
            }
        });
        if (!any.load(std::memory_order_relaxed)) {
            break;
        }
        for (size_t vid = 0; vid < to_revert.size(); ++vid) {
            if (!to_revert[vid].load(std::memory_order_relaxed)) {
                continue;
            }
            if (m_vertex_attribute[vid].m_is_rounded) {
                m_vertex_attribute[vid].m_pos = v_rational[vid];
                m_vertex_attribute[vid].m_is_rounded = false;
                m_all_rounded.store(false, std::memory_order_relaxed);
            }
        }
    }

    const auto vertices = get_vertices();
    size_t cnt_round_parallel = 0;
    for (const Tuple& v : vertices) {
        if (VA[v.vid(*this)].m_is_rounded) {
            ++cnt_round_parallel;
        }
    }

    // Final serial sweep: the parallel batch reverts a vertex whenever it lies in
    // any inverted tet, which can over-revert (a vertex may round fine once its
    // neighbours are committed). Retry the leftovers one at a time -- round() is a
    // no-op for the already-rounded majority, so this only pays for the few that
    // remain, and recovers vertices the batch conservatively reverted.
    for (const Tuple& v : vertices) {
        round(v);
    }

    size_t cnt_round = 0;
    for (const Tuple& v : vertices) {
        if (VA[v.vid(*this)].m_is_rounded) {
            ++cnt_round;
        }
    }

    logger().info(
        "Rounded vertices {}/{} (parallel {}, serial recovered {})",
        cnt_round,
        vertices.size(),
        cnt_round_parallel,
        cnt_round - cnt_round_parallel);

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });
}

void TetWildMesh::find_open_boundary()
{
    const auto fs = get_faces();
    const auto es = get_edges();
    std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);

    // for open boundary envelope
    std::vector<Eigen::Vector3d> v_posf(vert_capacity());
    std::vector<Eigen::Vector2i> open_boundaries;

    for (size_t i = 0; i < vert_capacity(); i++) {
        v_posf[i] = m_vertex_attribute[i].m_posf;
    }

    // count incident surface faces per edge (parallel; atomic_ref increments the shared array)
    threading::parallel_for(
        threading::range(0, fs.size()),
        [&](const threading::range& r) {
            for (size_t i = r.begin(); i < r.end(); i++) {
                const Tuple& f = fs[i];
                if (!m_face_attribute[f.fid(*this)].m_is_surface_fs) continue;
                const size_t eid1 = f.eid(*this);
                const size_t eid2 = f.switch_edge(*this).eid(*this);
                const size_t eid3 = f.switch_vertex(*this).switch_edge(*this).eid(*this);
                std::atomic_ref<int>(edge_on_open_boundary[eid1])
                    .fetch_add(1, std::memory_order_relaxed);
                std::atomic_ref<int>(edge_on_open_boundary[eid2])
                    .fetch_add(1, std::memory_order_relaxed);
                std::atomic_ref<int>(edge_on_open_boundary[eid3])
                    .fetch_add(1, std::memory_order_relaxed);
            }
        },
        NUM_THREADS);

    // collect open-boundary edges (exactly one incident surface face), parallel with a
    // per-chunk merge. The collected order differs from serial but the set is identical
    // and the boundary envelope built from it is order-independent.
    {
        std::mutex ob_mutex;
        threading::parallel_for(
            threading::range(0, es.size()),
            [&](const threading::range& r) {
                std::vector<Eigen::Vector2i> local;
                for (size_t i = r.begin(); i < r.end(); i++) {
                    const Tuple& e = es[i];
                    if (edge_on_open_boundary[e.eid(*this)] != 1) continue;
                    const size_t v1 = e.vid(*this);
                    const size_t v2 = e.switch_vertex(*this).vid(*this);
                    std::atomic_ref<bool>(m_vertex_extra[v1].m_is_on_open_boundary).store(true);
                    std::atomic_ref<bool>(m_vertex_extra[v2].m_is_on_open_boundary).store(true);
                    local.emplace_back(v1, v2);
                }
                if (local.empty()) return;
                std::lock_guard<std::mutex> lk(ob_mutex);
                open_boundaries.insert(open_boundaries.end(), local.begin(), local.end());
            },
            NUM_THREADS);
    }

    wmtk::logger().info("open boundary num: {}", open_boundaries.size());

    if (open_boundaries.size() == 0) {
        return;
    }

    // init the order-2 envelope (surface boundaries and non-manifold edges)
    //
    // It follows the surface envelope's choice of predicate rather than being hard-wired to
    // the sampled one. Both are containment tests against the same input at the same epsilon,
    // so having `use_sample_envelope: false` hold for the surface but not for its boundary
    // curves was an accident of the exact envelope not supporting edges until now.
    if (!m_order2_envelope) {
        m_order2_envelope = std::make_shared<SampleEnvelope>(m_envelope && m_envelope->use_exact);
    }
    // A FRACTION of eps here, deliberately, where the surface envelope in tetwild.cpp uses
    // all of it. The two are not symmetric: on the surface, widening the envelope REMOVES a
    // constraint that was blocking collapses, which is the whole point of that change; on an
    // open-boundary curve there is no such blockage to relieve, so the extra room is only
    // freedom for the boundary to wander within eps -- more geometry to resolve at the same
    // final quality. See /order2_envelope_ratio in the spec for the measurement.
    m_order2_envelope->init(
        v_posf,
        open_boundaries,
        m_params.epsr * m_params.diag_l * m_tet_params.order2_envelope_ratio);
}

bool TetWildMesh::is_open_boundary_edge(const Tuple& e)
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    if (!m_vertex_extra[v1].m_is_on_open_boundary || !m_vertex_extra[v2].m_is_on_open_boundary)
        return false;

    return !m_order2_envelope->is_outside(
        std::array<Eigen::Vector3d, 2>{
            {m_vertex_attribute[v1].m_posf, m_vertex_attribute[v2].m_posf}});
}

bool TetWildMesh::is_open_boundary_edge(const std::array<size_t, 2>& e)
{
    size_t v1 = e[0];
    size_t v2 = e[1];
    if (!m_vertex_extra[v1].m_is_on_open_boundary || !m_vertex_extra[v2].m_is_on_open_boundary)
        return false;

    return !m_order2_envelope->is_outside(
        std::array<Eigen::Vector3d, 2>{
            {m_vertex_attribute[v1].m_posf, m_vertex_attribute[v2].m_posf}});
}

// After having called the following:
// embed_tri_in_poly_mesh(
//    tri_vrt_coords,
//    triangle_indexes,
//    tet_vrt_coords,
//    tet_indexes,
//    vertices,
//    facets,
//    cells,
//    facets_on_input,
//    verbose
// );
//
// the tracked surface can be verified by calling:
//
// if (!checkTrackedFaces(vertices, tri_vrt_coords, facets, facets_on_input, triangle_indexes)) {
//  ... at least a facet in 'facets_on_input' is not coplanar with any facet in 'triangle_indexes'
// }
//
// bool TetWildMesh::checkTrackedFaces(
//     std::vector<vol_rem::bigrational>& vol_coords,
//     const std::vector<double>& surf_coords,
//     std::vector<uint32_t>& facets,
//     std::vector<uint32_t>& facets_on_input,
//     const std::vector<uint32_t>& surf_tris)
// {
//     std::vector<uint32_t> fstart; // Vector containing the starting index of each face in 'facets'
//     for (uint32_t f_i = 0; f_i < facets.size(); f_i += (facets[f_i] + 1)) fstart.push_back(f_i);
//     vol_rem::bigrational v1[3], v2[3], v3[3], v4[3];
//     for (uint32_t ti : facets_on_input) // For each facet in the tracked surface
//     {
//         uint32_t start = fstart[ti];
//         uint32_t fnv = facets[start]; // num face vertices
//         const uint32_t* face_vrts = facets.data() + start + 1;
//         size_t i = 0;
//         for (; i < surf_tris.size(); i += 3) { // For each input triangular facet 'i'
//             v2[0] = surf_coords[surf_tris[i] * 3]; // Let v2,v3,v4 be the coordinates of its three
//                                                    // vertices
//             v2[1] = surf_coords[surf_tris[i] * 3 + 1];
//             v2[2] = surf_coords[surf_tris[i] * 3 + 2];
//             v3[0] = surf_coords[surf_tris[i + 1] * 3];
//             v3[1] = surf_coords[surf_tris[i + 1] * 3 + 1];
//             v3[2] = surf_coords[surf_tris[i + 1] * 3 + 2];
//             v4[0] = surf_coords[surf_tris[i + 2] * 3];
//             v4[1] = surf_coords[surf_tris[i + 2] * 3 + 1];
//             v4[2] = surf_coords[surf_tris[i + 2] * 3 + 2];
//             uint32_t v = 0;
//             for (; v < fnv; v++) { // For each vertex 'v' of 'ti'
//                 const uint32_t vid = face_vrts[v];
//                 v1[0] = vol_coords[vid * 3]; // Let v1 be v's coordinates
//                 v1[1] = vol_coords[vid * 3 + 1];
//                 v1[2] = vol_coords[vid * 3 + 2];
//                 if (orient3D(
//                         v1[0],
//                         v1[1],
//                         v1[2],
//                         v2[0],
//                         v2[1],
//                         v2[2],
//                         v3[0],
//                         v3[1],
//                         v3[2],
//                         v4[0],
//                         v4[1],
//                         v4[2]) != 0)
//                     break; // 'v' is not coplanar with triangular facet 'i'
//             }
//             if (v == fnv) break; // All vertices of 'ti' are coplanar with triangular facet 'i'
//         }
//         if (i == surf_tris.size())
//             return false; // 'ti' is not coplanar with any triangular facet in surf_tris
//     }
//     return true;
// }

} // namespace wmtk::components::tetwild