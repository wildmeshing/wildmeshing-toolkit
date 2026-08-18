
#include <queue>
#include <algorithm>
#include "TriWildMesh.h"

#include <tuple>

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/WindingNumber.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <wmtk/utils/predicates.hpp>
#include <igl/winding_number.h>
#include <igl/write_triangle_mesh.h>
#include <igl/read_triangle_mesh.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/GeoUtils.h>
// clang-format on

//#include <paraviewo/HDF5VTUWriter.hpp>
#include <bitset>
#include <limits>
#include <paraviewo/VTUWriter.hpp>
#include <wmtk/utils/partition_utils.hpp>

namespace wmtk::components::triwild {


void TriWildMesh::init_mesh(
    const MatrixXd& V,
    const std::vector<Vector2r>& V_rational,
    const MatrixXi& F,
    const MatrixXi& E,
    const std::vector<std::string>& tag_names,
    const MatrixXd& V_env,
    const MatrixXi& E_env)
{
    assert(V.cols() == 2);
    assert(F.cols() == 3);
    assert(E.cols() == 2);

    init(F);

    assert(check_mesh_connectivity_validity());

    m_vertex_attribute.resize(V.rows());
    m_edge_attribute.resize(F.rows() * 3);

    // Take the arrangement's EXACT positions, and round separately -- the 2D counterpart of
    // what VolumemesherInsertion does with v_rational. m_pos used to be to_rational(V.row(i)),
    // i.e. the rounded double converted back, which silently threw the arrangement's
    // exactness away before anything could use it.
    //
    // A vertex whose coordinates happen to have an exact double representation ("direct")
    // rounds for free: snapping it changes nothing, so it cannot invert a triangle. The rest
    // start un-rounded and the sweep below reclaims whichever ones can be rounded without
    // inverting anything.
    assert(V_rational.empty() || V_rational.size() == size_t(V.rows()));
    size_t n_indirect = 0;
    for (int i = 0; i < vert_capacity(); i++) {
        auto& va = m_vertex_attribute[i];
        if (V_rational.empty()) {
            // No exact input available (a caller that only has doubles, e.g. a unit test).
            va.m_pos = to_rational(Vector2d(V.row(i)));
            va.m_posf = V.row(i);
            va.m_is_rounded = true;
            continue;
        }
        va.m_pos = V_rational[i];
        va.m_posf = Vector2d(V_rational[i][0].to_double(), V_rational[i][1].to_double());
        va.m_is_rounded =
            (Rational(va.m_posf[0]) == va.m_pos[0]) && (Rational(va.m_posf[1]) == va.m_pos[1]);
        if (!va.m_is_rounded) {
            ++n_indirect;
            m_all_rounded.store(false, std::memory_order_relaxed);
        }
    }
    if (n_indirect > 0) {
        logger().info(
            "{} of {} arrangement vertices have no exact double representation; rounding "
            "them where it does not invert a triangle",
            n_indirect,
            vert_capacity());
        round_all_vertices();
    }

    // Init quality, and check that the arrangement handed over a uniformly, positively
    // oriented triangulation.
    //
    // This used to trip on the first anomaly and report "Tets with different orientations in
    // the input!", which named neither of the two things it actually catches and, because
    // the old state machine took the first bad face as evidence that the WHOLE mesh was
    // inverted, reported "fully inverted" whenever the only bad face happened to be the last
    // one. Count instead, and say what was found. Same acceptance -- an all-positive mesh
    // passes, anything else throws -- only the message changed.
    //
    // The orientation is judged in EXACT arithmetic, on m_pos, not on the rounded m_posf.
    // Judging it on doubles is what used to make about a third of the 20k 2D dataset
    // unusable: two arrangement vertices that are exactly distinct can round to the same
    // double, and any triangle using both then looks exactly degenerate even though the
    // arrangement is perfectly valid. With the rationals kept and only the safely-roundable
    // vertices rounded (above), a triangle that is still degenerate here is a real one.
    size_t n_degenerate = 0, n_negative = 0, n_total = 0;
    size_t first_bad_fid = std::numeric_limits<size_t>::max();
    std::array<size_t, 3> first_bad_vids = {{0, 0, 0}};
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        const auto vs = oriented_tri_vids(fid);
        const Vector2r& p0 = m_vertex_attribute[vs[0]].m_pos;
        const Vector2r& p1 = m_vertex_attribute[vs[1]].m_pos;
        const Vector2r& p2 = m_vertex_attribute[vs[2]].m_pos;
        const Rational d = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (p2[0] - p0[0]);
        if (!(d > 0)) {
            if (d == 0) {
                ++n_degenerate;
            } else {
                ++n_negative;
            }
            if (first_bad_fid == std::numeric_limits<size_t>::max()) {
                first_bad_fid = fid;
                first_bad_vids = vs;
            }
        }
        ++n_total;
        m_face_attribute[fid].m_quality = get_quality(t);
    }

    if (n_degenerate + n_negative > 0) {
        if (n_degenerate + n_negative == n_total) {
            log_and_throw_error(
                "Input mesh is fully inverted! This should not happen... Might be a bug.");
        }
        const auto& p0 = m_vertex_attribute[first_bad_vids[0]].m_posf;
        const auto& p1 = m_vertex_attribute[first_bad_vids[1]].m_posf;
        const auto& p2 = m_vertex_attribute[first_bad_vids[2]].m_posf;
        log_and_throw_error(
            "The arrangement produced {} zero-area and {} negatively oriented triangles out "
            "of {} (measured exactly, on the arrangement's rational coordinates). First is "
            "face {} = ({}, {}, {}) at ({}, {}), ({}, {}), ({}, {}).",
            n_degenerate,
            n_negative,
            n_total,
            first_bad_fid,
            first_bad_vids[0],
            first_bad_vids[1],
            first_bad_vids[2],
            p0[0],
            p0[1],
            p1[0],
            p1[1],
            p2[0],
            p2[1]);
    }

    // mark edges as on surface if they are in E
    for (int i = 0; i < E.rows(); i++) {
        std::array<size_t, 2> vids = {{(size_t)E(i, 0), (size_t)E(i, 1)}};
        const auto [e, eid] = tuple_from_edge(vids);
        if (!e.is_valid(*this)) {
            log_and_throw_error("Edge {} in E is not found in the mesh!", vids);
        }
        m_edge_attribute[eid].m_is_surface_fs = true;
        m_vertex_attribute[vids[0]].m_is_on_surface = true;
        m_vertex_attribute[vids[1]].m_is_on_surface = true;
    }

    // Feature points: the 0-dimensional features of the curve network, taken from the
    // constrained edges E. Valence 1 is an open polyline's endpoint, valence >= 3 a junction;
    // valence 2 is the interior of a curve and valence 0 is not on it at all.
    //
    // Without these, the collapse pass deletes open polylines outright. A polyline erodes by
    // legal collapses of its interior into its tip until one segment is left, and that
    // segment has a feature at BOTH ends -- which the existing order test permits, because it
    // only refuses collapsing a feature into a non-feature. Measured on the 2D dataset:
    // 215292 went from 28 open components after the arrangement to 0 in the output, and
    // 134005 from 18 to 15, entirely in the collapse passes.
    {
        std::vector<int> surf_valence(vert_capacity(), 0);
        for (int i = 0; i < E.rows(); ++i) {
            ++surf_valence[E(i, 0)];
            ++surf_valence[E(i, 1)];
        }
        size_t n_endpoints = 0, n_junctions = 0;
        for (size_t v = 0; v < vert_capacity(); ++v) {
            const int val = surf_valence[v];
            if (val == 0 || val == 2) {
                continue;
            }
            if (val == 1) {
                ++n_endpoints;
            } else {
                ++n_junctions;
                // The erosion above is an ENDPOINT property. A junction cannot erode a curve
                // away -- every curve through it stays constrained and inside the envelope --
                // while anchoring it can freeze the whole mesh, because on a self-intersecting
                // input almost every arrangement vertex is a crossing. See
                // Parameters::allow_junction_cleanup.
                if (m_tri_params.allow_junction_cleanup) {
                    continue;
                }
            }
            m_vertex_extra[v].m_feature_id = m_feature_points.size();
            m_feature_points.push_back(m_vertex_attribute[v].m_posf);
        }
        if (n_endpoints + n_junctions > 0) {
            logger().info(
                "feature points: {} polyline endpoints, {} junctions; anchoring {} of them "
                "within {:.6} of their input positions",
                n_endpoints,
                n_junctions,
                m_feature_points.size(),
                m_envelope_eps);
        }
    }

    // init envelope
    if (m_envelope) {
        log_and_throw_error("Envelope was already initialized once.");
    }
    assert(m_V_envelope.empty() && m_E_envelope.empty());

    // Around the original input curves, not around the arrangement's constrained edges:
    // after simplification those are the coarsened curves, and the optimizer must stay near
    // what the user gave us. That is what makes the (opt-in) sanity check below a real
    // check rather than a tautology: it asks whether the simplified curves are still inside.
    m_V_envelope.resize(V_env.rows());
    for (size_t i = 0; i < m_V_envelope.size(); ++i) {
        m_V_envelope[i] = V_env.row(i);
    }
    m_E_envelope.resize(E_env.rows());
    for (size_t i = 0; i < m_E_envelope.size(); ++i) {
        m_E_envelope[i] = E_env.row(i);
    }

    m_envelope = std::make_shared<SampleEnvelope>(!m_tri_params.use_sample_envelope);
    m_envelope->init(m_V_envelope, m_E_envelope, m_envelope_eps);
    logger().info(
        "Envelope: {} (eps {:.6})",
        m_envelope->use_exact ? "EXACT" : "sampled",
        m_envelope_eps);

    // Sanity check: All surface edges must be inside the envelope. Opt-in: see
    // Parameters::check_envelope_at_init for why it is not worth its cost by default.
    if (m_tri_params.check_envelope_at_init) {
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

    // Track the bounding box. This is the box of the *domain* -- the arrangement covers the
    // input's bounding box grown by the background grid -- which is not m_tri_params.box_min /
    // box_max: those come from the input curves and set the tolerances. Deriving it from V
    // keeps this check self-consistent whatever the input box is.
    const Vector2d domain_min = V.colwise().minCoeff();
    const Vector2d domain_max = V.colwise().maxCoeff();

    const auto edges = get_edges();
    for (size_t i = 0; i < edges.size(); i++) {
        const auto vids = get_edge_vids(edges[i]);
        int on_bbox = -1;
        for (int k = 0; k < 2; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == domain_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == domain_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == domain_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == domain_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        if (edges[i].switch_face(*this)) {
            log_and_throw_error("Boundary edge {} is not on the boundary!", vids);
        }

        const size_t eid = edges[i].eid(*this);
        m_edge_attribute[eid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });

    // add tag names
    for (size_t i = 0; i < tag_names.size(); ++i) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }

    //// rounding
    size_t cnt_round = 0;

    for (int i = 0; i < vert_capacity(); i++) {
        Tuple v = tuple_from_vertex(i);
        if (round(v)) {
            cnt_round++;
        }
    }

    if (cnt_round < vert_capacity()) {
        logger().info("Rounded {}/{}", cnt_round, vert_capacity());
    } else {
        logger().info("All rounded!", cnt_round, vert_capacity());
    }

    // Last, because it reads positions and the surface tags set above.
    if (m_params.sizing_field_from_features) {
        init_sizing_field();
    }
}


namespace {
/// Distance from `p` to the segment `a`-`b`.
double point_segment_distance(const Vector2d& p, const Vector2d& a, const Vector2d& b)
{
    const Vector2d ab = b - a;
    const double l2 = ab.squaredNorm();
    if (l2 <= 0.) return (p - a).norm();
    const double t = std::clamp((p - a).dot(ab) / l2, 0., 1.);
    return (p - (a + t * ab)).norm();
}
} // namespace

void TriWildMesh::init_sizing_field()
{
    // The 2D counterpart of TetWildMesh::init_sizing_field. Two differences from that one,
    // both deliberate:
    //
    //  * That function is dead code -- nothing calls it -- and its edge-distance half never
    //    contributed anyway: `min_ev_dist` starts at 100000 and is only assigned when
    //    `min_ev_dist < ev_dist`, so it takes a MAXIMUM and the value never moves off its
    //    sentinel. Only the vertex-distance term ever reached `min_dist`. Here the edge term
    //    is a real minimum, and it skips edges incident to the vertex -- whose distance is
    //    zero by construction, which is presumably what the inverted test was hiding.
    //
    //  * It is actually called, from init_mesh, so the field carries geometry before the
    //    first pass rather than staying at 1 until the optimizer stalls.
    //
    // What it measures is local feature size: how finely the curve network is resolved at a
    // vertex, and how close another branch of it passes. Without this the sizing scalar is
    // 1 everywhere, so the target length is a flat `l` that detailed regions can never reach
    // -- the envelope refuses every collapse long before the length gate is satisfied. See
    // OptimizerParameters::debug_edge_length_match for how to measure the result.
    const double min_refine_scalar =
        m_params.sizing_field_min_eps_ratio * m_params.l_min / m_params.l;
    // Same 1.8 as tetwild: the radius the refinement is graded out over.
    const double R = m_params.l * 1.8;

    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        if (!m_vertex_attribute[vid].m_is_on_surface) continue;
        const Vector2d& p = m_vertex_attribute[vid].m_posf;

        double min_dist = std::numeric_limits<double>::max();

        // How close a DIFFERENT branch of the network passes, vertex-wise. A neighbour joined
        // to v by a feature edge is the same branch, and its distance is only how finely the
        // input happens to be sampled there -- seeding the field with that makes the output
        // inherit the input's tessellation instead of its geometry. Measured on
        // challenging_triwild_162463: including them grew the mesh 12x (16174 -> 198064
        // elements) and left 52% of edges LONGER than target. Skip them.
        for (const Tuple& u : get_one_ring_edges_for_vertex(v)) {
            const size_t uid = u.vid(*this);
            if (uid == vid || !m_vertex_attribute[uid].m_is_on_surface) continue;
            if (m_edge_attribute[u.eid(*this)].m_is_surface_fs) continue; // same branch
            min_dist = std::min(min_dist, (p - m_vertex_attribute[uid].m_posf).norm());
        }

        // How close another branch of the network comes: distance to the feature edges of the
        // incident triangles that do not touch v.
        for (const Tuple& f : get_one_ring_tris_for_vertex(v)) {
            const std::array<Tuple, 3> fvs = oriented_tri_vertices(f);
            for (int i = 0; i < 3; ++i) {
                const Tuple e = fvs[i];
                if (!m_edge_attribute[e.eid(*this)].m_is_surface_fs) continue;
                const size_t a = e.vid(*this);
                const size_t b = e.switch_vertex(*this).vid(*this);
                if (a == vid || b == vid) continue; // distance zero, says nothing
                min_dist = std::min(
                    min_dist,
                    point_segment_distance(
                        p,
                        m_vertex_attribute[a].m_posf,
                        m_vertex_attribute[b].m_posf));
            }
        }

        if (min_dist == std::numeric_limits<double>::max()) continue;

        const double refine_scalar = std::max(min_dist / m_params.l, min_refine_scalar);
        double& own = m_vertex_attribute[vid].m_sizing_scalar;
        own = std::min(refine_scalar, own);

        // Grade it out to R so the interior does not jump from a refined boundary straight
        // back to the global target. Monotone: only ever lowers.
        std::vector<bool> visited(vert_capacity(), false);
        visited[vid] = true;
        std::queue<size_t> q;
        for (const Tuple& u : get_one_ring_edges_for_vertex(v)) {
            q.push(u.vid(*this));
        }
        while (!q.empty()) {
            const size_t uid = q.front();
            q.pop();
            if (visited[uid]) continue;
            visited[uid] = true;
            const double dist = (m_vertex_attribute[uid].m_posf - p).norm();
            if (dist > R) continue;
            m_vertex_attribute[uid].m_sizing_scalar = std::min(
                dist / R * (1 - refine_scalar) + refine_scalar,
                m_vertex_attribute[uid].m_sizing_scalar);
            for (const Tuple& n : get_one_ring_edges_for_vertex(uid)) {
                const size_t nid = n.vid(*this);
                if (!visited[nid]) q.push(nid);
            }
        }
    }
}

size_t TriWildMesh::refine_sizing_around_worst(double max_energy)
{
    const int n_rings = std::max(0, m_params.stuck_refine_rings);
    // Clamped above. Without the clamp a
    // single degenerate face (quality MAX_ENERGY) sets filter_energy astronomically high and
    // select_worst_cells then picks out only the degenerate faces -- refinement stops seeing
    // the merely-bad ones it exists to fix.
    const double filter_energy = std::min(std::max(max_energy / 100, m_params.stop_energy), 100.);

    // m_quality is the AMIPS2D energy itself here, so no cube root (unlike tetwild/simwild).
    const auto worst = utils::select_worst_cells(
        tri_capacity(),
        [this](size_t fid) { return tuple_from_tri(fid).is_valid(*this); },
        [this](size_t fid) { return m_face_attribute[fid].m_quality; },
        filter_energy,
        m_params.stuck_refine_num_worst);

    if (worst.empty()) {
        return 0;
    }

    // If force-split is on, record the LONGEST edge of each worst triangle.
    // split_all_edges force-splits exactly those edges (bypasses the length gate), so a
    // stuck sliver's long edge is split immediately -- WITHOUT changing the sizing field.
    m_force_split_edges.clear();
    if (m_params.stuck_refine_force_split) {
        for (const auto& [q, fid] : worst) {
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

    logger().info(
        "[stuck-refine] worst {} tris (maxE {:.4}), refined {} of {} region vertices, "
        "filter_energy {:.4}",
        worst.size(),
        worst.back().first,
        refined.size(),
        region.size(),
        filter_energy);
    return refined.size();
}


void TriWildMesh::write_msh_groups(std::string file, const bool write_envelope)
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

void TriWildMesh::compute_winding_numbers(
    const std::vector<MatrixXd>& Vs,
    const std::vector<MatrixXi>& Es)
{
    assert(Vs.size() == Es.size());

    const auto& faces = get_faces();
    MatrixXd C = MatrixXd::Zero(faces.size(), 2);
    for (size_t i = 0; i < faces.size(); i++) {
        const auto vs = oriented_tri_vids(faces[i]);
        for (size_t v : vs) {
            C.row(i) += m_vertex_attribute[v].m_posf;
        }
        C.row(i) /= 3;
    }

    m_tags_count = Vs.size();
    for (int64_t input_idx = 0; input_idx < static_cast<int64_t>(Vs.size()); ++input_idx) {
        // The inputs were already read (and their x,y extracted) when the initial mesh was
        // built; reuse them instead of parsing every file a second time.
        const MatrixXd& V = Vs[input_idx];
        MatrixXi E = Es[input_idx];
        assert(V.cols() == 2);
        assert(E.cols() == 2);

        Eigen::VectorXd W;
        utils::winding_number_2d(V, E, C, W, NUM_THREADS);

        if (W.maxCoeff() <= 0.5) {
            // all removed, let's invert.
            logger().info("Correcting winding number");
            for (auto i = 0; i < E.rows(); i++) {
                auto temp = E(i, 0);
                E(i, 0) = E(i, 1);
                E(i, 1) = temp;
            }
            utils::winding_number_2d(V, E, C, W, NUM_THREADS);
        }

        if (W.maxCoeff() <= 0.5) {
            logger().warn("No winding number above 0.5 for input {}", input_idx);
        }

        // store winding number in mesh
        for (int i = 0; i < faces.size(); ++i) {
            const size_t fid = faces[i].fid(*this);
            if (W(i) > 0.5) {
                m_face_attribute[fid].tags.insert(input_idx);
            }
        }
    }
}

void TriWildMesh::filter_with_input_winding_number()
{
    // A face is inside when it is inside at least one input, i.e. when it carries a tag.
    std::vector<size_t> rm_fids;
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (m_face_attribute[fid].tags.empty()) {
            rm_fids.emplace_back(fid);
        }
    }
    logger().info("Filter with input winding number: removing {} faces", rm_fids.size());
    remove_tris_by_ids(rm_fids);
}

void TriWildMesh::filter_with_flood_fill()
{
    // Find the flood-fill region that appears the most on the mesh boundary: that is the
    // one outside the input. Mirrors TetWildMesh::filter_with_flood_fill.
    std::map<int, size_t> id_counter;
    for (const Tuple& e : get_edges()) {
        if (e.switch_face(*this)) {
            continue; // interior edge
        }
        ++id_counter[m_face_attribute[e.fid(*this)].part_id];
    }

    if (id_counter.empty()) {
        logger().warn("Flood fill filter found no boundary edges. Nothing removed.");
        return;
    }
    if (id_counter.size() != 1) {
        logger().warn(
            "There were {} flood fill IDs found at the boundary. Using the one with most "
            "occurances.",
            id_counter.size());
    }

    int best_id = id_counter.begin()->first;
    size_t best_count = id_counter.begin()->second;
    for (const auto& [id, count] : id_counter) {
        if (count > best_count) {
            best_id = id;
            best_count = count;
        }
    }

    logger().info("Filter with flood fill ID {}", best_id);

    std::vector<size_t> rm_fids;
    for (const Tuple& t : get_faces()) {
        const size_t fid = t.fid(*this);
        if (m_face_attribute[fid].part_id == best_id) {
            rm_fids.emplace_back(fid);
        }
    }
    remove_tris_by_ids(rm_fids);
}

int TriWildMesh::flood_fill()
{
    int current_id = 0;
    const auto faces = get_faces();
    // Indexed by fid, not a std::map: this is a BFS over every face, so the per-lookup
    // red-black tree cost dominated the pass on large meshes.
    std::vector<char> visited(tri_capacity(), 0);
    const auto seen = [&visited](size_t fid) { return visited[fid] != 0; };

    for (const Tuple& t : faces) {
        size_t fid = t.fid(*this);
        if (seen(fid)) {
            continue;
        }

        visited[fid] = 1;
        m_face_attribute[fid].part_id = current_id;

        const Tuple f1 = t;
        const Tuple f2 = t.switch_edge(*this);
        const Tuple f3 = t.switch_vertex(*this).switch_edge(*this);

        std::queue<Tuple> bfs_queue;

        if (!m_edge_attribute[f1.eid(*this)].m_is_surface_fs) {
            auto oppo_t = f1.switch_face(*this);
            if (oppo_t.has_value()) {
                if (!seen((*oppo_t).fid(*this))) {
                    bfs_queue.push(*oppo_t);
                }
            }
        }
        if (!m_edge_attribute[f2.eid(*this)].m_is_surface_fs) {
            auto oppo_t = f2.switch_face(*this);
            if (oppo_t.has_value()) {
                if (!seen((*oppo_t).fid(*this))) {
                    bfs_queue.push(*oppo_t);
                }
            }
        }
        if (!m_edge_attribute[f3.eid(*this)].m_is_surface_fs) {
            auto oppo_t = f3.switch_face(*this);
            if (oppo_t.has_value()) {
                if (!seen((*oppo_t).fid(*this))) {
                    bfs_queue.push(*oppo_t);
                }
            }
        }

        while (!bfs_queue.empty()) {
            auto tmp = bfs_queue.front();
            bfs_queue.pop();
            size_t tmp_id = tmp.fid(*this);
            if (seen(tmp_id)) continue;

            visited[tmp_id] = 1;

            m_face_attribute[tmp_id].part_id = current_id;

            const Tuple f_tmp1 = tmp;
            const Tuple f_tmp2 = tmp.switch_edge(*this);
            const Tuple f_tmp3 = tmp.switch_vertex(*this).switch_edge(*this);

            if (!m_edge_attribute[f_tmp1.eid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp1.switch_face(*this);
                if (oppo_t.has_value()) {
                    if (!seen((*oppo_t).fid(*this))) {
                        bfs_queue.push(*oppo_t);
                    }
                }
            }
            if (!m_edge_attribute[f_tmp2.eid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp2.switch_face(*this);
                if (oppo_t.has_value()) {
                    if (!seen((*oppo_t).fid(*this))) {
                        bfs_queue.push(*oppo_t);
                    }
                }
            }
            if (!m_edge_attribute[f_tmp3.eid(*this)].m_is_surface_fs) {
                auto oppo_t = f_tmp3.switch_face(*this);
                if (oppo_t.has_value()) {
                    if (!seen((*oppo_t).fid(*this))) {
                        bfs_queue.push(*oppo_t);
                    }
                }
            }
        }

        current_id++;
    }
    return current_id;
}


std::pair<size_t, size_t> TriWildMesh::feature_retention(double* worst_ratio) const
{
    if (worst_ratio) {
        *worst_ratio = 0;
    }
    if (m_feature_points.empty()) {
        return {0, 0};
    }

    // One nearest-neighbour query per feature against a kd-tree of the live vertices.
    //
    // The first version of this walked every vertex for every feature, and called
    // get_vertices() -- which BUILDS AND RETURNS A FRESH VECTOR each time -- from inside that
    // loop. On 2D model 177574 (573k triangles, thousands of features) it was 100% of the
    // process's samples and burned the entire 1800 s sweep timeout on a mesh that had already
    // converged to 19.91 and finished rounding. It turned four converged models into
    // "failures". Diagnostics do not get to cost more than the thing they measure.
    std::vector<Vector3d> pts;
    pts.reserve(vert_capacity());
    for (const Tuple& v : get_vertices()) {
        const Vector2d& p = m_vertex_attribute[v.vid(*this)].m_posf;
        pts.emplace_back(p[0], p[1], 0);
    }
    if (pts.empty()) {
        return {0, m_feature_points.size()};
    }
    const KNN knn(pts);

    size_t kept = 0;
    for (const Vector2d& anchor : m_feature_points) {
        uint32_t idx = 0;
        double sq = 0;
        knn.nearest_neighbor(Vector3d(anchor[0], anchor[1], 0), idx, sq);
        // Retention is geometric on purpose: the invariant is "some vertex is still within
        // eps of this anchor", true whether or not that vertex carries the id. Ids drive the
        // per-collapse policy because they are local and cheap; counting them here would
        // under-report the legitimate merge the policy allows.
        if (sq <= m_feature_eps * m_feature_eps) {
            ++kept;
        } else if (worst_ratio) {
            // How far the nearest vertex actually is, in units of eps -- what separates an
            // anchor that drifted just past the ball from a curve that was deleted.
            *worst_ratio = std::max(*worst_ratio, std::sqrt(sq) / m_feature_eps);
        }
    }
    return {kept, m_feature_points.size()};
}

bool TriWildMesh::smoothing_position_is_allowed(const size_t vid, const Vector2d& p) const
{
    const size_t f = m_vertex_extra[vid].m_feature_id;
    if (f == NO_FEATURE || !m_tri_params.preserve_feature_points) {
        return true;
    }
    // A ball, not a pin. The vertex may move anywhere within eps of the feature it stands
    // for, which is exactly the guarantee the envelope gives everywhere else.
    return (p - m_feature_points[f]).squaredNorm() <= m_feature_eps * m_feature_eps;
}

bool TriWildMesh::collapse_breaks_feature(const size_t v1_id, const size_t v2_id) const
{
    if (!m_tri_params.preserve_feature_points) {
        return false;
    }
    // v1 disappears into v2, and v2 does not move. So the question is only whether the
    // feature v1 stood for is still represented afterwards, by v2.
    const size_t f1 = m_vertex_extra[v1_id].m_feature_id;
    if (f1 == NO_FEATURE) {
        return false;
    }
    // Distance is the whole test. An earlier version also refused outright when v2 already
    // stood for a DIFFERENT feature, on the theory that one of the two would be lost. That
    // is wrong, and expensively so: when two anchors sit within eps of each other the
    // survivor covers both, and forbidding the merge deadlocks the mesh. On the puzzle
    // integration model it left a degenerate triangle that the collapse pass exists to
    // remove, and the max energy sat at 1e50 -- MAX_ENERGY -- for all 82 iterations instead
    // of converging to 4.99 in 3.
    //
    // Two anchors further apart than eps are still protected, because then v2 -- sitting on
    // one of them -- is more than eps from the other and fails this same test.
    return (m_vertex_attribute[v2_id].m_posf - m_feature_points[f1]).squaredNorm() >
           m_feature_eps * m_feature_eps;
}


void TriWildMesh::write_vtu(const std::string& path) const
{
    // consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
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
    for (size_t j = 0; j < m_tags_count; ++j) {
        tags[j].setZero();
    }
    MatrixXd amips(tri_capacity(), 1);
    amips.setZero();
    MatrixXd flood_fill(tri_capacity(), 1);
    flood_fill.setZero();

    int index = 0;
    for (const Tuple& t : faces) {
        size_t fid = t.fid(*this);
        amips(index, 0) = m_face_attribute[fid].m_quality;
        for (size_t j = 0; j < m_tags_count; ++j) {
            tags[j](index, 0) = m_face_attribute[fid].tags.count(j) ? 1 : 0;
        }
        flood_fill(index, 0) = m_face_attribute[fid].part_id;

        const auto vs = oriented_tri_vids(t);
        for (size_t j = 0; j < 3; j++) {
            F(index, j) = (int)vs[j];
        }
        ++index;
    }

    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = 0; j < 2; ++j) {
            E(i, j) = (int)edges[i][j];
        }
    }

    for (const Tuple& v : get_vertices()) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
        v_sizing_field[vid] = m_vertex_attribute[vid].m_sizing_scalar;
    }

    paraviewo::VTUWriter writer;

    for (size_t j = 0; j < m_tags_count; ++j) {
        writer.add_cell_field(fmt::format("tag_{}", j), tags[j]);
    }
    writer.add_cell_field("quality", amips);
    writer.add_cell_field("flood_fill", flood_fill);
    writer.add_field("sizing_field", v_sizing_field);
    writer.write_mesh(path + ".vtu", V, F, paraviewo::CellType::Triangle);

    // surface
    {
        const auto surf_out_path = path + "_surf.vtu";
        paraviewo::VTUWriter surf_writer;
        surf_writer.add_field("sizing_field", v_sizing_field);

        logger().info("Write {}", surf_out_path);
        surf_writer.write_mesh(surf_out_path, V, E, paraviewo::CellType::Line);
    }
}


} // namespace wmtk::components::triwild
