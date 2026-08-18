#include "tetwild.hpp"
#include <wmtk/utils/DriverPrologue.hpp>
#include <wmtk/utils/Preallocation.hpp>

#include "Parameters.h"
#include "TetWildMesh.h"

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <cstdlib>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <functional>
#include <memory>
#include <vector>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/partition_utils.hpp>
#include <wmtk/utils/resolve_path.hpp>
#include "wmtk/utils/InsertTriangleUtils.hpp"
#include "wmtk/utils/Logger.hpp"

#include <igl/Timer.h>
#include <igl/boundary_facets.h>
#include <igl/euler_characteristic.h>
#include <igl/facet_components.h>
#include <igl/random_points_on_mesh.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_unreferenced.h>
#include <igl/write_triangle_mesh.h>
#include <spdlog/common.h>
#include <wmtk/utils/EnvelopeBudget.hpp>
#include <wmtk/utils/predicates.hpp>

#include <tetwild_spec.hpp>

namespace wmtk::components::tetwild {

/**
 * @brief Compute the euler characteristic for each connected component and return them in a sorted
 * vector.
 */
std::vector<int> compute_euler_characteristics(const MatrixXi& F)
{
    // get components
    VectorXi C;
    const int n_components = igl::facet_components(F, C);

    std::vector<int> n_faces(n_components, 0); // number of faces per component
    for (int i = 0; i < C.rows(); ++i) {
        n_faces[C[i]] += 1;
    }

    std::vector<MatrixXi> FF(n_components); // vector of components
    for (int i = 0; i < n_components; ++i) {
        FF[i].resize(n_faces[i], 3);
    }

    std::vector<int> face_counter(n_components, 0);
    for (int i = 0; i < C.rows(); ++i) {
        const int& cid = C[i];
        FF[cid].row(face_counter[cid]) = F.row(i);
        ++face_counter[cid];
    }
    // FF contains now individual components but with offsetted IDs
    for (int i = 0; i < n_components; ++i) {
        // Clean up vertex IDs for each component
        const MatrixXi& F_copy = FF[i];
        int n_vertices = F_copy.maxCoeff() + 1;
        const MatrixXd V = MatrixXd::Zero(n_vertices, 3);
        VectorXi I, J;
        MatrixXd NV;
        MatrixXi NF;
        igl::remove_unreferenced(V, F_copy, NV, NF, I, J);
        FF[i] = NF;
    }

    std::vector<int> euler_characteristics(n_components);
    for (int i = 0; i < n_components; ++i) {
        euler_characteristics[i] = igl::euler_characteristic(FF[i]);
    }

    std::sort(euler_characteristics.begin(), euler_characteristics.end());

    return euler_characteristics;
}

/**
 * @brief Euler characteristic (V - E) of each connected component of an edge network,
 * sorted. The 1D counterpart of compute_euler_characteristics above, and the same check
 * triwild runs on its curves: per component, 1 for a tree/open polyline, 0 for one loop,
 * and so on. Comparing the sorted per-component values of the input feature network against
 * the output's tagged edges catches exactly what the distance audits cannot -- components
 * merged, split, or lost outright.
 */
std::vector<int> curve_euler_characteristics(const std::vector<std::array<size_t, 2>>& edges)
{
    std::map<size_t, size_t> root; // union-find over the vertices that appear
    std::function<size_t(size_t)> find = [&](size_t v) -> size_t {
        while (root[v] != v) {
            root[v] = root[root[v]];
            v = root[v];
        }
        return v;
    };
    for (const auto& e : edges) {
        for (const size_t v : {e[0], e[1]}) {
            if (root.count(v) == 0) {
                root[v] = v;
            }
        }
        root[find(e[0])] = find(e[1]);
    }
    std::map<size_t, std::pair<int, int>> vc_ec; // component root -> (#V, #E)
    for (const auto& [v, r] : root) {
        (void)r;
        ++vc_ec[find(v)].first;
    }
    for (const auto& e : edges) {
        ++vc_ec[find(e[0])].second;
    }
    std::vector<int> ecs;
    ecs.reserve(vc_ec.size());
    for (const auto& [r, ve] : vc_ec) {
        (void)r;
        ecs.push_back(ve.first - ve.second);
    }
    std::sort(ecs.begin(), ecs.end());
    return ecs;
}

TetWildMesh::ExportStruct tetwild_with_export(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    const std::filesystem::path root = utils::verify_and_setup_logger(
        json_params,
        jse::embed::wmtk_tetwild_spec::tetwild_spec::spec(),
        false);
    const std::vector<std::string> input_paths = utils::resolve_input_paths(json_params, root);

    tetwild::Parameters params;

    std::string output_path = json_params["output"];
    bool skip_simplify = json_params["skip_simplify"];
    const bool simplify_use_link_condition = json_params["simplify_use_link_condition"];
    const bool simplify_use_sample_envelope = json_params["simplify_use_sample_envelope"];
    const double simplify_envelope_ratio = json_params["simplify_envelope_ratio"];
    bool use_sample_envelope = json_params["use_sample_envelope"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];
    std::string filter_option = json_params["filter"];

    // Roles for the 'hybrid' filter: one entry per input file, "volume" (tet-fill its
    // inside) or "surface" (keep as an embedded sheet). Curves and points already declare
    // their dimension by arriving through input_edges / input_points.
    std::vector<std::string> input_roles = json_params["input_roles"];
    std::vector<size_t> volume_input_ids;
    if (filter_option == "hybrid") {
        const size_t n_inputs = json_params["input"].size();
        if (input_roles.empty()) {
            input_roles.assign(n_inputs, "volume");
        }
        if (input_roles.size() != n_inputs) {
            log_and_throw_error(
                "input_roles has {} entries for {} inputs",
                input_roles.size(),
                n_inputs);
        }
        for (size_t k = 0; k < input_roles.size(); ++k) {
            if (input_roles[k] == "volume") {
                volume_input_ids.push_back(k);
            } else if (input_roles[k] != "surface") {
                log_and_throw_error(
                    "input_roles[{}] = '{}'; must be 'volume' or 'surface'",
                    k,
                    input_roles[k]);
            }
        }
        if (volume_input_ids.empty()) {
            logger().warn("filter='hybrid' with no volume-role input: no tets will be kept.");
        }
    } else if (!input_roles.empty()) {
        logger().warn("input_roles is only used by filter='hybrid'; ignoring it.");
    }

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.order2_envelope_ratio = json_params["order2_envelope_ratio"];
    params.feature_envelope_ratio = json_params["feature_envelope_ratio"];
    params.preserve_feature_points = json_params["preserve_feature_points"];
    params.allow_junction_cleanup = json_params["allow_junction_cleanup"];
    params.stop_energy = json_params["stop_energy"];
    params.split_high_valence_threshold = json_params["split_high_valence_threshold"];
    params.num_smoothing_passes = json_params["num_smoothing_passes"];
    params.interleaved_smoothing = json_params["interleaved_smoothing"];
    params.interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];

    // Coarsening pass.
    params.coarsen_pass = json_params["coarsen_pass"];
    params.coarsen_unbounded = json_params["coarsen_unbounded"];
    params.coarsen_local_smoothing_passes = json_params["coarsen_local_smoothing_passes"];
    params.coarsen_smooth_ring = json_params["coarsen_smooth_ring"];
    params.coarsen_global_smoothing_passes = json_params["coarsen_global_smoothing_passes"];
    params.coarsen_max_rounds = json_params["coarsen_max_rounds"];
    params.w_amips = json_params["w_amips"];
    params.smoothing_mode = json_params["smoothing_mode"];
    params.project_line_search_steps = json_params["project_line_search_steps"];
    params.project_line_search_nested_steps = json_params["project_line_search_nested_steps"];

    params.preserve_topology = json_params["preserve_topology"];

    params.debug_output = json_params["DEBUG_output"];
    params.perform_sanity_checks = json_params["DEBUG_sanity_checks"];

    params.allow_surface_swap = json_params["allow_surface_swap"];
    params.check_surface_topology = json_params["check_surface_topology"];

    // Stuck-element sizing refinement.
    params.stuck_refine_stall_eps = json_params["stuck_refine_stall_eps"];
    params.stuck_refine_cooldown = json_params["stuck_refine_cooldown"];
    params.stuck_refine_num_worst = json_params["stuck_refine_num_worst"];
    params.stuck_refine_rings = json_params["stuck_refine_rings"];
    params.stuck_refine_factor = json_params["stuck_refine_factor"];
    params.stuck_refine_force_split = json_params["stuck_refine_force_split"];
    params.stuck_refine_min_scalar = json_params["stuck_refine_min_scalar"];
    params.stuck_refine_gradation = json_params["stuck_refine_gradation"];

    // Skip good regions.
    params.skip_good_regions = json_params["skip_good_regions"];
    params.skip_good_regions_margin = json_params["skip_good_regions_margin"];

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    std::vector<size_t> modified_nonmanifold_v;
    // --- phase timing (TETWILD_PHASES) ---
    igl::Timer phase_timer;
    double t_load = 0, t_simplify = 0, t_optimize = 0, t_finalize = 0, t_output = 0;
    phase_timer.start();
    {
        // Merging vertices that are merely *close* is a topology change: it welds sheets
        // of the surface that pass near each other and so removes handles and tunnels.
        // Under preserve_topology only exactly coincident vertices may be merged, which is
        // a pure de-duplication of the file and leaves the topology alone. simwild makes
        // the same distinction (see read_image_msh.cpp).
        const double remove_duplicate_eps =
            params.preserve_topology ? 0.0 : double(json_params["remove_duplicate_eps"]);
        MatrixXd V;
        MatrixXi F;
        io::read_triangle_mesh(input_paths, V, F, remove_duplicate_eps);
        box_minmax.first = V.colwise().minCoeff();
        box_minmax.second = V.colwise().maxCoeff();
        VF_to_vectors(V, F, verts, tris);
    }
    // Feature inputs: an edge mesh to force into the tetrahedralization as tet edges, and
    // points to force in as tet vertices. Both join the bounding box BEFORE eps and the
    // background box derive from it -- they are input geometry, and a feature outside the
    // surface's box must still land inside the triangulated domain.
    std::vector<Vector3d> feature_edge_vertices;
    std::vector<std::array<size_t, 2>> feature_edges;
    std::vector<Vector3d> feature_points;
    {
        std::vector<std::string> edge_paths = json_params["input_edges"];
        std::vector<std::string> point_paths = json_params["input_points"];
        for (std::string& p : edge_paths) {
            p = resolve_path(root, p).string();
        }
        for (std::string& p : point_paths) {
            p = resolve_path(root, p).string();
        }
        for (size_t file = 0; file < edge_paths.size(); ++file) {
            const std::string& path = edge_paths[file];
            MatrixXd Ve;
            MatrixXi Ee;
            io::read_edge_mesh(path, Ve, Ee);
            logger()
                .info("Read feature edge mesh {}: #V = {}, #E = {}", path, Ve.rows(), Ee.rows());
            const size_t base = feature_edge_vertices.size();
            for (int i = 0; i < Ve.rows(); ++i) {
                feature_edge_vertices.emplace_back(Ve.row(i));
            }
            for (int i = 0; i < Ee.rows(); ++i) {
                feature_edges.push_back({{base + size_t(Ee(i, 0)), base + size_t(Ee(i, 1))}});
            }
            // A vertex of an edge file with no incident edge is a free point, same rule as
            // triwild's 2D inputs.
            std::vector<int> valence(Ve.rows(), 0);
            for (int i = 0; i < Ee.rows(); ++i) {
                ++valence[Ee(i, 0)];
                ++valence[Ee(i, 1)];
            }
            for (int v = 0; v < Ve.rows(); ++v) {
                if (valence[v] == 0) {
                    feature_points.emplace_back(Ve.row(v));
                }
            }
        }
        for (const std::string& path : point_paths) {
            MatrixXd Vp;
            MatrixXi Ep;
            io::read_edge_mesh(path, Vp, Ep);
            if (Ep.rows() > 0) {
                logger().warn(
                    "input_points file {} has {} edges; only its {} vertices are used",
                    path,
                    Ep.rows(),
                    Vp.rows());
            }
            logger().info("Read feature point file {}: #P = {}", path, Vp.rows());
            for (int i = 0; i < Vp.rows(); ++i) {
                feature_points.emplace_back(Vp.row(i));
            }
        }
        for (const Vector3d& p : feature_edge_vertices) {
            box_minmax.first = box_minmax.first.cwiseMin(p);
            box_minmax.second = box_minmax.second.cwiseMax(p);
        }
        for (const Vector3d& p : feature_points) {
            box_minmax.first = box_minmax.first.cwiseMin(p);
            box_minmax.second = box_minmax.second.cwiseMax(p);
        }
    }

    // The 0-dimensional features to anchor: every input point, and the feature network's
    // endpoints (valence 1) -- junctions (valence >= 3) too when allow_junction_cleanup is
    // off. Valence 2 is a curve interior: never anchored, the tube handles it.
    // Anchors are REGISTERED unconditionally; preserve_feature_points gates only the
    // collapse and smoothing policy. Same split as triwild, and it keeps the retention
    // audit honest when the guard is off -- it reports 0/N instead of nothing to check.
    std::vector<Vector3d> feature_anchors;
    {
        feature_anchors = feature_points;
        std::vector<int> valence(feature_edge_vertices.size(), 0);
        for (const auto& e : feature_edges) {
            ++valence[e[0]];
            ++valence[e[1]];
        }
        size_t n_endpoints = 0, n_junctions = 0;
        for (size_t v = 0; v < feature_edge_vertices.size(); ++v) {
            if (valence[v] == 0 || valence[v] == 2) {
                continue;
            }
            if (valence[v] == 1) {
                ++n_endpoints;
            } else {
                ++n_junctions;
                if (params.allow_junction_cleanup) {
                    continue;
                }
            }
            feature_anchors.push_back(feature_edge_vertices[v]);
        }
        if (n_endpoints + n_junctions > 0) {
            logger().info(
                "feature anchors: {} curve endpoints, {} junctions; anchoring {} points in "
                "total (junction cleanup {})",
                n_endpoints,
                n_junctions,
                feature_anchors.size(),
                params.allow_junction_cleanup ? "on" : "off");
        }
    }

    t_load = phase_timer.getElapsedTime();
    phase_timer.start(); // surface simplification begins

    // Informational input-topology report; gated behind DEBUG_euler because the
    // Euler-characteristic computation is expensive on meshes with many components.
    // It is also needed by the preserve_topology check at the end, which compares it
    // against the output -- without this the comparison is between two empty vectors and
    // silently passes whatever happened to the topology.
    const bool compute_euler = json_params["DEBUG_euler"] || params.preserve_topology;
    std::vector<int> ecs_input;
    if (compute_euler) {
        Eigen::MatrixXi F(tris.size(), 3);
        for (int i = 0; i < tris.size(); ++i) {
            F.row(i) = Eigen::Vector3i((int)tris[i][0], (int)tris[i][1], (int)tris[i][2]);
        }

        ecs_input = compute_euler_characteristics(F);
        logger().info("Input euler characteristic: {}", ecs_input);
    }

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = params.epsr * diag;
    shortest_edge_collapse::ShortestEdgeCollapse surf_mesh(
        verts,
        NUM_THREADS,
        !use_sample_envelope);
    surf_mesh.set_use_link_condition(simplify_use_link_condition);
    // The simplification and the tetrahedralisation used to share one envelope object at
    // the same eps, which leaves the optimizer no room: a simplification free to place a
    // vertex right at the limit hands the optimizer a mesh where almost every move is
    // already outside. Measured on crown.obj, a third of the simplified vertices sat
    // outside the envelope before the tet phase began, and ~94% of smoothing attempts were
    // then vetoed.
    //
    // Simplifying inside a fraction of the envelope reserves the rest as headroom.
    //
    // The tetrahedralisation gets the WHOLE eps, not half of it. This used to be
    // envelope_size / 2, which left the optimizer working inside half the tolerance its
    // result is judged against -- the Hausdorff check below uses params.eps == epsr * diag.
    // Half the caller's tolerance was simply unreachable, and the headroom argument above
    // does not justify it: reserving headroom for the optimizer is what
    // simplify_envelope_ratio does, one line down, so the two halvings compounded and the
    // simplification ran at eps/4.
    //
    // Measured on the 15 Thingi10K models that exhausted max_iterations = 80 at
    // stop_energy 10: 14 of 14 run converge, in 10-24 iterations (101954: 80 iterations
    // ending at a degenerate 4.64e16 and 69884 tets -> 13 iterations at 9.83 and 28198
    // tets). Every measured Hausdorff distance stays under eps, the largest at 92% of it,
    // so the contract the envelope exists to enforce is unchanged -- it is now the same
    // number the acceptance check uses.
    const double tet_eps = envelope_size;
    const double simplify_eps = tet_eps * simplify_envelope_ratio;
    logger().info(
        "envelope eps: simplification {:.6} ({:.0f}% of tetrahedralisation {:.6})",
        simplify_eps,
        100 * simplify_envelope_ratio,
        tet_eps);

    surf_mesh.create_mesh(verts.size(), tris, modified_nonmanifold_v, simplify_eps);
    assert(surf_mesh.check_mesh_connectivity_validity());

    if (skip_simplify == false) {
        logger().info("input {} simplification", input_paths);
        {
            size_t nm_e = 0;
            for (const auto& e : surf_mesh.get_edges()) {
                if (surf_mesh.edge_valence(e) > 2) {
                    ++nm_e;
                }
            }
            size_t nm_v = 0;
            for (const auto& t : surf_mesh.get_vertices()) {
                if (!surf_mesh.is_manifold_vertex(t.vid(surf_mesh))) {
                    ++nm_v;
                }
            }
            logger().info(
                "simplification input non-manifold: {} edges, {} vertices. Link condition {}.",
                nm_e,
                nm_v,
                simplify_use_link_condition ? "on" : "off");
        }
        // SampleEnvelope::init builds both the exact structure and the sampled BVH, and
        // is_outside picks between them per query, so switching predicates for the
        // simplification alone is a flag flip with nothing to rebuild. Restoring it
        // afterwards no longer matters to the tet phase -- that now gets its own
        // envelope object at the full eps, built below -- but surf_mesh outlives this
        // block and is written out, so leave it as the caller asked for.
        const bool saved_use_exact = surf_mesh.m_envelope.use_exact;
        if (simplify_use_sample_envelope) {
            logger().info("simplification uses the sampled envelope; tet phase unaffected");
            surf_mesh.m_envelope.use_exact = false;
        }

        if (!params.preserve_topology) {
            logger().warn(
                "TODO the simplification still preserves topology as the toolkit does not "
                "support non-manifold meshes, to fix");
        }
        surf_mesh.collapse_shortest(0);

        surf_mesh.m_envelope.use_exact = saved_use_exact;
        surf_mesh.consolidate_mesh();
        if (!surf_mesh.check_mesh_connectivity_validity()) {
            log_and_throw_error("Simplification produced an invalid mesh connectivity.");
        }
        logger().info(
            "simplified: #v = {}, #f = {}",
            surf_mesh.get_vertices().size(),
            surf_mesh.get_faces().size());
    } else {
        logger().info("skip simplification");
    }
    surf_mesh.write_triangle_mesh(output_path + "_simplified_input.obj");


    //// get the simplified input
    std::vector<Eigen::Vector3d> vsimp(surf_mesh.vert_capacity());
    std::vector<std::array<size_t, 3>> fsimp(surf_mesh.tri_capacity());
    for (auto& t : surf_mesh.get_vertices()) {
        auto i = t.vid(surf_mesh);
        vsimp[i] = surf_mesh.vertex_attrs[i].pos;
    }

    for (auto& t : surf_mesh.get_faces()) {
        auto i = t.fid(surf_mesh);
        auto vs = surf_mesh.oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            fsimp[i][j] = vs[j].vid(surf_mesh);
        }
    }
    t_simplify = phase_timer.getElapsedTime(); // surface simplification done


    // /////////
    // // Prepare Envelope and parameter for TetWild
    // /////////


    params.init(box_minmax.first, box_minmax.second);

    // The surface handed to the arrangement is taken as the simplification left it.
    //
    // This used to run wmtk::remove_duplicates(vsimp, fsimp, 1e-10 * diag_l) here, which
    // merged vertices wrongly. It is not a proximity merge: it forwards to
    // igl::remove_duplicate_vertices, which SNAPS coordinates to an epsilon grid and
    // takes unique rows, so whether two vertices merge depends on which side of a cell
    // boundary they fall on rather than on how far apart they are. Two vertices 2*eps
    // apart merge when they land in the same cell; two a tenth of eps apart survive when
    // a boundary runs between them.
    //
    // Nothing here needs it. collapse_shortest() is followed by consolidate_mesh(), so
    // the ids are already contiguous, vert_capacity()/tri_capacity() are the live counts,
    // and vsimp/fsimp are exactly sized with no stale slots. The simplification also
    // maintains a valid manifold connectivity -- checked immediately above by
    // check_mesh_connectivity_validity() -- so there are no coincident vertices or
    // duplicated faces for it to find.

    // Which surface is the optimizer's envelope built around, and at what radius?
    //
    // Default: the ORIGINAL input at the full tet eps, like the simplification one but wider.
    //
    // With optimize_envelope_around_simplified: the SIMPLIFIED surface at the REMAINING
    // tolerance, tet_eps - simplify_eps. The triangle-inequality budget is unchanged -- the
    // simplification is already within simplify_eps of the input, so anything within
    // (tet_eps - simplify_eps) of the simplification is within tet_eps of the input -- but the
    // geometry now starts at the CENTRE of the envelope it is judged against rather than
    // somewhere inside it. That matters because the envelope is a hard veto, not a penalty: a
    // surface handed over already close to the boundary has most of its moves refused, and
    // deliberately starving that headroom (simplify_envelope_ratio 0.95) was enough to turn a
    // converging run into a diverging one on 1368052.
    //
    // The narrowing is charged for the simplification's deviation, so it applies only when a
    // simplification actually ran. Under skip_simplify the "simplified" surface IS the input,
    // there is nothing to charge, and subtracting anyway would confine the optimizer to a
    // fraction of the tolerance its result is judged against -- for no gain, since the
    // geometry already starts centred in the full envelope.
    const bool env_around_simplified = json_params["optimize_envelope_around_simplified"];
    const bool charge_simplify = env_around_simplified && !skip_simplify;
    const double opt_eps = wmtk::utils::optimization_envelope_eps(
        tet_eps,
        simplify_eps,
        env_around_simplified,
        /*simplification_ran=*/!skip_simplify);

    auto tet_envelope = std::make_shared<wmtk::SampleEnvelope>(!use_sample_envelope);
    {
        std::vector<Eigen::Vector3d> env_V;
        std::vector<Eigen::Vector3i> env_F;
        if (env_around_simplified) {
            env_V.resize(vsimp.size());
            env_F.resize(fsimp.size());
            for (size_t i = 0; i < vsimp.size(); ++i) env_V[i] = vsimp[i];
            for (size_t i = 0; i < fsimp.size(); ++i) {
                env_F[i] << (int)fsimp[i][0], (int)fsimp[i][1], (int)fsimp[i][2];
            }
        } else {
            env_V.resize(verts.size());
            env_F.resize(tris.size());
            for (size_t i = 0; i < verts.size(); ++i) env_V[i] = verts[i];
            for (size_t i = 0; i < tris.size(); ++i) {
                env_F[i] << (int)tris[i][0], (int)tris[i][1], (int)tris[i][2];
            }
        }
        tet_envelope->init(env_V, env_F, opt_eps);
    }

    // params.eps is deliberately NOT narrowed to match.
    //
    // It looks like it should be -- the veto now uses opt_eps -- but params.eps also sets
    // l_min, the minimum edge length, and the smoothing energy scale. Halving it halves l_min,
    // which is a RESOLUTION change, not an envelope one: measured on 106838 it took the output
    // from 200k to 580k tets (2.9x) and on 116060 from 133k to 285k (2.1x), at unchanged final
    // quality and ~40% more wall time. That swamps the effect under test. Leaving params.eps
    // alone keeps this experiment to the single variable it is about: which surface the
    // envelope is built around, and how much room the geometry starts with inside it.

    logger().info(
        "tetrahedralisation envelope: {} (eps {:.6}) around the {}",
        tet_envelope->use_exact ? "EXACT" : "sampled",
        std::sqrt(tet_envelope->eps2),
        !env_around_simplified ? "input"
        : charge_simplify      ? "SIMPLIFIED surface"
                               : "input (skip_simplify: nothing to charge, full eps)");

    if (json_params["DEBUG_disable_envelope"]) {
        logger().warn(
            "DEBUG_disable_envelope: envelope checks are OFF for the tetrahedralisation. "
            "The output has no containment guarantee and is for diagnosis only.");
        tet_envelope->disabled = true;
    }

    tetwild::TetWildMesh mesh(params, tet_envelope, NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh, json_params);

    /////////////////////////////////////////////////////

    igl::Timer timer;
    timer.start();
    std::vector<size_t> partition_id(vsimp.size());
    wmtk::partition_vertex_morton(
        vsimp.size(),
        [&vsimp](auto i) { return vsimp[i]; },
        std::max(NUM_THREADS, 1),
        partition_id);


    // triangle insertion with volumeremesher on the simplified mesh
    // std::vector<vol_rem::bigrational> embedded_vertices;
    // std::vector<uint32_t> embedded_facets;
    // std::vector<uint32_t> embedded_cells;
    // std::vector<uint32_t> embedded_facets_on_input;
    std::vector<Vector3r> v_rational;
    std::vector<std::array<size_t, 3>> facets;
    std::vector<bool> is_v_on_input;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<bool> tet_face_on_input_surface;

    logger().info("simplified: #v = {}, #f = {}", vsimp.size(), fsimp.size());

    igl::Timer insertion_timer;
    insertion_timer.start();

    // Exact arrangement of the simplified surface against a Delaunay background
    // mesh; the remesher's own tets are used directly, so no Steiner points.
    utils::EmbedFeaturesResult features_out;
    mesh.insertion_by_volumeremesher(
        vsimp,
        fsimp,
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface,
        feature_edge_vertices,
        feature_edges,
        feature_points,
        &features_out);

    if (!feature_edges.empty() || !feature_points.empty()) {
        size_t tiling_edges = 0;
        for (const auto& t : features_out.edge_tiling) {
            tiling_edges += t.size();
        }
        size_t points_found = 0;
        for (const int64_t v : features_out.point_vertex) {
            points_found += v >= 0 ? 1 : 0;
        }
        logger().info(
            "features after insertion: {} edges tiled by {} tet edges, {}/{} points are "
            "output vertices",
            features_out.edge_tiling.size(),
            tiling_edges,
            points_found,
            features_out.point_vertex.size());
    }

    logger().info("=== finished insertion");

    // generate new mesh
    tetwild::TetWildMesh mesh_new(params, tet_envelope, NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh_new, json_params);
    mesh_new.m_input_names = json_params["input_names"].get<std::vector<std::string>>();

    const bool has_features = !feature_edges.empty() || !feature_points.empty();
    // The feature-curve tube: what the collapse guard checks tagged edges against, and what
    // curve-vertex smoothing will be pulled toward. Around the ORIGINAL input feature edges,
    // like every other envelope. Follows the surface envelope's choice of predicate.
    if (!feature_edges.empty()) {
        std::vector<Eigen::Vector2i> fe_env(feature_edges.size());
        for (size_t i = 0; i < feature_edges.size(); ++i) {
            fe_env[i] = Eigen::Vector2i(int(feature_edges[i][0]), int(feature_edges[i][1]));
        }
        mesh_new.m_feature_envelope = std::make_shared<SampleEnvelope>(!use_sample_envelope);
        mesh_new.m_feature_envelope->init(
            feature_edge_vertices,
            fe_env,
            params.epsr * params.diag_l * params.feature_envelope_ratio);
    }
    // The anchor ball has the tube's radius; input_points-only runs need it too.
    mesh_new.m_feature_eps = params.epsr * params.diag_l * params.feature_envelope_ratio;
    mesh_new.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface,
        has_features ? &features_out : nullptr,
        has_features && !feature_anchors.empty() ? &feature_anchors : nullptr);

    double insertion_time = insertion_timer.getElapsedTime();


    // mesh_new.output_faces(output_path + "after_insertion_surface.obj", [](auto& f) {
    //     return f.m_is_surface_fs;
    // });


    wmtk::logger().info("volume remesher insertion time: {:.4}s", insertion_time);

    // mesh_new.output_tetrahedralized_embedded_mesh(
    //     "tetrahedralized_embedded_mesh.txt",
    //     v_rational,
    //     facets,
    //     tets,
    //     tet_face_on_input_surface);

    // mesh_new.output_init_tetmesh("tetmesh_before_opt.txt");

    mesh_new.consolidate_mesh();

    // mesh_new.output_mesh(output_path + "after_insertion.msh");

    // mesh_new.output_faces("test_embed_output_bbox.obj", [](auto& f) {
    //     return f.m_is_bbox_fs != -1;
    // });

    // /////////mesh improvement
    phase_timer.start(); // optimization begins
    if (json_params["use_legacy_code"]) {
        logger().warn("Using legacy code for mesh improvement!");
        mesh_new.mesh_improvement_legacy(max_its);
    } else {
        mesh_new.mesh_improvement(max_its);
    }
    t_optimize = phase_timer.getElapsedTime(); // optimization done
    phase_timer.start(); // finalize (winding/flood/filter) begins

    // mesh_new.output_mesh(output_path + "after_optimization.msh");
    // mesh_new.output_faces(output_path + "after_optimization_surface.obj", [](auto& f) {
    //     return f.m_is_surface_fs;
    // });

    bool all_rounded = true;
    for (const auto& v : mesh_new.get_vertices()) {
        if (!mesh_new.m_vertex_attribute[v.vid(mesh_new)].m_is_rounded) {
            all_rounded = false;
            break;
        }
    }
    if (all_rounded) {
        wmtk::logger().info("All vertices are rounded");
    } else {
        wmtk::logger().error("Not all vertices rounded!");
    }

    // Winding-number / flood-fill annotations. They are required to filter the outside
    // region (filter != "none") and are otherwise written only as output annotation
    // fields. On large meshes the three winding-number evaluations dominate the finalize
    // phase, so skip_winding_number lets a caller that does not filter and does not need
    // the annotations opt out of them. When filtering is requested the flag is ignored
    // (the winding number is needed), with a warning.
    // Feature collections, taken BEFORE any filter deletes tets: explicit features are
    // user input, and a filter discarding the region they live in must not erase them from
    // the feature outputs and audits (they do disappear from the tet mesh itself). Same
    // rule in 2D. With filter='none' nothing is deleted and this equals the final state.
    std::vector<std::array<Vector3d, 3>> hybrid_sheet_tris; // hybrid filter only
    std::vector<std::array<Vector3d, 2>> hybrid_curve_segs;
    std::vector<Vector3d> hybrid_anchor_pts;
    std::pair<size_t, size_t> hybrid_retention{0, 0};
    double hybrid_retention_worst = 0;
    const bool features_present = !feature_edges.empty() || !feature_points.empty();
    if (features_present) {
        for (const auto& e : mesh_new.get_edges()) {
            if (!mesh_new.m_feature_edge_attribute[e.eid(mesh_new)].m_is_feature_edge) {
                continue;
            }
            hybrid_curve_segs.push_back(
                {{mesh_new.m_vertex_attribute[e.vid(mesh_new)].m_posf,
                  mesh_new.m_vertex_attribute[e.switch_vertex(mesh_new).vid(mesh_new)].m_posf}});
        }
        for (const auto& v : mesh_new.get_vertices()) {
            const size_t vid = v.vid(mesh_new);
            if (mesh_new.m_vertex_extra[vid].m_feature_point_id != TetWildMesh::NO_FEATURE) {
                hybrid_anchor_pts.push_back(mesh_new.m_vertex_attribute[vid].m_posf);
            }
        }
        hybrid_retention = mesh_new.feature_retention(&hybrid_retention_worst);
    }

    const bool skip_winding = json_params["skip_winding_number"] && filter_option == "none";
    if (json_params["skip_winding_number"] && filter_option != "none") {
        logger().warn(
            "skip_winding_number is set but filter='{}' requires the winding number; "
            "computing it anyway.",
            filter_option);
    }
    if (!skip_winding) {
        // Precompute the tets and their barycenters once; all winding-number passes
        // below reuse them (they used to each rebuild get_tets() + the barycenters).
        const auto finalize_tets = mesh_new.get_tets();
        const Eigen::MatrixXd finalize_barycenters = mesh_new.tet_barycenters(finalize_tets);

        // apply input winding number
        mesh_new.compute_winding_number(finalize_tets, finalize_barycenters, verts, tris);
        // apply tracked surface winding number
        mesh_new.compute_winding_number(finalize_tets, finalize_barycenters);
        // apply flood fill
        {
            int num_parts = mesh_new.flood_fill();
            logger().info("flood fill parts {}", num_parts);
        }
        // compute per-input winding number (reuse in-memory verts/tris to avoid re-read)
        mesh_new
            .compute_winding_numbers(input_paths, finalize_tets, finalize_barycenters, verts, tris);
    } else {
        logger().info("Skipping winding-number and flood-fill computation (skip_winding_number)");
    }

    // ////winding number
    if (filter_option == "input") {
        mesh_new.filter_with_input_surface_winding_number();
    } else if (filter_option == "tracked") {
        mesh_new.filter_with_tracked_surface_winding_number();
    } else if (filter_option == "flood") {
        // Flood fill (a serial BFS over all tets) is only needed to identify the
        // outside connected component for this filter. It used to run
        // unconditionally just to color the output part_id field, which is a very
        // expensive no-op on large multi-component meshes (e.g. ~1-2 min of
        // serial BFS on 765k tets / 6600 parts). Only run it when it is used.
        const int num_parts = mesh_new.flood_fill();
        logger().info("flood fill parts {}", num_parts);
        mesh_new.filter_with_flood_fill();
    } else if (filter_option == "hybrid") {
        // Collect everything that lives on scaffolding tets BEFORE deleting them: the
        // surface-role sheets (tracked faces none of whose incident tets stay), every
        // tagged feature curve, the anchor positions, and the pre-filter retention -- the
        // audit that means something is the one taken while the features still exist in
        // the tet mesh.
        std::vector<bool> keep(mesh_new.tet_capacity(), false);
        for (const auto& t : mesh_new.get_tets()) {
            const size_t tid = t.tid(mesh_new);
            const auto& wn = mesh_new.m_tet_attribute[tid].m_winding_number_per_input;
            for (const size_t k : volume_input_ids) {
                if (k < wn.size() && wn[k] > 0.5) {
                    keep[tid] = true;
                    break;
                }
            }
        }
        for (const auto& f : mesh_new.get_faces()) {
            if (!mesh_new.m_face_attribute[f.fid(mesh_new)].m_is_surface_fs) {
                continue;
            }
            bool any_kept = keep[f.tid(mesh_new)];
            const auto oppo = f.switch_tetrahedron(mesh_new);
            if (oppo.has_value()) {
                any_kept = any_kept || keep[(*oppo).tid(mesh_new)];
            }
            if (any_kept) {
                continue; // a volume boundary (or interior) face; the tets represent it
            }
            const size_t v1 = f.vid(mesh_new);
            const size_t v2 = f.switch_vertex(mesh_new).vid(mesh_new);
            const size_t v3 = f.switch_edge(mesh_new).switch_vertex(mesh_new).vid(mesh_new);
            hybrid_sheet_tris.push_back(
                {{mesh_new.m_vertex_attribute[v1].m_posf,
                  mesh_new.m_vertex_attribute[v2].m_posf,
                  mesh_new.m_vertex_attribute[v3].m_posf}});
        }
        mesh_new.filter_with_roles(volume_input_ids);
    } else if (filter_option != "none") {
        logger().error("Unknown filter option '{}'. No filtering performed.", filter_option);
    }
    mesh_new.consolidate_mesh();

    t_finalize = phase_timer.getElapsedTime(); // winding/flood/filter done
    phase_timer.start(); // output (surface extraction) begins

    double time = timer.getElapsedTime();
    wmtk::logger().info("total time {:.4}s", time);
    if (mesh_new.tet_size() == 0) {
        log_and_throw_error("Empty Output after Filter!");
    }

    Eigen::MatrixXd matV; // all vertices
    Eigen::MatrixXi matF; // surface faces
    {
        auto outface = std::vector<std::array<size_t, 3>>();
        for (const auto& f : mesh_new.get_faces()) {
            if (filter_option == "none") {
                // output tracked surface
                const size_t fid = f.fid(mesh_new);
                if (!mesh_new.m_face_attribute[fid].m_is_surface_fs) {
                    continue;
                }
            } else {
                auto res = mesh_new.switch_tetrahedron(f);
                if (res) {
                    continue;
                }
            }
            auto verts = mesh_new.get_face_vertices(f);
            std::array<size_t, 3> vids = {
                {verts[0].vid(mesh_new), verts[1].vid(mesh_new), verts[2].vid(mesh_new)}};
            auto vs = mesh_new.oriented_tet_vertices(f);
            for (int j = 0; j < 4; j++) {
                if (std::find(vids.begin(), vids.end(), vs[j].vid(mesh_new)) == vids.end()) {
                    auto res = wmtk::utils::predicates::orient3d(
                        mesh_new.m_vertex_attribute[vids[0]].m_posf,
                        mesh_new.m_vertex_attribute[vids[1]].m_posf,
                        mesh_new.m_vertex_attribute[vids[2]].m_posf,
                        mesh_new.m_vertex_attribute[vs[j].vid(mesh_new)].m_posf);
                    if (res == wmtk::utils::predicates::Orientation::NEGATIVE)
                        std::swap(vids[1], vids[2]);
                    break;
                }
            }
            outface.emplace_back(vids);
        }
        matV = Eigen::MatrixXd::Zero(mesh_new.vert_capacity(), 3);
        for (const auto& v : mesh_new.get_vertices()) {
            auto vid = v.vid(mesh_new);
            matV.row(vid) = mesh_new.m_vertex_attribute[vid].m_posf;
        }
        matF.resize(outface.size(), 3);
        for (auto i = 0; i < outface.size(); i++) {
            matF.row(i) << (int)outface[i][0], (int)outface[i][1], (int)outface[i][2];
        }

        wmtk::logger().info("#output faces = {}", outface.size());
    }
    t_output = phase_timer.getElapsedTime(); // output (surface extraction) done

    wmtk::logger().info(
        "TETWILD_PHASES threads {} load {:.6}s simplify {:.6}s insertion {:.6}s "
        "optimization {:.6}s finalize {:.6}s output {:.6}s",
        NUM_THREADS,
        t_load,
        t_simplify,
        insertion_time,
        t_optimize,
        t_finalize,
        t_output);

    // Surface deviation + Euler Characteristic
    //
    // Two one-sided distances, which are easy to confuse and mean very different things:
    //
    //   d(output -> input)  every point of the output surface is within eps of the input.
    //                       This is the envelope invariant the optimizer enforces, so it is
    //                       the one worth comparing against eps and the one throw_on_fail
    //                       reacts to.
    //
    //   d(input -> output)  every point of the input is within eps of the output, i.e.
    //                       whether the output still *covers* the input. Nothing in the
    //                       pipeline promises this. Simplification is allowed to remove
    //                       features and the arrangement can drop thin sheets, either of
    //                       which makes it large while containment is untouched. Reported
    //                       as a diagnostic, never gated on.
    //
    // This used to compute only the second one, compare it to eps, and warn that "Hausdorff
    // distance is larger than the envelope" -- which the envelope never promised. On a
    // Thingi10K sponge whose output legitimately holds 116k of the input's 282k faces that
    // read 53x eps while containment was at 0.34x.
    //
    // Both are measured against `verts`/`tris`, i.e. the input *after* the load-time weld at
    // remove_duplicate_eps, which is the same mesh the envelope itself was built from. That
    // matters: the weld tolerance is relative to the diagonal and can be the same order as
    // eps, so measuring against the file on disk instead would fold the welding displacement
    // into the number and overstate the drift.
    double hausdorff_distance = -1; // d(output -> input), the invariant
    double coverage_distance = -1; // d(input -> output), diagnostic only
    std::vector<int> ecs_output;
    std::vector<int> ecs_curves_in, ecs_curves_out;
    {
        Eigen::MatrixXd V(verts.size(), 3);
        for (int i = 0; i < verts.size(); ++i) {
            V.row(i) = verts[i];
        }
        Eigen::MatrixXi F(tris.size(), 3);
        for (int i = 0; i < tris.size(); ++i) {
            F.row(i) = Eigen::Vector3i((int)tris[i][0], (int)tris[i][1], (int)tris[i][2]);
        }

        if (json_params["DEBUG_hausdorff"]) {
            // Both directions are estimated by sampling, and a max over samples converges
            // slowly: the old count of 10000 under-reported by 21x on one of the models
            // above (0.027 against a true 0.58), which is enough to turn a bad result into
            // an apparent pass. This is opt-in diagnostic code, so pay for a stable answer.
            const int n_samples = 100000;

            SampleEnvelope env_in(true);
            env_in.init(V, F, params.eps);

            SampleEnvelope env_out(true);
            env_out.init(matV, matF, params.eps);

            // Max distance from points sampled on (Vs,Fs) to the surface behind `target`.
            const auto max_deviation = [&](const Eigen::MatrixXd& Vs,
                                           const Eigen::MatrixXi& Fs,
                                           const SampleEnvelope& target) {
                Eigen::MatrixXd B;
                Eigen::VectorX<int64_t> FI; // must be int64 for new MSVC compiler
                Eigen::MatrixXd X;
                igl::random_points_on_mesh(n_samples, Vs, Fs, B, FI, X);

                double worst = -1;
                for (int i = 0; i < X.rows(); ++i) {
                    const Eigen::Vector3d p = X.row(i);
                    Eigen::Vector3d r;
                    worst = std::max(worst, target.nearest_point(p, r));
                }
                return worst < 0 ? worst : std::sqrt(worst);
            };

            hausdorff_distance = max_deviation(matV, matF, env_in);
            coverage_distance = max_deviation(V, F, env_out);

            logger().info(
                "surface deviation: containment d(output->input) = {:.4} | envelope = {:.4}",
                hausdorff_distance,
                params.eps);
            if (hausdorff_distance > params.eps) {
                logger().warn(
                    "Output is outside the envelope; the containment invariant "
                    "was violated.");
            } else {
                logger().info("Output is inside the envelope (as expected).");
            }
            // No comparison against eps: the pipeline does not promise this direction.
            logger().info(
                "surface deviation: coverage d(input->output) = {:.4} (diagnostic; large "
                "means the output no longer covers part of the input)",
                coverage_distance);
        }

        // Feature-curve deviation, the 1D counterpart of the surface block above:
        // containment (tagged edges near the input curves -- the invariant the tube veto
        // enforces) and coverage (input curves near the tagged edges -- diagnostic, the
        // direction nothing enforces). Sampled point-to-segment on both sides; feature
        // networks are small next to surfaces, so brute force over the segments is fine.
        if (json_params["DEBUG_hausdorff"] && !feature_edges.empty()) {
            // Collected pre-filter: under a filter the mesh no longer carries them.
            const std::vector<std::array<Vector3d, 2>>& tagged = hybrid_curve_segs;
            const auto point_to_segments = [](const Vector3d& p,
                                              const std::vector<std::array<Vector3d, 2>>& segs) {
                double best = std::numeric_limits<double>::infinity();
                for (const auto& s : segs) {
                    const Vector3d d = s[1] - s[0];
                    const double dd = d.squaredNorm();
                    double t = dd > 0 ? (p - s[0]).dot(d) / dd : 0.0;
                    t = std::clamp(t, 0.0, 1.0);
                    best = std::min(best, (p - (s[0] + t * d)).squaredNorm());
                }
                return best;
            };
            std::vector<std::array<Vector3d, 2>> input_segs(feature_edges.size());
            for (size_t i = 0; i < feature_edges.size(); ++i) {
                input_segs[i] = {
                    {feature_edge_vertices[feature_edges[i][0]],
                     feature_edge_vertices[feature_edges[i][1]]}};
            }
            const auto sweep = [&](const std::vector<std::array<Vector3d, 2>>& from,
                                   const std::vector<std::array<Vector3d, 2>>& to) {
                double worst = -1;
                for (const auto& s : from) {
                    const int n = 32;
                    for (int i = 0; i <= n; ++i) {
                        const Vector3d p = s[0] + (double(i) / n) * (s[1] - s[0]);
                        worst = std::max(worst, point_to_segments(p, to));
                    }
                }
                return worst < 0 ? worst : std::sqrt(worst);
            };
            const double feat_containment = tagged.empty() ? -1 : sweep(tagged, input_segs);
            const double feat_coverage = tagged.empty() ? std::numeric_limits<double>::infinity()
                                                        : sweep(input_segs, tagged);
            const double feat_eps = params.epsr * params.diag_l * params.feature_envelope_ratio;
            logger().info(
                "feature deviation: {} tagged edges | containment d(tagged->input) = {:.4} | "
                "tube = {:.4}",
                tagged.size(),
                feat_containment,
                feat_eps);
            if (feat_containment > feat_eps) {
                logger().warn("Tagged feature edges left the tube; the veto was violated.");
            }
            logger().info(
                "feature deviation: coverage d(input->tagged) = {:.4} (diagnostic; large "
                "means part of an input curve is no longer represented)",
                feat_coverage);
        }

        // The Euler-characteristic check is a topology sanity check. It is expensive on
        // meshes with many components (tens of seconds), so it is off by default and only
        // computed when explicitly requested (DEBUG_euler) or when it is actually needed
        // for the preserve_topology throw check below.
        if (compute_euler) {
            logger().info("Input euler characteristic: {}", ecs_input);
            ecs_output = compute_euler_characteristics(matF);
            logger().info("Output euler characteristic: {}", ecs_output);
            if (ecs_input != ecs_output) {
                logger().warn("Output topology is not the same as the input topology!");
            }
        }

        // The same check for the feature curves: input network vs the tagged output edges.
        // Always computed when features exist (the networks are tiny next to the surface);
        // the preserve_topology throw is with the other throws below.
        if (!feature_edges.empty()) {
            std::vector<std::array<size_t, 2>> tagged_pairs;
            std::map<std::array<double, 3>, size_t> vid_of;
            for (const auto& seg : hybrid_curve_segs) {
                std::array<size_t, 2> pair;
                for (int j = 0; j < 2; ++j) {
                    const std::array<double, 3> key = {{seg[j][0], seg[j][1], seg[j][2]}};
                    pair[j] = vid_of.emplace(key, vid_of.size()).first->second;
                }
                tagged_pairs.push_back(pair);
            }
            ecs_curves_in = curve_euler_characteristics(feature_edges);
            ecs_curves_out = curve_euler_characteristics(tagged_pairs);
            logger().info(
                "Euler characteristic, feature curves: input {} | tagged output {}",
                ecs_curves_in,
                ecs_curves_out);
            if (ecs_curves_in != ecs_curves_out) {
                logger().warn("Feature-curve topology is not the same as the input's!");
            }
        }

        // The anchor invariant, measured on the finished mesh.
        if (json_params["DEBUG_feature_retention"]) {
            double worst_ratio = 0;
            // Measured pre-filter: the anchors' vertices may have been deleted with a
            // discarded region, but the features were preserved up to that point and
            // survive in the feature outputs.
            auto [kept, total] = hybrid_retention;
            worst_ratio = hybrid_retention_worst;
            if (total > 0) {
                if (kept == total) {
                    logger().info("feature points retained: {}/{}", kept, total);
                } else {
                    logger().warn(
                        "feature points retained: {}/{} -- the worst is {:.2f} x eps from "
                        "the nearest vertex",
                        kept,
                        total,
                        worst_ratio);
                }
            }
        }
    }

    /////////output
    auto [max_energy, avg_energy] = mesh_new.get_max_avg_energy();
    wmtk::logger().info("final max energy = {} avg = {}", max_energy, avg_energy);

    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["#t"] = mesh_new.tet_size();
        report["#v"] = mesh_new.vertex_size();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = NUM_THREADS;
        report["#iterations"] = mesh_new.m_iterations_used;
        // What the final coarsening pass bought. All zero when coarsen_pass is off, which is
        // also how an A/B tells the two arms apart.
        report["coarsen_accepted"] = mesh_new.m_coarsen_stats.accepted;
        report["coarsen_t_before"] = mesh_new.m_coarsen_stats.cells_before;
        report["coarsen_t_after"] = mesh_new.m_coarsen_stats.cells_after;
        report["coarsen_max_energy_before"] = mesh_new.m_coarsen_stats.max_energy_before;
        report["coarsen_max_energy_after"] = mesh_new.m_coarsen_stats.max_energy_after;
        report["time"] = time;
        // d(output -> input); the key kept its name so existing consumers keep working,
        // but it now holds the containment direction rather than the coverage one.
        report["hausdorff"] = hausdorff_distance;
        report["coverage"] = coverage_distance;
        report["all_rounded"] = all_rounded;
        report["input_euler_characteristic"] = ecs_input;
        report["output_euler_characteristic"] = ecs_output;
        report["insertion_and_preprocessing"] = insertion_time;
        fout << std::setw(4) << report;
        fout.close();
    }

    // check metrics
    if (json_params["throw_on_fail"]) {
        if (!all_rounded) {
            log_and_throw_error("Not all vertices rounded!");
        }
        if (max_energy > params.stop_energy) {
            log_and_throw_error("Max energy is too large.");
        }
        // Regression guard for the sizing-refinement stall: an input that normally converges
        // in a handful of iterations and suddenly needs the whole budget means the
        // stuck-refine trigger stopped firing when it should. That is a silent slowdown --
        // the mesh still reaches stop_energy, so no energy or envelope assertion catches it.
        const int max_expected_its = json_params["max_expected_iterations"];
        if (max_expected_its > 0 && mesh_new.m_iterations_used > max_expected_its) {
            log_and_throw_error(
                "Converged, but needed {} iterations against an expected maximum of {}.",
                mesh_new.m_iterations_used,
                max_expected_its);
        }
        // Containment only. Coverage is deliberately not checked here: a run that
        // legitimately simplifies away a feature would otherwise be reported as a failure.
        if (hausdorff_distance > params.eps) {
            log_and_throw_error(
                "Output is outside the envelope by {} (eps {}).",
                hausdorff_distance,
                params.eps);
        }
        if (params.preserve_topology && ecs_input != ecs_output) {
            log_and_throw_error("Input topology was not preserved.");
        }
        if (params.preserve_topology && ecs_curves_in != ecs_curves_out) {
            log_and_throw_error("Feature-curve topology was not preserved.");
        }
    }


    if (json_params["write_vtu"]) {
        mesh_new.save_paraview(output_path, false);
    }

    mesh_new.output_mesh(output_path + "_final.msh");

    if (filter_option == "hybrid") {
        mesh_new.output_hybrid_mesh(
            output_path + "_hybrid.msh",
            hybrid_sheet_tris,
            hybrid_curve_segs,
            hybrid_anchor_pts);
    }

    igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);

    // The tracked feature curves, as an edge mesh -- the 1D counterpart of _surface.obj.
    // Only written when features exist, so featureless runs are byte-identical.
    if (features_present) {
        // From the pre-filter collections: a filter must not erase user-supplied features
        // from the feature outputs (they do leave the tet mesh with their region).
        std::ofstream fout(output_path + "_features.obj");
        std::map<std::array<double, 3>, size_t> vid_of;
        const auto obj_vertex = [&](const Vector3d& p) {
            const std::array<double, 3> key = {{p[0], p[1], p[2]}};
            const auto [it, inserted] = vid_of.emplace(key, vid_of.size() + 1);
            if (inserted) {
                fout << "v " << p[0] << " " << p[1] << " " << p[2] << "\n";
            }
            return it->second;
        };
        std::vector<std::array<size_t, 2>> obj_edges;
        for (const auto& seg : hybrid_curve_segs) {
            obj_edges.push_back({{obj_vertex(seg[0]), obj_vertex(seg[1])}});
        }
        std::vector<size_t> obj_points;
        for (const Vector3d& p : hybrid_anchor_pts) {
            obj_points.push_back(obj_vertex(p));
        }
        for (const auto& e : obj_edges) {
            fout << "l " << e[0] << " " << e[1] << "\n";
        }
        for (const size_t pid : obj_points) {
            fout << "p " << pid << "\n";
        }
    }

    wmtk::logger().info("======= finish =========");

    return mesh_new.export_mesh_data();
}


void tetwild(nlohmann::json json_params)
{
    auto e = tetwild_with_export(json_params);
    logger().info("V ({},{})", e.V.rows(), e.V.cols());
    logger().info("T ({},{})", e.T.rows(), e.T.cols());
    logger().info("F ({},{})", e.F.rows(), e.F.cols());
}

} // namespace wmtk::components::tetwild