#include "tetwild.hpp"
#include <wmtk/utils/DriverPrologue.hpp>
#include <wmtk/utils/Preallocation.hpp>

#include "Parameters.h"
#include "TetWildMesh.h"

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <cstdlib>
#include <wmtk/io/read_triangle_mesh.hpp>

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
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

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.order2_envelope_ratio = json_params["order2_envelope_ratio"];
    params.stop_energy = json_params["stop_energy"];
    params.split_high_valence_threshold = json_params["split_high_valence_threshold"];
    params.num_smoothing_passes = json_params["num_smoothing_passes"];
    params.interleaved_smoothing = json_params["interleaved_smoothing"];
    params.interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];
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

    // Built around the ORIGINAL input, like the simplification one, but at the full tet eps.
    // A separate object because surf_mesh's is deliberately tighter now.
    auto tet_envelope = std::make_shared<wmtk::SampleEnvelope>(!use_sample_envelope);
    {
        std::vector<Eigen::Vector3d> env_V(verts.size());
        std::vector<Eigen::Vector3i> env_F(tris.size());
        for (size_t i = 0; i < verts.size(); ++i) env_V[i] = verts[i];
        for (size_t i = 0; i < tris.size(); ++i) {
            env_F[i] << (int)tris[i][0], (int)tris[i][1], (int)tris[i][2];
        }
        tet_envelope->init(env_V, env_F, tet_eps);
    }

    logger().info(
        "tetrahedralisation envelope: {} (eps {:.6})",
        tet_envelope->use_exact ? "EXACT" : "sampled",
        std::sqrt(tet_envelope->eps2));

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
    mesh.insertion_by_volumeremesher(
        vsimp,
        fsimp,
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

    logger().info("=== finished insertion");

    // generate new mesh
    tetwild::TetWildMesh mesh_new(params, tet_envelope, NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh_new, json_params);
    mesh_new.m_input_names = json_params["input_names"].get<std::vector<std::string>>();

    mesh_new.init_from_Volumeremesher(
        v_rational,
        facets,
        is_v_on_input,
        tets,
        tet_face_on_input_surface);

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
    }


    if (json_params["write_vtu"]) {
        mesh_new.save_paraview(output_path, false);
    }

    mesh_new.output_mesh(output_path + "_final.msh");

    igl::write_triangle_mesh(output_path + "_surface.obj", matV, matF);

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