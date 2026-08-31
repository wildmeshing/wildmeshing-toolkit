#include "topological_offset.hpp"
#include <igl/Timer.h>
#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <topological_offset_spec.hpp>
#include <vector>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Preallocation.hpp>
#include <wmtk/utils/resolve_path.hpp>
#include "Parameters.h"
#include "TopoOffsetTetMesh.h"
#include "TopoOffsetTriMesh.h"

using namespace wmtk::components::simwild;


namespace wmtk::components::topological_offset {
void topological_offset(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    // verify input and inject defaults
    {
        const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
        jse::JSE spec_engine;
        spec_engine.strict = true;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }

    const std::filesystem::path root = json_params["input_dir"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    // load input file path
    std::string input_path = resolve_path(root, json_params["input"]).string();

    // load params
    Parameters params(json_params);
    const std::string offset_selection_str = json_params["offset_selection"];
    bool check_manifoldness = json_params["check_manifoldness"];

    std::filesystem::path output_filename = params.output_path;
    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later

    int NUM_THREADS = params.num_threads;

    // input must be .msh
    if (std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // read input data, init params from bounding box
    InputData input_data = read_image_msh(input_path);
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());
    if (params.debug_output) {
        logger().info("====== input parameters =======");
        logger().info("target_distance: {}", params.target_distance);
        logger().info("offset_dhat_factor: {}", params.offset_dhat_factor);
        logger().info("front_conv_rel: {}", params.front_conv_rel);
        logger().info("===============================");
    }

    if (input_data.T_input.cols() == 3) { // input is a 2d tri mesh
        logger().info("Input mesh (2D trimesh): {}", input_path);

        // THE TWO LENGTHS' ROLES IN 2D (Uday, 2026-08-31): front_conv_rel is THE accuracy --
        // the vertex bar and the chord-resolution threshold alike, tighter = more turns and a
        // finer front -- and offset_envelope_rel is only the leash on the operation passes.
        // Accuracy finer than the leash is unreachable: operations licensed to dent the front
        // by more than the sag threshold mint new refinable edges every turn (measured with
        // the roles swapped: leash 0.05 over threshold 0.025, annots 8 -> 19 turns, refinable
        // count bouncing 1 -> 5 at turn 18). Hence the hard requirement, not a warning.
        if (params.offset_envelope_rel > params.front_conv_rel) {
            log_and_throw_error(
                "offset_envelope_rel {} must be <= front_conv_rel {}: the operation corridor "
                "(the leash) cannot be wider than the convergence accuracy, or the operations "
                "keep denting the front past the resolution threshold and the loop chases the "
                "damage forever",
                params.offset_envelope_rel,
                params.front_conv_rel);
        }

        // initialize mesh
        TopoOffsetTriMesh mesh(params, NUM_THREADS);
        wmtk::set_preallocation_factor_from_json(mesh, json_params);
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names,
            input_data.envelope_name);

        // label input complex
        mesh.m_offset_params.offset_selection =
            expression_parser::parse(offset_selection_str, mesh.m_tag_name_to_id);
        mesh.label_input_complex();

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().warn("Empty input complex. Output mesh is same as the input.");
            mesh.write_msh_groups(output_filename.string());
            return;
        }

        // check for inversions in input mesh
        auto tris_before = mesh.get_faces();
        if (!mesh.invariants(tris_before)) {
            std::string bad_tris_str = "";
            for (const TriMesh::Tuple& t : tris_before) {
                std::vector<TriMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tris_str += (std::to_string(t.fid(mesh)) + " ");
                }
            }
            log_and_throw_error("Inverted input element. Aborting. Bad tri ids: {}", bad_tris_str);
        }
        tris_before.clear();

        // The BVH is needed by construction; the POTENTIAL is not -- execute_offset() and
        // everything it calls (simplicial_embedding, marching_tris, set_offset_tri_tags, the
        // split hooks) reference m_offset_potential zero times, now that the growth pass is
        // gone and marching places vertices at plain edge midpoints. So the potential is built
        // AFTER the offset exists, exactly as in 3D.
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // set initial counts
        mesh.m_init_counts[0] = mesh.get_vertices().size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();

        // output input complex and entire mesh as vtu
        if (mesh.m_offset_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // The per-tag containment envelopes were captured in init_from_image(), while the
        // region tags were still the input's own: execute_offset() replaces the tags of every
        // face the band grows through, which cuts each region's boundary curve short at the
        // band, and a tube built afterwards would pin the junction where region meets offset.
        // See TopoOffsetTriMesh::init_surfaces_and_boundaries().

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.execute_offset(output_filename);

        // Now that the offset exists, the potential can be built against it.
        mesh.init_offset_potential();
        mesh.write_phi_grid(output_filename.string(), mesh.m_offset_params.phi_grid_resolution);

        // inversion check
        auto tris = mesh.get_faces();
        bool noninverted = mesh.invariants(tris);
        if (!noninverted) {
            std::string bad_tris_str = "";
            for (const TriMesh::Tuple& t : tris) {
                std::vector<TriMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tris_str += (" " + std::to_string(t.fid(mesh)));
                }
            }
            // mesh.write_msh_groups(output_filename.string()); // DEBUG: write .msh anyway
            log_and_throw_error("INVERSION DURING OFFSET! bad tri ids: {}", bad_tris_str);
        }

        // Did conservative growth run out of room? Reported here, not inside the optimization,
        // because it is a property of the offset as constructed -- the optimization cannot move
        // vertices OFF the bounding box (they are smoothed along it, and collapse requires the
        // survivor to carry at least as many bbox faces), so a band clipped here stays clipped.
        mesh.warn_if_offset_reaches_domain_boundary();

        // offset region manifoldness check
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed.");
            } else {
                // mesh.write_msh_groups(output_filename.string()); // DEBUG: write .msh anyway
                log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
            }
        }

        // Unconditional, matching 3D: construction leaves the band boundary on background-cell
        // boundaries, so skipping the optimization does not yield a coarser offset, it yields
        // one whose defining property is simply unmet at an error set by the input mesh's
        // resolution rather than by anything the user asked for.
        //
        // ...unless WMTK_OFFSET_SKIP_OPTIMIZE=1, which is exactly a request for that
        // un-optimized output. The caveat above still holds and is then the POINT.
        if (TopoOffsetTetMesh::skip_optimization()) {
            logger().warn("[skip-optimize] optimize_offset() SKIPPED");
        } else {
            mesh.optimize_offset(output_filename);
        }

        // As in 3D: the check above ran on the offset as constructed, and optimization
        // re-triangulates it, so the property has to be re-established afterwards.
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed after optimization.");
            } else {
                logger().error("Offset region is NOT manifold after optimization!");
            }
        }

        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // report
        const std::string report_file = json_params["report"];
        if (!report_file.empty()) {
            std::ofstream f_out(report_file);
            nlohmann::json report;
            report["before #v"] = mesh.m_init_counts[0];
            report["before #e"] = mesh.m_init_counts[1];
            report["before #f"] = mesh.m_init_counts[2];
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            if (!mesh.optimization_metrics.empty()) {
                // THREE MEASURES, ONE CRITERION -- the same shape as the 3D block below, and the
                // full argument is written out there.
                //
                // CONVERGENCE IS max_grad AND NOTHING ELSE: the placement gradient
                // |grad (Phi - c)^2| over the offset boundary, at band vertices AND at interior
                // samples of band edges. max_grad_at_vertex / max_grad_in_edge split it by where
                // the max was measured, which is what says whether a failing run wants smoothing
                // or refinement.
                //
                // THE OTHER TWO SERIES ARE REPORTED FACTS, NOT CRITERIA. max_l2_dist_err is the
                // true Euclidean error; max_phi_residual is the same miss measured in Phi and
                // divided by the level-set slope to put it back in length units. Expect them to
                // disagree with each other and with convergence wherever several primitives are
                // active.
                std::vector<double> max_dist_err, avg_dist_err, max_residual, avg_residual,
                    max_grad, avg_grad, max_grad_at_vertex, max_grad_in_edge;
                for (const auto& m : mesh.optimization_metrics) {
                    max_dist_err.push_back(m[0]);
                    avg_dist_err.push_back(m[1]);
                    max_residual.push_back(m[2]);
                    avg_residual.push_back(m[3]);
                    max_grad.push_back(m[4]);
                    avg_grad.push_back(m[5]);
                    max_grad_at_vertex.push_back(m[6]);
                    max_grad_in_edge.push_back(m[7]);
                }
                report["optimization_metrics"]["max_l2_dist_err"] = max_dist_err;
                report["optimization_metrics"]["avg_l2_dist_err"] = avg_dist_err;
                report["optimization_metrics"]["max_phi_residual"] = max_residual;
                report["optimization_metrics"]["avg_phi_residual"] = avg_residual;
                report["optimization_metrics"]["max_grad"] = max_grad;
                report["optimization_metrics"]["avg_grad"] = avg_grad;
                report["optimization_metrics"]["max_grad_at_vertex"] = max_grad_at_vertex;
                report["optimization_metrics"]["max_grad_in_edge"] = max_grad_in_edge;

                std::vector<int> splits, collapses, swaps;
                for (const auto& c : mesh.op_counts) {
                    splits.push_back(c[0]);
                    collapses.push_back(c[1]);
                    swaps.push_back(c[2]);
                }
                report["op_counts"]["splits"] = splits;
                report["op_counts"]["collapses"] = collapses;
                report["op_counts"]["swaps"] = swaps;

                std::vector<int> born, recollapsed, recollapsed_same_pass;
                for (const auto& c : mesh.churn_counts) {
                    born.push_back(c[0]);
                    recollapsed.push_back(c[1]);
                    recollapsed_same_pass.push_back(c[2]);
                }
                report["churn"]["split_born"] = born;
                report["churn"]["recollapsed"] = recollapsed;
                report["churn"]["recollapsed_same_pass"] = recollapsed_same_pass;

                // Read from the run rather than recomputed from the arrays: optimize_offset()
                // breaks out of the loop the moment it converges, so its own verdict is the
                // authority and cannot drift from the criterion it applied.
                report["converged"] = mesh.m_converged;
                report["offset_gradient_tolerance"] = mesh.offset_gradient_tolerance();
                // The measured scale that tolerance is a fraction of; without it the
                // tolerance is an unreadable absolute number. 2D only -- 3D normalizes
                // analytically and has no measured reference to report.
                report["gradient_reference"] = mesh.gradient_reference();
                report["offset_residual_tolerance"] = mesh.offset_residual_tolerance();
                report["offset_level"] = mesh.m_offset_potential->target_level();
                report["offset_dhat"] = mesh.m_offset_potential->dhat();
            }
            f_out << std::setw(4) << report;
            f_out.close();
        }

        mesh.write_msh_groups(output_filename.string()); // write .msh with physical groups
        if (mesh.m_offset_params.save_vtu) { // write .vtu
            mesh.write_vtu(output_filename.string());
        }

        wmtk::logger().info("======= finish =========");
    } else { // input is a 3d tet mesh
        logger().info("Input mesh (3D tetmesh): {}", input_path);

        // initialize mesh
        TopoOffsetTetMesh mesh(params, NUM_THREADS);
        wmtk::set_preallocation_factor_from_json(mesh, json_params);
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names);

        // label input complex
        mesh.m_offset_params.offset_selection =
            expression_parser::parse(offset_selection_str, mesh.m_tag_name_to_id);
        mesh.label_input_complex();

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().warn("Empty input complex. Output mesh is same as the input.");
            mesh.write_msh_groups(output_filename.string());
            return;
        }

        // check for inversions in input mesh
        auto tets_before = mesh.get_tets();
        if (!mesh.invariants(tets_before)) {
            std::string bad_tets_str = "";
            for (const TetMesh::Tuple& t : tets_before) {
                std::vector<TetMesh::Tuple> tvec;
                tvec.push_back(t);
                if (!mesh.invariants(tvec)) {
                    bad_tets_str += (std::to_string(t.tid(mesh)) + " ");
                }
            }
            log_and_throw_error("Inverted input element. Aborting. Bad tet ids: {}", bad_tets_str);
        }
        tets_before.clear();

        // initial number of connected components
        size_t initial_num_comps = mesh.flood_fill();
        mesh.reset_connected_components();

        // The BVH is needed by construction; the POTENTIAL is not -- execute_offset() and
        // everything it calls (simplicial_embedding, marching_tets, set_offset_tet_tags, the
        // split hooks) reference m_offset_potential zero times, now that the growth pass is
        // gone and marching places vertices at plain edge midpoints. So the potential is built
        // AFTER the offset exists, which is what lets init_offset_potential() size dhat from
        // the offset it actually has to hold rather than from target_distance alone.
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.vertex_size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();
        mesh.m_init_counts[3] = mesh.tet_size();

        // output input complex and entire mesh as vtu
        if (mesh.m_offset_params.debug_output) {
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.execute_offset(output_filename);

        // Now that the offset exists, dhat can be sized to contain it. See
        // init_offset_potential(): dhat = max(offset_dhat_factor x delta, 2 x the furthest any
        // offset-surface vertex ended up from the input complex).
        mesh.init_offset_potential();
        mesh.write_phi_grid(output_filename.string(), mesh.m_offset_params.phi_grid_resolution);

        // checks
        {
            // inversion check
            auto tets = mesh.get_tets();
            bool noninverted = mesh.invariants(tets);
            if (!noninverted) {
                std::string bad_tets_str = "";
                for (const TetMesh::Tuple& t : tets) {
                    std::vector<TetMesh::Tuple> tvec;
                    tvec.push_back(t);
                    if (!mesh.invariants(tvec)) {
                        bad_tets_str += (" " + std::to_string(t.tid(mesh)));
                    }
                }
                // mesh.write_msh_groups(output_filename.string()); // DEBUG write .msh anyway
                log_and_throw_error("INVERSION DURING OFFSET! bad tet ids: {}", bad_tets_str);
            }

            // Did conservative growth run out of room? Reported here, not inside the
            // optimization, because it is a property of the offset as constructed -- the
            // optimization cannot move vertices OFF the bounding box, so a band
            // clipped here stays clipped.
            mesh.warn_if_offset_reaches_domain_boundary();

            // offset region manifoldness check
            if (check_manifoldness) {
                if (mesh.offset_is_manifold()) {
                    logger().info("Offset region manifold check passed.");
                } else {
                    // mesh.write_msh_groups(output_filename.string()); // DEBUG: write .msh anyway
                    log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
                }
            }

            // connected components check
            size_t final_num_comps = mesh.flood_fill();
            mesh.reset_connected_components();
            if (final_num_comps != initial_num_comps) {
                log_and_throw_error(
                    "# CONNECTED COMPONENTS MISMATCH: {} before, {} after",
                    initial_num_comps,
                    final_num_comps);
            } else {
                logger().info(
                    "connected components check passed. (# components={})",
                    initial_num_comps);
            }
        }

        // Unconditional -- 3D no longer has an un-optimized output, matching 2D. `optimize` is
        // not read here: now that the distance-field marching pass is gone, conservative growth
        // leaves the band boundary on background-cell boundaries, so skipping the optimization
        // does not yield a coarser offset, it yields one whose defining property is simply unmet
        // at an error set by the input mesh's resolution rather than by anything the user asked
        // for. See the corresponding note in .claude/CLAUDE.md.
        //
        // ...unless WMTK_OFFSET_SKIP_OPTIMIZE=1, which is exactly a request for that
        // un-optimized output. The caveat above still holds and is then the POINT, and it is a
        // strong one now that construction is midpoint marching with no growth pass: the offset
        // is wherever the input tetrahedralization put it. See
        // TopoOffsetTetMesh::skip_optimization().
        if (TopoOffsetTetMesh::skip_optimization()) {
            logger().warn("[skip-optimize] optimize_offset() SKIPPED");
        } else {
            mesh.optimize_offset(output_filename);
        }

        // The manifoldness check above ran on the offset as constructed. Optimization
        // then re-triangulates it -- splits, collapses and four kinds of swap all touch
        // the offset boundary -- so the property has to be re-established afterwards, not
        // assumed to survive.
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed after optimization.");
            } else {
                logger().error("Offset region is NOT manifold after optimization!");
            }
        }

        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // inversion check
        {
            auto tets = mesh.get_tets();
            bool noninverted = mesh.invariants(tets);
            if (!noninverted) {
                std::string bad_tets_str = "";
                for (const TetMesh::Tuple& t : tets) {
                    std::vector<TetMesh::Tuple> tvec;
                    tvec.push_back(t);
                    if (!mesh.invariants(tvec)) {
                        bad_tets_str += (" " + std::to_string(t.tid(mesh)));
                    }
                }
                // mesh.write_msh_groups(output_filename.string()); // DEBUG write .msh anyway
                log_and_throw_error("INVERSION DURING OFFSET! bad tet ids: {}", bad_tets_str);
            }
        }

        // report
        const std::string report_file = json_params["report"];
        if (!report_file.empty()) {
            std::ofstream f_out(report_file);
            nlohmann::json report;
            report["before #v"] = mesh.m_init_counts[0];
            report["before #e"] = mesh.m_init_counts[1];
            report["before #f"] = mesh.m_init_counts[2];
            report["before #t"] = mesh.m_init_counts[3];
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["after #t"] = mesh.tet_size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            if (!mesh.optimization_metrics.empty()) {
                // THREE MEASURES, ONE CRITERION.
                //
                // CONVERGENCE IS max_grad AND NOTHING ELSE -- the placement gradient
                // |grad (Phi - c)^2| over the offset surface, at band vertices AND at interior
                // samples of band faces, which is the same test for any potential.
                // max_grad_at_vertex / max_grad_in_face split it by where the max was measured,
                // which is what says whether a failing run wants smoothing or refinement.
                //
                // THE OTHER TWO SERIES ARE REPORTED FACTS, NOT CRITERIA. Nothing branches on
                // either; do not add a bar to them.
                //   max_l2_dist_err / avg_l2_dist_err -- the TRUE EUCLIDEAN error:
                //     | |p - nearest point on the input complex| - target_distance |, by BVH,
                //     per band vertex. Named l2 because that is exactly what it is, and because
                //     it is NOT what the run solves: the run minimises (Phi - c)^2, and Phi is
                //     only equal to the Euclidean distance where one primitive is active.
                //   max_phi_residual / avg_phi_residual -- the same miss measured in Phi, divided
                //     by the level-set slope to put it back in length units.
                //
                // Expect these two to DISAGREE with each other and with convergence wherever
                // several primitives are active -- a gap narrower than 2 x target_distance, a
                // reentrant corner. There the level set Phi = c may not exist at all, the
                // surface correctly settles at the nearest local minimum of (Phi - c)^2 instead,
                // and a converged run therefore reports a nonzero residual. That is the right
                // answer, not a failure.
                std::vector<double> max_dist_err, avg_dist_err, max_residual, avg_residual,
                    max_grad, avg_grad, max_grad_at_vertex, max_grad_in_face;
                for (const auto& m : mesh.optimization_metrics) {
                    max_dist_err.push_back(m[0]);
                    avg_dist_err.push_back(m[1]);
                    max_residual.push_back(m[2]);
                    avg_residual.push_back(m[3]);
                    max_grad.push_back(m[4]);
                    avg_grad.push_back(m[5]);
                    max_grad_at_vertex.push_back(m[6]);
                    max_grad_in_face.push_back(m[7]);
                }
                report["optimization_metrics"]["max_l2_dist_err"] = max_dist_err;
                report["optimization_metrics"]["avg_l2_dist_err"] = avg_dist_err;
                report["optimization_metrics"]["max_phi_residual"] = max_residual;
                report["optimization_metrics"]["avg_phi_residual"] = avg_residual;
                report["optimization_metrics"]["max_grad"] = max_grad;
                report["optimization_metrics"]["avg_grad"] = avg_grad;
                report["optimization_metrics"]["max_grad_at_vertex"] = max_grad_at_vertex;
                report["optimization_metrics"]["max_grad_in_face"] = max_grad_in_face;

                std::vector<int> splits, collapses, swaps;
                for (const auto& c : mesh.op_counts) {
                    splits.push_back(c[0]);
                    collapses.push_back(c[1]);
                    swaps.push_back(c[2]);
                }
                report["op_counts"]["splits"] = splits;
                report["op_counts"]["collapses"] = collapses;
                report["op_counts"]["swaps"] = swaps;

                std::vector<int> born, recollapsed, recollapsed_same_pass;
                for (const auto& c : mesh.churn_counts) {
                    born.push_back(c[0]);
                    recollapsed.push_back(c[1]);
                    recollapsed_same_pass.push_back(c[2]);
                }
                report["churn"]["split_born"] = born;
                report["churn"]["recollapsed"] = recollapsed;
                report["churn"]["recollapsed_same_pass"] = recollapsed_same_pass;

                // Read from the run rather than recomputed from the arrays as the 2D block does:
                // optimize_offset() breaks out of the loop the moment it converges, so its own
                // verdict is the authority and cannot drift from the criterion it applied.
                report["converged"] = mesh.m_converged;
                // Both bars: the one the verdict used, and the one the diagnostic residual is
                // normalized by. A consumer plotting either series needs the matching bar.
                report["offset_gradient_tolerance"] = mesh.offset_gradient_tolerance();
                report["offset_residual_tolerance"] = mesh.offset_residual_tolerance();
                report["offset_level"] = mesh.m_offset_potential->target_level();
                report["offset_dhat"] = mesh.m_offset_potential->dhat();
            }
            f_out << std::setw(4) << report;
            f_out.close();
        }

        // mesh.write_msh(output_filename.string()); // write .msh
        mesh.write_msh_groups(output_filename.string()); // write .msh with physical groups
        if (mesh.m_offset_params.save_vtu) { // write .vtu
            mesh.write_vtu(output_filename.string());
        }

        wmtk::logger().info("======= finish =========");
    }
}


} // namespace wmtk::components::topological_offset
