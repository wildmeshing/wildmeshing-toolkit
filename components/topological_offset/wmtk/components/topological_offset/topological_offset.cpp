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
        logger().info("relative_ball_threshold: {}", params.relative_ball_threshold);
        logger().info("===============================");
    }

    if (input_data.T_input.cols() == 3) { // input is a 2d tri mesh
        logger().info("Input mesh (2D trimesh): {}", input_path);

        // initialize mesh
        TopoOffsetTriMesh mesh(params, NUM_THREADS);
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

        // initialize BVH
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

        // Capture the region-boundary envelope while the region tags are still the input's own.
        // execute_offset() replaces the tags of every face the band grows through, which cuts
        // each region's boundary curve short at the band; an envelope built afterwards is a tube
        // around the truncated curve and pins the junction where region meets offset.
        if (mesh.m_offset_params.region_envelope_from_input) {
            mesh.init_region_boundary_envelope_from_input();
        }

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.execute_offset(output_filename);
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

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
        // vertices off the frozen bounding box, so a band clipped here stays clipped.
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

        // Unconditional -- 2D has no un-optimized output. `optimize` is not read here: after
        // conservative growth the band's boundary sits on background-mesh cell boundaries, so
        // its distance to the input complex is only accurate to the local cell size, and moving
        // it onto target_distance is entirely this call's job. Skipping it does not produce a
        // coarser offset, it produces one whose defining property is unmet. The parameter is
        // still honoured by the 3D branch below, which has its own distance-field insertion.
        mesh.optimize_offset(output_filename);

        // As in 3D: the check above ran on the offset as constructed, and optimization
        // re-triangulates it, so the property has to be re-established afterwards.
        if (check_manifoldness) {
            if (mesh.offset_is_manifold()) {
                logger().info("Offset region manifold check passed after optimization.");
            } else {
                logger().error("Offset region is NOT manifold after optimization!");
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
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            if (!mesh.optimization_metrics.empty()) {
                std::vector<double> max_dist_err, avg_dist_err, max_norm_dev, avg_norm_dev;
                for (const auto& m : mesh.optimization_metrics) {
                    max_dist_err.push_back(m[0]);
                    avg_dist_err.push_back(m[1]);
                    max_norm_dev.push_back(m[2]);
                    avg_norm_dev.push_back(m[3]);
                }
                report["optimization_metrics"]["max_dist_err"] = max_dist_err;
                report["optimization_metrics"]["avg_dist_err"] = avg_dist_err;
                report["optimization_metrics"]["max_norm_dev"] = max_norm_dev;
                report["optimization_metrics"]["avg_norm_dev"] = avg_norm_dev;

                std::vector<int> splits, collapses, swaps;
                for (const auto& c : mesh.op_counts) {
                    splits.push_back(c[0]);
                    collapses.push_back(c[1]);
                    swaps.push_back(c[2]);
                }
                report["op_counts"]["splits"] = splits;
                report["op_counts"]["collapses"] = collapses;
                report["op_counts"]["swaps"] = swaps;
                // Both criteria, matching optimize_offset()'s own test: MAX for distance, AVERAGE
                // for normal deviation (see the reasoning there). convergence_normal_deviation
                // <= 0 disables the angular half.
                const double nd_target = mesh.m_offset_params.convergence_normal_deviation;
                report["converged"] =
                    max_dist_err.back() <= mesh.m_offset_params.convergence_target &&
                    (nd_target <= 0. || avg_norm_dev.back() <= nd_target);
                report["convergence_target"] = mesh.m_offset_params.convergence_target;
                report["convergence_normal_deviation"] = nd_target;
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

        // initialize BVH
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
            // optimization cannot move vertices off the frozen bounding box, so a band clipped
            // here stays clipped.
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
        mesh.optimize_offset(output_filename);

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
                std::vector<double> max_dist_err, avg_dist_err, max_norm_dev, avg_norm_dev;
                for (const auto& m : mesh.optimization_metrics) {
                    max_dist_err.push_back(m[0]);
                    avg_dist_err.push_back(m[1]);
                    max_norm_dev.push_back(m[2]);
                    avg_norm_dev.push_back(m[3]);
                }
                report["optimization_metrics"]["max_dist_err"] = max_dist_err;
                report["optimization_metrics"]["avg_dist_err"] = avg_dist_err;
                report["optimization_metrics"]["max_norm_dev"] = max_norm_dev;
                report["optimization_metrics"]["avg_norm_dev"] = avg_norm_dev;

                std::vector<int> splits, collapses, swaps;
                for (const auto& c : mesh.op_counts) {
                    splits.push_back(c[0]);
                    collapses.push_back(c[1]);
                    swaps.push_back(c[2]);
                }
                report["op_counts"]["splits"] = splits;
                report["op_counts"]["collapses"] = collapses;
                report["op_counts"]["swaps"] = swaps;
                // Read from the run rather than recomputed from the arrays as the 2D block does:
                // optimize_offset() breaks out of the loop the moment it converges, so its own
                // verdict is the authority and cannot drift from the criterion it applied.
                report["converged"] = mesh.m_converged;
                report["convergence_target"] = mesh.m_offset_params.convergence_target;
                report["convergence_normal_deviation"] =
                    mesh.m_offset_params.convergence_normal_deviation;
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
