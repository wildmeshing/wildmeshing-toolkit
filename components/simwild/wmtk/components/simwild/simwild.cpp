#include "simwild.hpp"
#include <wmtk/utils/DriverPrologue.hpp>


#include <memory>
#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <wmtk/utils/EnvelopeBudget.hpp>
#include <wmtk/utils/Preallocation.hpp>
#include "Parameters.h"
#include "SimWildMesh.h"
#include "SimWildMeshTri.hpp"
#include "expression_parser/Parser.hpp"
#include "read_image_msh.hpp"

#include "simwild_spec.hpp"

namespace wmtk::components::simwild {

template <typename MeshT>
void apply_operation(MeshT& mesh, const nlohmann::json& json_params)
{
    using ExprPtr = expression_parser::ExpressionPtr;

    static_assert(
        std::is_same_v<MeshT, SimWildMesh> || std::is_same_v<MeshT, tri::SimWildMeshTri>,
        "MeshT must be either SimWildMesh or SimWildMeshTri");

    if constexpr (std::is_same_v<MeshT, SimWildMesh>) {
        logger().info("Use 3D operation");
    } else if constexpr (std::is_same_v<MeshT, tri::SimWildMeshTri>) {
        logger().info("Use 2D operation");
    }

    const std::string operation = json_params["operation"];
    if (operation == "remeshing") {
        mesh.set_sizing_field(json_params["sizing_field"]);
        mesh.set_quality_field(json_params["quality_field"]);
        mesh.mesh_improvement(json_params["max_iterations"]); // <-- tetwild
    } else if (operation == "fill_holes_topo") {
        const std::vector<std::string> fill_holes_tags_names = json_params["fill_holes_tags"];
        std::vector<CellTag> fill_holes_tags;
        for (const auto& tag_set_names : fill_holes_tags_names) {
            // fill_holes_tags.push_back(mesh.string_set_to_cell_tag(tag_set_names));
            const auto expr = expression_parser::parse(tag_set_names, mesh.m_tag_name_to_id);
            logger().info("Parsed fill_holes_tags expression: {}", expr->to_string());
            if (!expr->contains_only_and()) {
                log_and_throw_error("Only AND operation is allowed in fill_holes_tags expression.");
            }
            fill_holes_tags.push_back(expr->tags_involved());
        }
        const double raw_threshold = json_params["fill_holes_threshold"];
        const double threshold =
            raw_threshold < 0 ? std::numeric_limits<double>::infinity() : raw_threshold;
        mesh.fill_holes_topo(fill_holes_tags, threshold);
    } else if (operation == "tight_seal_topo") {
        // tight_seal_tag_sets is a list of lists: [[t1,t2],[t3,...]]
        const std::vector<std::vector<std::string>> tag_sets_names =
            json_params["tight_seal_tag_sets"];
        std::vector<std::vector<CellTag>> tag_sets;
        for (const auto& tag_set_names_vec : tag_sets_names) {
            std::vector<CellTag> tag_set_vec;
            for (const auto& tag_set_names : tag_set_names_vec) {
                auto expr = expression_parser::parse(tag_set_names, mesh.m_tag_name_to_id);
                logger().info("Parsed tight_seal_tag_sets expression: {}", expr->to_string());
                if (!expr->contains_only_and()) {
                    log_and_throw_error(
                        "Only AND operation is allowed in tight_seal_tag_sets expression.");
                }
                auto tag_set = expr->tags_involved();
                tag_set_vec.push_back(tag_set);
            }
            tag_sets.push_back(tag_set_vec);
        }
        const double raw_threshold = json_params["tight_seal_threshold"];
        const double threshold =
            raw_threshold < 0 ? std::numeric_limits<double>::infinity() : raw_threshold;
        mesh.tight_seal_topo(tag_sets, threshold);
    } else if (operation == "keep_lcc") {
        const std::vector<std::string> lcc_tags_names = json_params["keep_lcc_tags"];
        std::vector<CellTag> lcc_tags;
        for (const auto& tag_set_names : lcc_tags_names) {
            auto expr = expression_parser::parse(tag_set_names, mesh.m_tag_name_to_id);
            logger().info("Parsed keep_lcc_tags expression: {}", expr->to_string());
            if (!expr->contains_only_and()) {
                log_and_throw_error("Only AND operation is allowed in keep_lcc_tags expression.");
            }
            lcc_tags.push_back(expr->tags_involved());
        }
        const size_t n_lcc = json_params["keep_lcc_num"];
        mesh.keep_largest_connected_component(lcc_tags, n_lcc);
    } else if (operation == "resolve_overlaps") {
        const std::vector<std::vector<std::string>> tags_names =
            json_params["resolve_overlaps_tags"];
        std::vector<std::array<ExprPtr, 2>> tags;
        for (const auto& tag_set_names : tags_names) {
            if (tag_set_names.size() != 2) {
                log_and_throw_error(
                    "Each resolve_overlaps_tags entry must contain exactly two "
                    "expressions.");
            }
            std::array<ExprPtr, 2> tags_expr;
            for (size_t i = 0; i < 2; ++i) {
                tags_expr[i] = expression_parser::parse(tag_set_names[i], mesh.m_tag_name_to_id);
            }
            tags.push_back(tags_expr);
        }
        mesh.resolve_overlaps(tags);
    } else if (operation == "replace_tags") {
        const std::vector<std::string> tags_in_names = json_params["replace_tags_in"];
        const std::vector<std::string> tag_out_names = json_params["replace_tags_out"];
        if (tags_in_names.size() != tag_out_names.size()) {
            log_and_throw_error(
                "replace_tags_in and replace_tags_out must have the same number of expressions.");
        }
        std::vector<CellTag> tags_in;
        for (const auto& tag_set_names : tags_in_names) {
            const auto expr_in = expression_parser::parse(tag_set_names, mesh.m_tag_name_to_id);
            logger().info("Parsed tags_in expression: {}", expr_in->to_string());
            if (!expr_in->contains_only_and()) {
                log_and_throw_error("Only AND operation is allowed in replace_tags_in expression.");
            }
            tags_in.push_back(expr_in->tags_involved());
        }
        std::vector<CellTag> tags_out;
        for (const auto& tag_out_name : tag_out_names) {
            const auto expr_out = expression_parser::parse(tag_out_name, mesh.m_tag_name_to_id);
            logger().info("Parsed replace_tags_out expression: {}", expr_out->to_string());
            if (!expr_out->contains_only_and()) {
                log_and_throw_error(
                    "Only AND operation is allowed in replace_tags_out expression.");
            }
            CellTag tag_out = mesh.string_set_to_cell_tag(expr_out->tag_names_involved());
            tags_out.push_back(tag_out);
        }
        mesh.replace_tags(tags_in, tags_out);
    } else if (operation == "tag_priority") {
        const std::vector<std::string> tag_priority_names = json_params["tag_priority"];
        std::vector<int64_t> tag_priority;
        for (const auto& tag_name : tag_priority_names) {
            CellTag tag = mesh.string_set_to_cell_tag({tag_name});
            tag_priority.push_back(*tag.begin());
        }
        mesh.tag_priority(tag_priority);
    } else {
        log_and_throw_error("Unknown image simulation operation");
    }
}

namespace {

/**
 * @brief Does a simplification actually run, and so have to be charged against eps?
 *
 * SimWild simplifies in one of two places, and which one applies turns on preserve_topology:
 *
 *  - BEFORE insertion, in read_mesh / read_curves (read_image_msh.cpp), when
 *    `!preserve_topology && !skip_simplify`. Simplification can change the topology of the
 *    input, so this route only runs where the caller has said that is acceptable.
 *  - AFTER insertion, SimWildMesh::simplify() from run_3D, when
 *    `preserve_topology && !skip_simplify && operation == "remeshing"`. It collapses at
 *    eps_simplify with the quality check off and then REBUILDS the envelope around the
 *    result, so it displaces the envelope geometry exactly as the pre-insertion route does.
 *
 * The 2D route has no post-insertion counterpart -- SimWildMeshTri has no simplify() -- so
 * under preserve_topology nothing simplifies there.
 */
bool simplification_runs(const Parameters& params, bool is_3d)
{
    if (params.skip_simplify) return false;
    if (!params.preserve_topology) return true; // pre-insertion, both dimensions
    return is_3d && params.operation == "remeshing"; // post-insertion, 3D only
}

/**
 * @brief The eps the optimizer's envelope should use.
 *
 * SimWild already builds its envelope around the SIMPLIFIED geometry -- read_mesh in 3D and
 * the curve pipeline in 2D both hand over the simplified input, and SimWildMesh::simplify()
 * later rebuilds it around the simplified surface. So the part tetwild and triwild gain from
 * optimize_envelope_around_simplified, SimWild has always had.
 *
 * What it does NOT do is charge the simplification against the budget: the envelope uses the
 * full eps around geometry that has already moved up to eps_simplify from the input, so a
 * vertex may end up eps_simplify + eps away from it rather than eps. With the flag on, spend
 * only the remainder, which makes the total deviation eps as advertised.
 *
 * Narrowing here rather than at each init_envelope call is deliberate: every later rebuild
 * reads m_envelope_eps, so doing it once at construction covers them all -- including the
 * rebuild at the end of SimWildMesh::simplify() -- and cannot double-subtract.
 */
double optimization_envelope_eps(const Parameters& params, bool is_3d)
{
    return wmtk::utils::optimization_envelope_eps(
        params.eps,
        params.eps_simplify,
        params.optimize_envelope_around_simplified,
        simplification_runs(params, is_3d));
}

} // namespace

void run_3D(const nlohmann::json& json_params, const InputData& input_data)
{
    Parameters params(json_params);
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());

    igl::Timer timer;
    timer.start();

    simwild::SimWildMesh mesh(
        params,
        optimization_envelope_eps(params, /*is_3d=*/true),
        params.NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh, json_params);
    // first init envelope
    if (input_data.V_envelope.size() != 0) {
        bool use_exact = !json_params["use_sample_envelope"];
        mesh.init_envelope(input_data.V_envelope, input_data.F_envelope, use_exact);
    }
    if (input_data.V_input_r.size() == 0) {
        logger().info("Use float input for TetWild");
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names);
    } else {
        logger().warn("Use RATIONAL input for TetWild");
        mesh.init_from_image(
            input_data.V_input_r,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names);
    }

    auto write_unique_vtu = [&params, &mesh]() {
        static size_t vtu_counter = 0;
        if (params.write_vtu) {
            mesh.write_vtu(fmt::format("{}_{}", params.output_path, vtu_counter++));
        }
    };

    mesh.consolidate_mesh();

    write_unique_vtu();


    if (params.operation == "remeshing" && params.preserve_topology && !params.skip_simplify) {
        // collapse for getting the right edge length
        logger().info("Simplify after insertion");
        mesh.simplify();
    }

    // /////////apply operation
    apply_operation(mesh, json_params);

    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    logger().info("total optimization time {}s", time);

    // The output is what the caller consumes, and write_msh emits m_posf unconditionally --
    // for an un-rounded vertex that double is not the vertex's real position, so the written
    // mesh can contain inverted tets. Report it rather than letting it pass silently.
    const bool all_rounded = mesh.all_rounded();
    if (!all_rounded) {
        logger().error("Not all vertices rounded!");
    }

    // A SimWild surface is exactly the interface between unlike tags, so every face between two
    // differently tagged tets must be marked as surface. The operations maintain this
    // incrementally; check it once at the end rather than trusting that they did.
    if (!mesh.check_interface_faces_tagged(/*verbose=*/true)) {
        logger().error("Interface faces are not consistently tagged as surface!");
    }

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["#t"] = mesh.get_tets().size();
        report["#v"] = mesh.get_vertices().size();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = params.NUM_THREADS;
        report["time"] = time;
        report["all_rounded"] = all_rounded;
        fout << std::setw(4) << report;
        fout.close();
    }

    logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(params.output_path + ".msh", params.write_envelope);
    write_unique_vtu();
    if (params.write_vtu) {
        mesh.write_surface(params.output_path + "_surface.obj");
    }
}

void run_2D(const nlohmann::json& json_params, const InputData& input_data)
{
    Parameters params(json_params);
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());

    igl::Timer timer;
    timer.start();

    simwild::tri::SimWildMeshTri mesh(
        params,
        optimization_envelope_eps(params, /*is_3d=*/false),
        params.NUM_THREADS);
    wmtk::set_preallocation_factor_from_json(mesh, json_params);
    // first init envelope
    if (input_data.V_envelope.size() != 0) {
        mesh.init_envelope(input_data.V_envelope, input_data.F_envelope);
    }
    if (input_data.V_input_r.size() == 0) {
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names);
    } else {
        logger().warn("Use RATIONAL input for 2D");
        mesh.init_from_image(
            input_data.V_input_r,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names);
    }

    auto write_unique_vtu = [&params, &mesh]() {
        static size_t vtu_counter = 0;
        if (params.write_vtu) {
            mesh.write_vtu(fmt::format("{}_{}", params.output_path, vtu_counter++));
        }
    };

    mesh.consolidate_mesh();
    write_unique_vtu();

    // /////////apply operation
    apply_operation(mesh, json_params);

    mesh.consolidate_mesh();
    double time = timer.getElapsedTime();
    logger().info("total optimization time {}s", time);

    // write_msh emits m_posf unconditionally -- for an un-rounded vertex that double is not
    // the vertex's real position, so the written mesh can contain inverted faces. Report it
    // rather than letting it pass silently. Same check run_3D makes.
    bool all_rounded = true;
    for (const auto& v : mesh.get_vertices()) {
        if (!mesh.m_vertex_attribute[v.vid(mesh)].m_is_rounded) {
            all_rounded = false;
            break;
        }
    }
    if (!all_rounded) {
        logger().error("Not all vertices rounded!");
    }

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    const std::string report_file = json_params["report"];
    if (!report_file.empty()) {
        std::ofstream fout(report_file);
        nlohmann::json report;
        report["#t"] = mesh.get_faces().size();
        report["#v"] = mesh.get_vertices().size();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = params.NUM_THREADS;
        report["time"] = time;
        report["all_rounded"] = all_rounded;
        fout << std::setw(4) << report;
        fout.close();
    }

    logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    mesh.write_msh(params.output_path + ".msh", params.write_envelope);
    write_unique_vtu();
}

void simwild(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;

    const std::filesystem::path root = utils::verify_and_setup_logger(
        json_params,
        jse::embed::wmtk_simwild_spec::simwild_spec::spec(),
        true);
    const std::vector<std::string> input_paths = utils::resolve_input_paths(json_params, root);

    // std::filesystem::path output_filename = resolve_path(root, json_params["output"]);
    std::filesystem::path output_filename = json_params["output"];

    if (output_filename.has_extension() && output_filename.extension() != ".msh") {
        output_filename.replace_extension(".msh");
        logger().warn(
            "Extension of provided output filename is ignored. Output will be {}",
            output_filename.string());
    }
    output_filename.replace_extension(""); // extension is added back later
    json_params["output"] = output_filename.string(); // propagate resolved path to run_2D/run_3D

    auto get_unique_vtu_name = [&output_filename]() -> std::string {
        static size_t vtu_counter = 0;
        return fmt::format("{}_{}", output_filename.string(), vtu_counter++);
    };

    igl::Timer timer;
    timer.start();

    // Three input routes, one per kind of input:
    //   .msh          -> an already-tagged mesh, 2D or 3D by content (no insertion)
    //   surface OBJ   -> EmbedSurface, the 3D VolumeRemesher arrangement
    //   curve OBJ     -> EmbedCurves,  the 2D VolumeRemesher arrangement
    InputData input_data;
    const std::string extension = std::filesystem::path(input_paths[0]).extension().string();
    const int dimension = resolve_input_dimension(input_paths, json_params);
    if (extension == ".msh") {
        input_data = read_image_msh(input_paths[0]);
    } else if (dimension == 2) {
        input_data = read_curves(input_paths, output_filename.string(), json_params);
    } else {
        input_data = read_mesh(input_paths, output_filename.string(), json_params);
    }

    if (input_data.T_input.cols() == 4) {
        run_3D(json_params, input_data);
    } else {
        run_2D(json_params, input_data);
    }
    double time = timer.getElapsedTime();
    logger().info("total runtime {}s", time);
    logger().info("======= finish =========");
}

} // namespace wmtk::components::simwild