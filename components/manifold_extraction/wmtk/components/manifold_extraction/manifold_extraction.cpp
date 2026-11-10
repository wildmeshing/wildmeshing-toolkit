#include "manifold_extraction.hpp"
#include <igl/Timer.h>
#include <jse/jse.h>
#include <wmtk/components/topological_offset/Parameters.h>
#include <manifold_extraction_spec.hpp>
#include <optional>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/components/topological_offset/output_conversion.hpp>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>
#include "ManExtractTetMesh.h"
#include "ManExtractTriMesh.h"
#include "Parameters.h"

using namespace wmtk::components::simwild;
using namespace wmtk::components;

namespace wmtk::components::manifold_extraction {


static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "manifold_extraction");
    using wmtk::utils::resolve_path;

    const auto envelope_selector = json_params.value("envelope", nlohmann::json());
    if (component_io.memory()) json_params.erase("envelope");
    // verify input and inject defaults
    {
        const auto spec =
            jse::embed::wmtk_manifold_extraction_spec::manifold_extraction_spec::spec();
        jse::JSE spec_engine;
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
    std::string input_path = component_io.path(root, json_params["input"]);

    // load params
    Parameters man_params(json_params);
    const std::string tag_selection_str = json_params["tag_selection"];
    std::filesystem::path output_filename = man_params.output_path;
    int NUM_THREADS = 0;

    // debugging output filename
    std::filesystem::path debug_outfilename = output_filename;
    debug_outfilename.replace_extension("");

    // input must be .msh
    if (!component_io.memory() && std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // The `envelope` input: a mesh of the call on the memory route, a mesh file on the file
    // route (relative to input_dir). In 3D a labelled reference is held to as the offset holds one
    // (below), and handed on with the faces the carving made (hand_on_envelope()); in 2D it is
    // carried, handed on unchanged, while the extraction builds its envelopes around the mesh it
    // receives (TopoOffsetTriMesh).
    const std::string envelope_path =
        component_io.memory() ? "" : json_params["envelope"].get<std::string>();
    std::optional<MeshData> envelope_file;
    if (!envelope_path.empty()) envelope_file = io::read_mesh(resolve_path(root, envelope_path));
    const MeshData* envelope = envelope_file ? &*envelope_file
                               : !envelope_selector.is_null()
                                   ? &component_io.mesh(envelope_selector)
                                   : nullptr;

    // read image / MSH
    InputData input_data = component_io.memory()
                               ? read_mesh_data(component_io.mesh(input_path), envelope)
                               : read_image_msh(input_path, envelope);
    // After the output: a labelled reference goes on, so that a chain keeps passing the first
    // component's input, with `carved` added (topological_offset::with_offset_surface()): in 3D
    // the faces between a carved cell and the rest that bound a region, each under the labels of
    // the regions it separates -- new geometry, its own reference, as the offset's band boundary
    // (topological_offset::offset_rows(), the band here the carved cells). Unchanged where nothing
    // was carved, and in 2D. On the file route as <output>_envelope.msh. Until 2026-11 the
    // reference went on unchanged in 3D too, and simwild refused the carved faces outside it.
    const auto hand_on_envelope = [&](const topological_offset::OffsetRows& carved =
                                          topological_offset::OffsetRows()) {
        auto& outputs = component_io.result.first;
        if (input_data.envelope_reference) {
            outputs["envelope"] = carved.F.rows() > 0 ? topological_offset::with_offset_surface(
                                                            *input_data.envelope_reference,
                                                            carved)
                                                      : *input_data.envelope_reference;
        }
        if (!component_io.memory() && envelope && outputs.count("envelope"))
            io::write_mesh(output_filename.string() + "_envelope.msh", outputs.at("envelope"));
    };
    man_params.init(
        input_data.V_input.colwise().minCoeff(),
        input_data.V_input.colwise().maxCoeff());
    topological_offset::Parameters offset_params = man_params.generate_offset_params();
    offset_params.output_enabled = !component_io.memory();
    offset_params.init(
        input_data.V_input.colwise().minCoeff(),
        input_data.V_input.colwise().maxCoeff());


    // split into 2d and 3d cases
    if (input_data.T_input.cols() == 3) { // input is 2d trimesh
        logger().info("Input mesh (2d trimesh): {}", input_path);

        // initialize mesh. The tagged edges and the input points ride on the edges' input sets and
        // the vertices' point labels (TopoOffsetTriMesh::init_from_image()), which the
        // construction carries, as it only splits; nothing holds them, as nothing moves (in 3D the
        // construction holds them, below). Before init_from_image(): the input points' labels.
        // Until 2026-11 both were refused.
        ManExtractTriMesh mesh(man_params, offset_params, NUM_THREADS);
        mesh.m_point_names = input_data.point_names;
        mesh.m_input_point_labels = input_data.input_point_labels;
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names,
            "", // the curve group's tag keeps its default name, "curve"
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.F_orientation,
            input_data.input_point_vids);

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.get_vertices().size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();

        // start timer
        igl::Timer timer;
        timer.start();

        // label non manifold components as offset input
        mesh.m_man_params.tag_selection =
            expression_parser::parse(tag_selection_str, mesh.m_tag_name_to_id);
        size_t num_nmv = mesh.label_non_manifold();
        logger().info("\tNonmanifold vertices: {}", num_nmv);
        if (mesh.m_man_params.debug_output) { // save input complex
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // A boundary that is already manifold leaves an empty input complex: nothing to offset,
        // and the output is the input.
        if (num_nmv == 0) {
            logger().info("\tThe selection's boundary is manifold: the mesh is left as it is");
        } else {
            // initialize BVH
            mesh.init_input_complex_bvh();
            mesh.consolidate_mesh();

            // create offset
            mesh.construct_offset(output_filename);
        }

        // stop timer
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // write curve mesh
        if (!component_io.memory() && mesh.m_man_params.write_surface) {
            logger().info("Extracting curve mesh...");
            mesh.write_curve(output_filename.string());
        }

        // report
        const std::string report_file = json_params["report"];
        if (component_io.memory() || !report_file.empty()) {
            nlohmann::json report;
            report["before #v"] = mesh.m_init_counts[0];
            report["before #e"] = mesh.m_init_counts[1];
            report["before #f"] = mesh.m_init_counts[2];
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            component_io.result.second = report;
            if (!component_io.memory()) {
                std::ofstream f_out(report_file);
                f_out << std::setw(4) << report;
            }
        }

        // save output: the mesh, whose tags give the extracted curves
        topological_offset::collect_memory_output(mesh, component_io);
        if (!component_io.memory())
            io::write_mesh(output_filename.string() + ".msh", component_io.result.first.at("mesh"));
        hand_on_envelope();
        if (mesh.m_man_params.save_vtu) {
            mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        }
        wmtk::logger().info("======= finish =========");
    } else { // input is 3d tetmesh
        logger().info("Input mesh (3d tetmesh): {}", input_path);

        // initialize mesh. A labelled reference is held to (below), as the offset holds one, not
        // the envelope surface group. The tagged faces, the feature curves and the input points
        // are loaded as the offset's 3D route loads them, and held by its construction
        // (TopoOffsetTetMesh::build_envelopes(), hold_curves_and_points()): the tagged faces in
        // their labels' envelopes, the curves in their tubes, the points anchored, each written
        // back by collect_memory_output(). Before init_from_image(): the input points' labels.
        // Until 2026-11 all three were refused.
        const bool reference = input_data.envelope_reference.has_value();
        ManExtractTetMesh mesh(man_params, offset_params, NUM_THREADS);
        mesh.m_point_names = input_data.point_names;
        mesh.m_input_point_labels = input_data.input_point_labels;
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            reference ? MatrixXd() : input_data.V_envelope,
            reference ? MatrixXi() : input_data.F_envelope,
            input_data.tag_names,
            "", // the sheet group's tag keeps its default name, "surface"
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.F_orientation,
            input_data.feature_edge_names,
            &input_data.features,
            &input_data.features_placed);
        // A reference, held to by the code the offset's 3D route holds one with (its start checks
        // in TopoOffsetTetMesh::build_envelopes(): every label of the held faces in it, and every
        // held face received inside its labels' envelopes, exactly, before anything is carved;
        // in hold_curves_and_points(), every feature edge received inside the tubes around its
        // feature curves, and every input point anchored on one of its points of that label).
        // The construction only splits, so nothing moves after the checks.
        if (reference) {
            const io::HeldRows held = io::held_rows(
                input_data.V_envelope,
                input_data.F_envelope,
                input_data.F_envelope_labels);
            mesh.m_has_reference = true;
            mesh.m_reference_V = held.V;
            mesh.m_reference_F = held.F;
            mesh.m_reference_labels = held.labels;
            mesh.m_reference_curves = input_data.reference_curves;
            std::vector<Eigen::Vector3d> V(size_t(held.V.rows()));
            for (size_t i = 0; i < V.size(); ++i) V[i] = held.V.row(Eigen::Index(i)).transpose();
            std::vector<std::array<size_t, 3>> F(size_t(held.F.rows()));
            for (size_t i = 0; i < F.size(); ++i) {
                for (int j = 0; j < 3; ++j) F[i][j] = size_t(held.F(Eigen::Index(i), j));
            }
            mesh.set_input_curves(V, F, held.labels, &input_data.reference_curves);
            // The input points' anchors on its points (TetOptimizerMesh::ReferencePoints).
            mesh.set_reference_points(
                input_data.reference_points,
                input_data.reference_point_labels);
        }

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.vertex_size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();
        mesh.m_init_counts[3] = mesh.tet_size();

        // start timer
        igl::Timer timer;
        timer.start();

        // label non manifold components as offset input
        mesh.m_man_params.tag_selection =
            expression_parser::parse(tag_selection_str, mesh.m_tag_name_to_id);
        auto [num_nme, num_nmv] = mesh.label_non_manifold();
        logger().info("\tNonmanifold edges: {}", num_nme);
        logger().info("\tNonmanifold vertices: {}", num_nmv);
        if (mesh.m_man_params.debug_output) { // save input complex
            mesh.write_vtu(output_filename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // A boundary that is already manifold leaves an empty input complex: nothing to offset
        // (the offset construction crashed on it), and the output is the input.
        if (num_nme == 0 && num_nmv == 0) {
            logger().info("\tThe selection's boundary is manifold: the mesh is left as it is");
        } else {
            // initialize BVH
            mesh.init_input_complex_bvh();
            mesh.consolidate_mesh();

            // create offset
            mesh.construct_offset(output_filename);
        }

        // stop timer
        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // write surface mesh
        if (!component_io.memory() && mesh.m_man_params.write_surface) {
            logger().info("Extracting surface mesh...");
            mesh.write_surface(output_filename.string());
        }

        // report
        const std::string report_file = json_params["report"];
        if (component_io.memory() || !report_file.empty()) {
            nlohmann::json report;
            report["before #v"] = mesh.m_init_counts[0];
            report["before #e"] = mesh.m_init_counts[1];
            report["before #f"] = mesh.m_init_counts[2];
            report["before #t"] = mesh.m_init_counts[3];
            report["after #v"] = mesh.get_vertices().size();
            report["after #e"] = mesh.get_edges().size();
            report["after #f"] = mesh.get_faces().size();
            report["after #t"] = mesh.get_tets().size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            component_io.result.second = report;
            if (!component_io.memory()) {
                std::ofstream f_out(report_file);
                f_out << std::setw(4) << report;
            }
        }

        // save output: the mesh, whose tags give the extracted surface and the input complex
        topological_offset::collect_memory_output(mesh, component_io);
        if (!component_io.memory())
            io::write_mesh(output_filename.string() + ".msh", component_io.result.first.at("mesh"));
        hand_on_envelope(topological_offset::offset_rows(mesh));
        if (mesh.m_man_params.save_vtu) {
            mesh.write_vtu(debug_outfilename.string() + fmt::format("_{}", mesh.m_vtu_counter++));
        }
        wmtk::logger().info("======= finish =========");
    }
}


void manifold_extraction(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult manifold_extraction(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return std::move(context.result);
}

} // namespace wmtk::components::manifold_extraction
