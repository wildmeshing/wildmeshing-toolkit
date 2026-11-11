#include "topological_offset.hpp"
#include <igl/Timer.h>
#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <algorithm>
#include <optional>
#include <topological_offset_spec.hpp>
#include <vector>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/components/topological_offset/output_conversion.hpp>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>
#include "Parameters.h"
#include "TopoOffsetTetMesh.h"
#include "TopoOffsetTriMesh.h"

using namespace wmtk::components::simwild;


namespace wmtk::components::topological_offset {

static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "topological_offset");
    using wmtk::utils::resolve_path;

    const auto envelope_selector = json_params.value("envelope", nlohmann::json());
    if (component_io.memory()) json_params.erase("envelope");
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
    std::string input_path = component_io.path(root, json_params["input"]);

    // load params
    Parameters params(json_params);
    params.output_enabled = !component_io.memory();
    // Both dimensions: the correspondence is recorded by the march and indexes vertices the
    // optimization would split, collapse and renumber.
    if (params.save_offset_correspondence && params.optimize_offset) {
        log_and_throw_error(
            "save_offset_correspondence requires optimize_offset false: corr_input_vid is "
            "recorded by the march and not carried through the optimization");
    }
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
    if (!component_io.memory() && std::filesystem::path(input_path).extension() != ".msh") {
        log_and_throw_error("Input must be a .msh file.");
    }

    // The `envelope` input: a mesh of the call on the memory route, a mesh file on the file
    // route (relative to input_dir). A labelled reference (InputData::envelope_reference) is held
    // to in 3D, as simwild holds one (below); one without labels is the input's envelope surface
    // group, as a .msh's.
    const std::string envelope_path =
        component_io.memory() ? "" : json_params["envelope"].get<std::string>();
    std::optional<MeshData> envelope_file;
    if (!envelope_path.empty()) envelope_file = io::read_mesh(resolve_path(root, envelope_path));
    const MeshData* envelope = envelope_file ? &*envelope_file
                               : !envelope_selector.is_null()
                                   ? &component_io.mesh(envelope_selector)
                                   : nullptr;

    // read input data, init params from bounding box
    InputData input_data = component_io.memory()
                               ? read_mesh_data(component_io.mesh(input_path), envelope)
                               : read_image_msh(input_path, envelope);
    // After the output: a labelled reference goes on with `offset`, the band's boundary, each face
    // under the labels of the regions it separates, added (with_offset_surface()), so that a chain
    // keeps passing the first component's input -- the band's boundary is new, so it is its own
    // reference, a region's whose cells the band took included; unchanged where there is none
    // (2D, an empty input complex). On the file route as <output>_envelope.msh.
    const auto hand_on_envelope = [&](const OffsetRows& offset = OffsetRows()) {
        auto& outputs = component_io.result.first;
        if (input_data.envelope_reference) {
            outputs["envelope"] = offset.F.rows() > 0
                                      ? with_offset_surface(*input_data.envelope_reference, offset)
                                      : *input_data.envelope_reference;
        }
        if (!component_io.memory() && envelope && outputs.count("envelope"))
            io::write_mesh(output_filename.string() + "_envelope.msh", outputs.at("envelope"));
    };
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());
    if (params.debug_output) {
        logger().info("====== input parameters =======");
        for (size_t k = 0; k < params.offsets.size(); ++k) {
            logger().info(
                "offsets[{}]: selection '{}', target_distance {}",
                k,
                params.offsets[k].selection,
                params.offsets[k].target_distance);
        }
        logger().info("offset_dhat_factor: {}", params.offset_dhat_factor);
        logger().info(
            "front_conv: {} ({} x target_distance)",
            params.front_conv,
            params.front_conv_rel);
        logger().info("stencil_order: {}", params.stencil_order);
        logger().info("===============================");
    }

    if (input_data.T_input.cols() == 3) { // input is a 2d tri mesh
        // The 3D branch below, one dimension down, step for step; where 2D differs the comment
        // says why.
        logger().info("Input mesh (2D trimesh): {}", input_path);
        // Several entries of offsets, and growth_mode: the several-offsets and growth step of the
        // 2D lock-step port, not yet here. 2D's march and loop are the one-entry ones
        // (label_input_complex() refuses several entries), and its loop has no growth stages: a
        // growth_mode would run as "none", silently.
        if (params.offsets.size() > 1) {
            log_and_throw_error(
                "2D: {} entries of offsets; the 2D offset takes one until the several-offsets and "
                "growth step of the 2D lock-step port",
                params.offsets.size());
        }
        if (params.growth_mode != "none") {
            log_and_throw_error(
                "2D: growth_mode \"{}\"; the 2D offset runs \"none\" until the several-offsets and "
                "growth step of the 2D lock-step port",
                params.growth_mode);
        }
        // As in 3D: the energy's band term reads the euclidean distance to the input's segments
        // (ExactBandIntegral2D), and the smooth field has no band term of its own yet.
        if (params.offset_field != "euclidean") {
            log_and_throw_error(
                "offset_field \"{}\" is not available in 2D: the energy's band term reads the "
                "euclidean distance to the input, and this field has no band term yet",
                params.offset_field);
        }
        // A labelled reference: 3D holds to it (below). The 2D offset's envelopes are per-tag
        // tubes around the mesh's own tracked edges (build_boundary_envelopes()), with no route
        // to build them around a reference's segments, and its segments given to
        // init_from_image() as the envelope would become the curve group, a curve to offset.
        // Refused until the envelope-passing step of the 2D port, rather than run without it.
        if (input_data.envelope_reference) {
            log_and_throw_error(
                "2D: the envelope given is a labelled reference, which the 2D offset does not hold "
                "to yet (the envelope-passing step of the 2D lock-step port); give no envelope, or "
                "one without labels");
        }
        TopoOffsetTriMesh mesh(params, NUM_THREADS);
        // Before init_from_image(): the input points' labels, as triwild and simwild set them,
        // which it makes selectable and which the anchors keep
        // (TopoOffsetTriMesh::hold_curves_and_points()). As in 3D.
        mesh.m_point_names = input_data.point_names;
        mesh.m_input_point_labels = input_data.input_point_labels;
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.tag_names,
            input_data.envelope_name,
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.F_orientation,
            input_data.input_point_vids);
        // 3D refuses a mesh that does not fill its box here (require_box_boundary()), its wall
        // never tracked (114b283134). The 2D offset still tracks its wall and holds it in the
        // tube of its region's tag (init_surfaces_and_boundaries()), so every 2D boundary edge is
        // tracked and the check would refuse every input: it comes with that port.

        // The per-tag containment envelopes must be captured in init_from_image(), while the
        // region tags are still the input's own: construct_offset() replaces the tags of every face
        // the band grows through, and a tube built afterwards would pin the junction where a
        // region meets the offset. See TopoOffsetTriMesh::init_surfaces_and_boundaries().

        // label input complex
        for (Parameters::Offset& o : mesh.m_offset_params.offsets)
            o.expression = expression_parser::parse(o.selection, mesh.m_tag_name_to_id);
        mesh.m_offset_params.parse_protected_tags(mesh.m_tag_name_to_id);
        mesh.label_input_complex();

        // A correspondence handed on from an earlier offset is carried, as in 3D: its offset
        // vertices keep their values (vertex i is V_input row i), and the march adds this run's.
        // Construction only splits, so every vertex keeps its id, and it throws rather than cut a
        // carried vertex's edge to its input vertex (split_edge_before()).
        if (params.save_offset_correspondence) {
            for (size_t v = 0; v < input_data.corr_input_vid.size(); ++v) {
                if (input_data.corr_input_vid[v] < 0) continue;
                mesh.m_vertex_extra[v].m_corr_input_vid = input_data.corr_input_vid[v];
                mesh.m_vertex_extra[v].m_corr_carried = true;
            }
        }

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().warn("Empty input complex. Output mesh is same as the input.");
            topological_offset::collect_memory_output(mesh, component_io);
            if (!component_io.memory())
                io::write_mesh(
                    output_filename.string() + ".msh",
                    component_io.result.first.at("mesh"));
            hand_on_envelope();
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

        // initial number of connected components
        size_t initial_num_comps = mesh.flood_fill();

        // The BVH is needed by construction; the potential is not -- construct_offset() and
        // everything it calls reference m_offset_potential zero times, since marching places
        // vertices at plain edge midpoints. So the potential is built after the offset exists,
        // and only when the optimization runs. As in 3D.
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // set initial counts
        mesh.m_init_counts[0] = mesh.get_vertices().size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();

        // output input complex and entire mesh as vtu
        if (mesh.m_offset_params.debug_output) {
            mesh.write_debug_frame("input");
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.construct_offset(output_filename);

        // checks
        {
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
                log_and_throw_error("INVERSION DURING OFFSET! bad tri ids: {}", bad_tris_str);
            }

            // Did conservative growth run out of room? Reported here, not inside the
            // optimization, because it is a property of the offset as constructed: the
            // optimization cannot move vertices off the bounding box, so a band clipped here
            // stays clipped.
            mesh.warn_if_offset_reaches_domain_boundary();

            // offset region manifoldness check
            if (check_manifoldness) {
                if (mesh.offset_is_manifold()) {
                    logger().info("Offset region manifold check passed.");
                } else {
                    log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
                }
            }

            // connected components check
            size_t final_num_comps = mesh.flood_fill();
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

        // optimize_offset (json, default true). Construction leaves the band boundary on
        // background-cell boundaries, so skipping the optimization does not yield a coarser
        // offset but one whose defining property is unmet at an error set by the input mesh's
        // resolution; false is for inspecting the constructed offset.
        if (mesh.m_offset_params.optimize_offset) {
            // The potential, against the constructed offset. Only the optimization reads it, so
            // a construction-only run builds none. As in 3D.
            mesh.init_offset_potential();
            mesh.write_phi_grid(output_filename.string(), mesh.m_offset_params.phi_grid_resolution);

            mesh.optimize_offset(output_filename);
            mesh.audit_input_curves();

            // As in 3D: the check above ran on the offset as constructed, and optimization
            // re-triangulates it, so the property has to be re-established afterwards.
            if (check_manifoldness) {
                if (mesh.offset_is_manifold()) {
                    logger().info("Offset region manifold check passed after optimization.");
                } else {
                    logger().error("Offset region is NOT manifold after optimization!");
                }
            }
        } else {
            logger().info(
                "optimize_offset false: the optimization is skipped, the offset as constructed "
                "is the result");
            if (mesh.m_offset_params.phi_grid_resolution >= 2) {
                logger().warn(
                    "phi_grid_resolution {} ignored: the offset potential is built only for the "
                    "optimization",
                    mesh.m_offset_params.phi_grid_resolution);
            }
        }
        // 3D waits here for its last debug frame, written in the background; 2D writes its
        // frames in the foreground.

        double time = timer.getElapsedTime();
        wmtk::logger().info("total time {}s", time);

        // inversion check, after the optimization as well. As in 3D.
        {
            auto tris_after = mesh.get_faces();
            if (!mesh.invariants(tris_after)) {
                std::string bad_tris_str = "";
                for (const TriMesh::Tuple& t : tris_after) {
                    std::vector<TriMesh::Tuple> tvec;
                    tvec.push_back(t);
                    if (!mesh.invariants(tvec)) {
                        bad_tris_str += (" " + std::to_string(t.fid(mesh)));
                    }
                }
                log_and_throw_error("INVERSION DURING OFFSET! bad tri ids: {}", bad_tris_str);
            }
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
            if (!mesh.optimization_metrics.empty()) {
                // Convergence is max_grad and nothing else: the placement gradient
                // |grad (Phi - c)^2| over the front, at band vertices and at interior samples of
                // band edges; the at_vertex / in_edge split says whether a failing run wants
                // smoothing or refinement. The two error series are reported facts, not criteria
                // -- the full argument is written out in the 3D block below.
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


                // Read from the run, never recomputed from the arrays: optimize_offset() breaks
                // out of the loop the moment it converges, so its own verdict cannot drift from
                // the criterion it applied.
                report["converged"] = mesh.m_converged;
                report["offset_level"] = mesh.m_offset_potential->target_level();
                report["offset_dhat"] = mesh.m_offset_potential->dhat();
            }
            component_io.result.second = report;
            if (!component_io.memory()) {
                std::ofstream f_out(report_file);
                f_out << std::setw(4) << report;
            }
        }

        topological_offset::collect_memory_output(mesh, component_io);
        if (!component_io.memory())
            io::write_mesh(output_filename.string() + ".msh", component_io.result.first.at("mesh"));
        // No reference in 2D (refused above), so nothing to hand on with the band's boundary:
        // 3D's offset_rows() has no 2D twin until the envelope-passing step.
        hand_on_envelope();
        if (mesh.m_offset_params.save_vtu) { // write .vtu: the result, as the .msh
            mesh.write_vtu(output_filename.string(), nullptr, /*result=*/true);
        }

        wmtk::logger().info("======= finish =========");
    } else { // input is a 3d tet mesh
        logger().info("Input mesh (3D tetmesh): {}", input_path);
        // The energy's band term reads the euclidean distance to the input's triangles; the
        // smooth field has no band term of its own yet.
        if (params.offset_field != "euclidean") {
            log_and_throw_error(
                "offset_field \"{}\" is not available in 3D: the energy's band term reads the "
                "euclidean distance to the input, and this field has no band term yet",
                params.offset_field);
        }
        // initialize mesh. A labelled reference is held to (below), not the envelope surface
        // group: its triangles are the first component's input, not a sheet of this mesh.
        const bool reference = input_data.envelope_reference.has_value();
        TopoOffsetTetMesh mesh(params, NUM_THREADS);
        // Before init_from_image(): the input points' labels, as simwild sets them, which it makes
        // tags of and which the anchors keep (TopoOffsetTetMesh::hold_curves_and_points()).
        mesh.m_point_names = input_data.point_names;
        mesh.m_input_point_labels = input_data.input_point_labels;
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            reference ? MatrixXd() : input_data.V_envelope,
            reference ? MatrixXi() : input_data.F_envelope,
            input_data.tag_names,
            input_data.envelope_name,
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.F_orientation,
            input_data.feature_edge_names,
            &input_data.features,
            &input_data.features_placed);
        // The domain is the mesh's box, its wall held by the box rule as in tetwild and simwild
        // and never tracked: a mesh that does not fill its box -- a filtered tetwild output, its
        // boundary its tracked faces -- is refused (require_box_boundary()).
        mesh.require_box_boundary("topological_offset");
        // A reference, the first component's labelled input, held to as simwild holds one
        // (simwild.cpp's run_3D): the envelopes around its triangles of this mesh's labels
        // (io::held_rows(); TopoOffsetTetMesh::build_envelopes() builds them and runs the two
        // start checks), the order-2 tubes and the curve-end anchors around its curves, never
        // around the mesh received, and the feature tubes around its feature curves
        // (init_reference_feature_tubes(), from hold_curves_and_points()).
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

        // label input complex
        for (Parameters::Offset& o : mesh.m_offset_params.offsets)
            o.expression = expression_parser::parse(o.selection, mesh.m_tag_name_to_id);
        mesh.m_offset_params.parse_protected_tags(mesh.m_tag_name_to_id);
        mesh.label_input_complex();

        // A correspondence handed on from an earlier offset is carried: its offset vertices keep
        // their values (vertex i is V_input row i), and the march adds this run's. Construction
        // only splits, so every vertex keeps its id, and it throws rather than cut a carried
        // vertex's edge to its input vertex (split_edge_before()).
        if (params.save_offset_correspondence) {
            for (size_t v = 0; v < input_data.corr_input_vid.size(); ++v) {
                if (input_data.corr_input_vid[v] < 0) continue;
                mesh.m_vertex_extra[v].m_corr_input_vid = input_data.corr_input_vid[v];
                mesh.m_vertex_extra[v].m_corr_carried = true;
            }
        }

        // check empty input
        if (mesh.empty_input_complex()) {
            logger().warn("Empty input complex. Output mesh is same as the input.");
            topological_offset::collect_memory_output(mesh, component_io);
            if (!component_io.memory())
                io::write_mesh(
                    output_filename.string() + ".msh",
                    component_io.result.first.at("mesh"));
            hand_on_envelope();
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

        // The BVH is needed by construction; the potential is not -- construct_offset() and
        // everything it calls reference m_offset_potential zero times, since marching places
        // vertices at plain edge midpoints. So the potential is built after the offset exists,
        // and only when the optimization runs.
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();

        // record counts (mostly debugging, this is probably really slow)
        mesh.m_init_counts[0] = mesh.vertex_size();
        mesh.m_init_counts[1] = mesh.get_edges().size();
        mesh.m_init_counts[2] = mesh.get_faces().size();
        mesh.m_init_counts[3] = mesh.tet_size();

        // output input complex and entire mesh as vtu (DEBUG_remote_output: the complex alone)
        if (mesh.m_offset_params.debug_output) {
            if (mesh.stage_frames()) mesh.write_debug_frame("input");
            mesh.write_input_complex(output_filename.string() + "_input_complex");
        }

        // execute offset
        igl::Timer timer;
        timer.start();
        mesh.construct_offset(output_filename);

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
                log_and_throw_error("INVERSION DURING OFFSET! bad tet ids: {}", bad_tets_str);
            }

            // Did conservative growth run out of room? Reported here, not inside the
            // optimization, because it is a property of the offset as constructed: the
            // optimization cannot move vertices off the bounding box, so a band clipped here
            // stays clipped.
            mesh.warn_if_offset_reaches_domain_boundary();

            // offset region manifoldness check
            if (check_manifoldness) {
                if (mesh.offset_is_manifold()) {
                    logger().info("Offset region manifold check passed.");
                } else {
                    log_and_throw_error("OFFSET REGION IS NOT MANIFOLD");
                }
            }

            // connected components check
            size_t final_num_comps = mesh.flood_fill();
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

        // optimize_offset (json, default true), as in 2D. Conservative growth leaves the band
        // boundary on background-cell boundaries, so skipping the optimization does not yield a
        // coarser offset but one whose defining property is unmet at an error set by the input
        // mesh's resolution; false is for inspecting the constructed offset.
        if (mesh.m_offset_params.optimize_offset) {
            // The potential, against the constructed offset. Only the optimization reads it, so
            // a construction-only run builds none.
            mesh.init_offset_potential();
            mesh.write_phi_grid(output_filename.string(), mesh.m_offset_params.phi_grid_resolution);

            mesh.optimize_offset(output_filename);
            mesh.audit_input_curves();

            // The manifoldness check above ran on the offset as constructed. Optimization then
            // re-triangulates it -- splits, collapses and four kinds of swap all touch the
            // offset surface -- so the property has to be re-established afterwards, not
            // assumed to survive.
            if (check_manifoldness) {
                if (mesh.offset_is_manifold()) {
                    logger().info("Offset region manifold check passed after optimization.");
                } else {
                    logger().error("Offset region is NOT manifold after optimization!");
                }
            }
        } else {
            logger().info(
                "optimize_offset false: the optimization is skipped, the offset as constructed "
                "is the result");
            if (mesh.m_offset_params.phi_grid_resolution >= 2) {
                logger().warn(
                    "phi_grid_resolution {} ignored: the offset potential is built only for the "
                    "optimization",
                    mesh.m_offset_params.phi_grid_resolution);
            }
        }
        // The last debug frame is written in the background (write_debug_frame()); no frame
        // follows, so the run waits for it here.
        mesh.wait_debug_frame_write();

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
                log_and_throw_error("INVERSION DURING OFFSET! bad tet ids: {}", bad_tets_str);
            }
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
            report["after #t"] = mesh.tet_size();
            report["threads"] = NUM_THREADS;
            report["time"] = time;
            if (!mesh.optimization_metrics.empty()) {
                // The same eight series as the 2D block above, face samples in place of edge
                // samples. The two error series are reported facts, never criteria.
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


                // Read from the run, never recomputed from the arrays: optimize_offset() breaks
                // out of the loop the moment it converges, so its own verdict cannot drift from
                // the criterion it applied.
                report["converged"] = mesh.m_converged;
                report["offset_level"] = mesh.m_offset_potential->target_level();
                report["offset_dhat"] = mesh.m_offset_potential->dhat();
            }
            component_io.result.second = report;
            if (!component_io.memory()) {
                std::ofstream f_out(report_file);
                f_out << std::setw(4) << report;
            }
        }

        // mesh.write_msh(output_filename.string()); // write .msh
        topological_offset::collect_memory_output(mesh, component_io);
        if (!component_io.memory())
            io::write_mesh(output_filename.string() + ".msh", component_io.result.first.at("mesh"));
        hand_on_envelope(offset_rows(mesh));
        if (mesh.m_offset_params.save_vtu) { // write .vtu: the result, as the .msh
            mesh.write_vtu(output_filename.string(), nullptr, /*result=*/true);
        }

        wmtk::logger().info("======= finish =========");
    }
}


void topological_offset(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult topological_offset(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return std::move(context.result);
}

} // namespace wmtk::components::topological_offset
