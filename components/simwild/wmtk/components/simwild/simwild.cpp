#include "simwild.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/DriverPrologue.hpp>


#include <memory>
#include <optional>
#include <set>
#include <vector>

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/Types.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <wmtk/utils/EnvelopeBudget.hpp>
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
        if constexpr (std::is_same_v<MeshT, SimWildMesh>) {
            // The order-2 guarantee, audited: every edge still carrying the input's order-2 tag
            // lies in its tube (tag_order2_edges()). Edges the operations made order-2 carry no
            // tag and are not counted.
            const size_t n_out =
                mesh.tagged_edges_outside(SimWildMesh::FeatureEdgeAttributes::order2);
            if (n_out > 0) {
                logger().warn("{} order-2 edges outside their tube", n_out);
            } else {
                logger().info("order-2 edges outside their tube: 0");
            }
        }
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
    } else if (operation == "tight_seal_geom") {
        const std::string selection = json_params["tight_seal_geom_selection"];
        const std::vector<std::string> tag_names = json_params["tight_seal_geom_tags"];
        std::vector<int64_t> tags;
        if (selection == "detected") {
            if (!tag_names.empty()) {
                logger().warn(
                    "tight_seal_geom_selection is \"detected\", so tight_seal_geom_tags {} is "
                    "ignored",
                    tag_names);
            }
        } else {
            for (const auto& tag_name : tag_names) {
                const auto expr = expression_parser::parse(tag_name, mesh.m_tag_name_to_id);
                // the reader drops ambient (the untagged cells), so it is no tag to parse into
                if (expr->tag_names_involved().count("ambient") > 0) {
                    log_and_throw_error(
                        "tight_seal_geom_tags cannot hold ambient. Got {}",
                        tag_names);
                }
                const CellTag tag = expr->tags_involved();
                if (!expr->contains_only_and() || tag.size() != 1) {
                    log_and_throw_error(
                        "Each tight_seal_geom_tags entry must be a single tag. Got {}",
                        tag_name);
                }
                tags.push_back(*tag.begin());
            }
        }
        mesh.tight_seal_geom(
            tags,
            json_params["tight_seal_geom_distance"],
            json_params["tight_seal_geom_trace_rel_tol"],
            selection,
            json_params["tight_seal_geom_min_regions"],
            json_params["tight_seal_geom_sanity"]);
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
 * @brief Does a simplification actually run, so that there is a simplified geometry to build
 * the envelopes around?
 *
 * SimWild simplifies in one of two places, and which one applies turns on preserve_topology:
 *
 *  - BEFORE insertion, in read_mesh / read_curves (read_image_msh.cpp), when
 *    `!preserve_topology && simplify`. Simplification can change the topology of the
 *    input, so this route only runs where the caller has said that is acceptable.
 *  - AFTER insertion, SimWildMesh::simplify() from run_3D, when
 *    `preserve_topology && simplify && operation == "remeshing"`. It collapses at
 *    eps_simplify with the quality check on (EdgeCollapsing.cpp says why), and then, with
 *    envelope_reference "simplified", REBUILDS the envelope around the result, so it
 *    displaces the envelope geometry exactly as the pre-insertion route does; with "input"
 *    it restores the envelope around the input.
 *
 * Both routes run in both dimensions: run_2D calls SimWildMeshTri::simplify(), the 2D twin,
 * under the same condition. Until 2026-11 the 2D route had no post-insertion counterpart, and under
 * preserve_topology nothing simplified there.
 */
bool simplification_runs(const Parameters& params)
{
    if (!params.simplify) return false;
    if (!params.preserve_topology) return true; // pre-insertion
    return params.operation == "remeshing"; // post-insertion
}

/**
 * @brief The eps the optimizer's envelope uses, the full eps, after refusing an
 * envelope_reference the run cannot honour (utils::check_envelope_reference()).
 *
 * envelope_reference "input", the default: the envelope is around the input as read -- read_mesh
 * in 3D and the curve pipeline in 2D hand it over, and SimWildMesh::simplify() (2D:
 * SimWildMeshTri's) puts it back after simplifying. "simplified": around the simplified
 * geometry, which replaces the input. Both at the full eps; until 2026-11 "simplified" took what
 * the simplification left of it.
 *
 * Until 2026-10 SimWild built it around the simplified geometry whatever the flag said, and
 * charged the simplification only with the flag on: without it the envelope used the full eps
 * around geometry that had already moved up to eps_simplify from the input, so a vertex could
 * end up eps_simplify + eps away from it rather than eps.
 *
 * Both simplifications run at Parameters::eps_simplify: read_mesh derives it from the same
 * diagonal (InputData::diag).
 */
double envelope_eps_and_check(const Parameters& params, const InputData& input_data)
{
    const bool after_insertion = params.preserve_topology && simplification_runs(params);
    wmtk::utils::check_envelope_reference(
        params.eps,
        params.eps_simplify,
        params.envelope_around_simplified,
        input_data.simplified_before_insertion || after_insertion);
    return params.eps;
}

/**
 * @brief An `envelope` that is a labelled reference (InputData::envelope_reference) is held to
 * as given, so refused with the options that would not: the sampled envelope, since every check
 * against the reference is exact; envelope_reference "simplified", which rebuilds the envelopes
 * around this run's simplification (and without one is refused anyway).
 */
void check_reference_options(const Parameters& params)
{
    if (params.use_sample_envelope) {
        log_and_throw_error(
            "`envelope`: a reference is held to and checked against exactly, not with "
            "use_sample_envelope");
    }
    if (params.envelope_around_simplified) {
        log_and_throw_error(
            "`envelope`: a reference is held to as given, not with envelope_reference "
            "\"simplified\", which builds the envelopes around this run's simplification");
    }
}

/// The inputs of a surface (3D) or curve (2D) run, by their content
/// (io::ComponentIO::input_kind()).
struct InputLists
{
    /// The inputs the arrangement inserts and tags -- in 3D the surfaces, in 2D the curves --
    /// and json_params with their own input_names, input_type and input_transform, for the
    /// readers (read_mesh(), read_curves()), which index those by insertion.
    std::vector<std::string> inserted;
    nlohmann::json params;
    /// The selectors of the 3D curve inputs (feature curves) and of the point inputs, with their
    /// names.
    nlohmann::json curves = nlohmann::json::array(), points = nlohmann::json::array();
    std::vector<std::string> curve_names, point_names;
};

/**
 * @brief One input list: each entry what its content makes it, named by input_names at its
 * position in the list (tag_<i> without a name), with an input_type, when given, per entry. A
 * surface (3D) or a curve (2D) is inserted, input_type "volume" (the default) or "surface" (2D:
 * "curve"); in 3D a curve is a feature curve, input_type "curve", and in both dimensions bare
 * vertices are input points, input_type "point" -- their content leaves no choice. input_transform
 * applies to the inserted inputs alone, as it did. Until 2026-11 curves and points came in lists
 * of their own, input_edges and input_points, the curves named feature_<i>.
 */
InputLists split_inputs(
    const io::ComponentIO& component_io,
    const std::vector<std::string>& paths,
    const nlohmann::json& json_params,
    const int dimension)
{
    using Kind = io::ComponentIO::InputKind;
    const std::vector<std::string> names = json_params["input_names"];
    const nlohmann::json& types = json_params["input_type"];
    const nlohmann::json& transforms = json_params["input_transform"];
    if (!types.empty() && types.size() != paths.size()) {
        log_and_throw_error(
            "input_type needs one entry per input: {} entries for {} inputs",
            types.size(),
            paths.size());
    }
    InputLists out;
    out.params = json_params;
    out.params["input_names"] = nlohmann::json::array();
    out.params["input_type"] = nlohmann::json::array();
    out.params["input_transform"] = nlohmann::json::array();
    const char* inserted_kind = dimension == 3 ? "surface" : "curve";
    for (size_t i = 0; i < paths.size(); ++i) {
        const Kind kind = component_io.input_kind(paths[i]);
        const std::string name = i < names.size() ? names[i] : "tag_" + std::to_string(i);
        const std::string type = types.empty() ? "" : types[i].get<std::string>();
        if (kind == Kind::surface && dimension == 2) {
            log_and_throw_error(
                "{}: triangles in a 2D run, whose inputs are curves and points",
                paths[i]);
        }
        if (kind == (dimension == 3 ? Kind::surface : Kind::curve)) {
            if (!type.empty() && type != "volume" && type != inserted_kind) {
                log_and_throw_error(
                    "input_type \"{}\" for {}: a {} input is \"volume\" or \"{}\"{}",
                    type,
                    paths[i],
                    inserted_kind,
                    inserted_kind,
                    type == "surface" ? " (\"surface\" is 3D only)" : "");
            }
            out.inserted.push_back(paths[i]);
            out.params["input_names"].push_back(name);
            if (!types.empty()) out.params["input_type"].push_back(type);
            out.params["input_transform"].push_back(
                i < transforms.size() ? transforms[i] : nlohmann::json::array());
            continue;
        }
        const std::string own = kind == Kind::points ? "point" : "curve";
        if (!type.empty() && type != own) {
            log_and_throw_error(
                "input_type \"{}\" for {}: its content makes it a {} input, input_type \"{}\"",
                type,
                paths[i],
                own,
                own);
        }
        if (i < transforms.size() && !transforms[i].empty()) {
            log_and_throw_error(
                "input_transform for {}: a transform applies to the {} inputs, not to {}",
                paths[i],
                inserted_kind,
                kind == Kind::points ? "points" : "feature curves");
        }
        (kind == Kind::points ? out.points : out.curves).push_back(json_params["input"][i]);
        (kind == Kind::points ? out.point_names : out.curve_names).push_back(name);
    }
    if (out.inserted.empty()) {
        log_and_throw_error(
            "simwild needs a {} among its inputs: got {} curves and {} point inputs",
            inserted_kind,
            out.curves.size(),
            out.points.size());
    }
    return out;
}

} // namespace

void run_3D(const nlohmann::json& json_params, InputData& input_data, io::ComponentIO& component_io)
{
    Parameters params(json_params);
    params.output_enabled = !component_io.memory();
    params.init(
        input_data.V_input.colwise().minCoeff(),
        input_data.V_input.colwise().maxCoeff(),
        input_data.diag);

    igl::Timer timer;
    timer.start();

    simwild::SimWildMesh mesh(
        params,
        envelope_eps_and_check(params, input_data),
        params.NUM_THREADS);
    // A reference, the first component's labelled input: every envelope, tube and anchor around
    // it, never around the mesh received (SimWildMesh::m_has_reference).
    const bool reference = input_data.envelope_reference.has_value();
    std::optional<io::HeldRows> held;
    if (reference) {
        check_reference_options(params);
        held = io::held_rows(
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.F_envelope_labels);
        mesh.init_envelope(held->V, held->F);
        mesh.m_F_envelope_labels = held->labels;
        mesh.m_has_reference = true;
        mesh.m_reference_curves = input_data.reference_curves;
        // The input points' anchors on its points (TetOptimizerMesh::ReferencePoints).
        mesh.set_reference_points(input_data.reference_points, input_data.reference_point_labels);
    } else if (input_data.V_envelope.size() != 0) {
        // first init envelope
        mesh.init_envelope(input_data.V_envelope, input_data.F_envelope);
        mesh.m_F_envelope_labels = input_data.F_envelope_labels;
    }
    // Before init_from_image(): its reference checks name the feature curves' labels by these,
    // and its anchors keep the input points' labels, which the output's point block writes.
    mesh.m_feature_edge_names = input_data.feature_edge_names;
    mesh.m_point_names = input_data.point_names;
    mesh.m_input_point_labels = input_data.input_point_labels;
    // Around the input after a simplification before the insertion, the input's own curves: the
    // order-2 tubes and the curve anchors are built around them, and a rim, non-manifold edge or
    // curve end the simplification made is held around itself (TetOptimizerMesh::InputCurves).
    // Where the inputs meet, the input's curves are their intersections, which only an
    // arrangement of the input gives: there the edges are held around themselves too.
    if (!params.envelope_around_simplified && input_data.simplified_before_insertion) {
        if (input_data.F_envelope_labels.size() == size_t(input_data.F_envelope.rows())) {
            std::vector<Eigen::Vector3d> V(input_data.V_envelope.rows());
            for (size_t i = 0; i < V.size(); ++i) V[i] = input_data.V_envelope.row(i).transpose();
            std::vector<std::array<size_t, 3>> F(input_data.F_envelope.rows());
            for (size_t i = 0; i < F.size(); ++i) {
                for (int j = 0; j < 3; ++j) F[i][j] = size_t(input_data.F_envelope(i, j));
            }
            mesh.set_input_curves(
                V,
                F,
                input_data.F_envelope_labels,
                input_data.features.empty() ? nullptr : &input_data.features);
        } else {
            logger().warn(
                "envelope_reference \"input\": the input triangles' labels are not known, so "
                "the order-2 tubes and curve anchors are built around the simplified surface");
        }
    } else if (reference) {
        // The reference's own curves, as after a simplification the input's: an order-2 edge in
        // their tubes is held there, a curve end in reach of one of their ends anchored there.
        // An edge on none -- where inputs meet, or a curve the reference does not have -- is held
        // around itself, by the same rule (TetOptimizerMesh::InputCurves).
        std::vector<Eigen::Vector3d> V(held->V.rows());
        for (size_t i = 0; i < V.size(); ++i) V[i] = held->V.row(i).transpose();
        std::vector<std::array<size_t, 3>> F(held->F.rows());
        for (size_t i = 0; i < F.size(); ++i) {
            for (int j = 0; j < 3; ++j) F[i][j] = size_t(held->F(i, j));
        }
        mesh.set_input_curves(V, F, held->labels, &input_data.reference_curves);
    }
    if (input_data.V_input_r.size() == 0) {
        logger().info("Use float input for TetWild");
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names,
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.features.empty() ? nullptr : &input_data.features,
            &input_data.features_placed,
            input_data.F_orientation);
    } else {
        logger().warn("Use RATIONAL input for TetWild");
        mesh.init_from_image(
            input_data.V_input_r,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names,
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.features.empty() ? nullptr : &input_data.features,
            &input_data.features_placed,
            input_data.F_orientation);
    }

    auto write_unique_vtu = [&params, &mesh]() {
        static size_t vtu_counter = 0;
        if (params.write_vtu) {
            mesh.write_vtu(fmt::format("{}_{}", params.output_path, vtu_counter++));
        }
    };

    mesh.consolidate_mesh();

    write_unique_vtu();

    // The tagged tet mesh as the output .msh holds it: tets, their tags, the face tags, the
    // feature curves' edge tags (named by m_feature_edge_names, set before the initialization).
    auto tagged_outputs = [&mesh]() {
        MeshMap raw;
        mesh.write_vtu("mesh", &raw);
        io::compact_slot_outputs(mesh, raw);
        MeshMap outputs = io::named_outputs(std::move(raw));
        io::collect_tags(
            mesh,
            outputs,
            CellType::Tetrahedron,
            mesh.get_tets(),
            [&](const auto& t) -> const auto& { return mesh.m_tet_attribute[t.tid(mesh)].tags; });
        io::collect_input_set_tags<CellType::Triangle>(mesh, outputs, mesh.m_face_tag_names);
        io::collect_feature_edge_tags(mesh, outputs, mesh.m_feature_edge_names);
        io::collect_point_tags(mesh, outputs, mesh.m_point_names);
        return outputs;
    };

    // From here on the tracked surface -- the interfaces of the tags the input gave -- is recorded.
    const std::string history_output = json_params["history_output"];
    MeshMap history_input;
    if (!history_output.empty()) {
        // The tet mesh the history starts from, tags and all: for a surface input that is the
        // embedding, which no file holds otherwise. Next to the output on the file route; on the
        // memory route, the output mesh history_input.
        MeshData start = tagged_outputs().at("mesh");
        if (component_io.memory())
            history_input = io::named_outputs({{"mesh_history_input.msh", std::move(start)}});
        else
            io::write_mesh(params.output_path + "_history_input.msh", start);
        // A surface input: from the files, through the arrangement, to here.
        mesh.m_insertion_sources = std::move(input_data.facet_sources);
        mesh.record_surface_history(component_io.history_metadata(json_params));
    }
    // The volume around the tracked surface: its own choice, as it takes far more room.
    const std::string volume_history_output = json_params["volume_history_output"];
    if (!volume_history_output.empty()) {
        if (history_output.empty()) {
            log_and_throw_error(
                "volume_history_output needs history_output: the volume history "
                "follows the surface's");
        }
        mesh.record_volume_history();
    }

    if (params.operation == "remeshing" && params.preserve_topology && params.simplify) {
        // collapse for getting the right edge length
        logger().info("Simplify after insertion");
        mesh.simplify();
    }

    // /////////apply operation
    apply_operation(mesh, json_params);

    mesh.consolidate_mesh();
    // A tag operation hands on an envelope around its result (set_envelope_around_tags()).
    if (params.operation != "remeshing") mesh.set_envelope_around_tags();
    double time = timer.getElapsedTime();
    logger().info("total optimization time {}s", time);

    // The tag operations change tags only, and leave the surface flags to the next run: the
    // history ends where the flags are.
    if (const auto h = mesh.finish_surface_history(
            input_data.has_input_history ? &input_data.input_history : nullptr)) {
        component_io.save_history(*h, history_output);
    }

    // The output is what the caller consumes, and write_msh emits m_posf unconditionally --
    // for an un-rounded vertex that double is not the vertex's real position, so the written
    // mesh can contain inverted tets. Report it rather than letting it pass silently.
    const bool all_rounded = mesh.all_rounded();
    if (!all_rounded) {
        logger().error("Not all vertices rounded!");
    }

    // A SimWild surface is exactly the interface between unlike tags, so every face between two
    // differently tagged tets must be marked as surface. Remeshing maintains this incrementally;
    // check it once at the end rather than trusting that it did. The tag operations leave the
    // flags to the next run (above), so after them the check would only list the faces whose
    // tags they changed.
    if (params.operation == "remeshing" && !mesh.check_interface_faces_tagged(/*verbose=*/true)) {
        logger().error("Interface faces are not consistently tagged as surface!");
    }

    /////////output
    auto [max_energy, avg_energy] = mesh.get_max_avg_energy();
    const std::string report_file = json_params["report"];
    if (component_io.memory() || !report_file.empty()) {
        nlohmann::json report;
        report["#t"] = mesh.get_tets().size();
        report["#v"] = mesh.get_vertices().size();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = params.NUM_THREADS;
        report["time"] = time;
        report["all_rounded"] = all_rounded;
        component_io.result.second = report;
        if (!component_io.memory()) {
            std::ofstream fout(report_file);
            fout << std::setw(4) << report;
        }
    }

    logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    // One result for both routes: returned from memory calls, written as the .msh otherwise.
    {
        component_io.result.first = tagged_outputs();
        component_io.result.first.merge(history_input);
        // The volume history names the tags as the tet meshes' groups do -- the labels
        // io::collect_tags gives them, not the names, which may repeat or be "ambient" -- so
        // a tet's groups give its region.
        if (!volume_history_output.empty()) {
            const auto& labels = component_io.result.first.at("mesh")
                                     .cells.at(CellType::Tetrahedron)
                                     .attributes.at("tags")
                                     .labels;
            std::map<int64_t, std::string> tag_labels;
            for (size_t j = 0; j < mesh.m_tags_count && j < labels.size(); ++j) {
                tag_labels[int64_t(j)] = labels[j];
            }
            if (const auto vh = mesh.finish_volume_history(tag_labels)) {
                component_io.save_volume_history(*vh, volume_history_output);
            }
        }
        // The envelope handed on: a reference given, unchanged, so that a chain keeps passing the
        // first component's input; else the run's own, labelled (SimWildMesh::envelope_data()),
        // after a tag operation the one around its result. On the file route in a file of its
        // own, as the mesh file holds the mesh.
        if (!mesh.m_V_envelope.empty()) {
            component_io.result.first["envelope"] =
                reference ? *input_data.envelope_reference
                          : mesh.envelope_data(input_data.features, input_data.feature_edge_names);
        }
        if (!component_io.memory()) {
            io::write_mesh(params.output_path + ".msh", component_io.result.first.at("mesh"));
            if (component_io.result.first.count("envelope")) {
                io::write_mesh(
                    params.output_path + "_envelope.msh",
                    component_io.result.first.at("envelope"));
            }
        }
    }
    write_unique_vtu();
    if (params.write_vtu) {
        mesh.write_surface(params.output_path + "_surface.obj");
    }
}

void run_2D(
    const nlohmann::json& json_params,
    const InputData& input_data,
    io::ComponentIO& component_io)
{
    // A 2D mesh's interfaces are curves, which a surface history does not describe.
    if (!std::string(json_params["history_output"]).empty()) {
        log_and_throw_error(
            "history_output: simwild records a surface history in 3D only, and this input is "
            "2D");
    }
    Parameters params(json_params);
    params.output_enabled = !component_io.memory();
    params.init(
        input_data.V_input.colwise().minCoeff(),
        input_data.V_input.colwise().maxCoeff(),
        input_data.diag);

    igl::Timer timer;
    timer.start();

    simwild::tri::SimWildMeshTri mesh(
        params,
        envelope_eps_and_check(params, input_data),
        params.NUM_THREADS);
    // A reference, held to as in 3D (SimWildMeshTri::m_has_reference).
    const bool reference = input_data.envelope_reference.has_value();
    std::optional<io::HeldRows> held;
    if (reference) {
        check_reference_options(params);
        held = io::held_rows(
            input_data.V_envelope,
            input_data.F_envelope,
            input_data.F_envelope_labels);
        mesh.init_envelope(held->V, held->F);
        mesh.m_E_envelope_labels = held->labels;
        mesh.m_has_reference = true;
        mesh.set_reference_points(
            input_data.reference_points,
            input_data.reference_point_labels); // as in 3D
    } else if (input_data.V_envelope.size() != 0) {
        // first init envelope
        mesh.init_envelope(input_data.V_envelope, input_data.F_envelope);
        mesh.m_E_envelope_labels = input_data.F_envelope_labels;
    }
    // Around the input after a simplification, the input's own curve ends and junctions: the
    // anchors are centred on them (TriOptimizerMesh::m_input_curve_points), as in 3D. With a
    // reference, its own.
    if (!params.envelope_around_simplified && input_data.simplified_before_insertion) {
        mesh.set_input_curve_points(input_data.V_envelope, input_data.F_envelope);
    } else if (reference) {
        mesh.set_input_curve_points(held->V, held->F);
    }
    mesh.m_input_point_vids = input_data.input_point_vids;
    mesh.m_point_names = input_data.point_names; // as in 3D
    mesh.m_input_point_labels = input_data.input_point_labels;
    if (input_data.V_input_r.size() == 0) {
        mesh.init_from_image(
            input_data.V_input,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names,
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.F_orientation);
    } else {
        logger().warn("Use RATIONAL input for 2D");
        mesh.init_from_image(
            input_data.V_input_r,
            input_data.T_input,
            input_data.T_input_tag,
            input_data.tag_names,
            input_data.F_tagged,
            input_data.F_tags,
            input_data.face_tag_names,
            input_data.F_orientation);
    }

    auto write_unique_vtu = [&params, &mesh]() {
        static size_t vtu_counter = 0;
        if (params.write_vtu) {
            mesh.write_vtu(fmt::format("{}_{}", params.output_path, vtu_counter++));
        }
    };

    mesh.consolidate_mesh();
    write_unique_vtu();

    // As in run_3D: under preserve_topology the curves are simplified here, after the
    // insertion, rather than before it in read_curves.
    if (params.operation == "remeshing" && params.preserve_topology && params.simplify) {
        logger().info("Simplify after insertion");
        mesh.simplify();
    }

    // /////////apply operation
    apply_operation(mesh, json_params);

    mesh.consolidate_mesh();
    // As in 3D: a tag operation hands on an envelope around its result.
    if (params.operation != "remeshing") mesh.set_envelope_around_tags();
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
    if (component_io.memory() || !report_file.empty()) {
        nlohmann::json report;
        report["#t"] = mesh.get_faces().size();
        report["#v"] = mesh.get_vertices().size();
        report["max_energy"] = max_energy;
        report["avg_energy"] = avg_energy;
        report["eps"] = params.eps;
        report["threads"] = params.NUM_THREADS;
        report["time"] = time;
        report["all_rounded"] = all_rounded;
        component_io.result.second = report;
        if (!component_io.memory()) {
            std::ofstream fout(report_file);
            fout << std::setw(4) << report;
        }
    }

    // One result for both routes: returned from memory calls, written as the .msh otherwise.
    {
        MeshMap raw;
        mesh.write_vtu("mesh", &raw);
        io::compact_slot_outputs(mesh, raw);
        component_io.result.first = io::named_outputs(std::move(raw));
        io::collect_tags(
            mesh,
            component_io.result.first,
            CellType::Triangle,
            mesh.get_faces(),
            [&](const auto& t) -> const auto& { return mesh.m_face_attribute[t.fid(mesh)].tags; });
        // The curve inputs' edges, tagged by name, as the edge block of the mesh.
        io::collect_input_set_tags<CellType::Edge>(
            mesh,
            component_io.result.first,
            mesh.m_edge_tag_names);
        io::collect_point_tags(mesh, component_io.result.first, mesh.m_point_names);
        // The envelope handed on, as in 3D, with the input points.
        if (!mesh.m_V_envelope.empty()) {
            std::vector<Vector2d> points;
            for (const size_t v : input_data.input_point_vids) {
                points.emplace_back(input_data.V_input(v, 0), input_data.V_input(v, 1));
            }
            component_io.result.first["envelope"] =
                reference ? *input_data.envelope_reference : mesh.envelope_data(points);
        }
        if (!component_io.memory()) {
            io::write_mesh(params.output_path + ".msh", component_io.result.first.at("mesh"));
            if (component_io.result.first.count("envelope")) {
                io::write_mesh(
                    params.output_path + "_envelope.msh",
                    component_io.result.first.at("envelope"));
            }
        }
    }
    logger().info("final max energy = {} avg = {}", max_energy, avg_energy);
    write_unique_vtu();
}

static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "simwild");
    const auto envelope_selector = json_params.value("envelope", nlohmann::json());
    if (component_io.memory()) json_params.erase("envelope");
    using wmtk::utils::resolve_path;

    const std::filesystem::path root = utils::verify_and_setup_logger(
        json_params,
        jse::embed::wmtk_simwild_spec::simwild_spec::spec());
    const std::vector<std::string> input_paths = component_io.paths(root, json_params["input"]);

    // std::filesystem::path output_filename = resolve_path(root, json_params["output"]);
    std::filesystem::path output_filename = json_params["output"];

    // A mesh extension is dropped (the outputs add their own); any other dot is part of the
    // name, so "v3r0.005_rem" stays whole instead of becoming "v3r0".
    const std::string output_extension = output_filename.extension().string();
    const std::set<std::string> mesh_extensions = {".msh", ".vtu", ".obj", ".stl", ".off", ".ply"};
    if (mesh_extensions.count(output_extension)) {
        output_filename.replace_extension(""); // extension is added back later
        if (output_extension != ".msh") {
            logger().warn(
                "Extension of provided output filename is ignored. Output will be {}.msh",
                output_filename.string());
        }
    }
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
    const int dimension = resolve_input_dimension(
        input_paths,
        json_params,
        component_io.memory() ? &component_io : nullptr);
    // The `envelope` input: a mesh of the call on the memory route, a mesh file on the file
    // route (relative to input_dir), read by the shared reader. Surface and curve inputs build
    // their envelope from the input itself.
    const std::string envelope_path =
        component_io.memory() ? "" : json_params["envelope"].get<std::string>();
    const bool envelope_given = !envelope_selector.is_null() || !envelope_path.empty();
    if (dimension != 0 && envelope_given) {
        log_and_throw_error("`envelope` applies to tagged mesh inputs only");
    }
    // A tag operation holds nothing to an envelope, and the boundaries it leaves are new geometry
    // that the one given does not cover: it is not read, and the run hands on one around its
    // result (SimWildMesh::set_envelope_around_tags()). Until 2026-11 it was read, a reference
    // refused for a label the mesh holds that it lacked, and the run handed on none.
    const bool tag_operation = json_params["operation"] != "remeshing";
    if (tag_operation && envelope_given) {
        logger().info(
            "operation {}: the `envelope` given is not read; the envelope handed on is built "
            "around the result",
            json_params["operation"].get<std::string>());
    }
    std::optional<MeshData> envelope_file;
    if (!envelope_path.empty() && !tag_operation)
        envelope_file = io::read_mesh(resolve_path(root, envelope_path));
    const MeshData* envelope = envelope_file ? &*envelope_file
                               : !envelope_selector.is_null() && !tag_operation
                                   ? &component_io.mesh(envelope_selector)
                                   : nullptr;
    // A tagged mesh is the run's one input, as on tetwild's and triwild's tagged routes: the route
    // reads input[0] only, and until 2026-11 dropped any other input without a word.
    const auto only_input = [&](const char* route) {
        if (input_paths.size() > 1) {
            log_and_throw_error(
                "{} is simwild's only input: got {} inputs, and the route reads the first alone. "
                "Remesh the tagged mesh by itself, or give every input as a surface (2D: a curve "
                "network) to insert",
                route,
                input_paths.size());
        }
    };
    if (component_io.memory() && dimension == 0) {
        only_input("a tagged mesh");
        // Label images become regions here as on the file route (read_image_msh).
        MeshData input = component_io.mesh(input_paths.at(0));
        tags_from_label_images(input, input_paths.at(0));
        input_data = read_mesh_data(input, envelope);
    } else if (!component_io.memory() && extension == ".msh") {
        only_input("a .msh input");
        input_data = read_image_msh(input_paths[0], envelope);
    } else if (dimension == 2) {
        const InputLists lists = split_inputs(component_io, input_paths, json_params, 2);
        // Read by the code the 3D route reads them with; only their x and y are used.
        const utils::FeatureInputs features = utils::read_feature_inputs(
            component_io,
            root,
            nlohmann::json::array(),
            lists.points,
            {},
            lists.point_names);
        input_data = read_curves(
            lists.inserted,
            output_filename.string(),
            lists.params,
            component_io.memory() ? &component_io : nullptr,
            &features);
        input_data.input_point_labels = features.point_input;
        input_data.point_names = features.point_names;
    } else {
        const InputLists lists = split_inputs(component_io, input_paths, json_params, 3);
        // Read by the code tetwild reads them with.
        const utils::FeatureInputs features = utils::read_feature_inputs(
            component_io,
            root,
            lists.curves,
            lists.points,
            lists.curve_names,
            lists.point_names);
        input_data = read_mesh(
            lists.inserted,
            output_filename.string(),
            lists.params,
            component_io.memory() ? &component_io : nullptr,
            &features);
        if (!input_data.features.empty()) input_data.feature_edge_names = lists.curve_names;
        input_data.input_point_labels = features.point_input;
        input_data.point_names = features.point_names;
    }

    if (input_data.T_input.cols() == 4) {
        run_3D(json_params, input_data, component_io);
    } else {
        run_2D(json_params, input_data, component_io);
    }
    double time = timer.getElapsedTime();
    logger().info("total runtime {}s", time);
    logger().info("======= finish =========");
}

void simwild(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult simwild(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return context.take_result();
}

} // namespace wmtk::components::simwild