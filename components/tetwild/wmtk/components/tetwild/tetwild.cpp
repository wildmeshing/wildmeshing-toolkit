#include "tetwild.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>
#include <wmtk/utils/DriverPrologue.hpp>

#include "Parameters.h"
#include "TetWildMesh.h"

#include <jse/jse.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/Partitioning.h>
#include <cstdlib>
#include <wmtk/history/History.hpp>
#include <wmtk/history/InputHistory.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>

#include <wmtk/components/shortest_edge_collapse/ShortestEdgeCollapse.h>
#include <algorithm>
#include <cctype>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>
#include <wmtk/components/shortest_edge_collapse/Embedding.hpp>
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
#include <spdlog/common.h>
#include <wmtk/utils/EnvelopeBudget.hpp>
#include <wmtk/utils/FeatureInputs.hpp>
#include <wmtk/utils/WindingNumber.hpp>
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

/**
 * @brief The tagged route's input (read_tagged_mesh()), when `input` is a tet mesh: on the memory
 * route a named mesh with tets, on the file route a .msh with tets, read by the shared reader. Its
 * fields but the tags are dropped, as every reader drops them. Without one, nullopt: the inputs
 * are surfaces. A tet mesh is the only input: with any other, it throws.
 */
static std::optional<TaggedMesh> read_tagged_input(
    const io::ComponentIO& component_io,
    const std::vector<std::string>& input_paths)
{
    std::optional<TaggedMesh> tagged;
    for (const std::string& path : input_paths) {
        if (component_io.memory()) {
            const MeshData& mesh = component_io.mesh(path);
            if (!is_tet_mesh(mesh)) continue;
            if (input_paths.size() > 1) {
                log_and_throw_error(
                    "{}: a tagged tet mesh is tetwild's only input, not one of {}",
                    path,
                    input_paths.size());
            }
            tagged = read_tagged_mesh(mesh, path);
            continue;
        }
        std::string extension = std::filesystem::path(path).extension().string();
        for (char& c : extension) c = char(std::tolower(static_cast<unsigned char>(c)));
        if (extension != ".msh") continue;
        MeshData mesh = io::read_mesh(path);
        if (!is_tet_mesh(mesh)) continue;
        if (input_paths.size() > 1) {
            log_and_throw_error(
                "{}: a tagged tet mesh is tetwild's only input, not one of {}",
                path,
                input_paths.size());
        }
        tagged = read_tagged_mesh(mesh, path);
    }
    return tagged;
}

static TetWildMesh::ExportStruct run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "tetwild");
    // The `envelope` input names a mesh of the call on the memory route, as simwild's does.
    const auto envelope_selector = json_params.value("envelope", nlohmann::json());
    if (component_io.memory()) json_params.erase("envelope");
    using wmtk::utils::resolve_path;

    const std::filesystem::path root = utils::verify_and_setup_logger(
        json_params,
        jse::embed::wmtk_tetwild_spec::tetwild_spec::spec());
    const std::vector<std::string> inputs = component_io.paths(root, json_params["input"]);

    tetwild::Parameters params;
    params.output_enabled = !component_io.memory();

    std::string output_path = json_params["output"];
    bool simplify = json_params["simplify"];
    const bool simplify_use_sample_envelope = json_params["simplify_use_sample_envelope"];
    const double eps_simplify_rel = json_params["eps_simplify_rel"];
    const double eps_simplify = json_params["eps_simplify"];
    const bool simplify_boundary_envelope = json_params["simplify_boundary_envelope"];
    bool use_sample_envelope = json_params["use_sample_envelope"];
    int NUM_THREADS = json_params["num_threads"];
    int max_its = json_params["max_iterations"];
    std::string filter_option = json_params["filter"];

    params.epsr = json_params["eps_rel"];
    params.lr = json_params["length_rel"];
    params.order2_envelope_ratio = json_params["order2_envelope_ratio"];
    params.feature_envelope_ratio = json_params["feature_envelope_ratio"];
    params.stop_energy = json_params["stop_energy"];
    params.split_high_valence_threshold = json_params["split_high_valence_threshold"];
    params.num_smoothing_passes = json_params["num_smoothing_passes"];
    params.interleaved_smoothing = json_params["interleaved_smoothing"];
    params.interleaved_smoothing_passes = json_params["interleaved_smoothing_passes"];
    params.colored_smoothing = json_params["colored_smoothing"];
    params.check_surface_folds = json_params["check_surface_folds"];

    // Coarsening pass.
    params.coarsen_pass = json_params["coarsen_pass"];
    params.coarsen_unbounded = json_params["coarsen_unbounded"];
    params.coarsen_local_smoothing_passes = json_params["coarsen_local_smoothing_passes"];
    params.coarsen_smooth_ring = json_params["coarsen_smooth_ring"];
    params.coarsen_global_smoothing_passes = json_params["coarsen_global_smoothing_passes"];
    params.coarsen_max_rounds = json_params["coarsen_max_rounds"];
    params.coarsen_max_inner_passes = json_params["coarsen_max_inner_passes"];
    params.w_amips = json_params["w_amips"];
    params.smoothing_mode = json_params["smoothing_mode"];
    params.project_line_search_steps = json_params["project_line_search_steps"];
    params.project_line_search_nested_steps = json_params["project_line_search_nested_steps"];

    params.preserve_topology = json_params["preserve_topology"];
    params.per_input_envelopes = json_params["per_input_envelopes"];
    // preserve_topology holds the input simplification to the link condition as well, so the
    // surface handed to the arrangement has the input's topology.
    const bool simplify_use_link_condition = params.preserve_topology;

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

    // Recording the tracked surface: from the input files as read, through the weld, the
    // simplification and the insertion, to the output.
    const std::string history_output = json_params["history_output"];
    const bool record_history = !history_output.empty() && !json_params["use_legacy_code"];
    if (!history_output.empty() && !record_history) {
        logger().warn("history_output: not recorded, use_legacy_code rebuilds the mesh");
    }
    // Recording the feature curves (the curve inputs, with their ends, junctions and the point
    // inputs), the edges tagged `feature`: their own history and file, as the volume's, with or
    // without the surface's. From the inputs as read, through the insertion, to the output. The
    // legacy optimizer carries no curves.
    const std::string curve_history_output = json_params["curve_history_output"];
    const bool record_curves = !curve_history_output.empty() && !json_params["use_legacy_code"];
    if (!curve_history_output.empty() && !record_curves) {
        logger().warn("curve_history_output: not recorded, use_legacy_code rebuilds the mesh");
    }
    // The volume around it too, its own choice, recorded alongside from the insertion on (see
    // below). It follows the surface history. And the coarsening pass's local smoothing moves a
    // whole ball of vertices inside one collapse, of which the volume history, which records
    // the stars of the edge's ends, sees none: refused, as use_legacy_code is.
    const std::string volume_history_output = json_params["volume_history_output"];
    if (!volume_history_output.empty()) {
        if (!record_history) {
            log_and_throw_error(
                "volume_history_output needs history_output (and not "
                "use_legacy_code): the volume history follows the surface's");
        }
        if (params.coarsen_pass && params.coarsen_local_smoothing_passes > 0) {
            log_and_throw_error(
                "volume_history_output needs coarsen_local_smoothing_passes 0 with coarsen_pass: "
                "the volume history does not see the vertices a coarsening collapse smooths");
        }
    }

    // The tagged route: a tet mesh as the input -- on the memory route a named mesh with tets, on
    // the file route a .msh with tets -- continued from as it is, its tets kept, instead of a
    // background mesh built and cut by the insertion, so that a chain of components can go on
    // through tetwild. tetwild holds the mesh's tagged faces, feature curves and points
    // (read_tagged_mesh()), and no volumes: it carries no tet tags through its operations.
    // Skipped on it: the reading and cleaning of surfaces, the simplification, the insertion and
    // the input winding numbers, which are of input surfaces (wn_<i>). Until 2026-11 a tet mesh
    // threw on the memory route ("Expected triangle input"), and on the file route every
    // triangle of the .msh became one input surface, its tets and tags dropped.
    std::optional<TaggedMesh> tagged_mesh = read_tagged_input(component_io, inputs);
    const bool tagged = tagged_mesh.has_value();
    // One input list. On the surface route each entry is what its content makes it
    // (io::ComponentIO::input_kind()) -- triangles a surface, inserted and tracked as its face
    // tag; segments a curve, forced in as tet edges and held as a feature curve; bare vertices
    // points, forced in as tet vertices and anchored -- and each is named by input_names at its
    // position in the list, tag_<i> without a name (io::input_labels()). Until 2026-11 curves and
    // points came in lists of their own, input_edges and input_points, the curves named
    // feature_<i> after their position there.
    std::vector<std::string> input_paths; // the surfaces; on the tagged route the tet mesh
    std::vector<std::string> surface_labels, curve_labels, point_labels;
    nlohmann::json curve_inputs = nlohmann::json::array(), point_inputs = nlohmann::json::array();
    if (tagged) {
        input_paths = inputs;
    } else {
        const std::vector<std::string> labels = io::input_labels(
            json_params["input_names"].get<std::vector<std::string>>(),
            inputs.size());
        for (size_t i = 0; i < inputs.size(); ++i) {
            switch (component_io.input_kind(inputs[i])) {
            case io::ComponentIO::InputKind::surface:
                input_paths.push_back(inputs[i]);
                surface_labels.push_back(labels[i]);
                break;
            case io::ComponentIO::InputKind::curve:
                curve_inputs.push_back(json_params["input"][i]);
                curve_labels.push_back(labels[i]);
                break;
            case io::ComponentIO::InputKind::points:
                point_inputs.push_back(json_params["input"][i]);
                point_labels.push_back(labels[i]);
                break;
            }
        }
        if (input_paths.empty()) {
            log_and_throw_error(
                "tetwild needs a surface among its inputs: got {} curves and {} point inputs",
                curve_inputs.size(),
                point_inputs.size());
        }
    }
    // The legacy optimizer carries no curve tags: the feature curves and points would leave the
    // output without a word (mesh_improvement_legacy() re-tags the open boundaries only).
    if (json_params["use_legacy_code"] && (!curve_inputs.empty() || !point_inputs.empty())) {
        log_and_throw_error(
            "curve and point inputs: not with use_legacy_code, which carries no feature curves or "
            "points");
    }
    if (tagged) {
        // What the route has no use for, refused rather than ignored.
        if (!json_params["input_names"].empty()) {
            log_and_throw_error("input_names: a tagged mesh input names its labels itself");
        }
        if (filter_option == "input") {
            log_and_throw_error(
                "filter \"input\": a tagged mesh input has no input surface to take the winding "
                "number of; \"tracked\" and \"flood\" use its tagged faces");
        }
        if (json_params["use_legacy_code"]) {
            log_and_throw_error(
                "use_legacy_code: not with a tagged mesh input, whose labels the legacy "
                "optimizer does not keep");
        }
        if (tagged_mesh->F.empty()) {
            log_and_throw_error(
                "a tagged mesh input without tagged faces: tetwild holds a tracked surface, and "
                "the mesh has none");
        }
    }
    // The `envelope` input: a mesh of the call on the memory route, a mesh file on the file route
    // (relative to input_dir), read by the shared reader, as simwild's. The reference the first
    // component of a chain held its tags to, for this run to hold its own to.
    const std::string envelope_path =
        component_io.memory() ? "" : json_params["envelope"].get<std::string>();
    std::optional<MeshData> envelope_given;
    if (!envelope_selector.is_null() || !envelope_path.empty()) {
        if (!tagged) log_and_throw_error("`envelope` applies to a tagged mesh input only");
        envelope_given = envelope_path.empty()
                             ? component_io.mesh(envelope_selector.get<std::string>())
                             : io::read_mesh(resolve_path(root, envelope_path));
        // Held to as given, label by label, as simwild holds one (TetOptimizerMesh's reference):
        // checked exactly, so not with the sampled envelope; envelope_reference "simplified" has
        // no simplification to build around on this route and is refused below.
        if (!io::is_labelled_envelope(*envelope_given, /*is_3d=*/true)) {
            log_and_throw_error(
                "`envelope`: a tagged mesh input is held to a labelled reference, as tetwild, "
                "triwild and simwild hand theirs on, not to bare geometry");
        }
        if (use_sample_envelope) {
            log_and_throw_error(
                "`envelope`: a reference is held to and checked against exactly, not with "
                "use_sample_envelope");
        }
    }
    // The labels of the output's tags, the same on both routes. Of the surface route: the tracked
    // faces of each input surface and the tet edges tiling each input curve, named by their
    // inputs' labels (above), as triwild's and simwild's tags name an input. Of the tagged route:
    // the mesh's own labels, so that they leave as they came.
    const std::vector<std::string> face_tags = tagged ? tagged_mesh->face_names : surface_labels;
    const std::vector<std::string> feature_tags =
        tagged ? tagged_mesh->feature_names : curve_labels;
    // filter_inputs: the inputs that bound the domain a filter keeps, by their labels (bit i:
    // face_tags[i]). Every input by default, as before the key.
    InputSet bounding = ~InputSet(0);
    const auto filter_inputs = json_params["filter_inputs"].get<std::vector<std::string>>();
    if (!filter_inputs.empty()) {
        if (filter_option == "none") {
            log_and_throw_error("filter_inputs: with filter \"none\" nothing is filtered");
        }
        if (json_params["use_legacy_code"] && filter_option != "input") {
            log_and_throw_error(
                "filter_inputs: filter \"{}\" not with use_legacy_code, which keeps no face's "
                "inputs",
                filter_option);
        }
        bounding = 0;
        for (const std::string& name : filter_inputs) {
            const auto it = std::find(face_tags.begin(), face_tags.end(), name);
            if (it == face_tags.end()) {
                log_and_throw_error(
                    "filter_inputs: {} is not a surface input (the surface inputs: {})",
                    name,
                    face_tags);
            }
            bounding |= InputSet(1) << (it - face_tags.begin());
        }
        logger().info("filter \"{}\": the domain bounded by {} only", filter_option, filter_inputs);
    }

    history::SurfaceHistory input_history; // the files as read, welded, simplified
    history::CurveHistory curve_input_history; // the feature curve files as read

    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::vector<int> tri_input; // the input each of `tris` came from
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    std::vector<size_t> modified_nonmanifold_v;
    // --- phase timing (TETWILD_PHASES) ---
    igl::Timer phase_timer;
    double t_load = 0, t_simplify = 0, t_optimize = 0, t_finalize = 0, t_output = 0;
    phase_timer.start();
    // What both routes leave to the optimization and the output: the feature curves and points
    // held, the reference handed on (`envelope`), the mesh, the chain boundary of the oriented
    // surface it starts with (see log_orientation_check below), and the time from the insertion
    // or the initialization from a tagged mesh on.
    utils::FeatureInputs features;
    const std::vector<Vector3d>& feature_edge_vertices = features.edge_vertices;
    const std::vector<std::array<size_t, 2>>& feature_edges = features.edges;
    const std::vector<Vector3d>& feature_points = features.points;
    MeshData envelope_out;
    std::unique_ptr<tetwild::TetWildMesh> mesh_owner;
    size_t inserted_boundary = 0;
    double insertion_time = 0;
    igl::Timer timer;
    // The surface handed to the insertion, checked under preserve_topology below; -1: not checked
    // (a tagged mesh input is not inserted).
    int64_t inserted_crossing_pairs = -1, inserted_degenerate_triangles = -1;
    if (!tagged) {
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
            io::ReadProvenance read;
            component_io.triangles(
                input_paths,
                V,
                F,
                remove_duplicate_eps,
                -1,
                &tri_input,
                record_history ? &read : nullptr);
            // Each tracked face records the inputs it covers in max_inputs bits (see
            // SurfaceTagAttributes::input_set()), so a run with more cannot hold every face to its
            // own inputs. Refused here, before any work, rather than answered by the union.
            if (input_paths.size() > size_t(SurfaceTagAttributes::max_inputs)) {
                log_and_throw_error(
                    "{} input surfaces; at most {} are supported",
                    input_paths.size(),
                    SurfaceTagAttributes::max_inputs);
            }
            box_minmax.first = V.colwise().minCoeff();
            box_minmax.second = V.colwise().maxCoeff();
            VF_to_vectors(V, F, verts, tris);
            if (record_history) {
                // The files as they are, then the weld: each vertex renamed to the one it became.
                auto& h = input_history;
                for (Eigen::Index v = 0; v < read.V.rows(); ++v) {
                    h.vertices.push_back(read.V.row(v).transpose());
                }
                h.vertex_alive.assign(h.vertices.size(), 1);
                for (Eigen::Index f = 0; f < read.cells.rows(); ++f) {
                    h.faces.push_back(
                        {{uint64_t(read.cells(f, 0)),
                          uint64_t(read.cells(f, 1)),
                          uint64_t(read.cells(f, 2))}});
                }
                h.face_alive.assign(h.faces.size(), 1);
                history::Edit weld{history::EditKind::relabel, {}, {}, {}};
                for (const int n : read.vertex_map) {
                    weld.v.push_back(n < 0 ? history::kNoVertex : uint64_t(n));
                }
                for (uint64_t v = 0; v < verts.size(); ++v) weld.moves.push_back({v, verts[v]});
                h.edits.push_back(std::move(weld));
            }
        }
        // Feature inputs: an edge mesh to force into the tetrahedralization as tet edges, and
        // points to force in as tet vertices. Both join the bounding box BEFORE eps and the
        // background box derive from it -- they are input geometry, and a feature outside the
        // surface's box must still land inside the triangulated domain.
        // Read and grown over by the shared code simwild uses too (utils::read_feature_inputs()).
        // Each point labelled by its input's name (FeatureInputs::point_names): the mesh's point
        // block.
        io::ReadProvenance curves_read;
        features = utils::read_feature_inputs(
            component_io,
            root,
            curve_inputs,
            point_inputs,
            curve_labels,
            point_labels,
            record_curves ? &curves_read : nullptr);
        // The curves are inserted as read, welded by exact position and not simplified, so the
        // history goes from the inputs through that weld to the insertion's embed.
        if (record_curves) {
            curve_input_history = history::curves_from_features(curves_read, features);
        }
        features.extend_bbox(box_minmax.first, box_minmax.second);

        t_load = phase_timer.getElapsedTime();
        phase_timer.start(); // surface simplification begins

        double diag = (box_minmax.first - box_minmax.second).norm();
        const double envelope_size = params.epsr * diag;
        // Held by pointer so it can be dropped once vsimp/fsimp are extracted below: it is a full
        // TriMesh over the input, with its own envelope and preallocated slots, and nothing reads
        // it after the extraction -- but as a plain local it would outlive the whole optimization.
        auto surf_mesh_owner = std::make_unique<shortest_edge_collapse::ShortestEdgeCollapse>(
            verts,
            NUM_THREADS,
            !use_sample_envelope);
        auto& surf_mesh = *surf_mesh_owner;
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
        // does not justify it: reserving headroom for the optimizer is what the simplification's
        // own, smaller eps does (simplify_envelope_ratio 0.5 then), so the two halvings compounded
        // and the simplification ran at eps/4.
        //
        // Measured on the 15 Thingi10K models that exhausted max_iterations = 80 at
        // stop_energy 10: 14 of 14 run converge, in 10-24 iterations (101954: 80 iterations
        // ending at a degenerate 4.64e16 and 69884 tets -> 13 iterations at 9.83 and 28198
        // tets). Every measured Hausdorff distance stays under eps, the largest at 92% of it,
        // so the contract the envelope exists to enforce is unchanged -- it is now the same
        // number the acceptance check uses.
        const double tet_eps = envelope_size;
        // The simplification's eps is a length of its own, not a fraction of tet_eps: eps_simplify
        // when positive, else eps_simplify_rel of the same diagonal as tet_eps -- the two keys and
        // the rule of simwild's Parameters::init(), now in all three apps. The default, 5e-4 of the
        // diagonal, is 0.5 tet_eps at the default eps_rel, as simplify_envelope_ratio 0.5 was until
        // 2026-11. It does not follow eps_rel: under envelope_reference "input" one at or above
        // tet_eps is refused just below.
        const double simplify_eps = eps_simplify > 0 ? eps_simplify : eps_simplify_rel * diag;
        logger().info(
            "envelope eps: simplification {:.6} ({:.0f}% of tetrahedralisation {:.6})",
            simplify_eps,
            100 * simplify_eps / tet_eps,
            tet_eps);
        // Which geometry every envelope, tube and anchor is built around: the input, or the
        // simplified surface (see below, where they are built). Refused here, before any work:
        // around the input, a simplification that may leave the envelope, since the simplified
        // surface would start outside what it is held to; around the simplified surface, no
        // simplification.
        const bool env_around_simplified = json_params["envelope_reference"] == "simplified";
        wmtk::utils::check_envelope_reference(
            tet_eps,
            simplify_eps,
            env_around_simplified,
            /*simplification_ran=*/simplify);

        // The open boundary. Frozen, the simplification cannot touch anything enclosed by boundary
        // loops packed too tightly to leave a free vertex between them, and on Thingi10K 55928 that
        // is a 0.02-wide crumpled knot holding every boundary edge of the model: it survives as a
        // non-manifold tangle, the arrangement cuts it into ~460 sealed cells, and the optimizer --
        // refusing 5.3M collapses there in its first pass, on inversion and quality alone --
        // leaves ~570 tets with edges under 0.2% of the target length in one spot.
        //
        // With the envelope the boundary may move as long as it stays within a tube around the
        // input's boundary edges, the simplification counterpart of the optimizer's order-2
        // envelope and scaled the same way: order2_envelope_ratio of the simplification's eps,
        // where the optimizer's is of the surface eps, for the same headroom reason as above. The
        // optimizer's tube is built around the input's own boundary at order2_envelope_ratio * eps,
        // so a boundary of the input drifts at most that from it; under envelope_reference
        // "simplified" around the simplified one at the same radius. Until 2026-10 it was built
        // around the simplified boundary at the full eps: order2_envelope_ratio * (eps +
        // simplify_eps), 1.5 eps at the defaults.
        //
        // Not under preserve_topology: a hole narrower than the tube closes.
        const double simplify_boundary_eps = simplify_boundary_envelope && !params.preserve_topology
                                                 ? simplify_eps * params.order2_envelope_ratio
                                                 : 0.0;
        if (simplify_boundary_eps > 0) {
            logger().info("simplification boundary envelope eps {:.6}", simplify_boundary_eps);
        } else {
            logger().info("simplification freezes the open boundary");
        }

        surf_mesh.create_mesh(
            verts.size(),
            tris,
            modified_nonmanifold_v,
            simplify_eps,
            simplify_boundary_eps,
            tri_input,
            params.per_input_envelopes);
        assert(surf_mesh.check_mesh_connectivity_validity());

        if (simplify) {
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
            // Envelope::init builds both the exact structure and the sampled BVH, and
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

            // Without preserve_topology the simplification is free to change the topology: with the
            // link condition off it can create non-manifold edges and vertices, and with the
            // boundary envelope it closes holes and slits narrower than the tube. (This used to
            // warn that it "still preserves topology", which neither has been true of.)
            if (!params.preserve_topology) {
                logger().info(
                    "simplification may change the surface topology (link condition off, open "
                    "boundary {})",
                    simplify_boundary_eps > 0 ? "in a tube" : "frozen");
            }
            std::shared_ptr<history::Recorder> simplification;
            if (record_history) {
                simplification = surf_mesh.start_history(
                    [&surf_mesh](size_t v) { return surf_mesh.vertex_attrs.at(v).pos; });
            }
            surf_mesh.collapse_shortest(0);

            surf_mesh.m_envelope.use_exact = saved_use_exact;
            surf_mesh.consolidate_mesh();
            if (simplification) {
                surf_mesh.stop_history();
                if (!history::append(input_history, simplification->history())) {
                    log_and_throw_error(
                        "history_output: the simplification does not start at the input");
                }
            }
            if (!surf_mesh.check_mesh_connectivity_validity()) {
                log_and_throw_error("Simplification produced an invalid mesh connectivity.");
            }
            logger().info(
                "simplified: #v = {}, #f = {}",
                surf_mesh.get_vertices().size(),
                surf_mesh.get_faces().size());
        } else {
            logger().info("no simplification (simplify false)");
        }
        if (!component_io.memory())
            surf_mesh.write_triangle_mesh(output_path + "_simplified_input.obj");


        //// get the simplified input
        std::vector<Eigen::Vector3d> vsimp(surf_mesh.vert_capacity());
        std::vector<std::array<size_t, 3>> fsimp(surf_mesh.tri_capacity());
        for (auto& t : surf_mesh.get_vertices()) {
            auto i = t.vid(surf_mesh);
            vsimp[i] = surf_mesh.vertex_attrs[i].pos;
        }

        // Which input each simplified face came from: the arrangement's provenance names input
        // triangles, and the tracked faces it tiles them with are held to their inputs' envelopes.
        // -1 for a slot no face fills.
        std::vector<int> fsimp_input(fsimp.size(), -1);
        for (auto& t : surf_mesh.get_faces()) {
            auto i = t.fid(surf_mesh);
            auto vs = surf_mesh.oriented_tri_vertices(t);
            for (int j = 0; j < 3; j++) {
                fsimp[i][j] = vs[j].vid(surf_mesh);
            }
            fsimp_input[i] = surf_mesh.face_attrs[i].input;
        }
        surf_mesh_owner.reset(); // `surf_mesh` dangles from here on; nothing below may use it

        // Under preserve_topology, one exact check of the surface handed to the insertion: does it
        // cross itself? The link condition keeps the simplification's connectivity only, and the
        // envelope admits a fold of the surface through itself; the insertion then cuts the
        // crossing triangles along their intersection, which changes the topology. Not prevented,
        // warned about. Measured (2026-11) on the Octocat at 8 threads: about 1 run in 3 to 5 ended
        // with the Euler characteristic [2] -> [3], [3] right after the insertion already, and in 4
        // of 4 such runs the simplified surface had 2 to 5 crossing pairs in one fold; 0 in 7 of 7
        // runs that kept [2], and in the input. With this check, 10 more runs: 2 with 3 crossing
        // pairs, both [3]; 8 with 0, all [2]. It costs 3 ms on the Octocat's 7.8k simplified
        // triangles at 8 threads (12 ms serial) and 15 ms on the bunny's 32k (62 ms), against 4 s
        // and 12 s of simplification.
        if (params.preserve_topology) {
            logger().info("preserve_topology: checking the surface handed to the insertion");
            const auto crossings =
                shortest_edge_collapse::count_crossings(vsimp, fsimp, NUM_THREADS);
            inserted_crossing_pairs = crossings.pairs;
            inserted_degenerate_triangles = crossings.degenerate;
            if (crossings.pairs > 0 || crossings.degenerate > 0) {
                logger().warn(
                    "preserve_topology: the surface handed to the insertion crosses itself: {} "
                    "pairs of triangles meet outside their shared vertices and edges, and {} "
                    "triangles are degenerate. The output's topology may then differ from the "
                    "input's.",
                    crossings.pairs,
                    crossings.degenerate);
            } else {
                logger().info(
                    "preserve_topology: the surface handed to the insertion does not cross itself");
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
        // envelope_reference "input", the default: the ORIGINAL input at the full tet eps, like the
        // simplification one but wider.
        //
        // envelope_reference "simplified": the SIMPLIFIED surface, which replaces the input, at the
        // full tet eps too: the output is within tet_eps of it, so within simplify_eps + tet_eps of
        // the input. The geometry starts at the CENTRE of the envelope it is judged against rather
        // than somewhere inside it. That matters because the envelope is a hard veto, not a
        // penalty: a surface handed over already close to the boundary has most of its moves
        // refused, and deliberately starving that headroom (a simplification at 0.95 eps,
        // simplify_envelope_ratio 0.95 until 2026-11) was enough to turn a converging run into a
        // diverging one on 1368052. Until 2026-11 it was built at the remaining tolerance,
        // tet_eps - simplify_eps, keeping the output within tet_eps of the input.

        // The reference handed on (io::append_envelope_block()): that surface, each triangle with
        // its input's face tag; the feature curves, which every tube is built around as read, each
        // segment with its file's tag; the input points.
        auto tet_envelope = std::make_shared<wmtk::Envelope>(!use_sample_envelope);
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
            tet_envelope->init(env_V, env_F, tet_eps);
            const std::vector<int>& env_input = env_around_simplified ? fsimp_input : tri_input;
            std::vector<InputSet> F_sets(env_F.size(), 0);
            for (size_t i = 0; i < env_F.size() && i < env_input.size(); ++i) {
                if (env_input[i] >= 0) F_sets[i] = InputSet(1) << env_input[i];
            }
            io::append_envelope_block(
                envelope_out,
                io::geometry_data(env_V, env_F, CellType::Triangle),
                CellType::Triangle,
                F_sets,
                face_tags);
            std::vector<InputSet> E_sets;
            for (const int file : features.edge_input) {
                E_sets.push_back(file < SurfaceTagAttributes::max_inputs ? InputSet(1) << file : 0);
            }
            io::append_envelope_block(
                envelope_out,
                io::geometry_data(features.edge_vertices, features.edges, CellType::Edge),
                CellType::Edge,
                E_sets,
                feature_tags);
            io::append_envelope_points(
                envelope_out,
                features.points,
                features.point_input,
                features.point_names);
        }

        // One envelope per input, around the same surface as tet_envelope and at the same eps, with
        // the same predicate: what each tracked face is held to, through its input set
        // (TetOptimizerMesh::surface_envelope_for_face). tet_envelope stays the union for the uses
        // that need one -- the collapse's point prefilter, the order-2 and feature fallbacks, the
        // finalization. With one input the member would BE tet_envelope, so none is built and every
        // face is held to tet_envelope exactly as before.
        std::vector<std::shared_ptr<wmtk::Envelope>> input_envelopes;
        if (input_paths.size() > 1 && params.per_input_envelopes) {
            const std::vector<Eigen::Vector3d>& src_V = env_around_simplified ? vsimp : verts;
            const std::vector<std::array<size_t, 3>>& src_F = env_around_simplified ? fsimp : tris;
            const std::vector<int>& src_input = env_around_simplified ? fsimp_input : tri_input;
            for (size_t k = 0; k < input_paths.size(); ++k) {
                // This input's triangles over its own vertices only: an envelope keeps a copy of
                // the vertices it is given.
                std::vector<int64_t> local(src_V.size(), -1);
                std::vector<Eigen::Vector3d> V_k;
                std::vector<Eigen::Vector3i> F_k;
                for (size_t i = 0; i < src_F.size(); ++i) {
                    if (src_input[i] != int(k)) continue;
                    Eigen::Vector3i f;
                    for (int j = 0; j < 3; ++j) {
                        int64_t& l = local[src_F[i][j]];
                        if (l < 0) {
                            l = int64_t(V_k.size());
                            V_k.push_back(src_V[src_F[i][j]]);
                        }
                        f[j] = int(l);
                    }
                    F_k.push_back(f);
                }
                if (F_k.empty()) {
                    input_envelopes.push_back(nullptr); // no triangle, so no face to hold
                    continue;
                }
                auto env = std::make_shared<wmtk::Envelope>(!use_sample_envelope);
                env->init(V_k, F_k, tet_eps);
                input_envelopes.push_back(std::move(env));
            }
            logger().info(
                "per-input envelopes: {} inputs, each tracked face held to the envelopes of its "
                "own",
                input_envelopes.size());
        }

        logger().info(
            "tetrahedralisation envelope: {} (eps {:.6}) around the {}",
            tet_envelope->use_exact ? "EXACT" : "sampled",
            tet_envelope->eps(), // until 2026-11 sqrt(eps2), the sampled path's 0.42 eps
            env_around_simplified ? "SIMPLIFIED surface" : "input");

        if (json_params["DEBUG_disable_envelope"]) {
            logger().warn(
                "DEBUG_disable_envelope: envelope checks are OFF for the tetrahedralisation. "
                "The output has no containment guarantee and is for diagnosis only.");
            tet_envelope->disabled = true;
            for (const auto& env : input_envelopes) {
                if (env) env->disabled = true;
            }
        }

        // The background mesh exists only to be cut by the insertion; held by pointer so it can be
        // dropped as soon as the insertion has produced its output (see below).
        auto background_owner =
            std::make_unique<tetwild::TetWildMesh>(params, tet_envelope, NUM_THREADS);
        auto& mesh = *background_owner;

        /////////////////////////////////////////////////////

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
        // The input's orientation on the tracked faces, so the tracked-surface winding number
        // (filter "tracked", and the winding_number_tracked output field) is the input's own
        // rather than a guess. See SurfaceTagAttributes::m_orientation.
        std::vector<int8_t> tet_face_orientation;

        logger().info("simplified: #v = {}, #f = {}", vsimp.size(), fsimp.size());

        // The chain boundary of the surface handed to the insertion, for the orientation check
        // below: edges whose incident triangles do not cancel -- open, non-manifold or folded. The
        // tracked surface reproduces it, so it may have boundary exactly where this does. A closed
        // input can still get some here: a simplification that does not preserve topology can open
        // the surface or make it non-manifold.
        if (params.perform_sanity_checks) {
            std::map<std::array<size_t, 2>, int> net;
            for (const auto& f : fsimp) {
                for (int j = 0; j < 3; ++j) {
                    const size_t a = f[j], b = f[(j + 1) % 3];
                    if (a != b) net[{{std::min(a, b), std::max(a, b)}}] += a < b ? 1 : -1;
                }
            }
            for (const auto& [e, sum] : net) inserted_boundary += sum != 0;
        }

        igl::Timer insertion_timer;
        insertion_timer.start();

        // Exact arrangement of the simplified surface against a Delaunay background
        // mesh; the remesher's own tets are used directly, so no Steiner points.
        utils::EmbedFeaturesResult features_out;
        // With several inputs, each tracked face's inputs are taken from the arrangement whether or
        // not each is held to its own envelopes (per_input_envelopes): they are also the output's
        // face tags, one per input. As triwild takes its segments'. Until 2026-11 only with
        // per_input_envelopes, and without it every tracked face was input 0's.
        const bool per_input = input_paths.size() > 1;
        std::vector<InputSet> facet_input_set; // per facet, with several inputs only
        std::vector<std::vector<uint32_t>> facet_sources;
        // Per facet, each input's own orientation, with several inputs in a build that keeps one
        // count per input (SurfaceTagAttributes::m_input_orientation).
        std::vector<InputOrientations> facet_input_orientation;
        const bool per_input_orientation = per_input && SurfaceTagAttributes::per_input_orientation;
        mesh.insertion_by_volumeremesher(
            vsimp,
            fsimp,
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation,
            feature_edge_vertices,
            feature_edges,
            feature_points,
            &features_out,
            per_input ? &fsimp_input : nullptr,
            per_input ? &facet_input_set : nullptr,
            record_history ? &facet_sources : nullptr,
            per_input_orientation ? &facet_input_orientation : nullptr);

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
        background_owner.reset(); // `mesh` dangles from here on; nothing below may use it

        // generate new mesh
        mesh_owner = std::make_unique<tetwild::TetWildMesh>(params, tet_envelope, NUM_THREADS);
        auto& mesh_new = *mesh_owner;
        // Where each facet on the input lies there, by the positions its corners will have.
        for (size_t i = 0; i < facet_sources.size(); ++i) {
            if (facet_sources[i].empty()) continue;
            const auto& f = facets[i];
            mesh_new.m_insertion_sources.emplace_back(
                TetWildMesh::position_key(
                    to_double(v_rational[f[0]]),
                    to_double(v_rational[f[1]]),
                    to_double(v_rational[f[2]])),
                std::move(facet_sources[i]));
        }
        facet_sources = {};
        // Where each tet edge tiling a feature curve lies on the input segments, by the positions
        // its ends will have.
        if (record_curves) {
            mesh_new.set_insertion_curve_sources(features_out, [&v_rational](size_t v) {
                return to_double(v_rational[v]);
            });
        }
        // The wn_ fields are named after the inputs when input_names names them, else numbered.
        if (!json_params["input_names"].empty()) mesh_new.m_input_names = face_tags;
        // The input points' labels, which their anchors keep and the output writes as its point
        // block (io::collect_point_tags()).
        mesh_new.m_point_names = features.point_names;
        mesh_new.m_input_point_labels = features.point_input;

        // Around the input after a simplification, the input's own curves: the tubes and anchors
        // are built around them, and an order-2 edge or curve end the simplification made is held
        // around itself (TetOptimizerMesh::InputCurves). The input surface as the envelope has it,
        // each triangle with its input.
        if (!env_around_simplified && simplify) {
            assert(tri_input.size() == tris.size());
            std::vector<InputSet> tri_sets(tris.size());
            for (size_t i = 0; i < tris.size(); ++i) tri_sets[i] = InputSet(1) << tri_input[i];
            mesh_new.set_input_curves(verts, tris, tri_sets, &features);
        }

        const bool has_features = !feature_edges.empty() || !feature_points.empty();
        // The feature-curve tubes -- what the collapse guard checks tagged edges against, and what
        // curve-vertex smoothing is pulled toward -- are built with the tags, around the input
        // feature edges and one per input curve (TetOptimizerMesh::tag_feature_edges()),
        // following the surface envelope's choice of predicate.
        mesh_new.init_from_Volumeremesher(
            v_rational,
            facets,
            is_v_on_input,
            tets,
            tet_face_on_input_surface,
            &tet_face_orientation,
            has_features ? &features_out : nullptr,
            per_input ? &facet_input_set : nullptr,
            &features,
            per_input_orientation ? &facet_input_orientation : nullptr);
        mesh_new.m_input_envelopes.set(std::move(input_envelopes));

        // The insertion's output now lives in mesh_new. These are function locals, so without this
        // they -- the rational coordinates above all -- would be held through the optimization,
        // which is where the run peaks.
        std::vector<Vector3r>().swap(v_rational);
        std::vector<std::array<size_t, 3>>().swap(facets);
        std::vector<bool>().swap(is_v_on_input);
        std::vector<std::array<size_t, 4>>().swap(tets);
        std::vector<bool>().swap(tet_face_on_input_surface);
        std::vector<int8_t>().swap(tet_face_orientation);
        std::vector<InputSet>().swap(facet_input_set);
        std::vector<InputOrientations>().swap(facet_input_orientation);
        std::vector<Eigen::Vector3d>().swap(vsimp);
        std::vector<std::array<size_t, 3>>().swap(fsimp);
        std::vector<int>().swap(fsimp_input);

        insertion_time = insertion_timer.getElapsedTime();


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
    } else {
        // The tagged route (read_tagged_input() above): the mesh is the input as it is.
        const TaggedMesh& in = *tagged_mesh;
        features = in.features;
        box_minmax = {in.V[0], in.V[0]};
        for (const Vector3d& p : in.V) {
            box_minmax.first = box_minmax.first.cwiseMin(p);
            box_minmax.second = box_minmax.second.cwiseMax(p);
        }
        // An `envelope` given: the reference held to, its triangles of the mesh's face labels
        // (io::held_rows()), each with its labels, its feature curves of the mesh's feature
        // labels and its points of the mesh's point labels, matched by name.
        std::optional<io::HeldRows> held;
        utils::FeatureInputs reference_curves;
        MatrixXd reference_points;
        std::vector<InputSet> reference_point_labels;
        if (envelope_given) {
            const auto bits_named = [&](const std::string& name) {
                InputSet bits = 0;
                for (size_t i = 0; i < face_tags.size(); ++i)
                    if (face_tags[i] == name) bits |= InputSet(1) << i;
                return bits;
            };
            io::LabelledReference reference = io::read_labelled_envelope(
                *envelope_given,
                3,
                bits_named,
                feature_tags,
                features.point_names);
            held = io::held_rows(reference.V, reference.F, reference.F_labels);
            reference_curves = std::move(reference.curves);
            reference_points = std::move(reference.P);
            reference_point_labels = std::move(reference.P_labels);
        }
        // The input, as the surface route's is its input files: the reference when one is
        // given, else the tracked surface as received, on vertices of its own. What the envelopes
        // are built around, and what the output is measured against (DEBUG_hausdorff, the Euler
        // characteristics); F_sets, each triangle's labels.
        std::vector<InputSet> F_sets;
        if (held) {
            for (Eigen::Index v = 0; v < held->V.rows(); ++v) {
                verts.push_back(held->V.row(v).transpose());
            }
            for (Eigen::Index r = 0; r < held->F.rows(); ++r) {
                tris.push_back(
                    {{size_t(held->F(r, 0)), size_t(held->F(r, 1)), size_t(held->F(r, 2))}});
            }
            F_sets = held->labels;
            // The curves the output's are measured against: the reference's.
            features.edge_vertices = reference_curves.edge_vertices;
            features.edges = reference_curves.edges;
            features.edge_input = reference_curves.edge_input;
        } else {
            std::vector<int64_t> local(in.V.size(), -1);
            for (const auto& f : in.F) {
                std::array<size_t, 3> g;
                for (int k = 0; k < 3; ++k) {
                    int64_t& l = local[f[k]];
                    if (l < 0) {
                        l = int64_t(verts.size());
                        verts.push_back(in.V[f[k]]);
                    }
                    g[k] = size_t(l);
                }
                tris.push_back(g);
            }
            F_sets = in.F_sets;
        }
        t_load = phase_timer.getElapsedTime();
        logger().info("tagged mesh input: its tets kept, no simplification and no insertion");

        // eps_rel and length_rel are of the mesh's bounding box diagonal, its domain's, as
        // simwild's are of a tagged mesh's.
        params.init(box_minmax.first, box_minmax.second);
        const double tet_eps = params.eps;
        const double feature_eps = params.epsr * params.diag_l * params.feature_envelope_ratio;
        wmtk::utils::check_envelope_reference(
            tet_eps,
            0,
            json_params["envelope_reference"] == "simplified",
            /*simplification_ran=*/false);
        std::vector<Eigen::Vector3i> env_F(tris.size());
        for (size_t i = 0; i < tris.size(); ++i) {
            env_F[i] << int(tris[i][0]), int(tris[i][1]), int(tris[i][2]);
        }
        auto tet_envelope = std::make_shared<wmtk::Envelope>(!use_sample_envelope);
        tet_envelope->init(verts, env_F, tet_eps);
        // The reference handed on: an `envelope` given, unchanged, so that a chain keeps passing
        // the first component's input; else the tracked surface as received, each triangle with
        // its labels, and the feature curves and points, as the surface route hands on its input.
        if (envelope_given) {
            envelope_out = std::move(*envelope_given);
        } else {
            io::append_envelope_block(
                envelope_out,
                io::geometry_data(verts, env_F, CellType::Triangle),
                CellType::Triangle,
                F_sets,
                face_tags);
            std::vector<InputSet> E_sets;
            for (const int j : features.edge_input) E_sets.push_back(InputSet(1) << j);
            io::append_envelope_block(
                envelope_out,
                io::geometry_data(features.edge_vertices, features.edges, CellType::Edge),
                CellType::Edge,
                E_sets,
                feature_tags);
            io::append_envelope_points(
                envelope_out,
                features.points,
                features.point_input,
                features.point_names);
        }
        logger().info(
            "tetrahedralisation envelope: {} (eps {:.6}) around the {}",
            tet_envelope->use_exact ? "EXACT" : "sampled",
            tet_envelope->eps(),
            held ? "reference given (`envelope`)" : "tagged faces as received");

        mesh_owner = std::make_unique<tetwild::TetWildMesh>(params, tet_envelope, NUM_THREADS);
        auto& mesh_new = *mesh_owner;
        mesh_new.m_input_names = face_tags;
        mesh_new.m_feature_edge_names = feature_tags;
        mesh_new.m_point_names = features.point_names; // as on the surface route
        mesh_new.m_input_point_labels = features.point_input;
        // With a reference, as simwild holds one (TetOptimizerMesh::m_has_reference): the order-2
        // tubes and the curve-end anchors around its curves, an edge on none held around itself
        // (TetOptimizerMesh::InputCurves), and the input points' anchors on its points
        // (TetOptimizerMesh::ReferencePoints), set before the initialization builds them.
        if (held) {
            mesh_new.m_has_reference = true;
            mesh_new.m_reference_curves = reference_curves;
            mesh_new.set_input_curves(verts, tris, F_sets, &reference_curves);
            mesh_new.set_reference_points(reference_points, reference_point_labels);
        }
        timer.start();
        igl::Timer init_timer;
        init_timer.start();
        mesh_new.init_from_tagged_mesh(in);
        if (held) mesh_new.require_reference_face_labels(F_sets);
        // One envelope per label, around its triangles, at the same eps and with the same
        // predicate: what each tracked face is held to through its input set, as each input
        // file's on the surface route. With one label the member would be tet_envelope, so none is
        // built.
        if (face_tags.size() > 1) {
            const size_t n =
                mesh_new.init_input_envelopes(verts, env_F, F_sets, tet_eps, !use_sample_envelope);
            if (n > 0) {
                logger().info(
                    "per-label envelopes: {} labels, each tracked face held to the envelopes of "
                    "its own",
                    n);
            }
        }
        // With a reference, the feature tubes around its curves rather than the edges received,
        // and, before the optimization holds the mesh to it, what was received must already lie
        // in it: the exact check simwild's remeshing runs.
        if (held) {
            mesh_new.init_reference_feature_tubes(feature_eps, tet_envelope->use_exact);
            mesh_new.check_reference_containment(tet_eps);
        }
        if (json_params["DEBUG_disable_envelope"]) {
            logger().warn(
                "DEBUG_disable_envelope: envelope checks are OFF for the tetrahedralisation. "
                "The output has no containment guarantee and is for diagnosis only.");
            tet_envelope->disabled = true;
            for (size_t i = 0; i < mesh_new.m_input_envelopes.size(); ++i) {
                if (const auto& env = mesh_new.m_input_envelopes.member(i)) env->disabled = true;
            }
        }
        if (params.perform_sanity_checks && mesh_new.m_tracks_orientation) {
            inserted_boundary = mesh_new.tracked_surface_boundary().size();
        }
        insertion_time = init_timer.getElapsedTime();
        tagged_mesh.reset(); // its arrays now live in mesh_new
    }
    auto& mesh_new = *mesh_owner;

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

    // The oriented tracked surface is an integer 2-chain whose boundary the operations preserve:
    // empty for a closed input, on the input's open boundary otherwise. Cheap to state, and the
    // one check that catches an operation forgetting to carry the orientation.
    const auto log_orientation_check = [&](const char* when) {
        if (!params.perform_sanity_checks || !mesh_new.m_tracks_orientation) return;
        const size_t n_boundary = mesh_new.tracked_surface_boundary().size();
        const size_t n_faces = mesh_new.oriented_tracked_faces().size();
        if (inserted_boundary == 0 && n_boundary > 0) {
            logger().warn(
                "oriented tracked surface {}: {} faces, {} boundary edges although the inserted "
                "surface is closed -- the tracked winding number is not an integer near them",
                when,
                n_faces,
                n_boundary);
        } else {
            logger().info(
                "oriented tracked surface {}: {} faces, {} boundary edges (the inserted surface "
                "has {}: open, non-manifold or folded edges)",
                when,
                n_faces,
                n_boundary,
                inserted_boundary);
        }
    };
    log_orientation_check(tagged ? "as received" : "after insertion");

    mesh_new.consolidate_mesh();

    // The tracked surface's topology right after the insertion, by the rule of the input's and
    // the output's: what the optimization keeps, so with filter "none" the output's.
    std::vector<int> ecs_inserted;
    if (compute_euler) {
        std::vector<Eigen::Vector3i> tracked;
        for (const auto& f : mesh_new.get_faces()) {
            if (!mesh_new.m_face_attribute[f.fid(mesh_new)].m_is_surface_fs) continue;
            const auto vs = mesh_new.get_face_vertices(f);
            tracked.emplace_back(
                int(vs[0].vid(mesh_new)),
                int(vs[1].vid(mesh_new)),
                int(vs[2].vid(mesh_new)));
        }
        Eigen::MatrixXi F(tracked.size(), 3);
        for (size_t i = 0; i < tracked.size(); ++i) F.row(i) = tracked[i];
        ecs_inserted = compute_euler_characteristics(F);
        logger().info("Inserted euler characteristic: {}", ecs_inserted);
    }

    // From here on the tracked surface is recorded, starting where the insertion left it.
    if (record_history) mesh_new.record_surface_history(component_io.history_metadata(json_params));
    if (record_curves) mesh_new.record_curve_history(component_io.history_metadata(json_params));
    // The whole volume around it too, if asked: the map of the volume starts at the tet mesh
    // written here, and ends at the mesh before the filter (<output>_history_output.msh). Both
    // are files next to the output; on the memory route, which has none, the output meshes
    // history_input and history_output.
    MeshMap history_meshes;
    if (!volume_history_output.empty()) {
        if (component_io.memory()) {
            history_meshes["mesh_history_input.msh"] = mesh_new.tet_mesh_data();
        } else {
            io::write_mesh(output_path + "_history_input.msh", mesh_new.tet_mesh_data());
        }
        mesh_new.record_volume_history();
    }

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
        // The open-boundary guarantee, audited: every edge still carrying the order2 tag lies in
        // its tube (tag_order2_edges()). Edges the operations made open carry no tag and are not
        // counted. Not after the legacy optimizer: it rebuilds the mesh, whose open boundaries are
        // tagged again from its output, so there is no input tube to audit against.
        const size_t n_out =
            mesh_new.tagged_edges_outside(TetWildMesh::FeatureEdgeAttributes::order2);
        if (n_out > 0) {
            logger().warn("{} order-2 edges outside their tube", n_out);
        } else {
            logger().info("order-2 edges outside their tube: 0");
        }
    }
    log_orientation_check("after optimization");
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

    // Feature collections, taken BEFORE any filter deletes tets: explicit features are
    // user input, and a filter discarding the region they live in must not erase them from
    // the audits (they do disappear from the tet mesh itself). Same rule in 2D. With
    // filter='none' nothing is deleted and this equals the final state.
    std::vector<std::array<Vector3d, 2>> hybrid_curve_segs;
    std::pair<size_t, size_t> hybrid_retention{0, 0};
    double hybrid_retention_worst = 0;
    const bool features_present = !feature_edges.empty() || !feature_points.empty();
    if (features_present) {
        for (const auto& e : mesh_new.get_edges()) {
            if (!mesh_new.m_feature_edge_attribute[e.eid(mesh_new)].has(
                    TetWildMesh::FeatureEdgeAttributes::feature)) {
                continue;
            }
            hybrid_curve_segs.push_back(
                {{mesh_new.m_vertex_attribute[e.vid(mesh_new)].m_posf,
                  mesh_new.m_vertex_attribute[e.switch_vertex(mesh_new).vid(mesh_new)].m_posf}});
        }
        hybrid_retention = mesh_new.feature_retention(&hybrid_retention_worst);
    }

    // The finalization's per-tet annotations, each computed only when something uses it:
    //
    //   input winding numbers  the whole input's, which filter "input" uses, and each input's
    //                          (the wn_<i> output fields). When a filter needs them, or with
    //                          skip_winding_number false (it defaults to true).
    //   tracked winding number filter "tracked", or compute_tracked_winding_number.
    //   flood-fill parts       filter "flood", or compute_flood_fill. A serial BFS over every
    //                          tet: ~1-2 min on 765k tets / 6600 parts.
    //
    // The two last ones used to run on every call just to fill their output fields. A tagged
    // mesh input has no input surfaces, so no input winding numbers.
    //
    // With filter_inputs, the one the filter reads -- winding_number_input under "input",
    // winding_number_tracked under "tracked", part under "flood" -- is of those inputs only, so
    // that the others do not bound what it keeps; the others are of every input.
    const bool skip_winding =
        tagged || (json_params["skip_winding_number"] && filter_option == "none");
    // skip_winding_number is the default; a filter that needs the winding numbers computes them
    // regardless, so there is nothing to warn about.
    const bool need_tracked =
        filter_option == "tracked" || json_params["compute_tracked_winding_number"];
    const bool need_parts = filter_option == "flood" || json_params["compute_flood_fill"];
    {
        igl::Timer step_timer;
        // Precompute the tets and their barycenters once; every winding-number pass reuses
        // them.
        std::vector<TetWildMesh::Tuple> finalize_tets;
        Eigen::MatrixXd finalize_barycenters;
        if (!skip_winding || need_tracked) {
            finalize_tets = mesh_new.get_tets();
            finalize_barycenters = mesh_new.tet_barycenters(finalize_tets);
        }
        if (!skip_winding) {
            step_timer.start();
            mesh_new.compute_input_winding_numbers(
                finalize_tets,
                finalize_barycenters,
                verts,
                tris,
                tri_input,
                int(input_paths.size()),
                filter_option == "input" ? bounding : ~InputSet(0));
            logger().info(
                "finalize: input winding numbers ({} inputs) {:.4}s",
                input_paths.size(),
                step_timer.getElapsedTime());
        } else {
            logger().info(
                "Skipping the input winding numbers ({})",
                tagged ? "a tagged mesh input has no input surfaces" : "skip_winding_number");
        }
        if (need_tracked) {
            step_timer.start();
            mesh_new.compute_winding_number(
                finalize_tets,
                finalize_barycenters,
                {},
                {},
                filter_option == "tracked" ? bounding : ~InputSet(0));
            logger().info("finalize: tracked winding number {:.4}s", step_timer.getElapsedTime());
        }
        if (need_parts) {
            step_timer.start();
            const int num_parts =
                mesh_new.flood_fill(filter_option == "flood" ? bounding : ~InputSet(0));
            logger().info(
                "finalize: flood fill, {} parts {:.4}s",
                num_parts,
                step_timer.getElapsedTime());
        }
    }

    // The mesh the volume history ends at: all of it, before the filter drops a part. Both
    // routes consolidate it here, so they record the same histories.
    if (!volume_history_output.empty()) {
        if (component_io.memory()) {
            history_meshes["mesh_history_output.msh"] = mesh_new.tet_mesh_data();
        } else {
            io::write_mesh(output_path + "_history_output.msh", mesh_new.tet_mesh_data());
        }
    }

    // ////winding number
    if (filter_option == "input") {
        mesh_new.filter_with_input_surface_winding_number();
    } else if (filter_option == "tracked") {
        mesh_new.filter_with_tracked_surface_winding_number();
    } else if (filter_option == "flood") {
        mesh_new.filter_with_flood_fill();
    } else if (filter_option != "none") {
        logger().error("Unknown filter option '{}'. No filtering performed.", filter_option);
    }
    mesh_new.consolidate_mesh();
    // A filter removes the tracked faces of the tets it drops, which no operation maps. A tagged
    // mesh input's history starts at the mesh as received.
    if (const auto h = mesh_new.finish_surface_history(tagged ? nullptr : &input_history)) {
        component_io.save_history(*h, history_output);
    }
    if (const auto vh = mesh_new.finish_volume_history()) {
        component_io.save_volume_history(*vh, volume_history_output);
    }
    // As the surface's: a filter removes the tracked edges of the tets it drops.
    if (const auto ch = mesh_new.finish_curve_history(tagged ? nullptr : &curve_input_history)) {
        component_io.save_curve_history(*ch, curve_history_output);
    }

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

            Envelope env_in(true);
            env_in.init(V, F, params.eps);

            Envelope env_out(true);
            env_out.init(matV, matF, params.eps);

            // Max distance from points sampled on (Vs,Fs) to the surface behind `target`.
            const auto max_deviation =
                [&](const Eigen::MatrixXd& Vs, const Eigen::MatrixXi& Fs, const Envelope& target) {
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
            // A tagged mesh's curves are an input edge per edge and label (read_tagged_mesh()):
            // an edge of two labels is one edge of the network.
            std::set<std::array<size_t, 2>> network;
            for (const auto& e : feature_edges) {
                network.insert({{std::min(e[0], e[1]), std::max(e[0], e[1])}});
            }
            ecs_curves_in = curve_euler_characteristics(
                tagged ? std::vector<std::array<size_t, 2>>(network.begin(), network.end())
                       : feature_edges);
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
            // discarded region, but the features were preserved up to that point.
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
    if (component_io.memory() || !report_file.empty()) {
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
        report["inserted_euler_characteristic"] = ecs_inserted;
        report["output_euler_characteristic"] = ecs_output;
        report["inserted_crossing_pairs"] = inserted_crossing_pairs;
        report["inserted_degenerate_triangles"] = inserted_degenerate_triangles;
        report["insertion_and_preprocessing"] = insertion_time;
        component_io.result.second = report;
        if (!component_io.memory()) {
            std::ofstream fout(report_file);
            fout << std::setw(4) << report;
        }
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

    // One result for both routes, as triwild's: returned from memory calls, written as
    // <output>_final.msh otherwise. The tets with their fields (save_paraview()), and the output's
    // tags (face_tags, feature_tags, above), each with its input set, the faces oriented as their
    // input: the tracked surface its triangle block, the feature curves and the input points its
    // edge and point blocks. Nothing derived from the mesh: until 2026-11 the tracked surface and
    // the curves were also written as _surface.obj and _features.obj. Until 2026-11 too the file
    // had a writer of its own (TetWildMesh::output_mesh()), which named two fields apart from the
    // memory route (energy and sizing for t_energy and sizing_field) and wrote none of the vertex
    // flags (is_rounded, on_surface, on_open_boundary, order).
    {
        MeshMap raw;
        mesh_new.save_paraview("mesh", false, &raw);
        io::compact_slot_outputs(mesh_new, raw);
        component_io.result.first = io::named_outputs(std::move(raw));
        io::collect_input_set_tags<CellType::Triangle>(
            mesh_new,
            component_io.result.first,
            face_tags);
        io::collect_feature_edge_tags(mesh_new, component_io.result.first, feature_tags);
        io::collect_point_tags(mesh_new, component_io.result.first, mesh_new.m_point_names);
    }
    if (!component_io.memory()) {
        // The .msh numbers each input's winding-number field, wn_0, wn_1, ..., as triwild's .msh
        // does; the memory route and the VTU name it after the input (save_paraview()). All taken
        // out before any is put back, so that a name which is a number cannot overwrite another
        // input's field.
        MeshData msh = component_io.result.first.at("mesh");
        auto& fields = msh.cells.at(CellType::Tetrahedron).attributes;
        std::vector<std::pair<std::string, MeshAttribute>> numbered;
        for (size_t i = 0; i < mesh_new.m_input_names.size(); ++i) {
            auto field = fields.extract("wn_" + mesh_new.m_input_names[i]);
            if (field) {
                numbered.emplace_back("wn_" + std::to_string(i), std::move(field.mapped()));
            }
        }
        for (auto& [name, field] : numbered) fields[name] = std::move(field);
        io::write_mesh(output_path + "_final.msh", msh);
        // The reference the tags were held to, for the next component (the memory route's
        // outputs["envelope"]), in a file of its own: _final.msh is read as a mesh by gmsh and
        // polyfem.
        io::write_mesh(output_path + "_envelope.msh", envelope_out);
    }

    wmtk::logger().info("======= finish =========");

    if (component_io.memory()) {
        component_io.result.first.merge(io::named_outputs(std::move(history_meshes)));
        component_io.result.first["envelope"] = std::move(envelope_out);
        return {}; // the memory overload returns component_io.result instead
    }
    return mesh_new.export_mesh_data();
}


void tetwild(nlohmann::json json_params)
{
    io::ComponentIO context;
    auto e = run(std::move(json_params), context);
    logger().info("V ({},{})", e.V.rows(), e.V.cols());
    logger().info("T ({},{})", e.T.rows(), e.T.cols());
    logger().info("F ({},{})", e.F.rows(), e.F.cols());
}
TetWildMesh::ExportStruct tetwild_with_export(nlohmann::json json_params)
{
    io::ComponentIO context;
    return run(std::move(json_params), context);
}
ComponentResult tetwild(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return context.take_result();
}
} // namespace wmtk::components::tetwild
