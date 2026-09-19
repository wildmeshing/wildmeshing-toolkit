#include "polyfem_ops.hpp"

#include "ConstraintMatrices.hpp"
#include "DeformedMesh.hpp"
#include "Hdf5Writers.hpp"
#include "InterfaceSelection.hpp"
#include "MeshReduction.hpp"
#include "OuterLoops.hpp"
#include "PolyfemJson.hpp"
#include "PolyfemRunner.hpp"
#include "TaggedMesh.hpp"

#include <jse/jse.h>
#include <laplacian_smoothing_spec.hpp>
#include <minimum_separation_spec.hpp>
#include <polyfem_ops_spec.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <optional>
#include <set>

namespace wmtk::components::polyfem_ops {

namespace {

/// The operation's own parameters: everything the dispatcher does not own. `application` and
/// `operation` pick the code path, `inputs_only` is a C++-engine switch with no counterpart in
/// pysimwild, and `input_dir` is injected into every job by the JSON app (app/main.cpp) -- none of
/// the four may be offered to the pysimwild spec, which is strict and knows none of them.
///
/// `input_dir` is dropped rather than used to resolve relative paths, which is what the other
/// components do with it: pysimwild opens `input` and `output` exactly as written, and resolving
/// them here would put the generated files somewhere else than the Python engine puts them.
nlohmann::json operation_params(const nlohmann::json& json_params)
{
    nlohmann::json out = json_params;
    out.erase("application");
    out.erase("operation");
    out.erase("inputs_only");
    out.erase("input_dir");
    return out;
}

/// Translate a pysimwild spec.json into jse's dialect. The two validators agree on the rule
/// format except in two places, and the pysimwild files are the shared source of the rules --
/// they are what the Python engine (the oracle this port is checked against) validates with --
/// so the translation happens here and those files are never edited.
///
///  1. "type" as a list, e.g. ["string", "object"] on a selection, which is a bare region string
///     or a {region, filter, id} object. simwild.polyfem_ops.spec accepts the list; jse reads
///     "type" as a single string (jse.cpp, JSE::verify_rule) and throws on a list. jse's
///     equivalent is to repeat the pointer once per type: it requires exactly one rule with that
///     pointer to verify, and a string input can only satisfy the string copy, an object input
///     only the object copy.
///  2. An optional key whose own rule carries no "default". simwild.polyfem_ops.spec leaves such
///     a key simply absent; jse insists every optional key declares exactly one default and
///     spells "leave it absent" as the default "skip". Injecting anything else would be wrong:
///     a selection with no `filter` keeps its whole boundary, and any injected value would
///     silently narrow it.
///  3. A free-form object: `/amips_weights` is an object rule with no required/optional list and
///     one `/amips_weights/*` rule for its values, which simwild.polyfem_ops.spec reads as "any
///     key, each checked against the * rule". jse reads a missing optional list the other way --
///     no key is allowed -- and in strict mode refuses every non-empty dict (jse.cpp,
///     verify_rule_object via find_extra_keys), and its pointers match exactly, so a * rule is
///     never consulted for a concrete key. jse's equivalent therefore has to NAME the keys: the
///     ones the caller actually passed become the optional list and each gets its own copy of the
///     * rule, which the pass below then gives the "skip" default. This is the only translation
///     step that needs the parameters and not just the spec.
///
/// An optional key with no rule at all cannot be translated -- there is nothing to attach the
/// default to -- and jse rejects it. /protected_regions/*/region and /axes were in that state
/// until the pysimwild spec gained rules for them; every optional key reachable here now has one.
nlohmann::json to_jse_spec(const nlohmann::json& spec, const nlohmann::json& params)
{
    nlohmann::json out = nlohmann::json::array();
    for (const auto& rule : spec) {
        if (rule.contains("type") && rule["type"].is_array()) {
            for (const auto& type : rule["type"]) {
                nlohmann::json copy = rule;
                copy["type"] = type;
                out.push_back(copy);
            }
        } else {
            out.push_back(rule);
        }
    }

    // Free-form objects (3): name the caller's own keys. Only a top-level pointer is handled --
    // a deeper one would have to be walked through `params`, and neither spec has one.
    std::vector<nlohmann::json> named_keys;
    for (auto& rule : out) {
        const std::string pointer = rule["pointer"].get<std::string>();
        if (rule.value("type", "") != "object" || rule.contains("required") ||
            rule.contains("optional") || std::count(pointer.begin(), pointer.end(), '/') != 1) {
            continue;
        }
        const auto wildcard = std::find_if(out.begin(), out.end(), [&pointer](const auto& child) {
            return child["pointer"] == pointer + "/*";
        });
        if (wildcard == out.end()) {
            continue;
        }
        rule["optional"] = nlohmann::json::array();
        const auto value = params.find(pointer.substr(1));
        if (value == params.end() || !value->is_object()) {
            continue;
        }
        for (const auto& item : value->items()) {
            rule["optional"].push_back(item.key());
            nlohmann::json copy = *wildcard;
            copy["pointer"] = pointer + "/" + item.key();
            named_keys.push_back(copy);
        }
    }
    for (auto& rule : named_keys) {
        out.push_back(rule);
    }

    std::set<std::string> optional_children;
    std::set<std::string> has_default;
    for (const auto& rule : out) {
        if (rule.contains("default")) {
            has_default.insert(rule["pointer"].get<std::string>());
        }
        if (!rule.contains("optional")) {
            continue;
        }
        std::string base = rule["pointer"].get<std::string>();
        if (base == "/") {
            base.clear();
        }
        for (const auto& key : rule["optional"]) {
            optional_children.insert(base + "/" + key.get<std::string>());
        }
    }
    for (auto& rule : out) {
        const std::string pointer = rule["pointer"].get<std::string>();
        // insert(): only the first copy of a multi-type rule gets the default, because jse
        // requires exactly one defaulting rule per pointer.
        if (optional_children.count(pointer) != 0 && has_default.insert(pointer).second) {
            rule["default"] = "skip";
        }
    }
    return out;
}

/// One of the directories pysimwild puts the generated polyfem inputs and outputs in:
/// `sep_input`/`sep_output` or `smooth_input`/`smooth_output` beside the output stem. Mirrors
/// simwild.py (`out_dir = os.path.dirname(p["output"]) or "."`) followed by
/// minimum_separation.run / laplacian_smoothing.run.
///
/// Canonical, because both run() functions resolve the directory before use and the resolved
/// strings go verbatim into the simulation JSON: on this machine /tmp and /var are symlinks, so
/// an unresolved path would name the same directory by a different name than the Python does.
std::filesystem::path sim_dir(const std::string& output, const std::string& subdir)
{
    std::filesystem::path out_dir = std::filesystem::path(output).parent_path();
    if (out_dir.empty()) {
        out_dir = ".";
    }
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path sim = std::filesystem::canonical(out_dir) / subdir;
    std::filesystem::create_directories(sim);
    return std::filesystem::canonical(sim);
}

/**
 * @brief The protected_pin*.hdf5 files. Mirrors the `protected_regions` block of
 * `minimum_separation.run`.
 *
 * Every node of a matching cell is pinned exactly at rest (a polyfem HARD constraint, held by the
 * augmented Lagrangian) so the contact pushes only the unprotected side. An entry is a bare
 * expression (hold every component) or {"region", "axes"} (hold only those components, leaving
 * the region free to slide along the rest); entries sharing an axes spec go into ONE file, named
 * protected_pin.hdf5 or protected_pin_<axes>.hdf5, because polyfem takes a list of files.
 *
 * The Python's own check for unknown keys inside an entry is not mirrored: jse already rejects
 * them against the same spec, in strict mode, before this runs.
 *
 * Each file is written in inputs_only mode; otherwise its content goes into `memory` under the
 * same path instead.
 *
 * @return the file paths, in the order the Python appends them to `pin_paths`; they go into the
 * simulation JSON as `constraints.hard`.
 */
std::vector<std::string> protected_pins(
    const std::string& input,
    const nlohmann::json& protected_regions,
    const std::filesystem::path& sim_in_dir,
    const bool inputs_only,
    SolveInputs& memory)
{
    if (protected_regions.empty()) {
        return {};
    }
    // The Python reads the mesh dimension off the reduced mesh and loads the original mesh again
    // for the pin nodes; both give the mesh's own dimension, so one load answers both.
    const TaggedMesh mesh_for_pins(input);

    // Insertion-ordered grouping by the parsed axes, as the Python dict is.
    std::vector<std::pair<std::optional<std::vector<int>>, std::vector<std::string>>> groups;
    for (const auto& entry : protected_regions) {
        std::string expr;
        std::optional<std::vector<int>> axes;
        if (entry.is_object()) {
            expr = entry.at("region").get<std::string>();
            axes = parse_axes(
                entry.contains("axes") ? entry["axes"] : nlohmann::json(),
                mesh_for_pins.mesh_dim);
        } else {
            expr = entry.get<std::string>();
        }
        auto it = std::find_if(groups.begin(), groups.end(), [&axes](const auto& g) {
            return g.first == axes;
        });
        if (it == groups.end()) {
            groups.emplace_back(axes, std::vector<std::string>{expr});
        } else {
            it->second.push_back(expr);
        }
    }

    std::vector<std::string> pin_paths;
    for (const auto& [axes, exprs] : groups) {
        const std::vector<int64_t> pin_ids = select_region_nodes(mesh_for_pins, exprs);
        std::string suffix;
        if (axes.has_value()) {
            suffix = "_";
            for (const int a : *axes) suffix += "xyz"[a];
        }
        const std::filesystem::path pin_path =
            sim_in_dir / ("protected_pin" + suffix + ".hdf5");
        ConstraintHdf5 pin = pin_constraint(pin_ids, mesh_for_pins.mesh_dim, axes);
        pin_paths.push_back(std::filesystem::weakly_canonical(pin_path).string());
        if (inputs_only) {
            write_constraint_hdf5(pin_path.string(), pin);
            logger().info(
                "  pinned     : {}  ({} nodes, {})",
                pin_path.string(),
                pin_ids.size(),
                axes.has_value() ? suffix.substr(1) : "all axes");
        } else {
            memory.constraints.emplace(pin_paths.back(), std::move(pin));
        }
    }
    return pin_paths;
}

/// The reduced mesh and its material groups, which is what `build_polyfem_json` is given.
struct ReducedMesh
{
    std::filesystem::path path;
    mshio::MshSpec content;
    MeshInfo info;
};

/**
 * @brief The mesh polyfem actually solves on, and its material groups. Both operations do this,
 * in this order, for the same reason.
 *
 * The volumetric solve never runs on the caller's multi-tag mesh: WMTK writes one copy of a
 * multi-tagged cell per tag, and polyfem reads the copies as distinct elements -- it double-counts
 * AMIPS in separation and segfaults during constraint setup in smoothing. Collision filtering is
 * unaffected either way, because it works on the proxy mesh and not on body ids.
 *
 * The file is `<input stem>_polyfem.msh` in the simulation input directory, where pysimwild puts
 * it next to the other generated inputs. inputs_only writes it and reads the groups back off it,
 * as the Python does. A normal run writes no file and reads the groups off the content, in the
 * same order: the per-group volume is a running sum that divides every AMIPS weight in the
 * simulation JSON, so this is what keeps that JSON the same byte for byte in both modes (checked
 * in tests/test_polyfem_in_process.cpp).
 */
ReducedMesh reduce_mesh(
    const std::string& input,
    const std::vector<std::string>& ambient_like_tags,
    const std::filesystem::path& sim_in_dir,
    const bool inputs_only)
{
    ReducedMesh out;
    out.path = sim_in_dir / (std::filesystem::path(input).stem().string() + "_polyfem.msh");
    logger().info("[reduce mesh for polyfem]");
    out.content = polyfem_reduced_msh(input, ambient_like_tags);
    if (inputs_only) {
        write_polyfem_reduced_msh(out.path.string(), out.content);
        out.info = get_mesh_info(out.path.string());
    } else {
        out.info = get_mesh_info(out.content);
    }
    logger().info("Reduced material tags : {}  dim={}", out.info.tags, out.info.dim);
    return out;
}

/**
 * @brief The files of `make_interface_constraint`, under the names `build_polyfem_json` gives
 * them in `dir`: written in inputs_only mode, kept in `memory` under those names otherwise.
 *
 * `with_collision_proxy` is false in smoothing mode, whose simulation JSON has no contact block:
 * polyfem reads neither the linear map nor the body ids there, and the Python does not write them.
 */
void emit_interface_constraint(
    const InterfaceConstraint& generated,
    const std::filesystem::path& dir,
    const bool with_collision_proxy,
    const bool inputs_only,
    SolveInputs& memory)
{
    const auto path = [&dir](const char* name) { return (dir / name).string(); };
    logger().info("Writing:");
    // The OBJ is written in every mode, although a normal run hands polyfem the proxy from
    // `memory` and never reads this file: it is the only convenient way to see which faces the
    // selection picked, and the Python engine always writes it.
    write_collision_mesh_obj(path("interface_collision.obj"), generated.collision_mesh);
    if (inputs_only) {
        write_constraint_hdf5(path("interface_constraint.hdf5"), generated.fitting);
        write_constraint_hdf5(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
        if (with_collision_proxy) {
            write_linear_map_hdf5(path("interface_linear_map.hdf5"), generated.linear_map);
            write_collision_body_ids_txt(
                path("collision_body_ids.txt"),
                generated.collision_body_ids);
        }
        return;
    }
    memory.constraints.emplace(path("interface_constraint.hdf5"), generated.fitting);
    memory.constraints.emplace(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
    if (with_collision_proxy) {
        memory.collision_meshes.emplace(path("interface_collision.obj"), generated.collision_mesh);
        memory.linear_maps.emplace(path("interface_linear_map.hdf5"), generated.linear_map);
        memory.collision_body_ids.emplace(
            path("collision_body_ids.txt"),
            generated.collision_body_ids);
    }
}

} // namespace

PreparedOperation prepare_operation(nlohmann::json json_params)
{
    // The dispatcher keys only; an operation's own parameters are validated below against the
    // pysimwild spec.json that defines them, which stays the single source of the rules.
    {
        const auto spec = jse::embed::wmtk_polyfem_ops_spec::polyfem_ops_spec::spec();
        jse::JSE spec_engine;
        spec_engine.strict = true;

        nlohmann::json dispatch = nlohmann::json::object();
        for (const char* key : {"application", "input", "output", "operation", "inputs_only"}) {
            if (json_params.contains(key)) {
                dispatch[key] = json_params[key];
            }
        }
        if (!spec_engine.verify_json(dispatch, spec)) {
            log_and_throw_error(spec_engine.log2str());
        }
    }

    PreparedOperation out;
    out.operation = json_params["operation"].get<std::string>();
    out.inputs_only = json_params.value("inputs_only", false);
    const std::string& operation = out.operation;
    const bool inputs_only = out.inputs_only;

    nlohmann::json params = operation_params(json_params);
    const nlohmann::json op_spec = to_jse_spec(
        operation == "minimum_separation"
            ? jse::embed::wmtk_polyfem_ops_minimum_separation_spec::minimum_separation_spec::spec()
            : jse::embed::wmtk_polyfem_ops_laplacian_smoothing_spec::laplacian_smoothing_spec::
                  spec(),
        params);
    {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        if (!spec_engine.verify_json(params, op_spec)) {
            log_and_throw_error(spec_engine.log2str());
        }
        params = spec_engine.inject_defaults(params, op_spec);
    }

    out.input = params["input"].get<std::string>();
    out.output = params["output"].get<std::string>();
    const std::string& input = out.input;
    const std::string& output = out.output;

    if (operation == "minimum_separation") {
        // One pass gives both: the deduped sides the collision proxy is built from, and the
        // id-pair list that goes into contact.collision_pairs of the simulation JSON. It is also
        // where a malformed pair is caught.
        std::vector<Selection> sides;
        std::vector<std::array<int64_t, 2>> pairs;
        normalize_collision_pairs(params["collision_pairs"], sides, pairs);
        const OrderedJson polyfem_pairs = pairs;

        const std::filesystem::path sim_in_dir = sim_dir(output, "sep_input");
        out.sim_out_dir = sim_dir(output, "sep_output");
        logger().info("Input  : {}", input);
        logger().info("Output : {}.msh", output);
        // smooth_positions is false here and has no spec key: minimum_separation.run reads it
        // from cfg["smoothDisplacementsOrPositions"], whose OPT_DEFAULTS value is 0, and
        // simwild.py's minimum_separation engine never puts that key in cfg. Separation smooths
        // displacements (L u = 0), so the rest state stays an equilibrium of the penalty.
        emit_interface_constraint(
            make_interface_constraint(
                input,
                sides,
                params["use_graph_laplacian"],
                params["normalize_penalties"],
                params["scale"],
                /*smooth_positions=*/false),
            sim_in_dir,
            /*with_collision_proxy=*/true,
            inputs_only,
            out.inputs);

        ReducedMesh reduced =
            reduce_mesh(input, params["ambient_like_tags"], sim_in_dir, inputs_only);

        out.cfg = minimum_separation_cfg(params, polyfem_pairs);
        out.cfg["amips_weights"] = resolve_amips_weights(out.cfg);
        out.sim_json = build_polyfem_json(
            out.cfg,
            reduced.path,
            sim_in_dir,
            reduced.info,
            out.sim_out_dir / "solution.txt");
        if (!inputs_only) {
            out.inputs.meshes.emplace(
                out.sim_json["geometry"][0]["mesh"].get<std::string>(),
                std::move(reduced.content));
        }

        // The pins are made after the JSON and their paths are appended to it, as in run():
        // `constraints.hard` belongs to the operation, not to the shared builder.
        const std::vector<std::string> pin_paths =
            protected_pins(input, params["protected_regions"], sim_in_dir, inputs_only, out.inputs);
        if (!params["protected_regions"].empty()) {
            out.sim_json["constraints"]["hard"] = pin_paths;
        }
        out.sim_json_path = sim_in_dir / "separation.json";
    } else {
        // Smoothing mode: the simulation JSON has no contact block, so polyfem reads no collision
        // proxy (see emit_interface_constraint).
        std::vector<Selection> interfaces;
        std::vector<int64_t> ids_per_input;
        assign_selection_ids(params["interfaces"], interfaces, ids_per_input);

        const std::filesystem::path sim_in_dir = sim_dir(output, "smooth_input");
        out.sim_out_dir = sim_dir(output, "smooth_output");
        logger().info("Input  : {}", input);
        logger().info("Output : {}.msh", output);
        // Positions mode by default (laplacian_smoothing.run's own default, and the spec's):
        // in displacement mode the rest configuration is already a minimum and the solve
        // terminates with a zero gradient, so nothing moves.
        emit_interface_constraint(
            make_interface_constraint(
                input,
                interfaces,
                params["use_graph_laplacian"],
                params["normalize_penalties"],
                params["scale"],
                params["smooth_positions"]),
            sim_in_dir,
            /*with_collision_proxy=*/false,
            inputs_only,
            out.inputs);

        ReducedMesh reduced =
            reduce_mesh(input, params["ambient_like_tags"], sim_in_dir, inputs_only);

        // The reduced mesh's two groups are "ambient" and "body", which is the scheme
        // build_polyfem_json looks weights up in, so the resolved pair replaces whatever the
        // configuration carried.
        out.cfg = laplacian_smoothing_cfg(params);
        out.cfg["amips_weights"] = resolve_amips_weights(out.cfg);
        out.sim_json = build_polyfem_json(
            out.cfg,
            reduced.path,
            sim_in_dir,
            reduced.info,
            out.sim_out_dir / "solution.txt");
        if (!inputs_only) {
            out.inputs.meshes.emplace(
                out.sim_json["geometry"][0]["mesh"].get<std::string>(),
                std::move(reduced.content));
        }
        out.sim_json_path = sim_in_dir / "smoothing.json";
    }
    write_polyfem_json(out.sim_json_path, out.sim_json);
    return out;
}

void polyfem_ops(nlohmann::json json_params)
{
    PreparedOperation prepared = prepare_operation(std::move(json_params));
    if (prepared.inputs_only) {
        return;
    }

    // The polyfem linked into this process. The Python engine runs $POLYFEM_BIN instead; this one
    // needs no binary and never looks at the variable.
    const std::unique_ptr<PolyfemBackend> backend = in_process_backend(std::move(prepared.inputs));

    if (prepared.operation == "minimum_separation") {
        // The outer loop rewrites separation.json before every solve, so what stays on disk
        // afterwards is the last iteration's document -- the same file the Python leaves.
        const std::string strategy = prepared.cfg["strategy"];
        if (strategy == "dhat") {
            run_polyfem_dhat(
                *backend,
                prepared.sim_json,
                prepared.sim_json_path,
                prepared.sim_out_dir,
                prepared.cfg);
        } else {
            // jse has already refused anything but "dhat" and "stiffness" against the same
            // spec `run()` checks by hand, so there is no third branch to raise on.
            run_polyfem_stiffness(
                *backend,
                prepared.sim_json,
                prepared.sim_json_path,
                prepared.sim_out_dir,
                prepared.cfg);
        }
    } else {
        // One solve, no contact and no outer loop: `step_run_polyfem_single` writes the same
        // JSON again itself, which is mirrored rather than skipped so the two engines touch the
        // file the same number of times.
        run_polyfem_single(
            *backend,
            prepared.sim_json,
            prepared.sim_json_path,
            prepared.sim_out_dir);
    }

    logger().info("[write deformed msh]");
    // Applied to the ORIGINAL mesh, which is what preserves the caller's full tag set on the
    // output; the node tags match between the original and the reduced mesh, so solution.txt
    // indexes consistently against either.
    write_deformed_msh(
        prepared.input,
        prepared.sim_out_dir / "solution.txt",
        prepared.output + ".msh",
        prepared.cfg["scale"].get<double>());
}

} // namespace wmtk::components::polyfem_ops
