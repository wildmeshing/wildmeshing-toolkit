#include "PolyfemJson.hpp"

#include <minimum_separation_spec.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>

namespace wmtk::components::polyfem_ops {

namespace {

/// `dict.get(key, fallback)`: the stored value even when it is null, the fallback only when the
/// key is absent.
OrderedJson get_or(const OrderedJson& cfg, const std::string& key, const OrderedJson& fallback)
{
    const auto it = cfg.find(key);
    return it != cfg.end() ? OrderedJson(*it) : fallback;
}

/// Python truthiness, which is what the Python tests these configuration values with: false for
/// null, false, 0, an empty string and an empty container.
bool truthy(const OrderedJson& v)
{
    switch (v.type()) {
    case nlohmann::detail::value_t::null: return false;
    case nlohmann::detail::value_t::boolean: return v.get<bool>();
    case nlohmann::detail::value_t::number_integer:
    case nlohmann::detail::value_t::number_unsigned: return v.get<int64_t>() != 0;
    case nlohmann::detail::value_t::number_float: return v.get<double>() != 0.0;
    case nlohmann::detail::value_t::string: return !v.get<std::string>().empty();
    default: return !v.empty();
    }
}

/// `polyfem_utils.PARAVIEW_DEFAULTS`.
OrderedJson paraview_defaults()
{
    OrderedJson options;
    options["material"] = true;
    options["body_ids"] = true;
    options["tensor_values"] = false;
    options["nodes"] = false;

    OrderedJson out;
    out["file_name"] = "sim.pvd";
    out["surface"] = true;
    out["vismesh_rel_area"] = 1e7;
    out["options"] = options;
    return out;
}

} // namespace

const OrderedJson& opt_defaults()
{
    static const OrderedJson defaults = [] {
        // `_minsep_spec_defaults`: every root-level rule of the minimum_separation spec that
        // declares a default, keyed by its pointer without the leading slash.
        const nlohmann::json spec =
            jse::embed::wmtk_polyfem_ops_minimum_separation_spec::minimum_separation_spec::spec();
        std::map<std::string, nlohmann::json> s;
        for (const auto& rule : spec) {
            const std::string pointer = rule["pointer"].get<std::string>();
            if (std::count(pointer.begin(), pointer.end(), '/') == 1 && rule.contains("default")) {
                s[pointer.substr(1)] = rule["default"];
            }
        }

        OrderedJson out;
        // ---- derived from minimum_separation/spec.json (single source) ----
        out["scale"] = s.at("scale");
        out["useFitting"] = s.at("use_fitting");
        out["useLaplacian"] = s.at("use_laplacian");
        out["useGraphLaplacian"] = s.at("use_graph_laplacian");
        out["normalizePenalties"] = s.at("normalize_penalties");
        out["weight_fitting"] = s.at("weight_fitting");
        out["weight_laplacian"] = s.at("weight_laplacian");
        out["max_iterations"] = s.at("max_iterations");
        out["rtol"] = s.at("rtol");
        out["alpha_n"] = s.at("alpha_n");
        out["alpha_t"] = s.at("alpha_t");
        out["dhat_growth"] = s.at("dhat_growth");
        out["use_nh_body"] = s.at("use_nh_body");
        out["nh_youngs"] = s.at("nh_youngs");
        out["nh_poisson"] = s.at("nh_poisson");
        out["save_vtu"] = s.at("save_vtu");
        // ---- engine-only knobs (no spec counterpart) ----
        out["amips_weight_bg"] = 1e-6; // the "ambient" group
        out["amips_weight_body"] = 1e0; // the "body" group
        // NOT the spec default (that is -1 = auto): this is the resolved auto value for
        // strategy="dhat", which never adjusts kappa.
        out["barrier_stiffness"] = 1e6;
        out["sep"] = nullptr;
        out["smoothDisplacementsOrPositions"] = 0;
        // Volume normalization puts AMIPS in the same currency as the normalized fit/laplacian
        // penalties; by_count is the legacy fallback when volume normalization is disabled.
        out["amips_normalize_by_volume"] = true;
        out["amips_normalize_by_count"] = true;
        out["contact_enabled"] = true;
        return out;
    }();
    return defaults;
}

OrderedJson deep_merge(const OrderedJson& base, const OrderedJson& override_value)
{
    OrderedJson out = base;
    if (override_value.is_null()) {
        return out;
    }
    for (const auto& item : override_value.items()) {
        const auto existing = out.find(item.key());
        if (item.value().is_object() && existing != out.end() && existing->is_object()) {
            out[item.key()] = deep_merge(*existing, item.value());
        } else {
            out[item.key()] = item.value();
        }
    }
    return out;
}

OrderedJson geometry_block(const std::string& mesh_path, const OrderedJson& scale)
{
    OrderedJson transformation;
    transformation["scale"] = scale;

    OrderedJson geom;
    geom["mesh"] = mesh_path;
    geom["transformation"] = transformation;
    return geom;
}

OrderedJson resolve_amips_weights(const OrderedJson& cfg)
{
    OrderedJson legacy = get_or(cfg, "amips_weights", OrderedJson::object());
    if (!truthy(legacy)) {
        legacy = OrderedJson::object();
    }

    const OrderedJson ambient_w = get_or(
        cfg,
        "amips_ambient_weight",
        get_or(legacy, "ambient", opt_defaults().at("amips_weight_bg")));

    OrderedJson body_w = get_or(cfg, "amips_body_weight", nullptr);
    if (body_w.is_null()) {
        body_w = opt_defaults().at("amips_weight_body");
        for (const auto& item : legacy.items()) {
            if (item.key() != "ambient") {
                body_w = item.value();
                break;
            }
        }
    }

    OrderedJson out;
    out["ambient"] = ambient_w;
    out["body"] = body_w;
    return out;
}

OrderedJson build_polyfem_json(
    const OrderedJson& cfg,
    const std::filesystem::path& msh_path,
    const std::filesystem::path& out_dir,
    const MeshInfo& info,
    const std::filesystem::path& sol_path)
{
    const OrderedJson& opt = opt_defaults();
    const OrderedJson scale = get_or(cfg, "scale", opt.at("scale"));
    const OrderedJson sep = get_or(cfg, "sep", nullptr);
    const OrderedJson init_dhat = get_or(cfg, "init_dhat", sep);
    const OrderedJson collision_pairs = get_or(cfg, "collision_pairs", nullptr);
    const OrderedJson amips_weights_cfg = get_or(cfg, "amips_weights", OrderedJson::object());
    const bool use_fitting = truthy(get_or(cfg, "useFitting", opt.at("useFitting")));
    const bool use_laplacian = truthy(get_or(cfg, "useLaplacian", opt.at("useLaplacian"))) ||
                               truthy(get_or(cfg, "useGraphLaplacian", opt.at("useGraphLaplacian")));
    // AMIPS is integrated over element volumes in SOLVER units, so its raw magnitude carries a
    // hidden scale^dim factor (1e-9 in 3D at scale=1e-3) relative to the normalized fit/laplacian
    // penalties. Volume normalization divides each material's weight by its rest volume in solver
    // units, putting AMIPS in the same currency: a uniform strain eps costs ~w*eps^2 exactly as a
    // uniform displacement u costs w_fit*u^2. The legacy by-count mode only divides by the element
    // count and applies when volume normalization is off.
    const bool use_nh_body = truthy(get_or(cfg, "use_nh_body", opt.at("use_nh_body")));
    const OrderedJson nh_youngs = get_or(cfg, "nh_youngs", opt.at("nh_youngs"));
    const OrderedJson nh_poisson = get_or(cfg, "nh_poisson", opt.at("nh_poisson"));
    const bool amips_norm_by_volume =
        truthy(get_or(cfg, "amips_normalize_by_volume", opt.at("amips_normalize_by_volume")));
    const bool amips_norm_by_count =
        truthy(get_or(cfg, "amips_normalize_by_count", opt.at("amips_normalize_by_count")));

    // tag -> physical-group name, so amips_weights can be keyed by the numeric tag or by the
    // group name.
    std::map<int64_t, std::string> tag_to_name;
    for (const auto& [name, tag] : info.name_to_tag) {
        tag_to_name[tag] = name;
    }

    OrderedJson materials = OrderedJson::array();
    for (const int64_t tag : info.tags) {
        const auto name_it = tag_to_name.find(tag);
        const bool has_name = name_it != tag_to_name.end();
        // Which material is the ambient one is decided by its group NAME, as the Python decides
        // it. The reduced mesh this JSON is built on has exactly two physical groups, 1 =
        // "ambient" and 2 = "body", so the `tag == 0` test both engines used to make never fired
        // and ambient silently took the body's default AMIPS weight.
        const bool is_ambient = has_name && name_it->second == "ambient";
        // The Python also tries `amips_weights_cfg.get(tag)` with the INT key between these two;
        // a dict parsed from JSON can never have one, so the lookup is by the tag SPELLED OUT and
        // then by the group name.
        OrderedJson w = nullptr;
        const std::string tag_key = std::to_string(tag);
        if (amips_weights_cfg.contains(tag_key)) {
            w = amips_weights_cfg.at(tag_key);
        } else if (has_name && amips_weights_cfg.contains(name_it->second)) {
            w = amips_weights_cfg.at(name_it->second);
        }
        if (w.is_null()) {
            w = is_ambient ? opt.at("amips_weight_bg") : opt.at("amips_weight_body");
        }
        const auto vol_it = info.tag_to_volume.find(tag);
        const double vol_mesh = vol_it != info.tag_to_volume.end() ? vol_it->second : 0.0;
        // `scale ** mesh_dim` is CPython's float power, which is libm's pow; std::pow on two
        // doubles is the same call, so the two engines round the factor identically.
        const double vol_solver = vol_mesh * std::pow(scale.get<double>(), double(info.dim));
        if (amips_norm_by_volume) {
            if (vol_solver > 0) {
                w = w.get<double>() / vol_solver;
            }
        } else if (amips_norm_by_count) {
            const auto count_it = info.tag_to_count.find(tag);
            const int64_t n_elems = count_it != info.tag_to_count.end() ? count_it->second : 0;
            if (n_elems > 0) {
                w = w.get<double>() / double(n_elems);
            }
        }

        // AMIPS measures element SHAPE only: it is invariant under uniform scaling, so a body can
        // inflate for free. NeoHookean adds the missing volumetric term, which is what keeps a
        // body moving rigidly instead of swelling when contact pushes on all of its sides.
        //
        // The option turns EVERY group NeoHookean, ambient included, as the Python does, because
        // polyfem cannot solve a mixed list: State::formulation() accepts an array of differing
        // materials only when every entry is one of AssemblerUtils::elastic_materials(), AMIPS is
        // not one of them and MultiModel has no assembler to dispatch it to, so an AMIPS ambient
        // beside a NeoHookean body aborts the solve with "multimaterial supported only for
        // LinearElasticity and NeoHookean". All-NeoHookean is also the configuration the
        // volume-ratio measurement in the spec doc was made in. Each group is normalized by ITS
        // OWN rest volume, exactly as the AMIPS weights are.
        OrderedJson material;
        if (use_nh_body) {
            OrderedJson E = nh_youngs;
            if (amips_norm_by_volume && vol_solver > 0) {
                E = E.get<double>() / vol_solver; // same currency as the other penalties
            }
            material["id"] = tag;
            material["type"] = "NeoHookean";
            material["E"] = E;
            material["nu"] = nh_poisson;
            material["rho"] = 1.0;
        } else {
            material["id"] = tag;
            material["type"] = "AMIPS";
            material["weight"] = w;
            material["use_rest_pose"] = true;
        }
        materials.push_back(material);
    }

    const OrderedJson weight_fitting = get_or(cfg, "weight_fitting", opt.at("weight_fitting"));
    const OrderedJson weight_laplacian =
        get_or(cfg, "weight_laplacian", opt.at("weight_laplacian"));

    OrderedJson soft = OrderedJson::array();
    if (use_fitting) {
        OrderedJson entry;
        entry["data"] = (out_dir / "interface_constraint.hdf5").string();
        entry["weight"] = weight_fitting;
        soft.push_back(entry);
    }
    if (use_laplacian) {
        OrderedJson entry;
        entry["data"] = (out_dir / "interface_constraint_laplacian.hdf5").string();
        entry["weight"] = weight_laplacian;
        soft.push_back(entry);
    }

    const OrderedJson zero_val(std::vector<double>(info.dim, 0.0));
    const OrderedJson dim_flags(std::vector<bool>(info.dim, true));

    OrderedJson doc;
    doc["geometry"] = OrderedJson::array({geometry_block(
        std::filesystem::weakly_canonical(msh_path).string(),
        scale)});
    doc["materials"] = materials;
    doc["space"]["advanced"]["bc_method"] = "sample";
    if (truthy(get_or(cfg, "contact_enabled", opt.at("contact_enabled")))) {
        OrderedJson collision_mesh;
        collision_mesh["enabled"] = true;
        collision_mesh["mesh"] = (out_dir / "interface_collision.obj").string();
        collision_mesh["linear_map"] = (out_dir / "interface_linear_map.hdf5").string();
        collision_mesh["collision_body_ids"] = (out_dir / "collision_body_ids.txt").string();

        OrderedJson contact;
        contact["enabled"] = true;
        contact["friction_coefficient"] = 0.0;
        contact["epsv"] = 1e-3;
        contact["use_gcp_formulation"] = true;
        contact["dhat"] = init_dhat;
        contact["alpha_t"] = get_or(cfg, "alpha_t", opt.at("alpha_t"));
        contact["alpha_n"] = get_or(cfg, "alpha_n", opt.at("alpha_n"));
        contact["use_rest_shape_measure"] = get_or(cfg, "use_rest_shape_measure", false);
        contact["use_adaptive_dhat"] = get_or(cfg, "use_adaptive_dhat", false);
        contact["collision_pairs"] = collision_pairs;
        contact["collision_mesh"] = collision_mesh;
        doc["contact"] = contact;
    }

    OrderedJson solver;
    solver["max_threads"] = 0;
    solver["linear"]["solver"] = OrderedJson::array(
        {"Eigen::PardisoLDLT", "Eigen::CholmodDecomposition", "Eigen::AccelerateLDLT"});
    solver["nonlinear"]["grad_norm_tol"] = 1e-10;
    solver["nonlinear"]["rel_grad_norm_tol"] = 1e-8;
    solver["nonlinear"]["max_iterations"] = get_or(cfg, "nl_max_iterations", 1000);
    solver["nonlinear"]["allow_out_of_iterations"] = true;
    solver["nonlinear"]["Newton"]["residual_tolerance"] = 1e-3;
    solver["advanced"]["lump_mass_matrix"] = true;
    solver["contact"]["friction_convergence_tol"] = 0.01;
    solver["contact"]["friction_iterations"] = 1;
    solver["contact"]["barrier_stiffness"] =
        get_or(cfg, "barrier_stiffness", opt.at("barrier_stiffness"));
    doc["solver"] = solver;

    OrderedJson time;
    time["quasistatic"] = true;
    time["dt"] = 1;
    time["time_steps"] = 1;
    doc["time"] = time;

    OrderedJson dirichlet;
    dirichlet["id"] = "all";
    dirichlet["value"] = zero_val;
    dirichlet["dimension"] = dim_flags;
    OrderedJson boundary_conditions;
    boundary_conditions["rhs"] = std::vector<double>(3, 0.0);
    boundary_conditions["dirichlet_boundary"] = OrderedJson::array({dirichlet});
    doc["boundary_conditions"] = boundary_conditions;

    OrderedJson output;
    if (truthy(get_or(cfg, "save_vtu", opt.at("save_vtu")))) {
        output["paraview"] = deep_merge(paraview_defaults(), OrderedJson::object());
    }
    output["data"]["solution"] = std::filesystem::weakly_canonical(sol_path).string();
    output["data"]["advanced"]["reorder_nodes"] = true;
    doc["output"] = output;

    doc["input"]["data"] = OrderedJson::object();

    if (!soft.empty()) {
        doc["constraints"]["soft"] = soft;
    }
    return doc;
}

OrderedJson minimum_separation_cfg(const nlohmann::json& params, const OrderedJson& polyfem_pairs)
{
    OrderedJson cfg;
    cfg["input_msh"] = params["input"];
    // run() overwrites cfg["collision_pairs"] with the id pairs _normalize_collision_pairs
    // returns, before anything reads it; only those reach the polyfem JSON.
    cfg["collision_pairs"] = polyfem_pairs;
    cfg["sep"] = params["sep"];
    cfg["scale"] = params["scale"];
    cfg["useFitting"] = params["use_fitting"];
    cfg["useLaplacian"] = params["use_laplacian"];
    cfg["useGraphLaplacian"] = params["use_graph_laplacian"];
    cfg["normalizePenalties"] = params["normalize_penalties"];
    cfg["weight_fitting"] = params["weight_fitting"];
    cfg["weight_laplacian"] = params["weight_laplacian"];
    cfg["amips_weights"] = params["amips_weights"];
    cfg["max_iterations"] = params["max_iterations"];
    cfg["rtol"] = params["rtol"];
    cfg["nl_max_iterations"] = params["nl_max_iterations"];
    cfg["barrier_stiffness"] = params["barrier_stiffness"];
    cfg["alpha_n"] = params["alpha_n"];
    cfg["alpha_t"] = params["alpha_t"];
    cfg["save_vtu"] = params["save_vtu"];
    cfg["strategy"] = params["strategy"];
    cfg["dhat_growth"] = params["dhat_growth"];
    cfg["max_stiffness_multiplier"] = params["max_stiffness_multiplier"];
    cfg["protected_regions"] = params["protected_regions"];
    cfg["ambient_like_tags"] = params["ambient_like_tags"];
    cfg["use_nh_body"] = params["use_nh_body"];
    cfg["nh_youngs"] = params["nh_youngs"];
    cfg["nh_poisson"] = params["nh_poisson"];
    cfg["output_msh"] = params["output"].get<std::string>() + ".msh";
    if (params["init_dhat"].get<double>() > 0) {
        cfg["init_dhat"] = params["init_dhat"];
    }

    // run(): auto (<=0 or unset) barrier stiffness. The dhat ramp never adjusts kappa and too-soft
    // stalls at its fixed point, so it starts generous; the stiffness loop only ever raises kappa,
    // and a soft start does the bulk of the displacement on a cheap landscape.
    if (get_or(cfg, "barrier_stiffness", -1.0).get<double>() <= 0) {
        cfg["barrier_stiffness"] = cfg["strategy"] == "stiffness"
                                       ? OrderedJson(1.0)
                                       : opt_defaults().at("barrier_stiffness");
    }
    return cfg;
}

OrderedJson laplacian_smoothing_cfg(const nlohmann::json& params)
{
    OrderedJson cfg;
    cfg["input_msh"] = params["input"];
    cfg["scale"] = params["scale"];
    cfg["useFitting"] = params["use_fitting"];
    cfg["useLaplacian"] = params["use_laplacian"];
    cfg["useGraphLaplacian"] = params["use_graph_laplacian"];
    cfg["normalizePenalties"] = params["normalize_penalties"];
    cfg["weight_fitting"] = params["weight_fitting"];
    cfg["weight_laplacian"] = params["weight_laplacian"];
    cfg["max_iterations"] = params["max_iterations"];
    cfg["smoothDisplacementsOrPositions"] = params["smooth_positions"].get<bool>() ? 1 : 0;
    cfg["save_vtu"] = params["save_vtu"];
    cfg["ambient_like_tags"] = params["ambient_like_tags"];
    cfg["output_msh"] = params["output"].get<std::string>() + ".msh";
    if (!params["interfaces"].empty()) {
        cfg["interfaces"] = params["interfaces"];
    }

    // run(): smoothing has no contact and no outer loop.
    cfg["contact_enabled"] = false;
    if (cfg.contains("max_iterations") && !cfg.contains("nl_max_iterations")) {
        cfg["nl_max_iterations"] = cfg["max_iterations"];
    }
    // The configuration carries no amips_weights at all -- simwild.py's smoothing engine never
    // puts that key in it -- so the guard is always satisfied and the body weight is always 1e-4.
    bool has_non_ambient = false;
    for (const auto& item : get_or(cfg, "amips_weights", OrderedJson::object()).items()) {
        if (item.key() != "ambient") has_non_ambient = true;
    }
    if (!cfg.contains("amips_body_weight") && !has_non_ambient) {
        cfg["amips_body_weight"] = 1e-4;
    }
    return cfg;
}

void write_polyfem_json(const std::filesystem::path& path, const OrderedJson& doc)
{
    std::ofstream out(path);
    if (!out.is_open()) {
        log_and_throw_error("Unable to open {} for writing", path.string());
    }
    out << doc.dump(4);
}

} // namespace wmtk::components::polyfem_ops
