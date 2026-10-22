#include "CurveMap.hpp"
#include "SurfaceMap.hpp"
#include "surface_io.hpp"
#include "tracking.hpp"
#include "transfer.hpp"

#include <igl/Timer.h>
#include <jse/jse.h>
#include <fstream>
#include <sstream>
#include <wmtk/history/History.hpp>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <tracking_spec.hpp>

#include <algorithm>
#include <cmath>

namespace wmtk::components::tracking {

namespace {

nlohmann::json warnings_report(const MapWarnings& w)
{
    return {
        {"total", w.total()},
        {"not_a_disk", w.not_a_disk},
        {"flat_triangle", w.flat_triangle},
        {"inverted", w.inverted},
        {"merged_faces", w.merged_faces},
        {"unlocated", w.unlocated},
        {"replaced", w.replaced}};
}

void log_warnings(const char* what, const MapWarnings& w)
{
    logger().info(
        "{}: {} warning(s) -- not a disk {}, flat triangle {}, inverted {}, merged faces {}, "
        "unlocated {}, replaced faces {}",
        what,
        w.total(),
        w.not_a_disk,
        w.flat_triangle,
        w.inverted,
        w.merged_faces,
        w.unlocated,
        w.replaced);
    if (w.total() > 0) logger().warn("{}: the map is not a bijection everywhere", what);
}

/// The 3D position of a point on a complex.
Eigen::Vector3d position(
    const SurfacePoint& p,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 3>>& F)
{
    const auto& f = F[p.face];
    return p.bary[0] * V[f[0]] + p.bary[1] * V[f[1]] + p.bary[2] * V[f[2]];
}

/// One point per line, whitespace-separated numbers, '#' starting a comment. See the spec.
std::vector<std::vector<double>> read_points(const std::string& path, const size_t n)
{
    std::ifstream in(path);
    if (!in) log_and_throw_error("Could not read {}", path);
    std::vector<std::vector<double>> rows;
    std::string line;
    for (size_t number = 1; std::getline(in, line); ++number) {
        line = line.substr(0, line.find('#'));
        std::istringstream ss(line);
        std::vector<double> row;
        for (double x; ss >> x;) row.push_back(x);
        if (row.empty()) continue;
        if (row.size() != n) {
            log_and_throw_error("{}:{}: {} numbers, expected {}", path, number, row.size(), n);
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

uint64_t id_of(const double x)
{
    return x < 0 ? kNoId : uint64_t(x);
}

std::string id_text(const uint64_t id)
{
    return id == kNoId ? std::string("-1") : std::to_string(id);
}

/// The map operation: points on the input to the output, or back.
SurfaceMapOptions options_of(const nlohmann::json& json_params)
{
    SurfaceMapOptions options;
    if (json_params["smoothing"] == "barycentric") options.smoothing = Smoothing::barycentric;
    if (json_params["scaffold"] == "ring") options.scaffold = Scaffold::ring;
    return options;
}

void map_points(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root)
{
    using wmtk::utils::resolve_path;
    const std::string points_path = resolve_path(root, json_params["points"]).string();
    if (std::string(json_params["output_points"]).empty()) {
        log_and_throw_error("map: output_points is required");
    }
    const std::string output_points = resolve_path(root, json_params["output_points"]).string();
    const bool forward = json_params["direction"] == "to_output";
    igl::Timer timer;
    timer.start();
    const Tracking tracking(h, options_of(json_params));
    logger().info("map built in {:.3}s", timer.getElapsedTimeInSec());
    // A point named by a face is `face b0 b1 b2`, one named by vertices `v0 v1 v2 b0 b1 b2`.
    const size_t n_in = tracking.input_by_vertices() ? 6 : 4;
    const size_t n_out = tracking.output_by_vertices() ? 6 : 4;
    std::ofstream out(output_points);
    if (!out) log_and_throw_error("Could not write {}", output_points);
    out.precision(17);
    size_t lost = 0, total = 0;
    MapWarnings warnings;
    timer.start();
    if (forward) {
        std::vector<InputPoint> pts;
        for (const auto& r : read_points(points_path, n_in)) {
            InputPoint p;
            if (tracking.input_by_vertices()) {
                p.vertices = {{id_of(r[0]), id_of(r[1]), id_of(r[2])}};
                p.bary = Eigen::Vector3d(r[3], r[4], r[5]);
            } else {
                p.face = id_of(r[0]);
                p.bary = Eigen::Vector3d(r[1], r[2], r[3]);
            }
            pts.push_back(p);
        }
        total = pts.size();
        for (const OutputPoint& p : tracking.to_output(pts, &warnings)) {
            const bool none =
                tracking.output_by_vertices() ? p.vertices[0] == kNoId : p.face == kNoId;
            lost += none;
            if (tracking.output_by_vertices()) {
                for (const uint64_t v : p.vertices) out << id_text(none ? kNoId : v) << " ";
            } else {
                out << id_text(p.face) << " ";
            }
            out << p.bary[0] << " " << p.bary[1] << " " << p.bary[2] << "\n";
        }
    } else {
        std::vector<OutputPoint> pts;
        for (const auto& r : read_points(points_path, n_out)) {
            OutputPoint p;
            if (tracking.output_by_vertices()) {
                p.vertices = {{id_of(r[0]), id_of(r[1]), id_of(r[2])}};
                p.bary = Eigen::Vector3d(r[3], r[4], r[5]);
            } else {
                p.face = id_of(r[0]);
                p.bary = Eigen::Vector3d(r[1], r[2], r[3]);
            }
            pts.push_back(p);
        }
        total = pts.size();
        for (const InputPoint& p : tracking.to_input(pts, &warnings)) {
            const bool none =
                tracking.input_by_vertices() ? p.vertices[0] == kNoId : p.face == kNoId;
            lost += none;
            if (tracking.input_by_vertices()) {
                for (const uint64_t v : p.vertices) out << id_text(none ? kNoId : v) << " ";
            } else {
                out << id_text(p.face) << " ";
            }
            out << p.bary[0] << " " << p.bary[1] << " " << p.bary[2] << "\n";
        }
    }
    logger().info(
        "{} points mapped to the {} in {:.3}s, {} without an image; written to {}",
        total,
        forward ? "output" : "input",
        timer.getElapsedTimeInSec(),
        lost,
        output_points);
    log_warnings("mapping", warnings);
}

void log_warnings(const char* what, const CurveMapWarnings& w)
{
    logger().info(
        "{}: {} warning(s) -- topology {}, merged segments {}, overlapping {}, unlocated {}, "
        "replaced segments {}",
        what,
        w.total(),
        w.topology,
        w.merged_segments,
        w.overlapping,
        w.unlocated,
        w.replaced);
    if (w.total() > 0) logger().warn("{}: the map is not a bijection everywhere", what);
}

CurveMapOptions curve_options_of(const nlohmann::json& json_params)
{
    CurveMapOptions options;
    if (json_params["smoothing"] == "barycentric") options.smoothing = Smoothing::barycentric;
    return options;
}

/// The 3D position of a point on curves.
Eigen::Vector3d position(
    const CurvePoint& p,
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 2>>& S)
{
    return p.bary[0] * V[S[p.segment][0]] + p.bary[1] * V[S[p.segment][1]];
}

/// The map operation on curves: points on the input to the output, or back.
void map_curve_points(
    const history::CurveHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root)
{
    using wmtk::utils::resolve_path;
    const std::string points_path = resolve_path(root, json_params["points"]).string();
    if (std::string(json_params["output_points"]).empty()) {
        log_and_throw_error("map: output_points is required");
    }
    const std::string output_points = resolve_path(root, json_params["output_points"]).string();
    const bool forward = json_params["direction"] == "to_output";
    igl::Timer timer;
    timer.start();
    const CurveTracking tracking(h, curve_options_of(json_params));
    logger().info("map built in {:.3}s", timer.getElapsedTimeInSec());
    std::ofstream out(output_points);
    if (!out) log_and_throw_error("Could not write {}", output_points);
    out.precision(17);
    size_t lost = 0, total = 0;
    timer.start();
    if (forward) {
        std::vector<InputCurvePoint> pts;
        for (const auto& r : read_points(points_path, 3)) {
            pts.push_back({id_of(r[0]), Eigen::Vector2d(r[1], r[2])});
        }
        total = pts.size();
        for (const OutputCurvePoint& p : tracking.to_output(pts)) {
            const bool none =
                tracking.output_by_vertices() ? p.vertices[0] == kNoId : p.segment == kNoId;
            lost += none;
            if (tracking.output_by_vertices()) {
                for (const uint64_t v : p.vertices) out << id_text(none ? kNoId : v) << " ";
            } else {
                out << id_text(p.segment) << " ";
            }
            out << p.bary[0] << " " << p.bary[1] << "\n";
        }
    } else {
        std::vector<OutputCurvePoint> pts;
        for (const auto& r : read_points(points_path, tracking.output_by_vertices() ? 4 : 3)) {
            OutputCurvePoint p;
            if (tracking.output_by_vertices()) {
                p.vertices = {{id_of(r[0]), id_of(r[1])}};
                p.bary = Eigen::Vector2d(r[2], r[3]);
            } else {
                p.segment = id_of(r[0]);
                p.bary = Eigen::Vector2d(r[1], r[2]);
            }
            pts.push_back(p);
        }
        total = pts.size();
        for (const InputCurvePoint& p : tracking.to_input(pts)) {
            lost += p.segment == kNoId;
            out << id_text(p.segment) << " " << p.bary[0] << " " << p.bary[1] << "\n";
        }
    }
    logger().info(
        "{} points mapped to the {} in {:.3}s, {} without an image; written to {}",
        total,
        forward ? "output" : "input",
        timer.getElapsedTimeInSec(),
        lost,
        output_points);
    log_warnings("mapping", tracking.map().map_warnings());
}

/// The operations on a curve history (triwild's); `history_path` names it in messages.
void curves(
    const history::CurveHistory& h,
    const std::string& history_path,
    const nlohmann::json& json_params,
    const std::filesystem::path& root)
{
    igl::Timer timer;
    const std::string operation = json_params["operation"];
    if (operation == "map") {
        map_curve_points(h, json_params, root);
        return;
    }
    if (operation == "transfer") {
        log_and_throw_error("transfer carries a surface's fields; {} holds curves", history_path);
    }

    timer.start();
    const CurveTracking tracking(h, curve_options_of(json_params));
    const CurveMap& map = tracking.map();
    const auto& steps = map.steps_by_kind();
    logger().info(
        "map built in {:.3}s: {} splits, {} collapses, {} vertex moves, {} relabels, {} embeds, "
        "{} consolidations",
        timer.getElapsedTimeInSec(),
        steps[size_t(history::EditKind::split_edge)],
        steps[size_t(history::EditKind::collapse_edge)],
        steps[size_t(history::EditKind::move_vertices)],
        steps[size_t(history::EditKind::relabel)],
        steps[size_t(history::EditKind::embed)],
        steps[size_t(history::EditKind::consolidate)]);
    log_warnings("building the map", map.build_warnings());
    for (size_t i = 0; i < std::min<size_t>(map.warned_edits().size(), 5); ++i) {
        const size_t k = map.warned_edits()[i];
        const auto& e = h.edits[k];
        logger().warn(
            "  edit {} (kind {}): vertices {}, faces {}",
            k,
            int(e.kind),
            nlohmann::json(e.v).dump(),
            nlohmann::json(e.faces).dump());
    }

    const std::vector<FeaturePoint> features = tracking.feature_points();
    const size_t kept = size_t(std::count_if(features.begin(), features.end(), [](const auto& f) {
        return f.output_vertex != kNoId;
    }));
    logger().info(
        "{} feature points on the input (curve ends, junctions, input points), {} of them at an "
        "output vertex",
        features.size(),
        kept);
    if (operation == "features") {
        if (std::string(json_params["output_points"]).empty()) {
            log_and_throw_error("features: output_points is required");
        }
        const std::string output_points =
            wmtk::utils::resolve_path(root, json_params["output_points"]).string();
        std::ofstream out(output_points);
        if (!out) log_and_throw_error("Could not write {}", output_points);
        for (const FeaturePoint& f : features) {
            out << id_text(f.input_vertex) << " " << id_text(f.output_vertex) << "\n";
        }
        logger().info("feature points written to {}", output_points);
        return;
    }

    // info: the output's segment midpoints to the input and back, and how far they lie from
    // their preimages, against the output's mean segment length.
    const auto& V = map.output_vertices();
    const auto& S = map.output_segments();
    std::vector<CurvePoint> midpoints;
    double length = 0;
    for (uint64_t f = 0; f < S.size(); ++f) {
        if (!map.output_segment_alive()[f]) continue;
        midpoints.push_back({f, Eigen::Vector2d::Constant(0.5)});
        length += (V[S[f][0]] - V[S[f][1]]).norm();
    }
    timer.start();
    const auto in = map.to_input(midpoints);
    log_warnings("output midpoints to the input", map.map_warnings());
    const auto back = map.to_output(in);
    log_warnings("and back to the output", map.map_warnings());
    double max_err = 0;
    size_t lost = 0;
    std::vector<double> drift;
    for (size_t i = 0; i < midpoints.size(); ++i) {
        if (back[i].segment >= S.size()) {
            ++lost;
            continue;
        }
        max_err =
            std::max(max_err, (position(back[i], V, S) - position(midpoints[i], V, S)).norm());
        drift.push_back((position(midpoints[i], V, S) -
                         position(in[i], map.input_vertices(), map.input_segments()))
                            .norm());
    }
    logger().info(
        "{} output midpoints mapped to the input and back in {:.3}s: max displacement {:.3g}, {} "
        "lost",
        midpoints.size(),
        timer.getElapsedTimeInSec(),
        max_err,
        lost);
    if (!drift.empty() && length > 0) {
        std::sort(drift.begin(), drift.end());
        const double mean_length = length / double(midpoints.size());
        double mean = 0;
        for (const double d : drift) mean += d;
        mean /= double(drift.size());
        logger().info(
            "output midpoints lie from their preimages: mean {:.3g}, 99th percentile {:.3g}, max "
            "{:.3g} output segment lengths (mean segment {:.3g})",
            mean / mean_length,
            drift[drift.size() * 99 / 100] / mean_length,
            drift.back() / mean_length,
            mean_length);
    }
}

void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "tracking");
    using wmtk::utils::resolve_path;
    // On the memory route the history is the bytes of one, a producer's report["history"],
    // never a file name; the spec describes the file route's option, a path.
    std::vector<uint8_t> history_bytes;
    if (json_params.contains("history") && json_params["history"].is_binary()) {
        if (!component_io.memory()) {
            throw std::runtime_error(
                "tracking: history is a file on the file route; the bytes of one are taken "
                "on the memory route");
        }
        history_bytes = json_params["history"].get_binary();
        json_params["history"] = "";
    } else if (component_io.memory() && json_params.contains("history")) {
        throw std::runtime_error(
            "tracking: on the memory route history is the bytes of a history -- a "
            "component's report[\"history\"], or history::read_history / wm.read_history of a "
            "file -- not a file name");
    }
    {
        const auto spec = jse::embed::wmtk_tracking_spec::tracking_spec::spec();
        jse::JSE spec_engine;
        if (!spec_engine.verify_json(json_params, spec)) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }
    if (component_io.memory()) {
        // map and features read and write text files of points; in memory, map points
        // through Tracking and CurveTracking.
        if (json_params["operation"] == "map" || json_params["operation"] == "features") {
            throw std::runtime_error(
                "tracking: operation " + json_params["operation"].get<std::string>() +
                " reads and writes files of points; on the memory route only info and transfer "
                "are supported");
        }
        // The memory route writes no files: the meshes come back in the result.
        json_params["output_obj"] = "";
        json_params["output_tet_mesh"] = "";
    }
    const std::filesystem::path root = json_params["input_dir"];
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            set_file_logger(resolve_path(root, log_file_name).string());
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string history_path = component_io.memory()
                                         ? std::string("the history given")
                                         : resolve_path(root, json_params["history"]).string();
    igl::Timer timer;
    timer.start();
    // A surface's history or curves' (triwild's): the header says which.
    if ((component_io.memory() ? history::face_size(history_bytes)
                               : history::face_size(history_path)) == 2) {
        const history::CurveHistory h = component_io.memory()
                                            ? history::CurveHistory::from_bytes(history_bytes)
                                            : history::CurveHistory::load(history_path);
        logger().info(
            "loaded {} edits on curves from {} in {:.3}s",
            h.edits.size(),
            history_path,
            timer.getElapsedTimeInSec());
        curves(h, history_path, json_params, root);
        return;
    }
    if (json_params["operation"] == "features") {
        log_and_throw_error(
            "features lists a curve history's feature points; {} holds a surface",
            history_path);
    }
    timer.start();
    const history::SurfaceHistory h = component_io.memory()
                                          ? history::SurfaceHistory::from_bytes(history_bytes)
                                          : history::SurfaceHistory::load(history_path);
    logger().info(
        "loaded {} edits from {} in {:.3}s",
        h.edits.size(),
        history_path,
        timer.getElapsedTimeInSec());

    if (json_params["operation"] == "map") {
        map_points(h, json_params, root);
        return;
    }

    timer.start();
    const SurfaceMap map(h, options_of(json_params));
    const double build_time = timer.getElapsedTimeInSec();
    const auto& cells = map.cells_by_kind();
    logger().info(
        "map built in {:.3}s: {} edge splits, {} face splits, {} collapses, {} swaps, {} "
        "consolidations",
        timer.getElapsedTimeInSec(),
        cells[size_t(history::EditKind::split_edge)],
        cells[size_t(history::EditKind::split_face)],
        cells[size_t(history::EditKind::collapse_edge)],
        cells[size_t(history::EditKind::swap_edge)],
        cells[size_t(history::EditKind::consolidate)]);
    logger().info(
        "{} side(s) of collapses along the boundary or a feature curve; {} vertex moves, {} of "
        "them keeping barycentric coordinates",
        map.curve_collapses(),
        cells[size_t(history::EditKind::move_vertices)] + map.kept_moves(),
        map.kept_moves());
    if (map.folded_cells() > 0) {
        logger().info(
            "{} local maps span a patch the remesher folded over; the map stretches there",
            map.folded_cells());
    }
    log_warnings("building the map", map.build_warnings());
    for (size_t i = 0; i < std::min<size_t>(map.warned_edits().size(), 5); ++i) {
        const size_t k = map.warned_edits()[i];
        const auto& e = h.edits[k];
        logger().warn(
            "  edit {} (kind {}): vertices {}, faces {}",
            k,
            int(e.kind),
            nlohmann::json(e.v).dump(),
            nlohmann::json(e.faces).dump());
    }

    if (json_params["operation"] == "transfer") {
        const std::string input_name = json_params["input"];
        std::string input;
        SurfaceData in;
        if (component_io.memory()) {
            // The memory route never resolves a name as a file: the input is a mesh given.
            if (input_name.empty()) {
                throw std::runtime_error(
                    "tracking: a transfer on the memory route needs input, the mesh the "
                    "history starts from");
            }
            input = "input " + input_name;
            in = surface_from_mesh(component_io.mesh(input_name), input, true);
        } else {
            // The input: as given, or the files the history's application read, from its
            // parameters.
            std::vector<std::string> inputs;
            if (!input_name.empty()) {
                inputs.push_back(resolve_path(root, input_name).string());
            } else {
                const auto app = nlohmann::json::parse(h.metadata, nullptr, false);
                if (app.is_discarded() || !app.contains("input")) {
                    log_and_throw_error("{} names no input; give one", history_path);
                }
                // On the memory route the input was a mesh of the call, named by its key: a
                // file of that name here is some other file.
                if (app.value("route", "file") == "memory") {
                    log_and_throw_error(
                        "{} was recorded on the memory route, where its input was a mesh, not a "
                        "file; give input",
                        history_path);
                }
                const std::filesystem::path app_root =
                    app.contains("input_dir") ? std::filesystem::path(std::string(app["input_dir"]))
                                              : std::filesystem::path();
                const auto& in_json = app["input"];
                for (const auto& p :
                     in_json.is_array() ? in_json : nlohmann::json::array({in_json})) {
                    inputs.push_back(resolve_path(app_root, std::string(p)).string());
                    // A history recorded on the memory route names the input meshes, not files.
                    if (!std::filesystem::exists(inputs.back())) {
                        log_and_throw_error(
                            "{} names the input {}, which is not a file here; give one",
                            history_path,
                            inputs.back());
                    }
                }
            }
            input = nlohmann::json(inputs).dump();
            in = read_surface(inputs);
        }
        std::string why;
        if (!matches_input(map, in, why)) {
            log_and_throw_error(
                "{} is not the surface {} starts from: {}",
                input,
                history_path,
                why);
        }
        timer.start();
        MapWarnings warnings;
        const SurfaceData out = transfer_fields(map, in, &warnings);
        log_warnings("output centroids to the input", warnings);
        const std::string output = json_params["output"];
        if (component_io.memory()) {
            component_io.result.first["mesh"] = mesh_from_surface(out);
        } else {
            write_surface_msh(output, out);
        }
        const std::string tet_mesh = json_params["tet_mesh"];
        const std::string output_tet_mesh = json_params["output_tet_mesh"];
        if (!tet_mesh.empty()) {
            if (output_tet_mesh.empty() && !component_io.memory())
                log_and_throw_error("tet_mesh given without output_tet_mesh");
            // The surface's vertices by the tet mesh's own ids, which the history saved.
            if (h.output_vertex_ids.empty()) {
                log_and_throw_error(
                    "tet_mesh: {} ends at a surface output, not at a tet mesh's tracked surface",
                    history_path);
            }
            const auto rows = output_rows(map, h.output_vertex_ids);
            if (component_io.memory()) {
                MeshData with_fields = component_io.mesh(tet_mesh);
                add_surface(with_fields, out, rows, "tet_mesh " + tet_mesh);
                component_io.result.first["tet_mesh"] = std::move(with_fields);
            } else {
                write_surface_into_msh(
                    resolve_path(root, tet_mesh).string(),
                    output_tet_mesh,
                    out,
                    rows);
                logger().info(
                    "fields written onto the tracked surface of {}: {}",
                    tet_mesh,
                    output_tet_mesh);
            }
        }
        const std::string output_obj = json_params["output_obj"];
        if (!output_obj.empty()) {
            write_surface_obj(output_obj, out, json_params["uv_field"], json_params["texture"]);
            logger().info("textured output written to {}", output_obj);
        }
        std::vector<std::string> fields;
        for (const auto* f : {&out.face_fields, &out.corner_fields}) {
            for (const auto& [name, field] : *f) fields.push_back(name);
        }
        logger().info(
            "{} field(s) carried from {} faces onto {} in {:.3}s{}",
            fields.size(),
            in.faces.size(),
            out.faces.size(),
            timer.getElapsedTimeInSec(),
            component_io.memory() ? std::string() : "; written to " + output);
        component_io.result.second = {
            {"build_time_sec", build_time},
            {"build_warnings", warnings_report(map.build_warnings())},
            {"input_faces", in.faces.size()},
            {"output_faces", out.faces.size()},
            {"fields", fields},
            {"to_input_warnings", warnings_report(warnings)}};
        return;
    }

    nlohmann::json report;
    report["edits"] = h.edits.size();
    report["build_time_sec"] = build_time;
    report["edge_splits"] = cells[size_t(history::EditKind::split_edge)];
    report["face_splits"] = cells[size_t(history::EditKind::split_face)];
    report["collapses"] = cells[size_t(history::EditKind::collapse_edge)];
    report["swaps"] = cells[size_t(history::EditKind::swap_edge)];
    report["consolidations"] = cells[size_t(history::EditKind::consolidate)];
    report["curve_collapses"] = map.curve_collapses();
    report["vertex_moves"] = cells[size_t(history::EditKind::move_vertices)] + map.kept_moves();
    report["kept_moves"] = map.kept_moves();
    report["folded_cells"] = map.folded_cells();
    report["build_warnings"] = warnings_report(map.build_warnings());

    // info: the shape of the output's triangles, by how many feature edges they have.
    {
        const auto& V = map.output_vertices();
        const auto& F = map.output_faces();
        const auto& fe = map.output_feature_edges();
        std::array<std::vector<double>, 3> min_angles; // 0, 1, 2+ feature edges
        for (uint64_t f = 0; f < F.size(); ++f) {
            if (!map.output_face_alive()[f]) continue;
            double a_min = 180;
            int n_feature = 0;
            for (int k = 0; k < 3; ++k) {
                const Eigen::Vector3d& p = V[F[f][k]];
                const Eigen::Vector3d u = V[F[f][(k + 1) % 3]] - p, w = V[F[f][(k + 2) % 3]] - p;
                const double c = u.dot(w) / (u.norm() * w.norm());
                a_min = std::min(a_min, std::acos(std::clamp(c, -1., 1.)) * 180 / M_PI);
                uint64_t a = F[f][k], b = F[f][(k + 1) % 3];
                if (a > b) std::swap(a, b);
                n_feature += fe.count({{a, b}}) > 0;
            }
            min_angles[std::min(n_feature, 2)].push_back(a_min);
        }
        const char* what[3] = {"no feature edge", "one feature edge", "two or more"};
        const char* key[3] = {"no_feature_edge", "one_feature_edge", "two_or_more"};
        for (int c = 0; c < 3; ++c) {
            auto& a = min_angles[c];
            if (a.empty()) continue;
            std::sort(a.begin(), a.end());
            const auto below = [&a](double t) {
                return size_t(std::lower_bound(a.begin(), a.end(), t) - a.begin());
            };
            report["output_faces"][key[c]] = {
                {"faces", a.size()},
                {"min_angle", a.front()},
                {"min_angle_1st_percentile", a[a.size() / 100]},
                {"below_1_deg", below(1)},
                {"below_5_deg", below(5)},
                {"below_10_deg", below(10)}};
            logger().info(
                "output faces with {}: {} -- min angle {:.3g} deg, 1st percentile {:.3g}, below "
                "1 deg {}, below 5 deg {}, below 10 deg {}",
                what[c],
                a.size(),
                a.front(),
                a[a.size() / 100],
                below(1),
                below(5),
                below(10));
        }
    }

    // info: the output's face centroids to the input and back.
    std::vector<SurfacePoint> centroids;
    for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
        if (map.output_face_alive()[f]) centroids.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
    }
    timer.start();
    MapWarnings to_input_warnings, to_output_warnings;
    const auto in = map.to_input(centroids, &to_input_warnings);
    log_warnings("output centroids to the input", to_input_warnings);
    report["to_input_warnings"] = warnings_report(to_input_warnings);
    const auto back = map.to_output(in, &to_output_warnings);
    log_warnings("and back to the output", to_output_warnings);
    report["to_output_warnings"] = warnings_report(to_output_warnings);
    double max_err = 0;
    size_t lost = 0;
    for (size_t i = 0; i < centroids.size(); ++i) {
        if (back[i].face >= map.output_faces().size()) {
            ++lost;
            continue;
        }
        const double err = (position(back[i], map.output_vertices(), map.output_faces()) -
                            position(centroids[i], map.output_vertices(), map.output_faces()))
                               .norm();
        max_err = std::max(max_err, err);
    }
    logger().info(
        "{} output centroids mapped to the input and back in {:.3}s: max displacement {:.3g}, {} "
        "lost",
        centroids.size(),
        timer.getElapsedTimeInSec(),
        max_err,
        lost);
    report["centroids"] = centroids.size();
    report["round_trip_max_displacement"] = max_err;
    report["round_trip_lost"] = lost;

    // How far the map moves points: each output centroid from its preimage on the input, in 3D,
    // against the output's mean edge length.
    double edges = 0;
    size_t n_edges = 0;
    std::vector<double> drift;
    for (size_t i = 0; i < centroids.size(); ++i) {
        const auto& t = map.output_faces()[centroids[i].face];
        for (int k = 0; k < 3; ++k) {
            edges += (map.output_vertices()[t[k]] - map.output_vertices()[t[(k + 1) % 3]]).norm();
        }
        n_edges += 3;
        if (in[i].face >= map.input_faces().size()) continue;
        drift.push_back((position(centroids[i], map.output_vertices(), map.output_faces()) -
                         position(in[i], map.input_vertices(), map.input_faces()))
                            .norm());
    }
    if (!drift.empty() && n_edges > 0) {
        std::sort(drift.begin(), drift.end());
        const double mean_edge = edges / double(n_edges);
        double mean = 0;
        for (const double d : drift) mean += d;
        mean /= double(drift.size());
        logger().info(
            "output centroids lie from their preimages: mean {:.3g}, 99th percentile {:.3g}, max "
            "{:.3g} output edge lengths (mean edge {:.3g})",
            mean / mean_edge,
            drift[drift.size() * 99 / 100] / mean_edge,
            drift.back() / mean_edge,
            mean_edge);
        report["drift"] = {
            {"mean", mean / mean_edge},
            {"p99", drift[drift.size() * 99 / 100] / mean_edge},
            {"max", drift.back() / mean_edge},
            {"mean_edge", mean_edge}};
    }
    component_io.result.second = std::move(report);
}

} // namespace

void tracking(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult tracking(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return std::move(context.result);
}

} // namespace wmtk::components::tracking
