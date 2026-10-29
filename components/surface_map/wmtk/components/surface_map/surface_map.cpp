#include "surface_map.hpp"
#include "SurfaceMap.hpp"
#include "Tracking.hpp"
#include "surface_io.hpp"
#include "transfer.hpp"

#include <igl/Timer.h>
#include <jse/jse.h>
#include <fstream>
#include <sstream>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <surface_map_spec.hpp>

#include <algorithm>
#include <cmath>

namespace wmtk::components::surface_map {

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
    const std::string output_points = json_params["output_points"];
    if (output_points.empty()) log_and_throw_error("map: output_points is required");
    const bool forward = json_params["direction"] == "to_output";
    igl::Timer timer;
    timer.start();
    const Tracking tracking(h, options_of(json_params));
    logger().info("map built in {:.3}s", timer.getElapsedTimeInSec());
    const size_t n_out = tracking.output_by_vertices() ? 6 : 4;
    std::ofstream out(output_points);
    if (!out) log_and_throw_error("Could not write {}", output_points);
    out.precision(17);
    size_t lost = 0, total = 0;
    timer.start();
    if (forward) {
        std::vector<InputPoint> pts;
        for (const auto& r : read_points(points_path, 4)) {
            pts.push_back({id_of(r[0]), Eigen::Vector3d(r[1], r[2], r[3])});
        }
        total = pts.size();
        for (const OutputPoint& p : tracking.to_output(pts)) {
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
        for (const InputPoint& p : tracking.to_input(pts)) {
            lost += p.face == kNoId;
            out << id_text(p.face) << " " << p.bary[0] << " " << p.bary[1] << " " << p.bary[2]
                << "\n";
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

void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "surface_map");
    using wmtk::utils::resolve_path;
    {
        const auto spec = jse::embed::wmtk_surface_map_spec::surface_map_spec::spec();
        jse::JSE spec_engine;
        if (!spec_engine.verify_json(json_params, spec)) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }
    // The history is a file on both routes; map and transfer read and write files too (points,
    // the input surface, the transferred meshes), which have no in-memory form yet.
    if (component_io.memory() && json_params["operation"] != "info") {
        throw std::runtime_error(
            "surface_map: operation " + json_params["operation"].get<std::string>() +
            " reads and writes files; on the memory route only info is supported");
    }
    const std::filesystem::path root = json_params["input_dir"];
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            set_file_logger(resolve_path(root, log_file_name).string());
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string history_path = resolve_path(root, json_params["history"]).string();
    igl::Timer timer;
    timer.start();
    const history::SurfaceHistory h = history::SurfaceHistory::load(history_path);
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
        // The input: as given, or the files the history's application read, from its parameters.
        std::vector<std::string> inputs;
        if (!std::string(json_params["input"]).empty()) {
            inputs.push_back(resolve_path(root, json_params["input"]).string());
        } else {
            const auto app = nlohmann::json::parse(h.metadata, nullptr, false);
            if (app.is_discarded() || !app.contains("input")) {
                log_and_throw_error("{} names no input; give one", history_path);
            }
            const std::filesystem::path app_root =
                app.contains("input_dir") ? std::filesystem::path(std::string(app["input_dir"]))
                                          : std::filesystem::path();
            const auto& in_json = app["input"];
            for (const auto& p : in_json.is_array() ? in_json : nlohmann::json::array({in_json})) {
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
        const std::string input = nlohmann::json(inputs).dump();
        const std::string output = json_params["output"];
        const SurfaceData in = read_surface(inputs);
        std::string why;
        if (!matches_input(map, in, why)) {
            log_and_throw_error(
                "{} is not the surface {} starts from: {}",
                input,
                history_path,
                why);
        }
        timer.start();
        const SurfaceData out = transfer_fields(map, in);
        log_warnings("output centroids to the input", map.map_warnings());
        write_surface_msh(output, out);
        const std::string tet_mesh = json_params["tet_mesh"];
        const std::string output_tet_mesh = json_params["output_tet_mesh"];
        if (!tet_mesh.empty()) {
            if (output_tet_mesh.empty())
                log_and_throw_error("tet_mesh given without output_tet_mesh");
            write_surface_into_msh(resolve_path(root, tet_mesh).string(), output_tet_mesh, out);
            logger().info(
                "fields written onto the tracked surface of {}: {}",
                tet_mesh,
                output_tet_mesh);
        }
        const std::string output_obj = json_params["output_obj"];
        if (!output_obj.empty()) {
            write_surface_obj(output_obj, out, json_params["uv_field"], json_params["texture"]);
            logger().info("textured output written to {}", output_obj);
        }
        logger().info(
            "{} field(s) carried from {} faces onto {} in {:.3}s; written to {}",
            out.face_fields.size() + out.corner_fields.size(),
            in.faces.size(),
            out.faces.size(),
            timer.getElapsedTimeInSec(),
            output);
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
    const auto in = map.to_input(centroids);
    log_warnings("output centroids to the input", map.map_warnings());
    report["to_input_warnings"] = warnings_report(map.map_warnings());
    const auto back = map.to_output(in);
    log_warnings("and back to the output", map.map_warnings());
    report["to_output_warnings"] = warnings_report(map.map_warnings());
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

void surface_map(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult surface_map(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return std::move(context.result);
}

} // namespace wmtk::components::surface_map
