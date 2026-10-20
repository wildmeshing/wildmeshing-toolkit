#include "surface_map.hpp"
#include "SurfaceMap.hpp"
#include "surface_io.hpp"
#include "transfer.hpp"

#include <igl/Timer.h>
#include <jse/jse.h>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <surface_map_spec.hpp>

#include <algorithm>
#include <cmath>

namespace wmtk::components::surface_map {

namespace {

void log_warnings(const char* what, const MapWarnings& w)
{
    logger().info(
        "{}: {} warning(s) -- not a disk {}, flat triangle {}, inverted {}, merged faces {}, "
        "unlocated {}",
        what,
        w.total(),
        w.not_a_disk,
        w.flat_triangle,
        w.inverted,
        w.merged_faces,
        w.unlocated);
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

} // namespace

void surface_map(nlohmann::json json_params)
{
    using wmtk::utils::resolve_path;
    {
        const auto spec = jse::embed::wmtk_surface_map_spec::surface_map_spec::spec();
        jse::JSE spec_engine;
        if (!spec_engine.verify_json(json_params, spec)) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
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

    timer.start();
    const SurfaceMap map(h);
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
        const std::string input = resolve_path(root, json_params["input"]).string();
        const std::string output = json_params["output"];
        const SurfaceData in = read_surface_msh(input);
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
        for (int c = 0; c < 3; ++c) {
            auto& a = min_angles[c];
            if (a.empty()) continue;
            std::sort(a.begin(), a.end());
            const auto below = [&a](double t) {
                return size_t(std::lower_bound(a.begin(), a.end(), t) - a.begin());
            };
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
    const auto back = map.to_output(in);
    log_warnings("and back to the output", map.map_warnings());
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
}

} // namespace wmtk::components::surface_map
