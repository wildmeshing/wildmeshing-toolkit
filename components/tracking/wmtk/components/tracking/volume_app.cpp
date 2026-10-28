#include "volume_app.hpp"

#include "VolumeMap.hpp"
#include "tracking.hpp"
#include "volume_io.hpp"

#include <wmtk/utils/getRSS.h>
#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <igl/Timer.h>
#include <Eigen/Dense>

#include <algorithm>
#include <fstream>
#include <map>
#include <random>
#include <set>
#include <sstream>

namespace wmtk::components::tracking {

namespace {

VolumeMap load_map(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root)
{
    const std::string path = json_params["volume_history"];
    if (path.empty()) log_and_throw_error("volume operations need volume_history");
    auto vh = history::VolumeHistory::load(wmtk::utils::resolve_path(root, path).string());
    logger().info(
        "loaded {} operations around the tracked surface, {} regions",
        vh.edits.size(),
        vh.regions.size());
    return VolumeMap(h, std::move(vh));
}

void log_stats(const VolumeMap& map)
{
    const auto& s = map.stats();
    logger().info(
        "operation maps built: {}, failed: {}; patch pairs on their real geometry: {}, on a "
        "fallback embedding: {}; decompositions: {}",
        s.built,
        s.failed,
        s.real_pairs,
        s.fallback_pairs,
        s.decompositions);
    for (const auto& [why, n] : s.failures) logger().warn("  {} operation(s): {}", n, why);
    if (s.surface_not_kept > 0) {
        logger().warn(
            "{} operation(s) mapped as a bijection of the volume that does not keep the tracked "
            "surface (the boundary of an open sheet, a weld)",
            s.surface_not_kept);
    }
    if (s.region_changed + s.through_failed > 0) {
        logger().warn(
            "points that changed region: {}; through an operation with no map: {}",
            s.region_changed,
            s.through_failed);
    }
}

double percentile(std::vector<double> x, const double p)
{
    if (x.empty()) return 0;
    std::sort(x.begin(), x.end());
    return x[std::min(x.size() - 1, size_t(p / 100 * double(x.size())))];
}

} // namespace

void volume_info(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root)
{
    using wmtk::utils::resolve_path;
    igl::Timer timer;
    timer.start();
    const VolumeMap map = load_map(h, json_params, root);
    logger().info(
        "surface map and tracked faces in {:.3}s; memory {} MB",
        timer.getElapsedTimeInSec(),
        getCurrentRSS() >> 20);
    const std::string tet_mesh = json_params["tet_mesh"];
    const std::string output_tet_mesh = json_params["output_tet_mesh"];
    if (tet_mesh.empty()) {
        timer.start();
        map.build_all();
        logger().info("every operation's map built in {:.3}s", timer.getElapsedTimeInSec());
        log_stats(map);
        return;
    }
    // Points near the tracked surface of the mesh the history starts from: in the tets with a
    // vertex on a face between two regions, by volume, at random barycentric coordinates.
    const TetMeshData in = read_tet_msh(resolve_path(root, tet_mesh).string());
    std::vector<std::pair<std::array<uint64_t, 3>, size_t>> face_tets;
    face_tets.reserve(4 * in.tets.size());
    for (size_t t = 0; t < in.tets.size(); ++t) {
        for (int k = 0; k < 4; ++k) {
            std::array<uint64_t, 3> f;
            for (int c = 0, i = 0; c < 4; ++c) {
                if (c != k) f[i++] = in.tets[t][c];
            }
            std::sort(f.begin(), f.end());
            face_tets.emplace_back(f, t);
        }
    }
    std::sort(face_tets.begin(), face_tets.end());
    std::vector<char> on_surface(in.vertices.size(), 0);
    for (size_t i = 0; i + 1 < face_tets.size(); ++i) {
        const auto& [f, a] = face_tets[i];
        const auto& [g, b] = face_tets[i + 1];
        if (f == g && in.groups[a] != in.groups[b]) {
            for (const uint64_t v : f) on_surface[v] = 1;
        }
    }
    face_tets = {};
    // One region all round (tetwild): near the tracked surface where the volume history
    // starts -- the vertices at the surface history's positions there.
    bool any_interface =
        std::any_of(on_surface.begin(), on_surface.end(), [](char c) { return c; });
    if (!any_interface) {
        history::Replay r(h);
        for (uint64_t k = 0; k < map.history().surface_edit_offset && k < h.edits.size(); ++k) {
            r.apply(h.edits[k]);
        }
        std::set<std::array<double, 3>> surface;
        for (uint64_t v = 0; v < r.vertices().size(); ++v) {
            if (r.vertex_alive()[v])
                surface.insert({{r.vertices()[v][0], r.vertices()[v][1], r.vertices()[v][2]}});
        }
        for (size_t v = 0; v < in.vertices.size(); ++v) {
            const auto& x = in.vertices[v];
            on_surface[v] = surface.count({{x[0], x[1], x[2]}}) > 0;
        }
    }
    std::vector<size_t> at_surface;
    std::vector<double> vol;
    for (size_t t = 0; t < in.tets.size(); ++t) {
        const auto& T = in.tets[t];
        if (!(on_surface[T[0]] || on_surface[T[1]] || on_surface[T[2]] || on_surface[T[3]]))
            continue;
        at_surface.push_back(t);
        const auto& P = in.vertices;
        vol.push_back(
            std::abs((P[T[1]] - P[T[0]]).dot((P[T[2]] - P[T[0]]).cross(P[T[3]] - P[T[0]]))));
    }
    if (at_surface.empty())
        log_and_throw_error("{} has no tracked surface to sample near", tet_mesh);
    Eigen::Vector3d lo = in.vertices[0], hi = lo;
    for (const auto& v : in.vertices) {
        lo = lo.cwiseMin(v);
        hi = hi.cwiseMax(v);
    }
    const double diag = (hi - lo).norm();
    const size_t n = json_params["samples"];
    std::mt19937 rng(1);
    std::discrete_distribution<size_t> pick(vol.begin(), vol.end());
    std::uniform_real_distribution<double> u(0, 1);
    std::vector<VolumePoint> pts(4 * n);
    std::vector<size_t> from(n);
    const double eps = 1e-9 * diag;
    for (size_t i = 0; i < n; ++i) {
        from[i] = at_surface[pick(rng)];
        Eigen::Vector4d w(u(rng), u(rng), u(rng), u(rng));
        w /= w.sum();
        Eigen::Vector3d p = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) p += w[k] * in.vertices[in.tets[from[i]][k]];
        pts[i].position = p;
        for (int k = 0; k < 3; ++k)
            pts[n * (k + 1) + i].position = p + eps * Eigen::Vector3d::Unit(k);
    }
    logger().info("sampled; memory {} MB", getCurrentRSS() >> 20);
    timer.start();
    const auto there = map.to_output(pts);
    const double t_forward = timer.getElapsedTimeInSec();
    timer.start();
    const auto back = map.to_input(there);
    const double t_back = timer.getElapsedTimeInSec();
    logger().info("mapped there and back; memory {} MB", getCurrentRSS() >> 20);
    std::vector<double> conf, det, disp, trip;
    size_t moved = 0, kept = 0, located = 0;
    std::unique_ptr<TetMeshData> out;
    std::unique_ptr<TetLocator> loc;
    if (!output_tet_mesh.empty()) {
        out = std::make_unique<TetMeshData>(
            read_tet_msh(resolve_path(root, output_tet_mesh).string()));
        loc = std::make_unique<TetLocator>(*out);
    }
    for (size_t i = 0; i < n; ++i) {
        const Eigen::Vector3d y = there[i].position;
        Eigen::Matrix3d J;
        for (int k = 0; k < 3; ++k) J.col(k) = (there[n * (k + 1) + i].position - y) / eps;
        const Eigen::Vector3d sv = J.jacobiSvd().singularValues();
        const double d = (y - pts[i].position).norm() / diag;
        trip.push_back((back[i].position - pts[i].position).norm() / diag);
        if (loc) {
            const auto [t, b] = loc->locate(y);
            located += t >= 0;
            const auto& want = in.group_sets[in.groups[from[i]]];
            const auto g = std::find(out->group_sets.begin(), out->group_sets.end(), want);
            kept += g != out->group_sets.end() &&
                    loc->in_group(y, uint32_t(g - out->group_sets.begin()));
        }
        if (there[i].position == pts[i].position) continue; // never in an operation's tets
        ++moved;
        conf.push_back(sv[0] / sv[2]);
        det.push_back(J.determinant());
        disp.push_back(d);
    }
    log_stats(map);
    logger().info(
        "{} points near the tracked surface, {} moved; forward {:.3}s, back {:.3}s",
        n,
        moved,
        t_forward,
        t_back);
    logger().info(
        "composite, moved points: conformal p50 {:.3f} p99 {:.3g} max {:.3g} | det J p1 {:.3g}, "
        "<= 0: {} | moved / bbox diagonal p99 {:.2g} max {:.2g}",
        percentile(conf, 50),
        percentile(conf, 99),
        percentile(conf, 100),
        percentile(det, 1),
        std::count_if(det.begin(), det.end(), [](double x) { return x <= 0; }),
        percentile(disp, 99),
        percentile(disp, 100));
    if (loc) logger().info("regions kept: {} of {} ({} located in the output)", kept, n, located);
    logger().info(
        "round trip / bbox diagonal: p99 {:.2g} max {:.2g}",
        percentile(trip, 99),
        percentile(trip, 100));
}

void map_volume_points(
    const history::SurfaceHistory& h,
    const nlohmann::json& json_params,
    const std::filesystem::path& root)
{
    using wmtk::utils::resolve_path;
    const std::string points_path = resolve_path(root, json_params["points"]).string();
    const std::string output_points = json_params["output_points"];
    if (output_points.empty()) log_and_throw_error("map_volume: output_points is required");
    std::ifstream in(points_path);
    if (!in) log_and_throw_error("Could not read {}", points_path);
    std::vector<std::vector<double>> rows;
    std::string line;
    while (std::getline(in, line)) {
        line = line.substr(0, line.find('#'));
        std::istringstream ss(line);
        std::vector<double> row;
        for (double x; ss >> x;) row.push_back(x);
        if (row.empty()) continue;
        if (row.size() != 3 && row.size() != 5) {
            log_and_throw_error("{}: `x y z` or `tet b0 b1 b2 b3` per line", points_path);
        }
        rows.push_back(std::move(row));
    }
    const bool forward = json_params["direction"] == "to_output";
    std::ofstream out(output_points);
    if (!out) log_and_throw_error("Could not write {}", output_points);
    out.precision(17);
    const std::string tet_mesh = json_params["tet_mesh"];
    const std::string output_tet_mesh = json_params["output_tet_mesh"];
    if (!rows.empty() && rows[0].size() == 5) {
        // tets of the meshes: tet_mesh the history starts from, output_tet_mesh it ends at
        if (tet_mesh.empty() || output_tet_mesh.empty()) {
            log_and_throw_error("map_volume: tet points need tet_mesh and output_tet_mesh");
        }
        const std::string vh = json_params["volume_history"];
        const VolumeTracking tracking = VolumeTracking::load(
            resolve_path(root, json_params["history"]).string(),
            resolve_path(root, vh).string(),
            resolve_path(root, tet_mesh).string(),
            resolve_path(root, output_tet_mesh).string());
        std::vector<TetPoint> pts;
        for (const auto& r : rows) {
            if (r.size() != 5) log_and_throw_error("{}: mixed point kinds", points_path);
            pts.push_back(
                {r[0] < 0 ? kNoId : uint64_t(r[0]), Eigen::Vector4d(r[1], r[2], r[3], r[4])});
        }
        size_t lost = 0;
        for (const TetPoint& p : forward ? tracking.to_output(pts) : tracking.to_input(pts)) {
            lost += p.tet == kNoId;
            out << (p.tet == kNoId ? std::string("-1") : std::to_string(p.tet));
            for (int k = 0; k < 4; ++k) out << " " << p.bary[k];
            out << "\n";
        }
        log_stats(tracking.map());
        logger().info(
            "{} points mapped, {} with no tet, written to {}",
            pts.size(),
            lost,
            output_points);
        return;
    }
    const VolumeMap map = load_map(h, json_params, root);
    std::vector<VolumePoint> pts;
    for (const auto& r : rows) {
        if (r.size() != 3) log_and_throw_error("{}: mixed point kinds", points_path);
        pts.push_back({Eigen::Vector3d(r[0], r[1], r[2]), VolumePoint::kAnyRegion});
    }
    const auto there = forward ? map.to_output(pts) : map.to_input(pts);
    for (const VolumePoint& p : there) {
        out << p.position[0] << " " << p.position[1] << " " << p.position[2];
        if (p.region != VolumePoint::kAnyRegion) {
            for (const int64_t t : map.history().regions[p.region]) out << " " << t;
        }
        out << "\n";
    }
    log_stats(map);
    logger().info("{} points mapped, written to {}", pts.size(), output_points);
}

} // namespace wmtk::components::tracking
