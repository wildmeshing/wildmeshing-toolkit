#include "FeatureInputs.hpp"

#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Logger.hpp>

#include <string>

namespace wmtk::utils {

void FeatureInputs::extend_bbox(Vector3d& lo, Vector3d& hi) const
{
    for (const Vector3d& p : edge_vertices) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    for (const Vector3d& p : points) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
}

FeatureInputs read_feature_inputs(
    const io::ComponentIO& component_io,
    const std::filesystem::path& root,
    const nlohmann::json& edge_selectors,
    const nlohmann::json& point_selectors)
{
    FeatureInputs out;
    std::vector<std::string> edge_paths, point_paths;
    for (const auto& s : edge_selectors) edge_paths.push_back(component_io.path(root, s));
    for (const auto& s : point_selectors) point_paths.push_back(component_io.path(root, s));
    for (const std::string& path : edge_paths) {
        MatrixXd Ve;
        MatrixXi Ee;
        component_io.edges(path, Ve, Ee);
        logger().info("Read feature edge mesh {}: #V = {}, #E = {}", path, Ve.rows(), Ee.rows());
        const size_t base = out.edge_vertices.size();
        for (int i = 0; i < Ve.rows(); ++i) {
            out.edge_vertices.emplace_back(Ve.row(i));
        }
        for (int i = 0; i < Ee.rows(); ++i) {
            out.edges.push_back({{base + size_t(Ee(i, 0)), base + size_t(Ee(i, 1))}});
        }
        // A vertex of an edge file with no incident edge is a free point, same rule as
        // triwild's 2D inputs.
        std::vector<int> valence(Ve.rows(), 0);
        for (int i = 0; i < Ee.rows(); ++i) {
            ++valence[Ee(i, 0)];
            ++valence[Ee(i, 1)];
        }
        for (int v = 0; v < Ve.rows(); ++v) {
            if (valence[v] == 0) {
                out.points.emplace_back(Ve.row(v));
            }
        }
    }
    for (const std::string& path : point_paths) {
        MatrixXd Vp;
        MatrixXi Ep;
        component_io.edges(path, Vp, Ep);
        if (Ep.rows() > 0) {
            logger().warn(
                "input_points file {} has {} edges; only its {} vertices are used",
                path,
                Ep.rows(),
                Vp.rows());
        }
        logger().info("Read feature point file {}: #P = {}", path, Vp.rows());
        for (int i = 0; i < Vp.rows(); ++i) {
            out.points.emplace_back(Vp.row(i));
        }
    }
    return out;
}

EmbedFeaturesInput embed_features_input(
    const std::vector<Vector3d>& edge_vertices,
    const std::vector<std::array<size_t, 2>>& edges,
    const std::vector<Vector3d>& points,
    const Vector3d& lo,
    const Vector3d& hi)
{
    EmbedFeaturesInput vr_features;
    const auto in_box = [&](const Vector3d& p, double margin) {
        for (int d = 0; d < 3; ++d) {
            if (p[d] - margin < lo[d] || p[d] + margin > hi[d]) {
                return false;
            }
        }
        return true;
    };
    vr_features.edge_vrt_coord.reserve(3 * edge_vertices.size());
    for (const Vector3d& p : edge_vertices) {
        vr_features.edge_vrt_coord.insert(vr_features.edge_vrt_coord.end(), p.data(), p.data() + 3);
    }
    vr_features.edge_indices.reserve(2 * edges.size());
    for (const auto& e : edges) {
        const Vector3d& a = edge_vertices.at(e[0]);
        const Vector3d& b = edge_vertices.at(e[1]);
        const double apex_margin = 0.1 * (a - b).norm();
        if (!in_box(a, apex_margin) || !in_box(b, apex_margin)) {
            log_and_throw_error(
                "Feature edge ({}, {}) or its forcing-triangle apexes (offset 0.1 x its "
                "length {}) leave the background box [{}, {}]; the edge is too long "
                "relative to the box padding",
                a.transpose(),
                b.transpose(),
                (a - b).norm(),
                lo.transpose(),
                hi.transpose());
        }
        vr_features.edge_indices.push_back(uint32_t(e[0]));
        vr_features.edge_indices.push_back(uint32_t(e[1]));
    }
    // Point apexes are offset by 0.1x the estimated point spacing, itself bounded by the
    // point set's own bbox diagonal -- small against the box padding; checked with the
    // conservative margin of the padding itself would over-reject, so points are checked
    // with no margin and the remesher's own debug assert stays as the backstop.
    vr_features.point_coord.reserve(3 * points.size());
    for (const Vector3d& p : points) {
        if (!in_box(p, 0)) {
            log_and_throw_error(
                "Feature point ({}) lies outside the background box [{}, {}]",
                p.transpose(),
                lo.transpose(),
                hi.transpose());
        }
        vr_features.point_coord.insert(vr_features.point_coord.end(), p.data(), p.data() + 3);
    }
    return vr_features;
}

std::vector<size_t> placed_point_vertices(const EmbedFeaturesResult& placed)
{
    std::vector<size_t> vids;
    for (const int64_t v : placed.point_vertex) {
        if (v < 0) {
            log_and_throw_error("An input point is not a vertex of the initial mesh");
        }
        vids.push_back(size_t(v));
    }
    return vids;
}

} // namespace wmtk::utils
