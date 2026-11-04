#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/utils/EmbedTriangles.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <filesystem>
#include <vector>

namespace wmtk::io {
class ComponentIO;
}

namespace wmtk::utils {

/**
 * @brief The feature curves and points a 3D application forces into its tetrahedralization
 * (input_edges, input_points), as read.
 *
 * One implementation for tetwild and simwild, so the two read, insert and hold them alike:
 * read_feature_inputs() reads them, extend_bbox() grows the input bounding box over them,
 * embed_features_input() marshals them for embed_triangles_in_tets(), and the mesh holds what
 * the insertion made of them (TetOptimizerMesh::init_feature_envelope(), tag_feature_edges(),
 * anchor_feature_points() with placed_point_vertices()).
 */
struct FeatureInputs
{
    /// The vertices of every feature edge file, concatenated in file order.
    std::vector<Vector3d> edge_vertices;
    /// The feature edges, into edge_vertices.
    std::vector<std::array<size_t, 2>> edges;
    /// The feature points: every vertex of the point files, and every vertex of an edge file
    /// with no incident edge.
    std::vector<Vector3d> points;

    bool empty() const { return edges.empty() && points.empty(); }
    /// Grow [lo, hi] over every feature vertex and point. They are input geometry, and a
    /// feature outside the surface's box must still land inside the triangulated domain.
    void extend_bbox(Vector3d& lo, Vector3d& hi) const;
};

/**
 * @brief Read the feature edge files and point files, each selector resolved against `root`
 * (a mesh name on the memory route).
 *
 * Edge files are concatenated with their indices offset. A vertex of an edge file with no
 * incident edge is a feature point, the same rule as triwild's 2D inputs. A point file's edges
 * are ignored, with a warning.
 */
FeatureInputs read_feature_inputs(
    const io::ComponentIO& component_io,
    const std::filesystem::path& root,
    const nlohmann::json& edge_selectors,
    const nlohmann::json& point_selectors);

/**
 * @brief Marshal feature edges and points into the remesher's flat arrays, and check the
 * domain.
 *
 * The features AND their forcing-triangle apexes -- which stick out by up to 0.1x the edge
 * length along an axis -- must lie inside the background box [box_min, box_max], or the
 * remesher's non-Delaunay insertion cannot locate them. The caller grows the input bbox over
 * the features (FeatureInputs::extend_bbox()), so a violation means only one thing: a feature
 * edge so long that a tenth of it exceeds the box padding (diag/15). Thrown with that diagnosis
 * rather than letting the remesher assert.
 */
EmbedFeaturesInput embed_features_input(
    const std::vector<Vector3d>& edge_vertices,
    const std::vector<std::array<size_t, 2>>& edges,
    const std::vector<Vector3d>& points,
    const Vector3d& box_min,
    const Vector3d& box_max);

/// The output vertex of each input point, in input order. Each input point is the output vertex
/// exactly equal to it; one the insertion dropped is a contract violation, and throws.
std::vector<size_t> placed_point_vertices(const EmbedFeaturesResult& placed);

} // namespace wmtk::utils
