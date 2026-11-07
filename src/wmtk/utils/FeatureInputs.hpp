#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/io/ReadProvenance.hpp>
#include <wmtk/utils/EmbedTriangles.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <filesystem>
#include <string>
#include <vector>

namespace wmtk::io {
class ComponentIO;
}

namespace wmtk::utils {

/**
 * @brief The feature curves and points a 3D application forces into its tetrahedralization
 * (its curve and point inputs), as read.
 *
 * One implementation for tetwild and simwild, so the two read, insert and hold them alike:
 * read_feature_inputs() reads them, extend_bbox() grows the input bounding box over them,
 * embed_features_input() marshals them for embed_triangles_in_tets(), and the mesh holds what
 * the insertion made of them (TetOptimizerMesh::tag_feature_edges(),
 * anchor_feature_points() with placed_point_vertices()).
 */
struct FeatureInputs
{
    /// The vertices of every curve input, in input order, welded by exact position: vertices at
    /// the same coordinates, bit for bit, in one input or in two, are one, the first of them.
    /// So the ends of two curves that touch are one vertex, as the insertion places them, which
    /// the joined curve passes through. Until 2026-11 they were concatenated as given, and the input's curves had two
    /// components where the output, and the insertion, have one: under preserve_topology with
    /// throw_on_fail, tetwild's end-of-run check threw "Feature-curve topology was not preserved"
    /// on a run that kept it.
    std::vector<Vector3d> edge_vertices;
    /// The feature edges, into edge_vertices. A segment whose ends weld into one is dropped: it
    /// tiles nothing, and the insertion refuses it ("forced edge has coincident endpoints"), as
    /// it refused a segment between two vertices at one position before the weld.
    std::vector<std::array<size_t, 2>> edges;
    /// Per edge, the curve input it came from (its position among the curve inputs): the edge's
    /// input, as an input surface is a tracked face's (TetOptimizerMesh::tag_feature_edges()).
    std::vector<int> edge_input;
    /// The feature points: every vertex of the point inputs, and every vertex of a curve input
    /// with no incident edge.
    std::vector<Vector3d> points;
    /// Per point, the column of its label in point_names: the name of the input it is a vertex
    /// of. The names in the order they first label a point. Empty without the inputs' names
    /// (read_feature_inputs()): the points are then not labelled.
    std::vector<int> point_input;
    std::vector<std::string> point_names;

    bool empty() const { return edges.empty() && points.empty(); }
    /// Grow [lo, hi] over every feature vertex and point. They are input geometry, and a
    /// feature outside the surface's box must still land inside the triangulated domain.
    void extend_bbox(Vector3d& lo, Vector3d& hi) const;
};

/**
 * @brief Read the curve inputs and the point inputs (io::ComponentIO::input_kind()), each
 * selector resolved against `root` (a mesh name on the memory route).
 *
 * Curves are concatenated with their indices offset. A vertex of a curve input with no incident
 * edge is a feature point, the same rule as triwild's 2D inputs. With the inputs' names, one per
 * selector, each point is labelled by its input's (point_input, point_names).
 *
 * `read`, if given, gets the inputs as read, for a curve history to start from them
 * (history::curves_from_features()): the curve inputs' vertices and segments as given,
 * concatenated in input order, then the point inputs' vertices, as triwild appends its points;
 * and where each went: a curve vertex to its welded vertex of edge_vertices, a point to its row
 * after them, a segment to its row of edges (-1 for one of no length).
 */
FeatureInputs read_feature_inputs(
    const io::ComponentIO& component_io,
    const std::filesystem::path& root,
    const nlohmann::json& edge_selectors,
    const nlohmann::json& point_selectors,
    const std::vector<std::string>& edge_names = {},
    const std::vector<std::string>& point_names = {},
    io::ReadProvenance* read = nullptr);

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
