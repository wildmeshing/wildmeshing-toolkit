#pragma once

#include "CurveMap.hpp"
#include "SurfaceMap.hpp"

#include <wmtk/history/History.hpp>

#include <Eigen/Core>
#include <nlohmann/json.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace wmtk::components::tracking {

/// No face, no vertex: a point that has no image.
constexpr uint64_t kNoId = std::numeric_limits<uint64_t>::max();

/// A point on an application's input: a triangle of its input files -- the triangles of all of
/// them, in order, as the files hold them -- and barycentric coordinates with respect to that
/// triangle's corners in file order.
struct InputPoint
{
    uint64_t face = kNoId;
    Eigen::Vector3d bary = Eigen::Vector3d::Constant(1. / 3);
};

/**
 * @brief A point on an application's output surface.
 *
 * An application whose output is the remeshed surface itself (isotropic_remeshing) names it by
 * a triangle of its output file and barycentric coordinates with respect to its corners in file
 * order. One whose output is a tet mesh (tetwild, simwild), whose tracked surface is no element
 * of the file, names it by three vertices of the tet mesh -- the corners of a tracked face --
 * and barycentric coordinates in their order.
 */
struct OutputPoint
{
    /// Surface outputs: the output triangle.
    uint64_t face = kNoId;
    /// Tet outputs: the output's vertices at the corners of the tracked face holding the point.
    std::array<uint64_t, 3> vertices = {{kNoId, kNoId, kNoId}};
    Eigen::Vector3d bary = Eigen::Vector3d::Constant(1. / 3);
};

/**
 * @brief The map between an application's input and its output, read from the tracking file the
 * application wrote (its history_output option), in the terms its files use.
 *
 * Built from the history alone; see SurfaceMap for how. A point that cannot be mapped -- an
 * input triangle the application dropped, a tracked face the output does not have -- comes back
 * with no face (kNoId), and map().map_warnings() counts it.
 */
class Tracking
{
public:
    explicit Tracking(const history::SurfaceHistory& h, const SurfaceMapOptions& options = {});
    /// The tracking file an application wrote.
    static Tracking load(const std::string& path, const SurfaceMapOptions& options = {});

    /// Whether output points are named by vertex triplets (a tet output) rather than triangles.
    bool output_by_vertices() const { return !m_output_ids.empty(); }

    /// Input points to the output.
    std::vector<OutputPoint> to_output(const std::vector<InputPoint>& points) const;
    /// Output points to the input, the inverse map. A tet output's point may list the face's
    /// three vertices in any order, its barycentric coordinates following them.
    std::vector<InputPoint> to_input(const std::vector<OutputPoint>& points) const;

    /// The output's tracked surface: for a tet output, its tracked faces by the output's vertex
    /// ids, the triplets an OutputPoint may name; for a surface output, its triangles, by index.
    std::vector<std::array<uint64_t, 3>> output_triangles() const;

    /// The triangles of the input files, in order, as the history holds them.
    const std::vector<std::array<uint64_t, 3>>& input_faces() const { return m_map.input_faces(); }
    const SurfaceMap& map() const { return m_map; }

private:
    SurfaceMap m_map;
    /// The output's id of each vertex of the history's last state; empty for a surface output.
    std::vector<uint64_t> m_output_ids;
    /// Output vertex -> vertex of the last state.
    std::map<uint64_t, uint64_t> m_history_vertex;
};

/// A point on a curve application's input: a segment of its input curve files -- the segments of
/// all of them, in order, a polyline's consecutive vertex pairs as the files hold them -- and
/// barycentric coordinates with respect to that segment's ends in file order.
struct InputCurvePoint
{
    uint64_t segment = kNoId;
    Eigen::Vector2d bary = Eigen::Vector2d::Constant(0.5);
};

/**
 * @brief A point on an application's output curves.
 *
 * An application whose output is the curves themselves would name it by a segment of its output
 * file. One whose tracked curves are edges of a triangle mesh (triwild), which are no element of
 * the file, names it by two vertices of the mesh -- the ends of a tracked edge -- and barycentric
 * coordinates in their order.
 */
struct OutputCurvePoint
{
    /// Curve outputs: the output segment.
    uint64_t segment = kNoId;
    /// Triangle mesh outputs: the output's vertices at the ends of the tracked edge holding it.
    std::array<uint64_t, 2> vertices = {{kNoId, kNoId}};
    Eigen::Vector2d bary = Eigen::Vector2d::Constant(0.5);
};

/// A feature point of an application's input -- an end or a junction of its curves, or a point of
/// its point files -- by vertex of its files (the curve files', in order, then the point
/// files'), and the output vertex it ended at, kNoId if none.
struct FeaturePoint
{
    uint64_t input_vertex = kNoId;
    uint64_t output_vertex = kNoId;
};

/**
 * @brief The map between a curve application's input curves and its output's, read from the
 * tracking file the application wrote (its history_output option), in the terms its files use:
 * Tracking's counterpart for curves (triwild).
 *
 * Built from the history alone; see CurveMap for how. A point that cannot be mapped comes back
 * with no segment and no vertices (kNoId), and map().map_warnings() counts it.
 */
class CurveTracking
{
public:
    explicit CurveTracking(const history::CurveHistory& h, const CurveMapOptions& options = {});
    /// The tracking file an application wrote.
    static CurveTracking load(const std::string& path, const CurveMapOptions& options = {});

    /// Whether output points are named by vertex pairs (a triangle mesh output) rather than
    /// segments.
    bool output_by_vertices() const { return !m_output_ids.empty(); }

    /// Input points to the output.
    std::vector<OutputCurvePoint> to_output(const std::vector<InputCurvePoint>& points) const;
    /// Output points to the input, the inverse map. A point named by vertices may list them in
    /// either order, its barycentric coordinates following them.
    std::vector<InputCurvePoint> to_input(const std::vector<OutputCurvePoint>& points) const;

    /// The output's tracked curves: for an output by vertices, its tracked edges by the output's
    /// vertex ids, the pairs an OutputCurvePoint may name; otherwise its segments, by index.
    std::vector<std::array<uint64_t, 2>> output_segments() const;
    /// The segments of the input files, in order, as the history holds them.
    const std::vector<std::array<uint64_t, 2>>& input_segments() const
    {
        return m_map.input_segments();
    }
    /// The input's feature points and where each ended (see FeaturePoint).
    std::vector<FeaturePoint> feature_points() const;
    const CurveMap& map() const { return m_map; }

private:
    CurveMap m_map;
    /// The output's id of each vertex of the history's last state; empty for a curve output.
    std::vector<uint64_t> m_output_ids;
    /// Output vertex -> vertex of the last state.
    std::map<uint64_t, uint64_t> m_history_vertex;
};

/// The tracking application: maps points, transfers fields and reports on a tracking file
/// (tracking_spec.json).
void tracking(nlohmann::json json_params);

} // namespace wmtk::components::tracking
