#pragma once

#include "SurfaceMap.hpp"

#include <wmtk/history/SurfaceHistory.hpp>

#include <Eigen/Core>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace wmtk::components::surface_map {

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

} // namespace wmtk::components::surface_map
