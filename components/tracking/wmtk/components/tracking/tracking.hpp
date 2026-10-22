#pragma once

#include "SurfaceMap.hpp"

#include <wmtk/history/SurfaceHistory.hpp>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <vector>

namespace wmtk::components::tracking {

/// No face, no vertex: a point that has no image.
constexpr uint64_t kNoId = std::numeric_limits<uint64_t>::max();

/**
 * @brief A point on an application's input.
 *
 * A history that starts at input files (every application but simwild on a tet mesh) names it by
 * a triangle of those files -- the triangles of all of them, in order, as the files hold them --
 * and barycentric coordinates with respect to that triangle's corners in file order. One that
 * starts at a tet mesh's tracked surface (simwild on a .msh), which is no element of a file,
 * names it by three vertices of that mesh -- the corners of a tracked face -- and barycentric
 * coordinates in their order, as an OutputPoint of a tet output does.
 */
struct InputPoint
{
    /// Histories from input files: the input triangle.
    uint64_t face = kNoId;
    Eigen::Vector3d bary = Eigen::Vector3d::Constant(1. / 3);
    /// Histories from a tet mesh: its vertices at the corners of the tracked face holding the
    /// point. Last, so that {face, bary} still names a point on input files.
    std::array<uint64_t, 3> vertices = {{kNoId, kNoId, kNoId}};
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
 * with no face (kNoId), and the warnings of the call count it. Mapping never changes the object:
 * several threads may map through one at once.
 */
class Tracking
{
public:
    explicit Tracking(const history::SurfaceHistory& h, const SurfaceMapOptions& options = {});
    /// The tracking file an application wrote.
    static Tracking load(const std::string& path, const SurfaceMapOptions& options = {});

    /// Whether output points are named by vertex triplets (a tet output) rather than triangles.
    bool output_by_vertices() const { return !m_output.ids.empty(); }
    /// Whether input points are named by vertex triplets (a history that starts at a tet mesh's
    /// tracked surface) rather than triangles of the input files.
    bool input_by_vertices() const { return !m_input.ids.empty(); }

    /// Input points to the output. `warnings`, if given, gets where this call found the map not
    /// a bijection (SurfaceMap::to_output).
    std::vector<OutputPoint> to_output(
        const std::vector<InputPoint>& points,
        MapWarnings* warnings = nullptr) const;
    /// Output points to the input, the inverse map. A point named by vertices, on either side,
    /// may list the face's three vertices in any order, its barycentric coordinates following
    /// them.
    std::vector<InputPoint> to_input(
        const std::vector<OutputPoint>& points,
        MapWarnings* warnings = nullptr) const;

    /// The output's tracked surface: for a tet output, its tracked faces by the output's vertex
    /// ids, the triplets an OutputPoint may name; for a surface output, its triangles, by index.
    std::vector<std::array<uint64_t, 3>> output_triangles() const;

    /// The input's tracked surface likewise: for a history from a tet mesh, its tracked faces by
    /// the mesh's vertex ids, the triplets an InputPoint may name; otherwise the triangles of the
    /// input files, by index.
    std::vector<std::array<uint64_t, 3>> input_triangles() const;

    /// The triangles of the input files, in order, as the history holds them.
    const std::vector<std::array<uint64_t, 3>>& input_faces() const { return m_map.input_faces(); }
    const SurfaceMap& map() const { return m_map; }

private:
    /// A side of the map whose points may be named by the vertices of a mesh: the mesh's id of
    /// each vertex of the history's state there (empty: points are named by faces), and back.
    struct Naming
    {
        std::vector<uint64_t> ids;
        std::map<uint64_t, uint64_t> history_vertex;
    };
    /// A point named by vertices, as a point of `faces`: the live face with those corners, the
    /// coordinates in its corner order. No face if none has them.
    static SurfacePoint by_vertices(
        const Naming& naming,
        const std::vector<std::array<uint64_t, 3>>& faces,
        const std::map<std::array<uint64_t, 3>, uint64_t>& face_of,
        const std::array<uint64_t, 3>& vertices,
        const Eigen::Vector3d& bary);
    /// The live faces by their sorted corners.
    static std::map<std::array<uint64_t, 3>, uint64_t> faces_by_corners(
        const std::vector<std::array<uint64_t, 3>>& faces,
        const std::vector<uint8_t>& alive);
    /// The live faces of a side, by the naming's vertex ids if it has them.
    static std::vector<std::array<uint64_t, 3>> triangles(
        const Naming& naming,
        const std::vector<std::array<uint64_t, 3>>& faces,
        const std::vector<uint8_t>& alive);

    SurfaceMap m_map;
    /// The output's vertex ids (empty for a surface output) and the input mesh's (empty for a
    /// history from files).
    Naming m_output, m_input;
    /// The live faces of the last and the first state by their sorted corners, for the sides
    /// named by vertices.
    std::map<std::array<uint64_t, 3>, uint64_t> m_output_face_of, m_input_face_of;
};

/// The tracking application: maps points, transfers fields and reports on a tracking file
/// (tracking_spec.json).
void tracking(nlohmann::json json_params);
/**
 * @brief The memory route. `history` is the bytes of a history (a JSON binary value: a
 * component's report["history"]), never a file name. info returns its report. transfer takes its
 * surface from `meshes` by `input`, which it needs, and returns the output surface as "mesh",
 * the carried fields as triangle attributes; with `tet_mesh`, also a key of `meshes`, that mesh
 * with the fields on its tracked surface as "tet_mesh" (add_surface). map, whose points are
 * text files, throws.
 */
ComponentResult tracking(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::tracking
