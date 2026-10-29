#pragma once

#include "CurveMap.hpp"
#include "SurfaceMap.hpp"
#include "VolumeMap.hpp"
#include "volume_io.hpp"

#include <wmtk/history/History.hpp>

#include <Eigen/Core>
#include <nlohmann/json.hpp>
#include <wmtk/MeshData.hpp>

#include <array>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <optional>
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
 * with no segment and no vertices (kNoId), and the warnings of the call count it. Mapping never
 * changes the object: several threads may map through one at once.
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

    /// Input points to the output. `warnings`, if given, gets where this call found the map not
    /// a bijection (CurveMap::to_output).
    std::vector<OutputCurvePoint> to_output(
        const std::vector<InputCurvePoint>& points,
        CurveMapWarnings* warnings = nullptr) const;
    /// Output points to the input, the inverse map. A point named by vertices may list them in
    /// either order, its barycentric coordinates following them.
    std::vector<InputCurvePoint> to_input(
        const std::vector<OutputCurvePoint>& points,
        CurveMapWarnings* warnings = nullptr) const;

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

/// A map's warnings as a report holds them: each count by its name, and their total.
nlohmann::json warnings_report(const MapWarnings& w);
nlohmann::json warnings_report(const CurveMapWarnings& w);
/// A volume map's stats as a report holds them: each count by its name, the failures' reasons
/// with their counts.
nlohmann::json stats_report(const VolumeMapStats& s);

/// A point of a tet mesh file: one of its tets -- the rows io::read_mesh gives (TetMeshData) --
/// and barycentric coordinates with respect to its corners in file order.
struct TetPoint
{
    uint64_t tet = kNoId;
    Eigen::Vector4d bary = Eigen::Vector4d::Constant(0.25);
};

/**
 * @brief The map of a tet mesh's volume between the mesh its volume history starts from and the
 * one it ends at, in the terms of their files: Tracking's counterpart for the volume (simwild,
 * tetwild with volume_history_output).
 *
 * Built from the surface and volume histories the application saved (see VolumeMap for how),
 * and the two meshes, to name points by: the one the history starts from, which the
 * application writes as <output>_history_input.msh, and the one it ends at -- simwild's
 * <output>.msh; for tetwild, the mesh before its filter, <output>_history_output.msh, of which
 * <output>_final.msh holds the part kept (a point of the part dropped has no tet there). A point
 * keeps its region: where it lands on a face between two, it is named by a tet of
 * its own. One with no tet to land in -- outside the mesh given -- comes back with no tet
 * (kNoId), as does one that is no point: of a tet the mesh does not have, or with coordinates
 * that are not numbers. Without the meshes, it maps positions (map_positions()).
 *
 * Its const methods may be called from several threads at once (VolumeMap's are; the meshes
 * are not changed). A copy shares the map, its built operations and its stats, and the meshes.
 */
class VolumeTracking
{
public:
    /// The map of `volume`, recorded alongside `surface`, between `input_mesh`, where it starts,
    /// and `output_mesh`, where it ends (tet_mesh_data()): what load() reads from the files, as
    /// the memory route passes them.
    VolumeTracking(
        const history::SurfaceHistory& surface,
        history::VolumeHistory volume,
        const MeshData& input_mesh,
        const MeshData& output_mesh,
        const VolumeMapOptions& options = {});
    /// The map of `volume` alone, with no meshes to name points by: map_positions() only.
    VolumeTracking(
        const history::SurfaceHistory& surface,
        history::VolumeHistory volume,
        const VolumeMapOptions& options = {});
    /// The files an application wrote: the two histories and the two meshes.
    static VolumeTracking load(
        const std::string& surface_history,
        const std::string& volume_history,
        const std::string& input_mesh,
        const std::string& output_mesh,
        const VolumeMapOptions& options = {});

    /// Whether it has the meshes, which TetPoints are of.
    bool has_meshes() const { return m_in.has_value(); }
    /// Points of the input mesh to the output mesh. Throws without the meshes.
    std::vector<TetPoint> to_output(const std::vector<TetPoint>& points) const;
    /// Points of the output mesh to the input mesh, the inverse map. Throws without the meshes.
    std::vector<TetPoint> to_input(const std::vector<TetPoint>& points) const;
    /// Positions where the volume history starts mapped to where it ends (`forward`), or back,
    /// each with the region it ends in (an index into map().history().regions, kAnyRegion for
    /// one no operation took). `regions`, if given, has one per position: the region it is in,
    /// as a call the other way returned it; else it is the region of the tet an operation first
    /// finds it in. A position that is not a number comes back as it is.
    std::vector<VolumePoint> map_positions(
        const std::vector<Eigen::Vector3d>& positions,
        bool forward,
        const std::vector<uint32_t>& regions = {}) const;

    const VolumeMap& map() const { return *m_map; }
    /// The meshes; they throw without them.
    const TetMeshData& input_mesh() const;
    const TetMeshData& output_mesh() const;

private:
    std::vector<TetPoint> map_points(const std::vector<TetPoint>& points, bool forward) const;

    // Shared, all of it, by a copy: the map and the meshes are not changed by mapping.
    std::shared_ptr<const VolumeMap> m_map;
    std::optional<TetLocator> m_in, m_out;
};

/// The tracking application: maps points, transfers fields and reports on a tracking file
/// (tracking_spec.json).
void tracking(nlohmann::json json_params);
/**
 * @brief The memory route. `history` is the bytes of a history (a JSON binary value: a
 * component's report["history"]), never a file name; a surface's or curves'. info returns its
 * report. transfer, on a surface history, takes its surface from `meshes` by `input`, which it
 * needs, and returns the output surface as "mesh", the carried fields as triangle attributes;
 * with `tet_mesh`, also a key of `meshes`, that mesh with the fields on its tracked surface as
 * "tet_mesh" (add_surface). features, on a curve history, returns the feature points in the
 * report, as feature_points: pairs of an input and an output vertex, -1 for none. volume takes
 * `volume_history`, the bytes of a volume history (report["volume_history"]), and `tet_mesh`
 * and `output_tet_mesh`, keys of `meshes` (the producer's history_input and mesh or
 * history_output), and returns its report. map and map_volume, whose points are text files,
 * throw: Tracking, CurveTracking and VolumeTracking map points in memory.
 */
ComponentResult tracking(nlohmann::json json_params, const MeshMap& meshes);

} // namespace wmtk::components::tracking
