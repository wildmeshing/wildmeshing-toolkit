#pragma once

#include "ConnectedComponent.hpp"

#include <SimpleBVH/BVH.hpp>
#include <wmtk/Types.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

/**
 * Helpers of the geometric tight seal, SimWildMeshTri::tight_seal_geom (AnnotationsTri.cpp) and
 * SimWildMesh::tight_seal_geom (AnnotationsTet.cpp), which keep the operation itself. The helpers
 * below need no mesh; those that do are private members of the two mesh classes, defined in
 * TightSealGeom.cpp.
 */
namespace wmtk::components::simwild::tight_seal {

/// Whether `tag` is the tag named "ambient".
bool is_ambient(const std::map<int64_t, std::string>& tag_id_to_name, int64_t tag);

/// BVH of segments, given as consecutive pairs of points. Returns false if there are none.
bool segments_bvh(const std::vector<Vector2d>& vertices, SimpleBVH::BVH& bvh);
/// BVH of triangles, given as consecutive triples of points. Returns false if there are none.
bool triangles_bvh(const std::vector<Vector3d>& vertices, SimpleBVH::BVH& bvh);

/// The distance from p to the BVH's primitives.
double bvh_distance(const SimpleBVH::BVH& bvh, const Vector2d& p);
double bvh_distance(const SimpleBVH::BVH& bvh, const Vector3d& p);

/**
 * @brief Sphere tracing of a 1-Lipschitz function f along a + t dir, t in [0, L), towards the
 * level set f = level.
 *
 * From a point at f, the level set is at least |f - level| away in every direction, so a step
 * that long along the segment cannot cross it. `from_below`: f(a) < level and each step is
 * level - f; otherwise f(a) > level and each step is f - level. Returns the first t (of a + t dir)
 * where |f - level| <= tol, or nothing if the trace leaves [0, L) first: past the far end, the
 * level set is not on the segment; behind a, f(a) is already beyond the level.
 */
std::optional<double> sphere_trace(
    const std::function<double(const Vector2d&)>& f,
    const Vector2d& a,
    const Vector2d& dir,
    double L,
    double level,
    double tol,
    bool from_below);
std::optional<double> sphere_trace(
    const std::function<double(const Vector3d&)>& f,
    const Vector3d& a,
    const Vector3d& dir,
    double L,
    double level,
    double tol,
    bool from_below);

/// `regions` by the distance from p to their BVHs (region i's is bvhs[i]), nearest first; those at
/// the same distance stay in their order in `regions`.
std::vector<size_t> regions_by_distance(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector2d& p,
    const std::vector<size_t>& regions);
std::vector<size_t> regions_by_distance(
    const std::vector<SimpleBVH::BVH>& bvhs,
    const Vector3d& p,
    const std::vector<size_t>& regions);

/// The six edges of a tet, as pairs of its local vertex indices.
inline constexpr std::array<std::array<int, 2>, 6> tet_edges = {
    {{{0, 1}}, {{0, 2}}, {{0, 3}}, {{1, 2}}, {{1, 3}}, {{2, 3}}}};

/// Appends the convex polygon `poly`, fanned from its first corner, to `triangles` (consecutive
/// triples of points).
void fan_triangles(const std::vector<Vector3d>& poly, std::vector<Vector3d>& triangles);

/// What topology_fill needs of the mesh, by cell id (faces in 2D, tets in 3D).
struct TopologyFillMesh
{
    /// `regions`, indices into the tags, by the distance from the cell's centroid, nearest first.
    std::function<std::vector<size_t>(size_t cell, const std::vector<size_t>& regions)>
        regions_by_distance;
    /// Whether adding the cell to the region keeps the region's topology
    /// (wmtk/utils/ConsistentTopology.hpp).
    std::function<bool(size_t cell, const std::function<bool(size_t)>& in_region)>
        consistent_topology;
    /// The cells sharing an edge (2D) or a face (3D) with the cell.
    std::function<std::vector<size_t>(size_t cell)> neighbours;
    std::function<CellTag&(size_t cell)> tags;
};

/**
 * @brief The fill of tight_seal_geom's EXPERIMENTAL "fill" mode; no cell is split.
 *
 * Each cell of a component gets the nearest of the regions the component borders. Then, for each
 * of those regions, a breadth-first search grows the region's tag over the cells that got it: a
 * cell joins if it passes the topology test against the tag's cells, and a cell refused is asked
 * again when a neighbour joins (only that can change the answer: more of the region around the
 * cell without another shared facet leaves it refused). The cells still refused wait until every
 * region has been tried. Each then takes the nearest of the component's regions that passes the
 * test, so the second nearest if the nearest still refuses it, and so on; and they are gone over
 * until a pass labels none, since a cell may pass only once a neighbour has been labelled. Until
 * labelled, a cell of a component counts as in no region, whatever tags it came with; labelling it
 * replaces its tags of the component's regions, as the Voronoi fill does.
 *
 * components_regions[c] holds the regions of components[c], as indices into `tags`. filled[cell]
 * is set for each cell labelled. after_component is called after each component.
 *
 * Throws, after trying every component, if a cell fails the test with all of its component's
 * regions; the warnings before it name the components.
 */
void topology_fill(
    const std::vector<ConnectedComponent>& components,
    const std::vector<std::vector<size_t>>& components_regions,
    const std::vector<int64_t>& tags,
    const TopologyFillMesh& mesh,
    std::vector<char>& filled,
    const std::function<void()>& after_component);

/// Sets `fn` to nullptr when it goes out of scope: m_voronoi_split_fn must not outlive the
/// operation that set it.
template <typename Fn>
struct ClearOnExit
{
    Fn& fn;
    ~ClearOnExit() { fn = nullptr; }
};

} // namespace wmtk::components::simwild::tight_seal
