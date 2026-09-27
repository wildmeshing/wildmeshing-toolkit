#pragma once

#include "Hdf5Writers.hpp"
#include "TaggedMesh.hpp"

#include <array>
#include <string>
#include <vector>

namespace wmtk::components::simwild::polyfem_helpers {

/// Everything `constraints.load_mesh` returns but its node-tag map, which nothing here reads, in
/// the order of its 9-tuple.
struct LoadedMesh
{
    MatrixXd coords; ///< (n, mesh_dim), in mesh units
    std::vector<std::array<int64_t, 2>> interface_edges;
    int64_t total_n_nodes = 0;
    int mesh_dim = 3;
    std::vector<std::array<int64_t, 3>> interface_faces; ///< 3D only, empty in 2D
    std::vector<int64_t> collision_node_ids; ///< in collision-OBJ vertex order
    std::vector<std::array<int64_t, 2>> collision_edges_local; ///< over those local ids
    std::vector<std::vector<int64_t>> face_tags; ///< per face/edge, sorted id lists
};

/**
 * @brief Chain directed 2D edges into loops. Mirrors `constraints._orient_edge_loops_2d`.
 *
 * Each simple closed loop keeps the direction its input edges agree on -- the explicit
 * region/filter orientation, which is the only hole-safe source of truth (a cavity loop is wound
 * clockwise, an outer loop counter-clockwise; signed area cannot tell the material side). A loop
 * whose inputs contradict each other (the same interface selected from both sides, so no single
 * direction is correct for both bodies) throws. Non-loop components (open chains, branch points)
 * pass through with their original order and orientation.
 *
 * The Python signature also takes `coords`, which it never reads; it is not mirrored.
 */
std::vector<std::array<int64_t, 2>> orient_edge_loops_2d(
    const std::vector<std::array<int64_t, 2>>& oriented_edges);

/**
 * @brief The material-interface surfaces of a multi-tag mesh. Mirrors `constraints.load_mesh`
 * after its read of the file: every interface when `selections` is empty (the legacy
 * auto-detect), else those picked by the given selections.
 */
LoadedMesh load_mesh(const TaggedMesh& mesh, const std::vector<Selection>& selections);

/// What interface_collision.obj holds, one entry per line of it: the "v" lines, then either the
/// "f" lines (3D) or the "l" lines (2D), with the indices 0-based here and 1-based in the file.
struct CollisionObj
{
    std::vector<std::array<double, 3>> vertices; ///< mesh units (un-scaled); z is 0 in 2D
    std::vector<std::array<int64_t, 3>> faces;
    std::vector<std::array<int64_t, 2>> edges;
};

/// Mirrors `constraints.write_collision_mesh_obj` up to the write: which vertices, faces and edges
/// it writes, in its order. Vertices are in mesh units; polyfem applies the geometry
/// transformation from its JSON.
CollisionObj collision_mesh_obj(
    const MatrixXd& coords,
    const std::vector<int64_t>& node_ids,
    const std::vector<std::array<int64_t, 2>>& interface_edges,
    const std::vector<std::array<int64_t, 3>>& interface_faces,
    const std::vector<int64_t>& collision_node_ids,
    const std::vector<std::array<int64_t, 2>>& collision_edges_local);

/**
 * @brief Mirrors `minimum_separation._normalize_collision_pairs`.
 *
 * Turns [[side_A, side_B], ...] into the unique selections (identical selections dedupe to one
 * collision body) and the polyfem id pairs (duplicate id pairs collapse).
 */
void normalize_collision_pairs(
    const nlohmann::json& raw_pairs,
    std::vector<Selection>& unique,
    std::vector<std::array<int64_t, 2>>& polyfem_pairs);

/// The polyfem inputs `constraints.make_interface_constraint` writes, one member per file.
struct InterfaceConstraint
{
    ConstraintHdf5 fitting; ///< interface_constraint.hdf5
    ConstraintHdf5 laplacian; ///< interface_constraint_laplacian.hdf5
    CollisionObj collision_mesh; ///< interface_collision.obj
    LinearMapHdf5 linear_map; ///< interface_linear_map.hdf5
    std::vector<std::vector<int64_t>> collision_body_ids; ///< collision_body_ids.txt, line by line
};

/**
 * @brief Mirrors `constraints.make_interface_constraint` up to the writes.
 *
 * Which of the files an operation generates is the caller's: see `add_interface_constraint` in
 * PolyfemOperation.hpp. The Python's `dim` argument is not mirrored: no caller passes it, so it is
 * always the mesh dimension.
 *
 * `laplacian_row_factor_by_id` carries the per-interface Laplacian weight, as selection id ->
 * sqrt(weight / weight_laplacian), and holds an entry only for the selections that set their own
 * `weight`. EMPTY (the default, and what an operation without the key passes) means no node is
 * weighted and the Laplacian constraint is left exactly as it was.
 */
InterfaceConstraint make_interface_constraint(
    const TaggedMesh& mesh,
    const std::vector<Selection>& selections,
    bool use_graph,
    bool normalize,
    double scale,
    bool smooth_positions,
    const std::map<int64_t, double>& laplacian_row_factor_by_id = {});

} // namespace wmtk::components::simwild::polyfem_helpers
