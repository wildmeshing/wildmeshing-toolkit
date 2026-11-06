#pragma once

#include <wmtk/SurfaceTagAttributes.h>
#include <wmtk/MeshData.hpp>
#include <wmtk/Types.hpp>

#include <string>
#include <vector>

namespace wmtk::components::triwild {

/**
 * @brief A tagged triangle mesh as triwild's tagged route takes it, the 2D twin of tetwild's
 * TaggedMesh: the triangles, and what triwild holds on them -- tracked curves and points. No
 * regions: triwild tracks curves through its operations and writes no region tags (the inputs'
 * winding numbers, computed at the end, are fields, wn_<name>), so a mesh whose triangles carry
 * a label other than ambient is refused (read_tagged_mesh()).
 *
 * The edge block is read as simwild's read_mesh_data() reads a triangle mesh's: each label one
 * input, every row oriented by its vertex order. Rows of one edge are merged: their labels
 * united, their orientation one count, +1 along E's row where they all run that way, -1 where
 * they all run the other, 0 where both, as coincident input segments of opposite directions
 * cancel at the arrangement. The point block is input points, anchored where they are.
 */
struct TaggedTriMesh
{
    /// The vertices of the triangles (x, y), renumbered in order.
    MatrixXd V;
    /// The triangles, positively oriented (read_tagged_mesh() refuses any other).
    MatrixXi F;
    /// The tracked edges, one row per edge: E_sets[i] has bit j for label edge_names[j], and
    /// E_orientation[i] is the orientation along the row.
    MatrixXi E;
    std::vector<InputSet> E_sets;
    std::vector<int> E_orientation;
    std::vector<std::string> edge_names;
    /// The point block's vertices, one per row and label, each under the label point_labels[i]
    /// of point_names.
    std::vector<size_t> point_vids;
    std::vector<int> point_labels;
    std::vector<std::string> point_names;
};

/// Whether `mesh` has triangles: the tagged route's input, rather than curves.
bool is_triangle_mesh(const MeshData& mesh);

/**
 * @brief Read a tagged triangle mesh for triwild, throwing before any work on what triwild cannot
 * hold: triangles tagged other than ambient (a region, simwild's to remesh), cells other than
 * triangles, edges and points, vertices off the plane z = 0, triangles not positively oriented, an
 * edge block without tags, a row without a tag, rows labelled EnvelopeSurface, more labels than
 * SurfaceTagAttributes::max_inputs, a row on a vertex no triangle has. `what` names the input.
 * Fields as tetwild's tagged route takes them: `tags` read, every other dropped.
 */
TaggedTriMesh read_tagged_mesh(const MeshData& mesh, const std::string& what);

} // namespace wmtk::components::triwild
