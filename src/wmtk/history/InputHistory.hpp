#pragma once

#include "SurfaceHistory.hpp"

#include <wmtk/io/read_triangle_mesh.hpp>

#include <Eigen/Core>

#include <vector>

namespace wmtk::history {

/**
 * @brief A surface application's history from its input file as read.
 *
 * The file's triangles, as io::read_triangle_mesh read them; a relabel to the welded mesh the
 * application ran on; then the history the application's mesh recorded from it (TriMesh::
 * start_history right after create_mesh). A user's point on the input is then a triangle of the
 * file itself. The recorded history's feature edges are taken back onto the file's vertices,
 * where the relabel carries them over.
 *
 * @param read      what read_triangle_mesh returned as provenance
 * @param welded    the positions of the mesh it returned, the one the application ran on
 * @param recorded  the history recorded from that mesh
 * @throws if `recorded` does not start at the welded mesh
 */
SurfaceHistory from_input(
    const io::ReadProvenance& read,
    const std::vector<Eigen::Vector3d>& welded,
    const SurfaceHistory& recorded);

} // namespace wmtk::history
