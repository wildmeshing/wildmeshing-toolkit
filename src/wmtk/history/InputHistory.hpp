#pragma once

#include "History.hpp"

#include <wmtk/io/read_edge_mesh.hpp>
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

/**
 * @brief A curve application's history from its input files as read, to the network it inserts.
 *
 * The files' segments, as io::read_edge_mesh read them; a relabel to the welded network the
 * application ran on; the input simplification's collapses on that network; and a relabel onto
 * the rows the simplification left. A user's point on the input is then a segment of the files
 * themselves. The feature points are the welded network's vertices with other than two
 * segments -- curve ends, junctions, input points -- each named by the first vertex of the files
 * that became it.
 *
 * @param read                the files as read -- the curve files' vertices and segments, then
 *                            the point files' vertices -- and where each went in V_in, E_in
 * @param V_in, E_in          the welded network (two or three columns; z is dropped)
 * @param collapses           utils::simplify_segments' collapses on V_in, E_in; empty if it did
 *                            not run
 * @param V_out, E_out        the network the simplification left, the one inserted
 * @param surviving_vertices  for each row of V_out, its row of V_in
 * @throws if the collapses do not leave E_out's segments
 */
CurveHistory curves_from_input(
    const io::EdgeReadProvenance& read,
    const MatrixXd& V_in,
    const MatrixXi& E_in,
    const std::vector<Edit>& collapses,
    const MatrixXd& V_out,
    const MatrixXi& E_out,
    const std::vector<int>& surviving_vertices);

} // namespace wmtk::history
