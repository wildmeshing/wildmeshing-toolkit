#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/io/ReadProvenance.hpp>

namespace wmtk::io {

/// The cleaning read_edge_mesh does, on a mesh already in memory. `provenance`, if not null,
/// receives V and E as given and where each vertex and segment went.
void clean_edge_mesh(
    MatrixXd& V,
    MatrixXi& E,
    double tol_rel = -1,
    double tol_abs = -1,
    ReadProvenance* provenance = nullptr);

/**
 * @brief Reads an edge mesh from a file.
 *
 * Coincident vertices are merged and the edges remapped onto the survivors; edges that
 * become degenerate (both endpoints merged into one) and duplicated edges are dropped.
 * Vertices with no incident edge are kept: in 2D they are meaningful free points that the
 * arrangement still triangulates.
 *
 * The tolerance can be given absolutely or relative to the bounding-box diagonal. If both
 * are negative, no merging happens.
 *
 * @param path The file path to read the mesh from.
 * @param V Output vertex positions. Size is #V by 3.
 * @param E Output edge indices. Size is #E by 2.
 * @param tol_rel Merge tolerance relative to the bounding-box diagonal. Negative disables.
 * @param tol_abs Absolute merge tolerance. Takes precedence over tol_rel when non-negative.
 * @param provenance If not null, receives the mesh as the file holds it and where it went.
 */
void read_edge_mesh(
    const std::string& path,
    MatrixXd& V,
    MatrixXi& E,
    double tol_rel = -1,
    double tol_abs = -1,
    ReadProvenance* provenance = nullptr);

} // namespace wmtk::io