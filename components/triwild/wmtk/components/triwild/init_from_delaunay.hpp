#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::triwild {

/**
 * @brief Initializes a triangulation from a 2D point set and edge set using constrained Delaunay
 * triangulation. The output is a triangulation of the bounding box of the input points that
 * contains all edges and vertices from the input.
 *
 * @param V input vertices (Nx2)
 * @param E input edges (Mx2)
 * @param V_out output vertices (Kx2)
 * @param F_out output faces (Lx3)
 * @param E_out output edges (Px2) - the constrained edges from the input
 */
void init_from_delaunay_box_mesh(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out);

void init_from_paths(
    const std::vector<std::string>& input_paths,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out);

} // namespace wmtk::components::triwild