#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::components::triwild {

/**
 * @brief Initializes a triangulation from a 2D point set and edge set by computing the exact
 * arrangement of the edges (vol_rem::embed_seg_in_tri_mesh). The output is a triangulation of
 * the bounding box of the input points that contains all edges and vertices from the input.
 *
 * The input edges are a soup: they may cross, overlap, be duplicated or be degenerate. Crossings
 * become new output vertices, so E_out generally has more (and shorter) edges than E.
 *
 * @param V input vertices (Nx2)
 * @param E input edges (Mx2)
 * @param V_out output vertices (Kx2)
 * @param F_out output faces (Lx3)
 * @param E_out output edges (Px2) - the output edges tiling the input edges
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