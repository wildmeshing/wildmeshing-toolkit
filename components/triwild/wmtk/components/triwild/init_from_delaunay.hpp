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

/**
 * @brief Read every input edge mesh and concatenate them into one segment network.
 *
 * Reading is separate from the arrangement because the bounding box, the envelope and the
 * simplification all have to happen in between: the tolerance is relative to the input's
 * bounding box, and the simplification runs before the arrangement so the arrangement only
 * ever sees the coarsened curves.
 *
 * The union is what gets simplified and arranged, so curves sharing a boundary stay
 * coincident. The per-input copies are handed back untouched for the winding-number pass.
 *
 * @param input_paths        files to read
 * @param remove_duplicate_eps  merge input vertices closer than this fraction of the
 *                           bounding-box diagonal (negative disables)
 * @param V_all, E_all       the concatenated segment network
 * @param Vs_out, Es_out     the per-input meshes as read, so the winding-number pass does
 *                           not have to re-read every file from disk
 */
void read_input_curves(
    const std::vector<std::string>& input_paths,
    double remove_duplicate_eps,
    MatrixXd& V_all,
    MatrixXi& E_all,
    std::vector<MatrixXd>& Vs_out,
    std::vector<MatrixXi>& Es_out);

} // namespace wmtk::components::triwild