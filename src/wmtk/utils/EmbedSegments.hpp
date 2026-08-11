#pragma once

#include <wmtk/Types.hpp>

namespace wmtk::utils {

/**
 * @brief Initializes a triangulation from a 2D point set and edge set by computing the exact
 * arrangement of the edges (vol_rem::embed_seg_in_tri_mesh). The output is a triangulation of
 * the bounding box of the input points that contains all edges and vertices from the input.
 *
 * The input edges are a soup: they may cross, overlap, be duplicated or be degenerate. Crossings
 * become new output vertices, so E_out generally has more (and shorter) edges than E.
 *
 * The arrangement's vertices are EXACT: a crossing between two segments generally has no
 * double representation, and vol_rem::embed_seg_in_tri_mesh returns bigrationals for exactly
 * that reason. They are handed back in V_rational, and V_out is only their rounding.
 *
 * Rounding alone is not enough to work with, which is why both are returned. Two vertices
 * that are exactly distinct can round to the SAME double, and any triangle using both is
 * then exactly degenerate -- on the 20k 2D dataset that made about a third of the models
 * unusable. The 3D path has always kept the rationals (VolumemesherInsertion's v_rational);
 * this is the 2D counterpart.
 *
 * @param V input vertices (Nx2)
 * @param E input edges (Mx2)
 * @param V_out output vertices (Kx2), the rounding of V_rational
 * @param V_rational output vertices, exact (size K)
 * @param F_out output faces (Lx3)
 * @param E_out output edges (Px2) - the output edges tiling the input edges
 */
void embed_segments(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    std::vector<Vector2r>& V_rational,
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

} // namespace wmtk::utils