#pragma once

#include <wmtk/Types.hpp>

#include <string>
#include <vector>

namespace wmtk::components::simwild {

/**
 * @brief Turn a set of 2D curve networks into one tagged triangle mesh.
 *
 * The 2D counterpart of EmbedSurface, member for member, with each 3D primitive replaced by
 * the 2D one:
 *
 *   EmbedSurface                              EmbedCurves
 *   ------------------------------------      ------------------------------------------
 *   read_triangle_mesh + 4x4 transform        read_input_curves + 3x3 transform
 *   simplify_surface (ShortestEdgeCollapse)   simplify_curves (simplify_segments)
 *   delaunay_box_mesh + embed_tri_in_poly     embed_segments (grid is inside it)
 *   winding_number  (solid angle)             winding_number_2d
 *   T_emb (Nx4), T_tags (#T x #inputs)        F_emb (Nx3), F_tags (#F x #inputs)
 *
 * The tagging contract is identical and is what makes this a *simwild* input rather than a
 * triwild one: tag i is set on a cell when input i's winding number at the cell's barycenter
 * exceeds 0.5, so a cell inside two inputs carries both tags and the material interfaces fall
 * out of where adjacent cells disagree.
 *
 * Unlike the 3D path there is no separate background mesh to build: the 2D arrangement
 * triangulates every point it is handed, so the background grid is just extra points appended
 * to the same array (wmtk::utils::embed_segments does that).
 */
class EmbedCurves
{
public:
    /**
     * @param input_paths       one curve network per entry, as OBJ 'v'/'l' polylines
     * @param input_transform   3x3 homogeneous transform per input; identity where omitted
     * @param tol_rel, tol_abs  merge input vertices closer than this (negative disables)
     */
    EmbedCurves(
        const std::vector<std::string>& input_paths,
        const std::vector<Matrix3d>& input_transform = {},
        const double tol_rel = -1,
        const double tol_abs = -1);

    /**
     * @brief Simplify the input curves while staying within the eps envelope.
     *
     * Must be a small envelope: the winding-number tags below are evaluated against the
     * ORIGINAL per-input curves, not the simplified union, so moving the curves too far
     * makes the tags disagree with the geometry. Same constraint EmbedSurface documents.
     */
    void
    simplify_curves(const double eps, const bool use_exact_envelope, const int num_threads = 0);

    /**
     * @brief Compute the exact arrangement of the (simplified) curve union.
     *
     * @return true when every arrangement vertex has an exact double representation.
     */
    bool embed_curves();

    /**
     * @brief Remove vertices not referenced by any output face.
     */
    void consolidate();

    const MatrixXd& V_emb() const { return m_V_emb; }
    const MatrixXr& V_emb_r() const { return m_V_emb_r; }
    const MatrixXi& F_emb() const { return m_F_emb; }
    const MatrixSi& F_tags() const { return m_F_tags; }
    /// The simplified input curves -- what the envelope is built from.
    const MatrixXd& V_curves() const { return m_V_curves; }
    const MatrixXi& E_curves() const { return m_E_curves; }
    /// The constrained edges of the arrangement, i.e. the output edges tiling the input.
    const MatrixXi& E_constrained() const { return m_E_constrained; }

    std::pair<Vector2d, Vector2d> bbox_curves_minmax() const;

    void write_curves_obj(const std::string& filename) const;

    int m_num_threads = 0;

private:
    /**
     * @brief One binary tag column per input, set where that input's winding number at a
     * face barycenter exceeds 0.5.
     *
     * Auto-corrects an inverted input orientation and warns when an input claims nothing, as
     * triwild's compute_winding_numbers does and the 3D tag_from_winding_number does not.
     */
    void tag_from_winding_number();

private:
    /// Per input, as read: kept separate because the winding number is per input.
    std::vector<MatrixXd> m_Vs;
    std::vector<MatrixXi> m_Es;

    /// The union, which is what gets simplified and arranged.
    MatrixXd m_V_curves;
    MatrixXi m_E_curves;

    MatrixXd m_V_emb;
    MatrixXr m_V_emb_r;
    MatrixXi m_F_emb;
    MatrixXi m_E_constrained;
    MatrixSi m_F_tags;
};

} // namespace wmtk::components::simwild
