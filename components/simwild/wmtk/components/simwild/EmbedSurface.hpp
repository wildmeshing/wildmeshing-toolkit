#pragma once

#include <set>
#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/Delaunay.hpp>

namespace wmtk::components::simwild {

using ImageData = std::vector<std::vector<std::vector<size_t>>>;

/**
 * @brief Generate the tet mesh that contains all input vertices.
 *
 * This method is used by tetwild to initialize the tet mesh.
 *
 * @param envelope The envelope around the input surface. Used for filtering additional points that
 * would be otherwise very close to the surface.
 * @param vertices The vertices on the input surface.
 * @param points The output points from the tetrahedralization.
 * @param tets The tets from the tetrahedralization.
 * @param box_min The box corner with the lowest coordinate values.
 * @param box_max The box corner with the largest coordinate values.
 */
void delaunay_box_mesh(
    const wmtk::Envelope& envelope,
    const MatrixXd& vertices,
    std::vector<wmtk::delaunay::Point3D>& points,
    std::vector<wmtk::delaunay::Tetrahedron>& tets,
    Vector3d& box_min,
    Vector3d& box_max);

/**
 * @brief Embed a surface in the given volumetric mesh.
 *
 * @param V_surface Surface vertices.
 * @param F_surface Surface faces (triangles).
 * @param V_vol Tet vertices.
 * @param T_vol Tets.
 * @param[out] V_emb Vertices after embedding.
 * @param[out] T_emb Tets after embedding.
 * @param[out] F_on_surface Faces that are on the surface.
 * @param F_input Which input surface each row of F_surface came from, or SIZE_MAX for a row
 * that belongs to none. Pass empty to skip the provenance below.
 * @param[out] F_on_surface_inputs Parallel to F_on_surface: the input surfaces tiling each
 * output surface face, from the arrangement's own triangle provenance. Usually one; more
 * where two inputs meet coplanarly, which is exactly the case a geometric look-up cannot
 * tell apart.
 */
void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb,
    MatrixXi& F_on_surface,
    const bool perform_sanity_checks = false,
    const std::vector<size_t>& F_input = {},
    std::vector<std::vector<size_t>>* F_on_surface_inputs = nullptr);

/**
 * A class for reading an image and converting it into a tet mesh.
 */
class EmbedSurface
{
public:
    /**
     * @brief Input from meshes.
     */
    EmbedSurface(
        const std::vector<std::string>& img_filenames,
        const std::vector<Matrix4d>& img_transform = {},
        const double tol_rel = -1,
        const double tol_abs = -1);

    /**
     * @brief Simplify the input surface while staying within the eps envelope.
     *
     * @param eps The absolute envelope thickness.
     */
    void simplify_surface(const double eps, const int num_threads = 0);

    /**
     * @brief Merge vertices that are closer than eps.
     */
    void remove_duplicates(const double eps);

    /**
     * @brief Run the arrangement and tag the resulting cells.
     *
     * @param flood_fill Unify the tags of each region bounded by the surface.
     * @param tag_from_winding_number Decide each cell's tags by evaluating the winding
     * number of every input at its centroid (the default), rather than by propagating them
     * across the arrangement's own surface provenance. See tag_from_provenance.
     */
    bool embed_surface(const bool flood_fill = false, const bool tag_from_winding_number = true);

    /**
     * @brief Remove unreferenced vertices.
     */
    void consolidate();

    const MatrixXd& V_emb() const { return m_V_emb; }
    const MatrixXr& V_emb_r() const { return m_V_emb_r; }
    const MatrixXd& V_surface() const { return m_V_surface; }
    const MatrixXi& T_emb() const { return m_T_emb; }
    const MatrixSi& T_tags() const { return m_T_tags; }
    const MatrixXi& F_on_surface() const { return m_F_on_surface; }
    const MatrixXi& F_surface() const { return m_F_surface; }

    /**
     * @brief Write surface as read from image.
     *
     * The surface is all the contours in the image, i.e., the surface in between voxels with
     * different value.
     */
    void write_surf_off(const std::string& filename) const;
    /**
     * @brief Write embedded surface.
     *
     * This writes all the vertices that exist in the volume and all triangles that are representing
     * the embedded surface.
     */
    void write_emb_surf_off(const std::string& filename) const;

    void write_emb_msh(const std::string& filename) const;
    void write_emb_vtu(const std::string& filename) const;

    std::pair<Vector3d, Vector3d> bbox_minmax() const;
    std::pair<Vector3d, Vector3d> bbox_surf_minmax() const;

    std::vector<Eigen::Vector3d> V_surf_to_vector() const;
    std::vector<std::array<size_t, 3>> F_surf_to_vector() const;

private:
    void V_surf_from_vector(const std::vector<Eigen::Vector3d>& verts);
    void F_surf_from_vector(const std::vector<std::array<size_t, 3>>& tris);

    void tag_from_winding_number();

    /**
     * @brief Decide the cell tags from the arrangement's surface provenance.
     *
     * The exact alternative to tag_from_winding_number. m_F_tags_surface says which inputs
     * each surface face belongs to, so tags propagate combinatorially: start from a cell of
     * the padded box, which is outside everything, and walk the tet adjacency graph toggling
     * membership of input i whenever a face belonging to input i is crossed. Every input
     * surface is closed, so the parity along any two paths to the same cell agrees; if it
     * does not, the input was not closed and this throws rather than returning a tagging
     * that depends on the traversal order.
     */
    void tag_from_provenance();

    /**
     * @brief Count where the tagging and the arrangement's surface disagree.
     *
     * Everything downstream reads the surface off the tags -- SimWildMesh marks a face as
     * surface exactly when its two tets' tags differ -- so a tagging is only right if it
     * reproduces the surface the arrangement computed. Diagnostic, under DEBUG_sanity_checks.
     */
    void check_tag_surface_invariant(const std::string& how) const;

private:
    std::vector<std::string> m_img_filenames;
    std::vector<ImageData> m_img_datas;

    // the surface separating all tags
    MatrixXd m_V_surface;
    MatrixXi m_F_surface;

    // per row of m_F_surface: which input it came from, or SIZE_MAX for none
    std::vector<size_t> m_F_input;

    // per row of m_F_on_surface: which inputs tile it, from the arrangement's provenance.
    // Only filled when the provenance tagging is asked for.
    std::vector<std::vector<size_t>> m_F_tags_surface;

    std::vector<size_t> modified_nonmanifold_v;

    // the embedding
    MatrixXd m_V_emb;
    MatrixXr m_V_emb_r;
    MatrixXi m_T_emb;
    // triangles of the embedding representing the surface
    MatrixXi m_F_on_surface;
    // tags on the tets
    MatrixSi m_T_tags;
    std::vector<std::set<int64_t>> m_tags;

    // input from triangle meshes
    std::vector<MatrixXd> Vs;
    std::vector<MatrixXi> Fs;

public:
    bool m_smooth_surface = false;
    bool m_perform_sanity_checks = false;
    int m_num_threads = 0;
};

} // namespace wmtk::components::simwild