#pragma once

#include <wmtk/Types.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/utils/Delaunay.hpp>

namespace wmtk::components::image_simulation {

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
 */
void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb);

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
 */
void embed_surface(
    const MatrixXd& V_surface,
    const MatrixXi& F_surface,
    const MatrixXd& V_vol,
    const MatrixXi& T_vol,
    MatrixXr& V_emb,
    MatrixXi& T_emb,
    MatrixXi& F_on_surface);

void tag_tets_from_image(
    const std::string& filename,
    const MatrixXd& V,
    const MatrixXi& T,
    VectorXi& T_tags);

void tag_tets_from_image(
    const std::vector<std::vector<std::vector<size_t>>>& data,
    const MatrixXd& V,
    const MatrixXi& T,
    VectorXi& T_tags);

/**
 * A class for reading an image and converting it into a tet mesh.
 */
class EmbedSurface
{
public:
    EmbedSurface(const std::string& img_filename);

    void simplify_surface();

    void remove_duplicates();

    void embed_surface();

    /**
     * @brief Remove unreferenced vertices.
     */
    void consolidate();

    const MatrixXd& V_emb() const { return m_V_emb; }
    const MatrixXi& T_emb() const { return m_T_emb; }
    const VectorXi& T_tags() const { return m_T_tags; }
    const MatrixXi& F_on_surface() const { return m_F_on_surface; }

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

    std::vector<Eigen::Vector3d> V_surf_to_vector() const;
    std::vector<std::array<size_t, 3>> F_surf_to_vector() const;

private:
    void V_surf_from_vector(const std::vector<Eigen::Vector3d>& verts);
    void F_surf_from_vector(const std::vector<std::array<size_t, 3>>& tris);

private:
    std::string m_img_filename;
    std::vector<std::vector<std::vector<size_t>>> m_img_data;

    // the surface separating all tags
    MatrixXd m_V_surface;
    MatrixXi m_F_surface;

    std::vector<size_t> modified_nonmanifold_v;

    // the embedding
    MatrixXd m_V_emb;
    MatrixXi m_T_emb;
    // triangles of the embedding representing the surface
    MatrixXi m_F_on_surface;
    // tags on the tets
    VectorXi m_T_tags;
};

} // namespace wmtk::components::image_simulation