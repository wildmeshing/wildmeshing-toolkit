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

void tag_tets_from_image(const std::string& filename, const MatrixXd& V, const MatrixXi& T, VectorXi& T_tags);

} // namespace wmtk::components::image_simulation