#include "read_triangle_mesh.hpp"

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {

void read_triangle_mesh(
    const std::string& path,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    double tol_rel,
    double tol_abs)
{
    if (tol_abs >= 0 && tol_rel >= 0) {
        log_and_throw_error(
            "Only one of tol_abs and tol_rel can be non-negative. Got abs = {} and rel = {}",
            tol_abs,
            tol_rel);
    }

    if (!igl::read_triangle_mesh(path, V, F)) {
        log_and_throw_error("Could not read mesh {}", path);
    }

    if (tol_abs < 0 && tol_rel >= 0) {
        const MatrixXd box_min = V.colwise().minCoeff();
        const MatrixXd box_max = V.colwise().maxCoeff();
        const double diag = (box_max - box_min).norm();
        tol_abs = tol_rel * diag;
    }

    // remove duplicated vertices
    if (tol_abs >= 0) {
        VectorXi SVI, SVJ;
        MatrixXd temp_V = V;
        igl::remove_duplicate_vertices(temp_V, tol_abs, V, SVI, SVJ);
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; ++j) {
                F(i, j) = SVJ[F(i, j)];
            }
        }
    }

    // remove faces with duplicated vertices (degenerate faces)
    {
        std::vector<bool> valid_face(F.rows(), true);
        size_t n_degenerate_faces = 0;
        for (int i = 0; i < F.rows(); i++) {
            if (F(i, 0) == F(i, 1) || F(i, 1) == F(i, 2) || F(i, 0) == F(i, 2)) {
                valid_face[i] = false;
                ++n_degenerate_faces;
            }
        }
        if (n_degenerate_faces > 0) {
            logger().warn(
                "Input mesh has faces with duplicated vertex IDs. {} of {} faces will be removed.",
                n_degenerate_faces,
                F.rows());
            std::vector<Vector3i> valid_tris;
            valid_tris.reserve(F.rows());
            for (int i = 0; i < F.rows(); i++) {
                if (valid_face[i]) {
                    valid_tris.push_back(F.row(i));
                }
            }
            F.resize(valid_tris.size(), 3);
            for (int i = 0; i < valid_tris.size(); i++) {
                F.row(i) = valid_tris[i];
            }
        }
    }

    // remove unreferenced vertices
    {
        MatrixXd temp_V = V;
        MatrixXi temp_F = F;
        VectorXi I;
        igl::remove_unreferenced(temp_V, temp_F, V, F, I);
    }
}

} // namespace wmtk::io