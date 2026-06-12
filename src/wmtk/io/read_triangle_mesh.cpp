#include "read_triangle_mesh.hpp"

#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {

/**
 * @brief Clean the input triangle mesh by removing duplicated vertices, degenerate faces, and
 * unreferenced vertices.
 *
 * @param V Input vertex positions. Will be modified in-place to remove duplicated and unreferenced
 * vertices.
 * @param F Input face indices. Will be modified in-place to remove degenerate faces and update
 * vertex indices after removing duplicated vertices.
 * @param tol_rel Relative tolerance for identifying duplicated vertices. If non-negative, tol_abs
 * will be ignored and tol_rel * diag will be used as the absolute tolerance, where diag is the
 * diagonal length of the bounding box of the input mesh.
 * @param tol_abs Absolute tolerance for identifying duplicated vertices. If non-negative, vertices
 * that are within this distance will be considered duplicates and merged. If negative, no vertex
 * merging will be performed.
 */
void clean_triangle_mesh(MatrixXd& V, MatrixXi& F, double tol_rel = -1, double tol_abs = -1)
{
    if (tol_abs >= 0 && tol_rel >= 0) {
        log_and_throw_error(
            "Only one of tol_abs and tol_rel can be non-negative. Got abs = {} and rel = {}",
            tol_abs,
            tol_rel);
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

void read_triangle_mesh(
    const std::string& path,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    double tol_rel,
    double tol_abs)
{
    if (!igl::read_triangle_mesh(path, V, F)) {
        log_and_throw_error("Could not read mesh {}", path);
    }

    clean_triangle_mesh(V, F, tol_rel, tol_abs);
}

void read_triangle_mesh(
    const std::vector<std::string>& paths,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F,
    double tol_rel,
    double tol_abs)
{
    V.resize(0, 3);
    F.resize(0, 3);
    for (const std::string p : paths) {
        if (!std::filesystem::exists(p)) {
            log_and_throw_error("File {} does not exist", p);
        }
        MatrixXd V_single;
        MatrixXi F_single;
        if (!igl::read_triangle_mesh(p, V_single, F_single)) {
            log_and_throw_error("Could not read mesh {}", p);
        }
        assert(V_single.cols() == 3);
        assert(F_single.cols() == 3);

        const int nV_old = V.rows();
        const int nF_old = F.rows();

        V.conservativeResize(V.rows() + V_single.rows(), 3);
        V.block(nV_old, 0, V_single.rows(), 3) = V_single;

        F_single.array() += nV_old;
        F.conservativeResize(F.rows() + F_single.rows(), 3);
        F.block(nF_old, 0, F_single.rows(), 3) = F_single;
    }

    clean_triangle_mesh(V, F, tol_rel, tol_abs);
}

} // namespace wmtk::io