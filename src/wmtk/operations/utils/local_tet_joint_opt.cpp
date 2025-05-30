#include "local_tet_joint_opt.hpp"
#include <igl/opengl/glfw/Viewer.h>
#include <fstream>
#include <iostream>
#include <wmtk/utils/orient.hpp>

namespace wmtk {
namespace operations {
namespace utils {

// Implementation of SymmetricDirichletEnergy operator
inline double SymmetricDirichletEnergy::operator()(const Eigen::Matrix3d& F, Eigen::Matrix3d* dE_dF)
    const
{
    const Eigen::Matrix3d Finv = F.inverse();
    const double energy = 0.5 * (F.squaredNorm() + Finv.squaredNorm());
    if (dE_dF) {
        *dE_dF = F - Finv.transpose();
    }
    return energy;
}

// Implementation of precompute_reference
void precompute_reference(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    std::vector<TetPrecomp>& P)
{
    const int m = static_cast<int>(T.rows());
    P.resize(m);
    for (int ti = 0; ti < m; ++ti) {
        const int i0 = T(ti, 0), i1 = T(ti, 1), i2 = T(ti, 2), i3 = T(ti, 3);
        Eigen::Matrix3d X;
        X.col(0) = V.row(i1) - V.row(i0);
        X.col(1) = V.row(i2) - V.row(i0);
        X.col(2) = V.row(i3) - V.row(i0);
        TetPrecomp pc;
        pc.Xinv = X.inverse();
        pc.volume = std::abs(X.determinant()) / 6.0;
        P[ti] = pc;
    }
}

// Implementation of compute_energy_and_gradient_fast
template <typename DerivedVParam, typename DerivedT, typename EnergyFunctor>
double compute_energy_and_gradient_fast(
    const Eigen::MatrixBase<DerivedVParam>& V_param,
    const Eigen::MatrixBase<DerivedT>& T,
    const std::vector<TetPrecomp>& P,
    Eigen::MatrixXd& grad,
    const EnergyFunctor& energy_functor)
{
    const int n = static_cast<int>(V_param.rows());
    grad.setZero(n, 3);

    double total_E = 0.0;
    double total_volume = 0.0;
    const int m = static_cast<int>(T.rows());

    for (int ti = 0; ti < m; ++ti) {
        const int i0 = T(ti, 0), i1 = T(ti, 1), i2 = T(ti, 2), i3 = T(ti, 3);
        const TetPrecomp& pc = P[ti];

        // Current edge matrix Xp
        Eigen::Matrix3d Xp;
        Xp.col(0) = V_param.row(i1) - V_param.row(i0);
        Xp.col(1) = V_param.row(i2) - V_param.row(i0);
        Xp.col(2) = V_param.row(i3) - V_param.row(i0);

        const Eigen::Matrix3d F = Xp * pc.Xinv; // deformation gradient

        Eigen::Matrix3d dE_dF;
        const double Ei = energy_functor(F, &dE_dF) * pc.volume;
        total_E += Ei;
        total_volume += pc.volume;
        const Eigen::Matrix3d dE_dXp = dE_dF * pc.Xinv.transpose() * pc.volume;

        // Scatter to vertex gradients
        grad.row(i1) += dE_dXp.col(0).transpose();
        grad.row(i2) += dE_dXp.col(1).transpose();
        grad.row(i3) += dE_dXp.col(2).transpose();
        grad.row(i0) -= (dE_dXp.col(0) + dE_dXp.col(1) + dE_dXp.col(2)).transpose();
    }
    grad /= total_volume;
    return total_E / total_volume;
}

void local_tet_joint_opt(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T_before,
    const Eigen::MatrixXi& T_after,
    Eigen::MatrixXd& V_param,
    const std::vector<int>& constraint_vids,
    bool debug_mode)
{
    // Precompute reference data for all tetrahedra
    std::vector<TetPrecomp> P;

    // Create a joint tetrahedron matrix by concatenating T_before and T_after
    Eigen::MatrixXi T_joint(T_before.rows() + T_after.rows(), 4);
    T_joint.topRows(T_before.rows()) = T_before;
    T_joint.bottomRows(T_after.rows()) = T_after;

    // Precompute reference data for the joint tetrahedra
    precompute_reference(V, T_joint, P);
    Eigen::MatrixXd grad;
    double energy =
        compute_energy_and_gradient_fast(V_param, T_joint, P, grad, SymmetricDirichletEnergy());
    std::cout << "energy: " << energy << std::endl;
    std::cout << "grad: \n" << grad << std::endl;
    // Set z-axis gradient to zero for all constraint vertices
    for (const int vid : constraint_vids) {
        if (vid >= 0 && vid < grad.rows()) {
            // Zero out the z-component (third column) of the gradient
            grad(vid, 2) = 0.0;
        } else {
            std::cout << "vid: " << vid << " is out of range" << std::endl;
        }
    }

    std::cout << "After zeroing z-gradient for constraint vertices:" << std::endl;
    std::cout << "grad: \n" << grad << std::endl;

    // Gradient descent with line search
    const double initial_step_size = 0.1;
    const double step_reduction_factor = 0.5;
    const int max_iterations = 100;
    const double convergence_threshold = 1e-6;
    const int max_line_search_iterations = 20;

    // Create a copy of the initial parameters
    Eigen::MatrixXd V_current = V_param;
    double current_energy = energy;

    for (int iter = 0; iter < max_iterations; ++iter) {
        // Compute the descent direction (negative gradient)
        Eigen::MatrixXd descent_direction = -grad;

        // Line search to find appropriate step size
        double step_size = initial_step_size;
        bool valid_step_found = false;

        for (int line_search_iter = 0; line_search_iter < max_line_search_iterations;
             ++line_search_iter) {
            // Print current iteration and step size
            std::cout << "Line search iteration " << line_search_iter
                      << ", step size: " << step_size << std::endl;
            // Try the step
            Eigen::MatrixXd V_next = V_current + step_size * descent_direction;

            // Check for tet inversions
            bool has_inverted_tets = false;
            for (int t = 0; t < T_joint.rows(); ++t) {
                const int i0 = T_joint(t, 0);
                const int i1 = T_joint(t, 1);
                const int i2 = T_joint(t, 2);
                const int i3 = T_joint(t, 3);

                const Eigen::Vector3d p0 = V_next.row(i0);
                const Eigen::Vector3d p1 = V_next.row(i1);
                const Eigen::Vector3d p2 = V_next.row(i2);
                const Eigen::Vector3d p3 = V_next.row(i3);

                if (wmtk::utils::wmtk_orient3d(p0, p1, p2, p3) >= 0) {
                    has_inverted_tets = true;
                    std::cout << "Tet " << t << " is inverted" << std::endl;
                    break;
                }
            }

            if (has_inverted_tets) {
                // Reduce step size and try again
                step_size *= step_reduction_factor;
                continue;
            }

            // Compute energy at the new position
            Eigen::MatrixXd new_grad;
            double new_energy = compute_energy_and_gradient_fast(
                V_next,
                T_after,
                P,
                new_grad,
                SymmetricDirichletEnergy());

            // Check if energy decreased
            if (new_energy < current_energy) {
                // Accept the step
                V_current = V_next;
                current_energy = new_energy;
                grad = new_grad;

                // Set z-axis gradient to zero for all constraint vertices
                for (const int vid : constraint_vids) {
                    if (vid >= 0 && vid < grad.rows()) {
                        grad(vid, 2) = 0.0;
                    }
                }

                valid_step_found = true;
                break;
            } else {
                std::cout << "current energy: " << current_energy << " new energy: " << new_energy
                          << std::endl;

                // Reduce step size and try again
                step_size *= step_reduction_factor;
            }
        }

        if (!valid_step_found) {
            std::cout << "Line search failed to find a valid step at iteration " << iter
                      << std::endl;
            break;
        }

        // Check for convergence
        double grad_norm = grad.norm();
        std::cout << "Iteration " << iter << ": energy = " << current_energy
                  << ", gradient norm = " << grad_norm << std::endl;

        if (grad_norm < convergence_threshold) {
            std::cout << "Converged after " << iter + 1 << " iterations." << std::endl;
            break;
        }
    }

    if (debug_mode) {
        // Create viewer
        igl::opengl::glfw::Viewer viewer;

        // Store meshes for visualization
        Eigen::MatrixXd meshes[3] = {V, V_param, V_current};
        int current_mesh = 0;

        // Extract surface triangles from the tetrahedral mesh
        Eigen::MatrixXi F_surface;
        std::vector<std::array<int, 3>> surface_triangles;

        // Use a simple method to extract the surface: process each face of every tetrahedron
        for (int i = 0; i < T_before.rows(); ++i) {
            // Four faces of the tetrahedron
            std::array<std::array<int, 3>, 4> tet_faces = {
                {{T_before(i, 1), T_before(i, 0), T_before(i, 2)},
                 {T_before(i, 1), T_before(i, 2), T_before(i, 3)},
                 {T_before(i, 0), T_before(i, 3), T_before(i, 2)},
                 {T_before(i, 1), T_before(i, 3), T_before(i, 0)}}};

            // Add all faces
            for (const auto& face : tet_faces) {
                surface_triangles.push_back(face);
            }
        }

        // Convert surface triangles to Eigen matrix
        F_surface.resize(surface_triangles.size(), 3);
        for (size_t i = 0; i < surface_triangles.size(); ++i) {
            F_surface.row(i) << surface_triangles[i][0], surface_triangles[i][1],
                surface_triangles[i][2];
        }

        // Set initial mesh and rendering properties
        viewer.data().set_mesh(meshes[current_mesh], F_surface);
        viewer.data().set_face_based(true);
        viewer.data().show_lines = true;

        // Add keyboard callback to switch between meshes
        viewer.callback_key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int) {
            if (key == '0') {
                current_mesh = 0; // V
                viewer.data().set_mesh(meshes[current_mesh], F_surface);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface);
                return true;
            } else if (key == '1') {
                current_mesh = 1; // V_param
                viewer.data().set_mesh(meshes[current_mesh], F_surface);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface);
                return true;
            } else if (key == '2') {
                current_mesh = 2; // V_current
                viewer.data().set_mesh(meshes[current_mesh], F_surface);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface);
                return true;
            }
            return false;
        };

        viewer.launch();
    }
    // Update the output parameters
    V_param = V_current;
    std::cout << "Final energy: " << current_energy << std::endl;
    // Compute energy and gradient for the entire mesh
}

// Explicit template instantiations
template double
compute_energy_and_gradient_fast<Eigen::MatrixXd, Eigen::MatrixXi, SymmetricDirichletEnergy>(
    const Eigen::MatrixBase<Eigen::MatrixXd>& V_param,
    const Eigen::MatrixBase<Eigen::MatrixXi>& T,
    const std::vector<TetPrecomp>& P,
    Eigen::MatrixXd& grad,
    const SymmetricDirichletEnergy& energy_functor);

} // namespace utils
} // namespace operations
} // namespace wmtk
