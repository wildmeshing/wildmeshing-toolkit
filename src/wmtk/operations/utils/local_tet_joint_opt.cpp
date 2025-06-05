#include "local_tet_joint_opt.hpp"
#include <igl/opengl/glfw/Viewer.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiPlugin.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <wmtk/utils/orient.hpp>
namespace wmtk {
namespace operations {
namespace utils {
void debug_visualization(
    const Eigen::MatrixXi& T_before,
    const Eigen::MatrixXi& T_after,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXd& V_param,
    const Eigen::MatrixXd& V_current,
    double current_energy)
{
    // Print debug information
    std::cout << "\n=== Debug Information ===" << std::endl;
    std::cout << "\nT_before (" << T_before.rows() << "x" << T_before.cols() << "):" << std::endl;
    std::cout << T_before << std::endl;
    std::cout << "\nT_after (" << T_after.rows() << "x" << T_after.cols() << "):" << std::endl;
    std::cout << T_after << std::endl;
    // Set high precision for floating point output
    std::cout << std::fixed << std::setprecision(15);
    std::cout << "\nV (" << V.rows() << "x" << V.cols() << "):" << std::endl;
    std::cout << V << std::endl;
    std::cout << "\nV_param (" << V_param.rows() << "x" << V_param.cols() << "):" << std::endl;
    std::cout << V_param << std::endl;
    std::cout << "\nV_current (" << V_current.rows() << "x" << V_current.cols()
              << "):" << std::endl;
    std::cout << V_current << std::endl;
    // Reset to default precision
    std::cout << std::defaultfloat;
    std::cout << "\n=========================" << std::endl;
    // Create viewer
    igl::opengl::glfw::Viewer viewer;
    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiPlugin plugin;
    viewer.plugins.push_back(&plugin);
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    plugin.widgets.push_back(&menu);
    // Store meshes for visualization
    Eigen::MatrixXd meshes[3] = {V, V_param, V_current};
    std::vector<std::string> mesh_names = {
        "Original (V)",
        "Parameterized (V_param)",
        "Optimized (V_current)"};
    static int current_mesh = 0;
    static int tet_topology = 0; // 0: before, 1: after
    static int selected_tet = 0;
    static bool show_single_tet = false;
    // Extract surface triangles from the tetrahedral mesh
    Eigen::MatrixXi F_surface_before, F_surface_after;
    // Extract all tetrahedral faces for visualization
    F_surface_before.resize(T_before.rows() * 4, 3);
    F_surface_after.resize(T_after.rows() * 4, 3);
    for (int i = 0; i < T_before.rows(); ++i) {
        F_surface_before.row(i * 4 + 0) << T_before(i, 1), T_before(i, 0), T_before(i, 2);
        F_surface_before.row(i * 4 + 1) << T_before(i, 1), T_before(i, 2), T_before(i, 3);
        F_surface_before.row(i * 4 + 2) << T_before(i, 0), T_before(i, 3), T_before(i, 2);
        F_surface_before.row(i * 4 + 3) << T_before(i, 1), T_before(i, 3), T_before(i, 0);
    }
    for (int i = 0; i < T_after.rows(); ++i) {
        F_surface_after.row(i * 4 + 0) << T_after(i, 1), T_after(i, 0), T_after(i, 2);
        F_surface_after.row(i * 4 + 1) << T_after(i, 1), T_after(i, 2), T_after(i, 3);
        F_surface_after.row(i * 4 + 2) << T_after(i, 0), T_after(i, 3), T_after(i, 2);
        F_surface_after.row(i * 4 + 3) << T_after(i, 1), T_after(i, 3), T_after(i, 0);
    }
    // Helper function to compute tet volume
    auto compute_tet_volume = [](const Eigen::MatrixXd& vertices, const Eigen::Vector4i& tet) {
        Eigen::Vector3d v0 = vertices.row(tet[0]);
        Eigen::Vector3d v1 = vertices.row(tet[1]);
        Eigen::Vector3d v2 = vertices.row(tet[2]);
        Eigen::Vector3d v3 = vertices.row(tet[3]);
        Eigen::Matrix3d M;
        M.col(0) = v1 - v0;
        M.col(1) = v2 - v0;
        M.col(2) = v3 - v0;
        return std::abs(M.determinant()) / 6.0;
    };
    auto update_mesh_display = [&]() {
        viewer.data().clear();
        if (show_single_tet) {
            // Display single tet faces
            Eigen::MatrixXi F_single_tet(4, 3);
            if (tet_topology == 0) { // before
                if (selected_tet < T_before.rows()) {
                    F_single_tet.row(0) << T_before(selected_tet, 1), T_before(selected_tet, 0),
                        T_before(selected_tet, 2);
                    F_single_tet.row(1) << T_before(selected_tet, 1), T_before(selected_tet, 2),
                        T_before(selected_tet, 3);
                    F_single_tet.row(2) << T_before(selected_tet, 0), T_before(selected_tet, 3),
                        T_before(selected_tet, 2);
                    F_single_tet.row(3) << T_before(selected_tet, 1), T_before(selected_tet, 3),
                        T_before(selected_tet, 0);
                }
                viewer.data().set_mesh(meshes[current_mesh], F_single_tet);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface_before);
            } else { // after
                if (selected_tet < T_after.rows()) {
                    F_single_tet.row(0) << T_after(selected_tet, 1), T_after(selected_tet, 0),
                        T_after(selected_tet, 2);
                    F_single_tet.row(1) << T_after(selected_tet, 1), T_after(selected_tet, 2),
                        T_after(selected_tet, 3);
                    F_single_tet.row(2) << T_after(selected_tet, 0), T_after(selected_tet, 3),
                        T_after(selected_tet, 2);
                    F_single_tet.row(3) << T_after(selected_tet, 1), T_after(selected_tet, 3),
                        T_after(selected_tet, 0);
                }
                viewer.data().set_mesh(meshes[current_mesh], F_single_tet);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface_after);
            }
        } else {
            // Display full mesh
            if (tet_topology == 0) {
                viewer.data().set_mesh(meshes[current_mesh], F_surface_before);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface_before);
            } else {
                viewer.data().set_mesh(meshes[current_mesh], F_surface_after);
                viewer.core().align_camera_center(meshes[current_mesh], F_surface_after);
            }
        }
        viewer.data().set_face_based(true);
    };
    // Customize the menu
    menu.callback_draw_viewer_menu = [&]() {
        // Draw parent menu content
        menu.draw_viewer_menu();
        // Add mesh selection group
        if (ImGui::CollapsingHeader("Mesh Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
            // Mesh selection combo
            if (ImGui::Combo(
                    "Vertex Data",
                    &current_mesh,
                    "Original (V)\0Parameterized (V_param)\0Optimized (V_current)\0\0")) {
                update_mesh_display();
            }
            // Display current energy information
            ImGui::Text("Current Energy: %.6f", current_energy);
            ImGui::Text("Mesh Vertices: %d", static_cast<int>(meshes[current_mesh].rows()));
            // Buttons for quick mesh switching
            if (ImGui::Button("Original Mesh", ImVec2(-1, 0))) {
                current_mesh = 0;
                update_mesh_display();
            }
            if (ImGui::Button("Parameterized Mesh", ImVec2(-1, 0))) {
                current_mesh = 1;
                update_mesh_display();
            }
            if (ImGui::Button("Optimized Mesh", ImVec2(-1, 0))) {
                current_mesh = 2;
                update_mesh_display();
            }
        }
        // Add topology selection group
        if (ImGui::CollapsingHeader("Topology Selection", ImGuiTreeNodeFlags_DefaultOpen)) {
            // Topology selection combo
            if (ImGui::Combo("Tet Topology", &tet_topology, "Before\0After\0\0")) {
                // Reset selected_tet when changing topology to ensure valid index
                selected_tet = 0;
                update_mesh_display();
            }
            if (tet_topology == 0) {
                ImGui::Text("Tetrahedra (Before): %d", static_cast<int>(T_before.rows()));
            } else {
                ImGui::Text("Tetrahedra (After): %d", static_cast<int>(T_after.rows()));
            }
            // Buttons for quick topology switching
            if (ImGui::Button("Before Topology", ImVec2(-1, 0))) {
                tet_topology = 0;
                selected_tet = 0;
                update_mesh_display();
            }
            if (ImGui::Button("After Topology", ImVec2(-1, 0))) {
                tet_topology = 1;
                selected_tet = 0;
                update_mesh_display();
            }
        }
        // Add single tet visualization
        if (ImGui::CollapsingHeader("Single Tet Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Checkbox("Show Single Tet", &show_single_tet)) {
                update_mesh_display();
            }
            if (show_single_tet) {
                int max_tets = (tet_topology == 0) ? T_before.rows() : T_after.rows();
                // Ensure selected_tet is within valid range
                if (selected_tet >= max_tets) {
                    selected_tet = max_tets - 1;
                }
                if (ImGui::SliderInt("Tet Index", &selected_tet, 0, max_tets - 1)) {
                    update_mesh_display();
                }
                // Input field for direct tet number entry
                int display_tet_number = selected_tet + 1; // Display 1-based indexing
                if (ImGui::InputInt("Tet Number (1-based)", &display_tet_number)) {
                    // Convert back to 0-based and clamp to valid range
                    selected_tet = std::max(0, std::min(display_tet_number - 1, max_tets - 1));
                    update_mesh_display();
                }
                ImGui::Text("Selected Tet %d (Faces 0-3)", selected_tet);
                if (tet_topology == 0 && selected_tet < T_before.rows()) {
                    ImGui::Text(
                        "Vertices: %d, %d, %d, %d",
                        T_before(selected_tet, 0),
                        T_before(selected_tet, 1),
                        T_before(selected_tet, 2),
                        T_before(selected_tet, 3));
                    double volume =
                        compute_tet_volume(meshes[current_mesh], T_before.row(selected_tet));
                    ImGui::Text("Volume: %.6e", volume);
                } else if (tet_topology == 1 && selected_tet < T_after.rows()) {
                    ImGui::Text(
                        "Vertices: %d, %d, %d, %d",
                        T_after(selected_tet, 0),
                        T_after(selected_tet, 1),
                        T_after(selected_tet, 2),
                        T_after(selected_tet, 3));
                    double volume =
                        compute_tet_volume(meshes[current_mesh], T_after.row(selected_tet));
                    ImGui::Text("Volume: %.6e", volume);
                }
                // Navigation buttons
                if (ImGui::Button("Previous Tet", ImVec2(-1, 0))) {
                    if (selected_tet > 0) {
                        selected_tet--;
                        update_mesh_display();
                    }
                }
                if (ImGui::Button("Next Tet", ImVec2(-1, 0))) {
                    if (selected_tet < max_tets - 1) {
                        selected_tet++;
                        update_mesh_display();
                    }
                }
            }
        }
        // Add rendering options
        if (ImGui::CollapsingHeader("Rendering Options", ImGuiTreeNodeFlags_DefaultOpen)) {
            static bool show_wireframe = true;
            if (ImGui::Checkbox("Show Wireframe", &show_wireframe)) {
                viewer.data().show_lines = show_wireframe;
            }
            // static bool face_based = true;
            // if (ImGui::Checkbox("Face Based", &face_based)) {
            //     viewer.data().set_face_based(face_based);
            // }
        }
    };
    // Set initial mesh and rendering properties
    update_mesh_display();
    viewer.data().set_face_based(true);
    viewer.data().show_lines = true;
    viewer.launch();
}


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
    // Print volumes for all tetrahedra
    std::cout << "Tetrahedra volumes:" << std::endl;
    for (size_t i = 0; i < P.size(); ++i) {
        std::cout << "Tet " << i << ": volume = " << P[i].volume << std::endl;
    }

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
                T_joint,
                P,
                new_grad,
                SymmetricDirichletEnergy());
            // Skip if energy is NaN
            if (std::isnan(new_energy)) {
                std::cout << "Energy is NaN, reducing step size" << std::endl;
                step_size *= step_reduction_factor;
                continue;
            }
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

    if (true) {
        // visualization for debugging
        debug_visualization(T_before, T_after, V, V_param, V_current, current_energy);
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
