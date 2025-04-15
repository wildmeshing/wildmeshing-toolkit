#include "TetMesh_Embedding.hpp"
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/colormap.h>
#include <igl/harmonic.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/remove_unreferenced.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <array>
#include <map>
#include <set>

namespace wmtk::operations::utils {

std::vector<int> embed_mesh(const Eigen::MatrixXi& T, const Eigen::MatrixXi& F_bd)
{
    // Convert T and F_bd to sets for easier operations
    std::unordered_set<int> T_vertices;
    std::unordered_set<int> F_bd_vertices;

    // Collect all vertices from tetrahedra
    for (int i = 0; i < T.rows(); ++i) {
        for (int j = 0; j < T.cols(); ++j) {
            T_vertices.insert(T(i, j));
        }
    }

    // Collect all vertices from boundary faces
    for (int i = 0; i < F_bd.rows(); ++i) {
        for (int j = 0; j < F_bd.cols(); ++j) {
            F_bd_vertices.insert(F_bd(i, j));
        }
    }

    // Find vertices that are in T but not in F_bd
    std::unordered_set<int> non_bd_vertices;
    for (const auto& v : T_vertices) {
        if (F_bd_vertices.find(v) == F_bd_vertices.end()) {
            non_bd_vertices.insert(v);
        }
    }

    // Initialize detailed statistics dictionary
    std::map<std::string, int> tet_stats = {
        {"bd_0", 0}, // 0 boundary vertices
        {"bd_1", 0}, // 1 boundary vertex
        {"bd_2", 0}, // 2 boundary vertices
        {"bd_3", 0}, // 3 boundary vertices
        {"bd_4", 0} // 4 boundary vertices
    };

    // Iterate through each tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // Count boundary vertices
        int bd_count = 0;
        for (int j = 0; j < T.cols(); ++j) {
            if (F_bd_vertices.find(T(i, j)) != F_bd_vertices.end()) {
                bd_count++;
            }
        }
        // Classify based on boundary vertex count
        tet_stats["bd_" + std::to_string(bd_count)]++;
    }

    // Print detailed statistics
    spdlog::info("Tetrahedron Statistics:");
    spdlog::info("Number of tetrahedra with 0 boundary vertices: {}", tet_stats["bd_0"]);
    spdlog::info("Number of tetrahedra with 1 boundary vertex: {}", tet_stats["bd_1"]);
    spdlog::info("Number of tetrahedra with 2 boundary vertices: {}", tet_stats["bd_2"]);
    spdlog::info("Number of tetrahedra with 3 boundary vertices: {}", tet_stats["bd_3"]);
    spdlog::info("Number of tetrahedra with 4 boundary vertices: {}", tet_stats["bd_4"]);

    // Create a dictionary to store the count of boundary neighbors for each non-boundary vertex
    std::unordered_map<int, int> bd_neighbor_counts;
    for (const auto& v : non_bd_vertices) {
        bd_neighbor_counts[v] = 0;
    }

    // Create sets to store boundary neighbors for each non-boundary vertex
    std::unordered_map<int, std::unordered_set<int>> bd_neighbor_sets;
    for (const auto& v : non_bd_vertices) {
        bd_neighbor_sets[v] = std::unordered_set<int>();
    }

    // For each tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // For each vertex in the tetrahedron
        for (int j = 0; j < T.cols(); ++j) {
            int v = T(i, j);
            // If this vertex is non-boundary
            if (non_bd_vertices.find(v) != non_bd_vertices.end()) {
                // Add boundary neighbors from this tetrahedron to the set
                for (int k = 0; k < T.cols(); ++k) {
                    int neighbor = T(i, k);
                    if (F_bd_vertices.find(neighbor) != F_bd_vertices.end() && neighbor != v) {
                        bd_neighbor_sets[v].insert(neighbor);
                    }
                }
            }
        }
    }

    // Calculate the number of unique boundary neighbors for each non-boundary vertex
    for (const auto& v : non_bd_vertices) {
        bd_neighbor_counts[v] = bd_neighbor_sets[v].size();
    }

    // Print boundary neighbor counts for each non-boundary vertex
    spdlog::info("Boundary neighbor counts for non-boundary vertices:");
    for (const auto& [vertex, count] : bd_neighbor_counts) {
        spdlog::info("Vertex {} has {} boundary neighbors", vertex, count);
    }

    // Convert non-boundary vertices set to vector and return
    std::vector<int> result(non_bd_vertices.begin(), non_bd_vertices.end());
    return result;
}

Eigen::MatrixXi find_F_top(const std::unordered_set<int>& non_bd_vertices, const Eigen::MatrixXi& T)
{
    std::set<std::array<int, 3>> F_top_set;

    // For each tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // Get all possible triangular faces from the tetrahedron
        std::array<std::array<int, 3>, 4> faces = {
            std::array<int, 3>{T(i, 0), T(i, 1), T(i, 2)},
            std::array<int, 3>{T(i, 0), T(i, 1), T(i, 3)},
            std::array<int, 3>{T(i, 0), T(i, 2), T(i, 3)},
            std::array<int, 3>{T(i, 1), T(i, 2), T(i, 3)}};

        // Check each face
        for (auto face : faces) {
            // If all vertices of the face are interior vertices
            if (non_bd_vertices.find(face[0]) != non_bd_vertices.end() &&
                non_bd_vertices.find(face[1]) != non_bd_vertices.end() &&
                non_bd_vertices.find(face[2]) != non_bd_vertices.end()) {
                // Sort vertices to ensure consistent orientation
                // std::sort(face.begin(), face.end());
                F_top_set.insert(face);
            }
        }
    }

    // Convert set to Eigen::MatrixXi
    Eigen::MatrixXi F_top(F_top_set.size(), 3);
    int row = 0;
    for (const auto& face : F_top_set) {
        F_top(row, 0) = face[0];
        F_top(row, 1) = face[1];
        F_top(row, 2) = face[2];
        row++;
    }

    return F_top;
}


void find_boundary_loop(const Eigen::MatrixXi& F, Eigen::VectorXi& bnd_loop)
{
    // 创建边到面的映射
    std::map<std::pair<int, int>, std::vector<int>> edge_to_face;

    // 遍历所有三角形
    for (int i = 0; i < F.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            int v1 = F(i, j);
            int v2 = F(i, (j + 1) % 3);

            // 确保边的顺序一致（小索引在前）
            if (v1 > v2) std::swap(v1, v2);

            edge_to_face[{v1, v2}].push_back(i);
        }
    }

    // 找出边界边（只被一个面引用的边）
    std::vector<std::pair<int, int>> boundary_edges;
    for (const auto& edge_faces : edge_to_face) {
        if (edge_faces.second.size() == 1) {
            boundary_edges.push_back(edge_faces.first);
        }
    }

    // 如果没有边界边，返回空结果
    if (boundary_edges.empty()) {
        bnd_loop.resize(0);
        return;
    }

    // 构建边界边的连接关系
    std::map<int, std::vector<int>> vertex_to_vertices;
    for (const auto& edge : boundary_edges) {
        vertex_to_vertices[edge.first].push_back(edge.second);
        vertex_to_vertices[edge.second].push_back(edge.first);
    }

    // 找出最长的边界环
    std::vector<int> longest_loop;
    std::set<int> visited;

    // 对每个可能的起始顶点尝试构建环
    for (const auto& entry : vertex_to_vertices) {
        if (visited.count(entry.first) > 0) continue;

        int start_vertex = entry.first;
        std::vector<int> current_loop;
        current_loop.push_back(start_vertex);
        visited.insert(start_vertex);

        int current = start_vertex;
        bool found_next = true;

        while (found_next) {
            found_next = false;

            for (int next : vertex_to_vertices[current]) {
                if (visited.count(next) == 0) {
                    current_loop.push_back(next);
                    visited.insert(next);
                    current = next;
                    found_next = true;
                    break;
                }
            }
        }

        // 检查是否形成了环（最后一个顶点连接到第一个顶点）
        for (int next : vertex_to_vertices[current]) {
            if (next == start_vertex) {
                // 找到了一个环
                if (current_loop.size() > longest_loop.size()) {
                    longest_loop = current_loop;
                }
                break;
            }
        }
    }

    // 将结果转换为Eigen::VectorXi
    bnd_loop.resize(longest_loop.size());
    for (int i = 0; i < longest_loop.size(); ++i) {
        bnd_loop(i) = longest_loop[i];
    }

    spdlog::info("找到边界环，包含 {} 个顶点", bnd_loop.size());
}

Eigen::MatrixXd parametrize_top(
    const Eigen::MatrixXi& F_top,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F_bd,
    const Eigen::MatrixXd& uv_bd,
    const Eigen::VectorXi& IM_uv_bd,
    const Eigen::MatrixXi& T,
    Eigen::VectorXi& IM_top,
    bool debug_mode)
{
    // First remove unreferenced vertices from F_top and V
    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    Eigen::VectorXi J;
    igl::remove_unreferenced(V, F_top, NV, NF, IM_top, J);

    // Use cleaned mesh data
    Eigen::MatrixXd V_clean_top = NV;
    Eigen::MatrixXi F_clean_top = NF;

    // Check if F_clean_top is empty
    if (F_clean_top.size() == 0) {
        spdlog::warn("F_clean_top is empty");
        return Eigen::MatrixXd();
    }

    spdlog::info("F_clean_top: {} rows", F_clean_top.rows());

    // Use libigl to get boundary loop
    Eigen::VectorXi bnd;
    // igl::boundary_loop(F_clean_top, bnd);
    // There is a weird bug in igl::boundary_loop, when there is only two triangles
    find_boundary_loop(F_clean_top, bnd);
    // Print F_clean_top matrix
    spdlog::info("F_clean_top matrix:");
    for (int i = 0; i < F_clean_top.rows(); ++i) {
        std::string row_str = "";
        for (int j = 0; j < F_clean_top.cols(); ++j) {
            row_str += std::to_string(F_clean_top(i, j)) + " ";
        }
        spdlog::info("  Row {}: {}", i, row_str);
    }

    // Print boundary loop bnd
    std::string bnd_str = "";
    for (int i = 0; i < bnd.size(); ++i) {
        bnd_str += std::to_string(bnd(i)) + " ";
    }
    spdlog::info("Boundary loop bnd: {}", bnd_str);
    spdlog::info("Boundary vertices count: {}", bnd.size());

    // Get boundary loop of F_bd
    Eigen::VectorXi bd_loop;
    igl::boundary_loop(F_bd, bd_loop);

    // Initialize UV coordinates
    Eigen::MatrixXd uv = Eigen::MatrixXd::Zero(V_clean_top.rows(), 2);

    // Process each boundary vertex
    for (int i = 0; i < bnd.size(); ++i) {
        int v_idx = bnd(i);
        // Find corresponding vertex index in original mesh
        int orig_v_idx = J(v_idx);
        spdlog::info("orig_v_idx: {}", orig_v_idx);
        // Find connected tetrahedra and adjacent boundary vertices
        std::vector<Eigen::Vector2d> neighbor_uvs;
        std::set<int> neighbor_verts;

        for (int t = 0; t < T.rows(); ++t) {
            bool contains_orig_v = false;
            for (int j = 0; j < 4; ++j) {
                if (T(t, j) == orig_v_idx) {
                    contains_orig_v = true;
                    break;
                }
            }

            if (contains_orig_v) {
                for (int j = 0; j < 4; ++j) {
                    int v = T(t, j);
                    if (v != orig_v_idx) {
                        // Check if v is in bd_loop
                        bool in_bd_loop = false;
                        for (int k = 0; k < bd_loop.size(); ++k) {
                            if (bd_loop(k) == v) {
                                in_bd_loop = true;
                                break;
                            }
                        }
                        if (in_bd_loop && neighbor_verts.find(v) == neighbor_verts.end()) {
                            neighbor_verts.insert(v);

                            if (v < IM_uv_bd.size() && IM_uv_bd(v) != -1) {
                                Eigen::Vector2d uv_coord = uv_bd.row(IM_uv_bd(v));
                                neighbor_uvs.push_back(uv_coord);
                                spdlog::info(
                                    "Found adjacent boundary vertex {} with UV coords: ({}, {})",
                                    v,
                                    uv_coord(0),
                                    uv_coord(1));
                            }
                        }
                    }
                }
            }
        }

        if (!neighbor_uvs.empty()) {
            Eigen::Vector2d avg_uv = Eigen::Vector2d::Zero();
            for (const auto& uv_coord : neighbor_uvs) {
                avg_uv += uv_coord;
            }
            avg_uv /= neighbor_uvs.size();
            uv.row(v_idx) = avg_uv;
            spdlog::info(
                "Final UV coordinates (average) for vertex {}: ({}, {})",
                orig_v_idx,
                avg_uv(0),
                avg_uv(1));
        }
    }


    if (bnd.size() < V_clean_top.rows()) {
        // Harmonic mapping for interior points
        Eigen::MatrixXd bnd_uv(bnd.size(), 2);
        for (int i = 0; i < bnd.size(); ++i) {
            bnd_uv.row(i) = uv.row(bnd(i));
        }
        igl::harmonic(V_clean_top, F_clean_top, bnd, bnd_uv, 1, uv);
    }


    if (debug_mode) {
        // Visualize parameterization result
        spdlog::info("Starting visualization of parameterization result");

        // Create 2D vertex matrix for visualization
        Eigen::MatrixXd V_uv(uv.rows(), 3);
        V_uv.leftCols(2) = uv;
        V_uv.col(2).setZero();

        // Use igl::opengl::glfw::Viewer to directly visualize the parameterization result
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_uv, F_clean_top);
        viewer.data().set_face_based(true);
        viewer.data().show_lines = true;
        viewer.launch();

        spdlog::info("Visualization of parameterization result completed");
        spdlog::info("end harmonic");
    }
    return uv;
}

void visualize_tet_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& T)
{
    spdlog::info("Starting tetrahedral mesh visualization");

    // Create viewer
    igl::opengl::glfw::Viewer viewer;
    // Directly visualize the surface of the tetrahedral mesh
    // Extract surface triangles from the tetrahedral mesh
    Eigen::MatrixXi F_surface;
    std::vector<std::array<int, 3>> surface_triangles;

    // Use a simple method to extract the surface: process each face of every tetrahedron
    for (int i = 0; i < T.rows(); ++i) {
        // Four faces of the tetrahedron
        std::array<std::array<int, 3>, 4> tet_faces = {
            {{T(i, 0), T(i, 1), T(i, 3)},
             {T(i, 0), T(i, 2), T(i, 1)},
             {T(i, 0), T(i, 3), T(i, 2)},
             {T(i, 1), T(i, 2), T(i, 3)}}};

        // Add all faces (simplified approach, ideally should detect which are on the surface)
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

    // Set mesh and rendering properties
    viewer.data().set_mesh(V, F_surface);
    viewer.data().set_face_based(true);
    viewer.data().show_lines = true;


    viewer.launch();
    spdlog::info("Tetrahedral mesh visualization completed");
}

void launch_debug_viewer(
    const Eigen::MatrixXd& V_bottom,
    const Eigen::MatrixXi& F_bottom,
    const Eigen::MatrixXd& uv_bottom)
{
    igl::opengl::glfw::Viewer viewer;

    viewer.data().clear();
    viewer.data().set_mesh(V_bottom, F_bottom);
    viewer.data().set_face_based(true);
    viewer.data().show_lines = true;
    viewer.data().line_width = 1.0f;
    viewer.data().line_color << 0.0f, 0.0f, 0.0f, 1.0f;

    viewer.core().align_camera_center(V_bottom, F_bottom);

    viewer.callback_key_pressed =
        [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) -> bool {
        if (key == '1') {
            viewer.data().clear();
            viewer.data().set_mesh(V_bottom, F_bottom);
            viewer.data().set_face_based(true);
            viewer.data().show_lines = true;
            viewer.data().line_width = 1.0f;
            viewer.data().line_color << 0.0f, 0.0f, 0.0f, 1.0f;
            viewer.core().align_camera_center(V_bottom, F_bottom);
            spdlog::info("Switched to 3D mesh view");
            return true;
        } else if (key == '2') {
            viewer.data().clear();
            viewer.data().set_mesh(uv_bottom, F_bottom);
            viewer.data().set_face_based(true);
            viewer.data().show_lines = true;
            viewer.data().line_width = 1.0f;
            viewer.data().line_color << 0.0f, 0.0f, 0.0f, 1.0f;
            viewer.data().set_colors(Eigen::RowVector3d(0.8, 0.8, 0.8));
            viewer.core().align_camera_center(uv_bottom, F_bottom);
            spdlog::info("Switched to UV parameterization view");
            return true;
        }
        return false;
    };

    spdlog::info("Launching viewer - Press 1 for 3D mesh, Press 2 for "
                 "UV parameterization");
    viewer.launch();
}

} // namespace wmtk::operations::utils