#include "TetMesh_Embedding.hpp"
#include <igl/barycenter.h>
#include <igl/boundary_loop.h>
#include <igl/colormap.h>
#include <igl/harmonic.h>
#include <igl/map_vertices_to_circle.h>
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

Eigen::MatrixXi extract_surface_without_vertex(
    const Eigen::MatrixXi& T,
    // const Eigen::MatrixXd& V,
    int vertex_to_remove)
{
    std::cout << "T: \n" << T << std::endl;
    // Iterate through all tetrahedra to find the face not containing vertex_to_remove
    // Assumption: All tets are connected to vertex_to_remove except one face
    std::vector<Eigen::Vector3i> surface_triangles;

    for (int i = 0; i < T.rows(); i++) {
        Eigen::Vector4i tet = T.row(i);
        std::cout << "i: " << i << std::endl;
        std::cout << "tet: " << tet << std::endl;
        // Find position of vertex_to_remove in tet
        int pos = -1;
        for (int j = 0; j < 4; j++) {
            if (tet[j] == vertex_to_remove) {
                pos = j;
                break;
            }
        }

        if (pos != -1) {
            // Order vertices based on tet orientation:
            // pos=0: (1,3,2), pos=1: (0,2,3), pos=2: (0,3,1), pos=3: (0,1,2)
            Eigen::Vector3i oriented_face;
            switch (pos) {
            case 0: oriented_face = Eigen::Vector3i(tet[1], tet[3], tet[2]); break;
            case 1: oriented_face = Eigen::Vector3i(tet[0], tet[2], tet[3]); break;
            case 2: oriented_face = Eigen::Vector3i(tet[0], tet[3], tet[1]); break;
            case 3: oriented_face = Eigen::Vector3i(tet[0], tet[1], tet[2]); break;
            }
            surface_triangles.push_back(oriented_face);
        }
    }

    // Convert vector to matrix
    Eigen::MatrixXi F0(surface_triangles.size(), 3);
    for (int i = 0; i < surface_triangles.size(); i++) {
        F0.row(i) = surface_triangles[i];
    }

    return F0;
}

std::tuple<Eigen::MatrixXd, Eigen::VectorXi, Eigen::MatrixXd, Eigen::MatrixXi>
harmonic_parameterization(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    Eigen::VectorXi IM, J;
    igl::remove_unreferenced(V, F, NV, NF, IM, J);

    Eigen::MatrixXd V_clean = NV;
    Eigen::MatrixXi F_clean = NF;

    Eigen::VectorXi bnd;
    igl::boundary_loop(F_clean, bnd);

    Eigen::MatrixXd bnd_uv;
    igl::map_vertices_to_circle(V_clean, bnd, bnd_uv);

    Eigen::MatrixXd uv;
    igl::harmonic(V_clean, F_clean, bnd, bnd_uv, 1, uv);

    return {uv, IM, V_clean, F_clean};
}

Eigen::MatrixXd lift_to_hemisphere(const Eigen::MatrixXd& uv, const Eigen::MatrixXi& F)
{
    // Get mesh boundary
    Eigen::VectorXi bnd;
    igl::boundary_loop(F, bnd);

    // Create a set of boundary vertex indices for quick lookup
    std::unordered_set<int> bnd_vertices;
    for (int i = 0; i < bnd.size(); i++) {
        bnd_vertices.insert(bnd(i));
    }

    // Use squared distance from first boundary point to origin as R^2
    double R2 = uv(bnd(0), 0) * uv(bnd(0), 0) + uv(bnd(0), 1) * uv(bnd(0), 1);

    // Create lifted 3D coordinates
    Eigen::MatrixXd V_hemisphere(uv.rows(), 3);
    V_hemisphere.leftCols(2) = uv; // Copy x,y coordinates

    // Compute z coordinates
    for (int i = 0; i < uv.rows(); i++) {
        if (bnd_vertices.find(i) != bnd_vertices.end()) {
            // Set boundary points z coordinate to 0
            V_hemisphere(i, 2) = 0;
        } else {
            // Lift interior points to hemisphere
            double x = uv(i, 0);
            double y = uv(i, 1);
            V_hemisphere(i, 2) = -std::sqrt(R2 - x * x - y * y);
        }
    }

    return V_hemisphere;
}

std::tuple<Eigen::MatrixXd, std::map<std::pair<int, int>, double>> tutte_embedding_case3(
    const std::vector<std::vector<int>>& F,
    const std::vector<int>& f0)
{
    // 1. Collect all vertices and index them 0..n-1
    std::set<int> verts_set;
    for (const auto& face : F) {
        verts_set.insert(face.begin(), face.end());
    }
    verts_set.insert(f0.begin(), f0.end());
    std::vector<int> verts(verts_set.begin(), verts_set.end());
    std::unordered_map<int, int> vid2idx;
    std::unordered_map<int, int> idx2vid;
    for (int i = 0; i < verts.size(); ++i) {
        vid2idx[verts[i]] = i;
        idx2vid[i] = verts[i];
    }
    int n = verts.size();

    // 2. Boundary indices
    std::vector<int> B;
    for (int v : f0) {
        B.push_back(vid2idx[v]);
    }
    std::set<int> Bset(B.begin(), B.end());

    // 3. Count interior edges from F
    std::map<std::pair<int, int>, int> edge_count;
    for (const auto& face : F) {
        int m = face.size();
        for (int k = 0; k < m; ++k) {
            int a = vid2idx[face[k]];
            int b = vid2idx[face[(k + 1) % m]];
            auto e = std::make_pair(std::min(a, b), std::max(a, b));
            edge_count[e]++;
        }
    }

    // 4. Build Laplacian L with ω=1 on interior edges using dense matrix
    Eigen::MatrixXd L = Eigen::MatrixXd::Zero(n, n);
    for (const auto& [e, c] : edge_count) {
        int i = e.first;
        int j = e.second;
        double w = 1.0;
        L(i, i) += w;
        L(j, j) += w;
        L(i, j) -= w;
        L(j, i) -= w;
    }

    // 5. Partition into boundary (B) and interior (I)
    std::vector<int> I;
    for (int i = 0; i < n; ++i) {
        if (Bset.find(i) == Bset.end()) {
            I.push_back(i);
        }
    }
    Eigen::MatrixXd LIB(I.size(), B.size());
    Eigen::MatrixXd LII(I.size(), I.size());
    for (int i = 0; i < I.size(); ++i) {
        for (int j = 0; j < B.size(); ++j) {
            LIB(i, j) = L(I[i], B[j]);
        }
        for (int j = 0; j < I.size(); ++j) {
            LII(i, j) = L(I[i], I[j]);
        }
    }

    // 6. Fix outer triangle positions
    Eigen::MatrixXd pB(3, 2);
    pB << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;
    Eigen::VectorXd xB = pB.col(0);
    Eigen::VectorXd yB = pB.col(1);

    // 7. Solve for interior coordinates
    Eigen::VectorXd xI = LII.ldlt().solve(-LIB * xB);
    Eigen::VectorXd yI = LII.ldlt().solve(-LIB * yB);

    // 8. Assemble uv mapping
    Eigen::MatrixXd uv(n, 2);
    for (int idx = 0; idx < B.size(); ++idx) {
        uv.row(B[idx]) = pB.row(idx);
    }
    for (int idx = 0; idx < I.size(); ++idx) {
        uv(I[idx], 0) = xI(idx);
        uv(I[idx], 1) = yI(idx);
    }

    // 9. Compute substitution matrix S = LIB^T * (LII^{-1} * LIB)
    Eigen::MatrixXd X = LII.colPivHouseholderQr().solve(LIB);
    Eigen::MatrixXd S = LIB.transpose() * X;

    // 10. Assemble stress dict
    std::map<std::pair<int, int>, double> stress;
    // interior edges
    for (const auto& [e, c] : edge_count) {
        int a = e.first;
        int b = e.second;
        stress[{idx2vid[a], idx2vid[b]}] = 1.0;
        stress[{idx2vid[b], idx2vid[a]}] = 1.0;
    }
    // boundary edges (loop of f0)
    for (int p = 0; p < 3; ++p) {
        int q = (p + 1) % 3;
        int i = B[p];
        int j = B[q];
        int va = idx2vid[i];
        int vb = idx2vid[j];
        double wb = -S(p, q);
        stress[{va, vb}] = wb;
        stress[{vb, va}] = wb;
    }

    return {uv, stress};
}

Eigen::MatrixXd compute_lifting(
    const std::vector<std::vector<int>>& F,
    const std::vector<int>& f0,
    const Eigen::MatrixXd& uv,
    const std::map<std::pair<int, int>, double>& stresses)
{
    // 1. Build combined face list:
    std::vector<std::vector<int>> F_all = F;

    F_all.push_back(f0);
    std::swap(F_all.back()[0], F_all.back()[1]); // Swap the first two elements of the boundary loop
                                                 // to make it face outward
    int num_faces = F_all.size();

    // 2. Build edge->faces adjacency
    std::map<std::pair<int, int>, std::vector<int>> edge2faces;
    for (int fid = 0; fid < num_faces; ++fid) {
        const auto& face = F_all[fid];
        int m = face.size();
        for (int i = 0; i < m; ++i) {
            int a = face[i];
            int b = face[(i + 1) % m];
            auto edge = std::minmax(a, b);
            edge2faces[edge].push_back(fid);
        }
    }

    // 3. Build face adjacency list
    std::vector<std::vector<int>> adj_faces(num_faces);
    for (const auto& [edge, fids] : edge2faces) {
        if (fids.size() == 2) {
            int f1 = fids[0];
            int f2 = fids[1];
            adj_faces[f1].push_back(f2);
            adj_faces[f2].push_back(f1);
        }
    }
    for (auto& neighbors : adj_faces) {
        std::sort(neighbors.begin(), neighbors.end());
    }
    // 4. Initialize plane params a (2-vector) and d (scalar)
    std::vector<Eigen::Vector2d> a(num_faces);
    std::vector<double> d(num_faces);
    int f_base = num_faces - 2; // last of F_all
    a[f_base] = Eigen::Vector2d(0.0, 0.0);
    d[f_base] = 0.0;

    // 5. BFS over faces starting from base
    std::queue<int> queue;
    std::set<int> visited;
    queue.push(f_base);
    visited.insert(f_base);
    while (!queue.empty()) {
        int fr = queue.front();
        queue.pop();
        const Eigen::Vector2d& ar = a[fr];
        double dr = d[fr];
        std::cout << "fr: " << fr << ", ar: " << ar.transpose() << ", dr: " << dr << std::endl;
        // for each neighbor face
        for (int fl : adj_faces[fr]) {
            if (visited.count(fl)) continue;
            std::cout << "fl: " << fl << std::endl;
            // find common edge
            std::set<int> shared;
            for (int v : F_all[fr]) {
                if (std::find(F_all[fl].begin(), F_all[fl].end(), v) != F_all[fl].end()) {
                    shared.insert(v);
                }
            }
            // find edge vertices i,j in common
            int i = -1, j = -1;
            for (int u : shared) {
                for (int v : shared) {
                    if (u < v && edge2faces.count(std::minmax(u, v))) {
                        i = u;
                        j = v;
                        break;
                    }
                }
                if (i != -1) break;
            }
            // determine orientation: want fl left of directed (i->j)
            Eigen::Vector2d pi = uv.row(i);
            Eigen::Vector2d pj = uv.row(j);
            // find third vertex k of fl
            int k = -1;
            for (int v : F_all[fl]) {
                if (v != i && v != j) {
                    k = v;
                    break;
                }
            }
            std::cout << "i: " << i << ", j: " << j << ", k: " << k << std::endl;
            Eigen::Vector2d pk = uv.row(k);
            // cross of (pj-pi) x (pk-pi)
            double cross = (pj - pi).x() * (pk - pi).y() - (pj - pi).y() * (pk - pi).x();

            std::cout << "cross: " << cross << std::endl;
            if (cross < 0) {
                continue;
                // swap to make fl on left
                // std::swap(i, j);
                // std::swap(pi, pj);
            }

            // stress on this edge
            // double wij = stresses.at(std::minmax(i, j));
            double wij = stresses.at(std::make_pair(i, j));
            // compute (pi - pj)_perp
            Eigen::Vector2d v = pi - pj;
            Eigen::Vector2d v_perp(-v.y(), v.x());
            // compute a[fl] and d[fl]
            a[fl] = wij * v_perp + ar;
            Eigen::Vector2d pj_perp(-pj.y(), pj.x());
            d[fl] = wij * pi.dot(pj_perp) + dr;
            visited.insert(fl);
            queue.push(fl);
        }
    }

    // 6. Compute z for each vertex
    // build vertex->incident-face map
    std::map<int, std::vector<int>> vert2faces;
    for (int fid = 0; fid < num_faces; ++fid) {
        for (int v : F_all[fid]) {
            vert2faces[v].push_back(fid);
        }
    }
    // Calculate the center of f_base in uv
    Eigen::Vector2d center(0, 0);
    for (int v : F_all[f_base]) {
        center += uv.row(v);
    }
    center /= F_all[f_base].size();

    Eigen::MatrixXd xyz(uv.rows(), 3);
    for (int v = 0; v < uv.rows(); ++v) {
        double x = uv(v, 0);
        double y = uv(v, 1);
        // pick first incident face
        int fid = vert2faces[v][0];
        double z = a[fid].dot(Eigen::Vector2d(x, y)) + d[fid];
        xyz(v, 0) = x - center(0);
        xyz(v, 1) = y - center(1);
        xyz(v, 2) = z;
    }

    for (int v : F_all[f_base]) {
        xyz(v, 2) = 0; // Force the z-coordinate of f_base's vertices to be 0
    }
    return xyz;
}

Eigen::MatrixXd embed_mesh_lift(const Eigen::MatrixXi& T, const Eigen::MatrixXd& V, int v0)
{
    // TODO: implement this function

    // get the surface without vertex v0
    Eigen::MatrixXi F0 = extract_surface_without_vertex(T, v0);
    std::cout << "F0: \n" << F0 << std::endl;

    // clean up unreferenced vertices(v0)
    Eigen::MatrixXd V_clean;
    Eigen::MatrixXi F_clean;
    Eigen::VectorXi IM, J;
    igl::remove_unreferenced(V, F0, V_clean, F_clean, IM, J);

    std::vector<std::vector<int>> F_list_vec;
    for (int i = 0; i < F_clean.rows(); ++i) {
        F_list_vec.push_back({F_clean(i, 0), F_clean(i, 1), F_clean(i, 2)});
    }
    Eigen::VectorXi bnd_loop;
    igl::boundary_loop(F_clean, bnd_loop);
    F_list_vec.push_back(std::vector<int>(bnd_loop.data(), bnd_loop.data() + bnd_loop.size()));
    std::vector<int> f0 = F_list_vec.front();
    F_list_vec.erase(F_list_vec.begin());

    // get the uv coordinates
    auto [uv, stress] = tutte_embedding_case3(F_list_vec, f0);

    auto check_uv_orientation = [](const Eigen::MatrixXd& uv, const Eigen::MatrixXi& F) {
        for (int i = 0; i < F.rows(); i++) {
            if (wmtk::utils::wmtk_orient2d(uv.row(F(i, 0)), uv.row(F(i, 1)), uv.row(F(i, 2))) > 0) {
                // std::cerr << "UV orientation check failed for F_clean[" << i << "]" << std::endl;
                return false;
            }
        }
        return true;
    };

    static int failure_count = 0;
    static int total_count = 0;
    total_count++;
    bool is_tutte_embedding_failed = false;
    if (!check_uv_orientation(-uv, F_clean.bottomRows(F_clean.rows() - 1))) {
        std::cerr << "UV orientation check failed for F_clean[1:end]." << std::endl;
        failure_count++;
        std::cerr << "Failure count: " << failure_count << std::endl;
        is_tutte_embedding_failed = true;
        // throw std::runtime_error("Invalid UV orientation detected.");
    }
    {
        std::cout << "F_list = [";
        for (const auto& face : F_list_vec) {
            std::cout << "[";
            for (size_t i = 0; i < face.size(); ++i) {
                std::cout << face[i];
                if (i < face.size() - 1) std::cout << ", ";
            }
            std::cout << "]";
            if (&face != &F_list_vec.back()) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "uv_array = [";
        for (int i = 0; i < uv.rows(); ++i) {
            std::cout << "[";
            for (int j = 0; j < uv.cols(); ++j) {
                std::cout << uv(i, j);
                if (j < uv.cols() - 1) std::cout << ", ";
            }
            std::cout << "]";
            if (i < uv.rows() - 1) std::cout << ", ";
        }
        std::cout << "]" << std::endl;

        std::cout << "stress = {";
        for (auto it = stress.begin(); it != stress.end(); ++it) {
            std::cout << "(" << it->first.first << ", " << it->first.second << "): " << it->second;
            if (std::next(it) != stress.end()) std::cout << ", ";
        }
        std::cout << "}" << std::endl;
    }

    auto xyz = compute_lifting(F_list_vec, f0, uv, stress);

    std::cout << "IM: " << IM << std::endl;
    Eigen::MatrixXd V_param = Eigen::MatrixXd::Zero(V.rows(), V.cols());
    for (int i = 0; i < IM.size(); ++i) {
        if (IM[i] != -1) {
            V_param.row(i) = xyz.row(IM[i]);
        }
    }
    V_param.row(v0) = Eigen::RowVector3d::Zero();
    {
        std::cout << "V_param = [" << std::endl;
        for (int i = 0; i < V_param.rows(); ++i) {
            std::cout << "  [";
            for (int j = 0; j < V_param.cols(); ++j) {
                std::cout << V_param(i, j);
                if (j < V_param.cols() - 1) std::cout << ", ";
            }
            std::cout << "]";
            if (i < V_param.rows() - 1) std::cout << "," << std::endl;
        }
        std::cout << std::endl << "]" << std::endl;
    }
    // utils::visualize_tet_mesh(V_param, T);

    std::cout << "Total failure count: " << failure_count << std::endl;
    std::cout << "Total count: " << total_count << std::endl;
    if (is_tutte_embedding_failed) {
        // TODO: try to fix the embedding later
        return Eigen::MatrixXd();
    } else {
        return V_param;
    }
}
} // namespace wmtk::operations::utils
