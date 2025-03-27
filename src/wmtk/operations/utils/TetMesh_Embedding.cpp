#include "TetMesh_Embedding.hpp"
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
                std::sort(face.begin(), face.end());
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

} // namespace wmtk::operations::utils