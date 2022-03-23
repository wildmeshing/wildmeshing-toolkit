#pragma once

#include "wmtk/TetMesh.h"

#include <Eigen/Core>
#include <array>
#include <vector>
#include "tbb/concurrent_map.h"


namespace wmtk {
/**
 * @brief Before triangle insertion, find which ones are already present in the mesh.
 * Note that the vertices are already the same, so just do a dictionary-find for the face
 * indices.
 * @param vertices
 * @param faces
 * @param output is_matched
 */
void match_tet_faces_to_triangles(
    const wmtk::TetMesh& m,
    const std::vector<Eigen::Vector3d>& vertices,
    const std::vector<std::array<size_t, 3>>& faces,
    tbb::concurrent_vector<bool>& is_matched,
    tbb::concurrent_map<std::array<size_t, 3>, std::vector<int>>& tet_face_tags);

bool remove_duplicates(
    std::vector<Eigen::Vector3d>& vertices,
    std::vector<std::array<size_t, 3>>& faces,
    double);
} // namespace wmtk