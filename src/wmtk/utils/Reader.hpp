#pragma once

#include <Eigen/Core>

namespace wmtk {
void eigen_to_wmtk_input(
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F);
void stl_to_manifold_wmtk_input(
    std::string input_path,
    double remove_duplicate_esp,
    std::pair<Eigen::Vector3d, Eigen::Vector3d>& box_minmax,
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    std::vector<size_t>& modified_nonmanifold_v);
void stl_to_manifold_wmtk_input(
    std::vector<std::string> input_paths,
    double remove_duplicate_esp,
    std::pair<Eigen::Vector3d, Eigen::Vector3d>& box_minmax,
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    std::vector<size_t>& modified_nonmanifold_v);
void resolve_duplicated_faces(const Eigen::MatrixXi& inF, Eigen::MatrixXi& outF);
} // namespace wmtk
