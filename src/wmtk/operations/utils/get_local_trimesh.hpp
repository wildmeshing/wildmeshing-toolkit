#pragma once
#include <wmtk/Mesh.hpp>

namespace wmtk {
class TriMesh;
class TetMesh;
} // namespace wmtk

namespace wmtk::operations::utils {
// V, F, and Id map
std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t>> get_local_trimesh(
    const wmtk::TriMesh& mesh,
    const wmtk::simplex::Simplex& simplex);

std::tuple<Eigen::MatrixXi, Eigen::MatrixXd, std::vector<int64_t>> get_local_tetmesh(
    const wmtk::TetMesh& mesh,
    const wmtk::simplex::Simplex& simplex);
} // namespace wmtk::operations::utils
