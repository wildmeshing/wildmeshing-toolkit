#include <geogram/mesh/mesh_io.h>
#include <igl/is_edge_manifold.h>
#include <igl/is_vertex_manifold.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/remove_unreferenced.h>
#include <igl/resolve_duplicated_faces.h>
#include <Eigen/Core>
#include <wmtk/utils/ManifoldUtils.hpp>
#include "Logger.hpp"
namespace wmtk {
void stl_to_eigen(std::string input_surface, Eigen::MatrixXd& VI, Eigen::MatrixXi& FI);
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
