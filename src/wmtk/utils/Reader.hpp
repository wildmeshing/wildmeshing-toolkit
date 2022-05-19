#include <geogram/mesh/mesh_io.h>
#include <Eigen/Core>
#include "Logger.hpp"
namespace wmtk {
void reader(std::string input_surface, Eigen::MatrixXd& VI, Eigen::MatrixXi& FI);
void input_formatter(
    std::vector<Eigen::Vector3d>& verts,
    std::vector<std::array<size_t, 3>>& tris,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F);
} // namespace wmtk
