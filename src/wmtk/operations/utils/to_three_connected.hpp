#pragma once

#include <Eigen/Core>

namespace wmtk {
namespace operations {
namespace utils {
void make_3_connected(const Eigen::MatrixXi& F, Eigen::MatrixXd& V);
}
} // namespace operations
} // namespace wmtk