#pragma once

#include <Eigen/Core>

namespace wmtk {
namespace operations {
namespace utils {
int make_3_connected(Eigen::MatrixXi& F, Eigen::MatrixXd& V);
}
} // namespace operations
} // namespace wmtk