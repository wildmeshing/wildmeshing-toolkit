#pragma once
#include <Eigen/Core>
namespace wmtk::operations::utils {

void flatten(
    const Eigen::MatrixXd& Vjoint,
    const Eigen::MatrixXi& Fjoint_before,
    const Eigen::MatrixXi& Fjoint_after,
    const Eigen::VectorXi& b_UV,
    const Eigen::VectorXd& bc_UV,
    Eigen::VectorXd& UVjoint_flat);

// TODO: add support for boundary cases
void local_joint_flatten(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,

    Eigen::MatrixXd& uv_before,
    Eigen::MatrixXd& uv_after);
} // namespace wmtk::operations::utils