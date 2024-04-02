#pragma once
#include <Eigen/Core>
namespace wmtk::operations::utils {

void flatten(
    const Eigen::MatrixXd& Vjoint,
    const Eigen::MatrixXi& Fjoint_before,
    const Eigen::MatrixXi& Fjoint_after,
    const Eigen::VectorXi& b_UV,
    const Eigen::VectorXd& bc_UV,
    Eigen::MatrixXd& UVjoint);

// TODO: add support for boundary cases
void local_joint_flatten(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint);
} // namespace wmtk::operations::utils