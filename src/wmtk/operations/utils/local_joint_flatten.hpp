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

void local_joint_flatten(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint,
    bool is_bd_v0,
    bool is_bd_v1);

void local_joint_flatten_smoothing(
    const Eigen::MatrixXi& F, // the F will not change during the smoothing
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXd& V_after,
    Eigen::MatrixXd& UV_joint,
    Eigen::MatrixXi& F_after_output);

} // namespace wmtk::operations::utils