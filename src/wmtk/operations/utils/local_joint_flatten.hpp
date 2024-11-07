#pragma once
#include <Eigen/Core>
namespace wmtk::operations::utils {

// void flatten(
//     const Eigen::MatrixXd& V_joint_before,
//     const Eigen::MatrixXd& V_joint_after,
//     const Eigen::MatrixXi& F_joint_before,
//     const Eigen::MatrixXi& F_joint_after,
//     // const Eigen::VectorXi& b_soft,
//     const std::vector<std::pair<int, int>>& b_hard,
//     Eigen::MatrixXd& UVjoint,
//     int n_iterations = 10);

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
    bool is_bd_v1,
    bool debug_mode = false);

void local_joint_flatten_smoothing(
    const Eigen::MatrixXi& F, // the F will not change during the smoothing
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    Eigen::MatrixXd& UV_joint,
    bool debug_mode = false);

void local_joint_flatten_swap(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    Eigen::MatrixXd& UV_joint,
    Eigen::VectorXi& local_vid_after_to_before_map);
} // namespace wmtk::operations::utils