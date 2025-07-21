#pragma once
#include <filesystem>
#include "track_operations.hpp"
using path = std::filesystem::path;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief Tracks multiple operations on a query curve from a directory.
 * This function processes multiple operations from the files in the specified directory and updates
 * the query curve accordingly.
 *
 * @param dirPath the path to the directory containing the operation logs
 * @param curve the query curve to be tracked
 * @param do_forward a boolean flag indicating whether to perform the operations in the forward
 * direction. Default is false.
 */
void track_line(path dirPath, query_curve& curve, bool do_forward = false);

// extension of the above function for multiple curves
void track_lines(
    path dirPath,
    std::vector<query_curve>& curves,
    bool do_forward = false,
    bool do_parallel = true);


// demo application on back tracking one curve
void back_track_one_curve_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir);

// demo application on forward tracking uv-isolines
void forward_track_iso_lines_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& uv_in,
    const Eigen::MatrixXi& Fuv_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int N = 5);

// check the result of iso-lines
void check_iso_lines(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const std::vector<query_curve>& curves_in,
    const std::vector<query_curve>& curves_out,
    bool render_before = true,
    bool render_after = true);

// only for debug, check iso-lines step by step
#ifdef DEBUG_CURVES
void check_iso_lines_step_by_step(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir);
#endif