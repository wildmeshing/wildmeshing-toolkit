#pragma once

#include <filesystem>
#include "track_operations.hpp"
using path = std::filesystem::path;

#include <nlohmann/json.hpp>
using json = nlohmann::json;

/**
 * @brief Tracks a single operation on a set of query points.
 *
 * This function processes a single operation from the operation log and updates the query points
 * accordingly.
 *
 * @param operation_log A JSON object containing the details of the operation to be tracked.
 * @param query_points A vector of query_point objects that will be updated based on the operation.
 * @param do_forward A boolean flag indicating whether to perform the operation in the forward
 * direction. Default is false.
 * @param use_rational A boolean flag indicating whether to use rational arithmetic for the
 * operation. Default is false.
 */
void track_point_one_operation(
    const json& operation_log,
    std::vector<query_point>& query_points,
    bool do_forward = false,
    bool use_rational = false);

/**
 * @brief Tracks multiple operations on a set of query points from a directory.
 *
 * This function processes multiple operations from the files in the specified directory and updates
 * the query points accordingly.
 *
 * @param dirPath The path to the directory containing the operation logs.
 * @param query_points A vector of query_point objects that will be updated based on the operations.
 * @param do_forward A boolean flag indicating whether to perform the operations in the forward
 * direction. Default is false.
 * @param use_rational A boolean flag indicating whether to use rational arithmetic for the
 * operations. Default is false.
 */
void track_point(
    path dirPath,
    std::vector<query_point>& query_points,
    bool do_forward = false,
    bool use_rational = false);


// runnable demo applications with libigl viewer, which render points sampled on the center of the
// triangles
void back_track_point_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    bool use_rational = false);

void forward_track_point_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    bool use_rational = false);

// render a picture, color is based on the triangle index
void render_index_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int W = 1280,
    int H = 800);

// application that transfer picture
void transfer_texture_app(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::MatrixXi& F_in_obj,
    const Eigen::MatrixXd& Vt_in_obj,
    const Eigen::MatrixXi& Ft_in_obj,
    const Eigen::MatrixXd& Vt_out,
    const Eigen::MatrixXi& Ft_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int width_out,
    int height_out);