#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

// igl
#include <igl/parallel_for.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <wmtk/utils/Rational.hpp>

struct query_point_tet
{
    int64_t t_id; // tet id
    Eigen::Vector4d bc; // barycentric coordinates
    Eigen::Vector4i tv_ids; // tet vertex ids
};

struct query_segment_tet
{
    int64_t t_id; // tet id
    Eigen::Vector4d bcs[2]; // barycentric coordinates
    Eigen::Vector4i tv_ids; // face vertex ids
};

struct query_curve_tet
{
    std::vector<query_segment_tet> segments;
    std::vector<int> next_segment_ids;
};

void handle_consolidate_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<query_point_tet>& query_points,
    bool forward = false);

void handle_consolidat_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    query_curve_tet& curve,
    bool forward = false);

// // Rational version of handle_collapse_edge_tet
// void handle_collapse_edge_tet_r(
//     const Eigen::MatrixXd& UV_joint,
//     const Eigen::MatrixXi& F_before,
//     const Eigen::MatrixXi& F_after,
//     const std::vector<int64_t>& v_id_map_joint,
//     const std::vector<int64_t>& id_map_before,
//     const std::vector<int64_t>& id_map_after,
//     std::vector<query_point>& query_points);

// void handle_collapse_edge_tet(
//     const Eigen::MatrixXd& UV_joint,
//     const Eigen::MatrixXi& F_before,
//     const Eigen::MatrixXi& F_after,
//     const std::vector<int64_t>& v_id_map_joint,
//     const std::vector<int64_t>& id_map_before,
//     const std::vector<int64_t>& id_map_after,
//     std::vector<query_point>& query_points,
//     bool use_rational = false);

// // curve version of the handle_collapse_edge
// void handle_collapse_edge_tet_curve(
//     const Eigen::MatrixXd& UV_joint,
//     const Eigen::MatrixXi& F_before,
//     const Eigen::MatrixXi& F_after,
//     const std::vector<int64_t>& v_id_map_joint,
//     const std::vector<int64_t>& id_map_before,
//     const std::vector<int64_t>& id_map_after,
//     query_curve& curve,
//     bool use_rational = false);

// split/swap/smooth operations
void handle_local_mapping_tet(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& T_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& T_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_tet>& query_points);

void parse_consolidate_file_tet(
    const json& operation_log,
    std::vector<int64_t>& tet_ids_maps,
    std::vector<int64_t>& vertex_ids_maps);

void parse_non_collapse_file_tet(
    const json& operation_log,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& T_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& T_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after);

// void parse_edge_collapse_file_tet(
//     const json& operation_log,
//     Eigen::MatrixXd& UV_joint,
//     Eigen::MatrixXi& F_before,
//     Eigen::MatrixXi& F_after,
//     std::vector<int64_t>& v_id_map_joint,
//     std::vector<int64_t>& id_map_before,
//     std::vector<int64_t>& id_map_after);
