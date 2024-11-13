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
    Eigen::Vector3d bcs[2]; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

struct query_curve_tet
{
    std::vector<query_segment> segments;
    std::vector<int> next_segment_ids;
};

void handle_consolidate_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<query_point_tet>& query_points,
    bool forward = false);
{
    std::cout << "Handling Consolidate" << std::endl;
    if (!forward) {
        // backward
        igl::parallel_for(query_points.size(), [&](int id) {
            query_point_tet& qp = query_points[id];
            if (qp.t_id >= 0) {
                if (tet_ids_maps[qp.t_id] != qp.t_id) {
                    qp.t_id = tet_ids_maps[qp.t_id];
                }
                for (int i = 0; i < 4; i++) {
                    if (vertex_ids_maps[qp.tv_ids[i]] != qp.tv_ids[i]) {
                        qp.tv_ids[i] = vertex_ids_maps[qp.tv_ids[i]];
                    }
                }
            }
        });
    } else {
        // forward
        igl::parallel_for(query_points.size(), [&](int id) {
            query_point_tet& qp = query_points[id];
            if (qp.t_id >= 0) {
                auto it = std::find(tet_ids_maps.begin(), tet_ids_maps.end(), qp.t_id);
                if (it != tet_ids_maps.end()) {
                    qp.t_id = std::distance(tet_ids_maps.begin(), it);
                }
                for (int i = 0; i < 4; i++) {
                    auto it_v =
                        std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qp.tv_ids[i]);
                    if (it_v != vertex_ids_maps.end()) {
                        qp.tv_ids[i] = std::distance(vertex_ids_maps.begin(), it_v);
                    } else {
                        std::cout << "Error: vertex not found" << std::endl;
                    }
                }
            }
        });
    }
}

void handle_consolidat_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    query_curve_tet& curve,
    bool forward = false)
{
    std::cout << "Handling Consolidate forward for curve" << std::endl;
    if (!forward) {
        // backward
        // TODO: maybe use igl::parallel_for
        for (int id = 0; id < curve.segments.size(); id++) {
            auto& qs = curve.segments[id];
            if (qs.t_id >= 0) {
                qs.t_id = tet_ids_maps[qs.t_id];
            }
            for (int j = 0; j < 4; j++) {
                qs.tv_ids[j] = vertex_ids_maps[qs.tv_ids[j]];
            }
        }
    } else {
        // forward
        for (int id = 0; id < curve.segments.size(); id++) {
            auto& qs = curve.segments[id];
            if (qs.t_id >= 0) {
                auto it = std::find(tet_ids_maps.begin(), tet_ids_maps.end(), qs.t_id);
                if (it != tet_ids_maps.end()) {
                    qs.t_id = std::distance(tet_ids_maps.begin(), it);
                }
                for (int j = 0; j < 4; j++) {
                    auto it_v =
                        std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qs.tv_ids[j]);
                    if (it_v != vertex_ids_maps.end()) {
                        qs.tv_ids[j] = std::distance(vertex_ids_maps.begin(), it_v);
                    } else {
                        std::cout << "Error: vertex not found" << std::endl;
                    }
                }
            }
        }
    }
}

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
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points);

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
