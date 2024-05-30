#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

// igl
#include <igl/parallel_for.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <igl/triangle_triangle_adjacency.h>

#include <wmtk/utils/Rational.hpp>

struct query_point
{
    int64_t f_id; // face id
    Eigen::Vector3d bc; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

struct query_segment
{
    int64_t f_id; // face id
    Eigen::Vector3d bcs[2]; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

struct query_curve
{
    std::vector<query_segment> segments;
    std::vector<int> next_segment_ids;
};

// Rational version of the above
struct query_point_r
{
    int64_t f_id; // face id
    Eigen::Vector3<wmtk::Rational> bc; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

struct query_segment_r
{
    int64_t f_id; // face id
    Eigen::Vector3<wmtk::Rational> bcs[2]; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

struct query_curve_r
{
    std::vector<query_segment_r> segments;
    std::vector<int> next_segment_ids;
};

template <typename qp_type>
void handle_consolidate(
    const std::vector<int64_t>& face_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<qp_type>& query_points)
{
    std::cout << "Handling Consolidate" << std::endl;
    igl::parallel_for(query_points.size(), [&](int id) {
        qp_type& qp = query_points[id];
        if (qp.f_id >= 0) {
            if (face_ids_maps[qp.f_id] != qp.f_id) {
                qp.f_id = face_ids_maps[qp.f_id];
            }
            for (int i = 0; i < 3; i++) {
                if (vertex_ids_maps[qp.fv_ids[i]] != qp.fv_ids[i]) {
                    qp.fv_ids[i] = vertex_ids_maps[qp.fv_ids[i]];
                }
            }
        }
    });
}

template <typename qc_type>
void handle_consolidate(
    const std::vector<int64_t>& face_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    qc_type& curve)
{
    std::cout << "Handling Consolidate" << std::endl;
    for (int id = 0; id < curve.segments.size(); id++) {
        auto& qs = curve.segments[id];
        if (qs.f_id >= 0) {
            if (face_ids_maps[qs.f_id] != qs.f_id) {
                qs.f_id = face_ids_maps[qs.f_id];
            }
            for (int j = 0; j < 3; j++) {
                if (vertex_ids_maps[qs.fv_ids[j]] != qs.fv_ids[j]) {
                    qs.fv_ids[j] = vertex_ids_maps[qs.fv_ids[j]];
                }
            }
        }
    }
}

template <typename qp_type>
void handle_consolidate_forward(
    const std::vector<int64_t>& face_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<qp_type>& query_points)
{
    std::cout << "Handling Consolidate forward" << std::endl;

    igl::parallel_for(query_points.size(), [&](int id) {
        qp_type& qp = query_points[id];
        if (qp.f_id >= 0) {
            auto it = std::find(face_ids_maps.begin(), face_ids_maps.end(), qp.f_id);
            if (it != face_ids_maps.end()) {
                qp.f_id = std::distance(face_ids_maps.begin(), it);
            }
            for (int i = 0; i < 3; i++) {
                auto it_v = std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qp.fv_ids[i]);
                if (it_v != vertex_ids_maps.end()) {
                    qp.fv_ids[i] = std::distance(vertex_ids_maps.begin(), it_v);
                }
            }
        }
    });
}

// TODO: Rational Version of this code
Eigen::Vector3d ComputeBarycentricCoordinates3D(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double eps = 1e-3);

Eigen::Vector3d ComputeBarycentricCoordinates2D(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c);

// Rational version of ComputeBarycentricCoordinates2D
Eigen::Vector3<wmtk::Rational> ComputeBarycentricCoordinates2D_r(
    const Eigen::Vector2<wmtk::Rational>& p,
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c);

void handle_collapse_edge(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points);

// Rational version of handle_collapse_edge
void handle_collapse_edge_r(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_r>& query_points);

bool intersectSegmentEdge(
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    const Eigen::Vector2d& d,
    Eigen::Vector2d& barycentric);

bool intersectSegmentEdge_r(
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c,
    const Eigen::Vector2<wmtk::Rational>& d,
    Eigen::Vector2<wmtk::Rational>& barycentric);

// curve version of the handle_collapse_edge
void handle_collapse_edge(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve& curve);

void handle_collapse_edge_r(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve_r& curve);

void handle_split_edge(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points);

void handle_swap_edge(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points);

void parse_consolidate_file(
    const json& operation_log,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps);

void parse_edge_split_file(
    const json& operation_log,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& F_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after);

void parse_edge_collapse_file(
    const json& operation_log,
    Eigen::MatrixXd& UV_joint,
    Eigen::MatrixXi& F_before,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& v_id_map_joint,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& id_map_after);
