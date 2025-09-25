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

template <typename CoordType = double>
struct query_point_t
{
    int64_t f_id; // face id
    Eigen::Vector3<CoordType> bc; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

// Type aliases for convenience
using query_point = query_point_t<double>;
using query_point_r = query_point_t<wmtk::Rational>;

template <typename CoordType = double>
struct query_segment_t
{
    int64_t f_id; // face id
    int64_t origin_segment_id; // original segment id when the segment was first created as input
    Eigen::Vector3<CoordType> bcs[2]; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

// Type aliases for convenience
using query_segment = query_segment_t<double>;
using query_segment_r = query_segment_t<wmtk::Rational>;

template <typename CoordType = double>
struct query_curve_t
{
    std::vector<query_segment_t<CoordType>> segments;
    std::vector<int> next_segment_ids;
    int prev_segment_id(int seg_id)
    {
        auto it = std::find(next_segment_ids.begin(), next_segment_ids.end(), seg_id);
        if (it == next_segment_ids.end()) {
            return -1;
        }
        return std::distance(next_segment_ids.begin(), it);
    }
};

// Type aliases for convenience
using query_curve = query_curve_t<double>;
using query_curve_r = query_curve_t<wmtk::Rational>;

// Helper functions for type conversion between double and wmtk::Rational
template <typename FromType, typename ToType>
query_point_t<ToType> convert_query_point(const query_point_t<FromType>& from)
{
    query_point_t<ToType> to;
    to.f_id = from.f_id;
    to.fv_ids = from.fv_ids;
    for (int i = 0; i < 3; ++i) {
        if constexpr (std::is_same_v<FromType, wmtk::Rational> && std::is_same_v<ToType, double>) {
            to.bc[i] = from.bc[i].to_double();
        } else if constexpr (
            std::is_same_v<FromType, double> && std::is_same_v<ToType, wmtk::Rational>) {
            to.bc[i] = wmtk::Rational(from.bc[i]);
        } else {
            to.bc[i] = ToType(from.bc[i]);
        }
    }
    // to.bc /= to.bc.sum();
    if (std::is_same_v<ToType, wmtk::Rational>) {
        to.bc /= to.bc.sum();
    }
    return to;
}

template <typename FromType, typename ToType>
query_segment_t<ToType> convert_query_segment(const query_segment_t<FromType>& from)
{
    query_segment_t<ToType> to;
    to.f_id = from.f_id;
    to.origin_segment_id = from.origin_segment_id;
    to.fv_ids = from.fv_ids;
    for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < 3; ++i) {
            if constexpr (
                std::is_same_v<FromType, wmtk::Rational> && std::is_same_v<ToType, double>) {
                to.bcs[j][i] = from.bcs[j][i].to_double();
            } else if constexpr (
                std::is_same_v<FromType, double> && std::is_same_v<ToType, wmtk::Rational>) {
                to.bcs[j][i] = wmtk::Rational(from.bcs[j][i]);
            } else {
                to.bcs[j][i] = ToType(from.bcs[j][i]);
            }
        }
    }
    to.bcs[0] /= to.bcs[0].sum();
    to.bcs[1] /= to.bcs[1].sum();
    return to;
}

template <typename FromType, typename ToType>
query_curve_t<ToType> convert_query_curve(const query_curve_t<FromType>& from)
{
    query_curve_t<ToType> to;
    to.segments.reserve(from.segments.size());
    for (const auto& seg : from.segments) {
        to.segments.push_back(convert_query_segment<FromType, ToType>(seg));
    }
    to.next_segment_ids = from.next_segment_ids;
    return to;
}

// Helper function for printing with to_double conversion
template <typename CoordType>
std::ostream& operator<<(std::ostream& os, const query_point_t<CoordType>& qp)
{
    if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        os << "query_point(f_id=" << qp.f_id << ", bc=(" << qp.bc[0].to_double() << ","
           << qp.bc[1].to_double() << "," << qp.bc[2].to_double() << "), fv_ids=(" << qp.fv_ids[0]
           << "," << qp.fv_ids[1] << "," << qp.fv_ids[2] << "))";
    } else {
        os << "query_point(f_id=" << qp.f_id << ", bc=(" << qp.bc[0] << "," << qp.bc[1] << ","
           << qp.bc[2] << "), fv_ids=(" << qp.fv_ids[0] << "," << qp.fv_ids[1] << ","
           << qp.fv_ids[2] << "))";
    }
    return os;
}

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
// Precompute cache for fast 2D barycentric evaluation per triangle
struct BarycentricPrecompute2D
{
    Eigen::Matrix<wmtk::Rational, 2, 2> Minv;
    Eigen::Matrix<wmtk::Rational, 2, 1> a;
};

// Get barycentric coordinates from cache
Eigen::Matrix<wmtk::Rational, 3, 1> barycentric_from_cache(
    const BarycentricPrecompute2D& bc,
    const Eigen::Matrix<wmtk::Rational, 2, 1>& p);

// Build cache from double UV/vertex array (internally converts to Rational)
std::vector<BarycentricPrecompute2D> build_barycentric_cache_2d(
    const Eigen::MatrixX<wmtk::Rational>& V2,
    const Eigen::MatrixXi& F);

std::vector<BarycentricPrecompute2D> build_barycentric_cache_2d_from_double(
    const Eigen::MatrixXd& V2,
    const Eigen::MatrixXi& F);

// IO - templated versions
template <typename CoordType>
void save_query_curves_t(
    const std::vector<query_curve_t<CoordType>>& curves,
    const std::string& filename);

template <typename CoordType>
std::vector<query_curve_t<CoordType>> load_query_curves_t(const std::string& filename);

// IO - backward compatibility functions
void save_query_curves(const std::vector<query_curve>& curves, const std::string& filename);
std::vector<query_curve> load_query_curves(const std::string& filename);

// IO - intersection reference functions
void save_intersection_reference(
    const std::vector<std::vector<int>>& intersection_reference,
    const std::string& filename);
std::vector<std::vector<int>> load_intersection_reference(const std::string& filename);


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

template <typename qc_type>
void handle_consolidate_forward(
    const std::vector<int64_t>& face_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    qc_type& curve)
{
    std::cout << "Handling Consolidate forward" << std::endl;
    for (int id = 0; id < curve.segments.size(); id++) {
        auto& qs = curve.segments[id];
        if (qs.f_id >= 0) {
            auto it = std::find(face_ids_maps.begin(), face_ids_maps.end(), qs.f_id);
            if (it != face_ids_maps.end()) {
                qs.f_id = std::distance(face_ids_maps.begin(), it);
            }
            for (int j = 0; j < 3; j++) {
                auto it_v = std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qs.fv_ids[j]);
                if (it_v != vertex_ids_maps.end()) {
                    qs.fv_ids[j] = std::distance(vertex_ids_maps.begin(), it_v);
                }
            }
        }
    }
}


void handle_collapse_edge(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points,
    bool use_rational = false);

// Rational version of handle_collapse_edge (uses rational internally but input/output are double)
void handle_collapse_edge_r(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache = nullptr);

void handle_collapse_edge_use_exact_predicate(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points);

// Templated version for native support of different coordinate types
template <typename CoordType>
void handle_collapse_edge_t(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<CoordType>>& query_points,
    bool use_rational = false,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache = nullptr);

// All rational version of handle_collapse_edge_t
void handle_collapse_edge_rational(
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);
// Unified function for split/swap/smooth operations - they all have the same interface
void handle_non_collapse_operation(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points,
    const std::string& operation_name = "non-collapse operation",
    bool use_rational = false);

// Rational version of handle_non_collapse_operation (uses rational internally but input/output are double)
void handle_non_collapse_operation_r(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points,
    const std::string& operation_name,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache = nullptr);

// Templated version for native support of different coordinate types
template <typename CoordType>
void handle_non_collapse_operation_t(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<CoordType>>& query_points,
    const std::string& operation_name = "non-collapse operation",
    bool use_rational = false,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache = nullptr);


void handle_non_collapse_operation_rational(
    const Eigen::MatrixX<wmtk::Rational>& V_before_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixX<wmtk::Rational>& V_after_r,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache = nullptr);

/***
 * Parse the operation log file
 */

void parse_consolidate_file(
    const json& operation_log,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps);

void parse_non_collapse_file(
    const json& operation_log,
    bool& is_skipped,
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


// Note: handle_one_segment and other curve-related functions have been moved to
// track_operations_curve.hpp
