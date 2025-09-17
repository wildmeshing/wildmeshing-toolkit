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

#include "track_operations.hpp" // For basic data structures

// Helper function to get all possible triangle IDs for a point (returns local triangle IDs) -
// templated
template <typename CoordType>
std::vector<int> get_possible_triangle_ids_t(
    const query_point_t<CoordType>& qp,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi);

// For backward compatibility
inline std::vector<int> get_possible_triangle_ids(
    const query_point& qp,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi)
{
    return get_possible_triangle_ids_t(qp, F_before, id_map_before, v_id_map_before, TT, TTi);
}

// Helper function to transform barycentric coordinates from one triangle representation to another
// - templated
template <typename CoordType>
std::pair<Eigen::Vector3<CoordType>, Eigen::Vector3i> transform_bc_to_triangle_t(
    const query_point_t<CoordType>& qp,
    int target_triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before);

// Backward compatibility version
std::pair<Eigen::Vector3d, Eigen::Vector3i> transform_bc_to_triangle(
    const query_point& qp,
    int target_triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before);

// Helper function to check if two points are in the same triangle and transform coordinates if
// needed - templated
template <typename CoordType>
struct SameTriangleResult_t
{
    bool in_same_triangle;
    int common_triangle_id;
    query_point_t<CoordType> transformed_qp1;
    query_point_t<CoordType> transformed_qp2;
};

template <typename CoordType>
SameTriangleResult_t<CoordType> check_and_transform_to_common_triangle_t(
    const query_point_t<CoordType>& qp1,
    const query_point_t<CoordType>& qp2,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi);

// Backward compatibility versions
struct SameTriangleResult
{
    bool in_same_triangle;
    int common_triangle_id;
    query_point transformed_qp1;
    query_point transformed_qp2;
};

SameTriangleResult check_and_transform_to_common_triangle(
    const query_point& qp1,
    const query_point& qp2,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi);

// Helper function to get candidate edges with their corresponding triangle indication - templated
struct EdgeTrianglePair
{
    std::pair<int, int> edge; // vertex pair (local vertex IDs)
    int triangle_id; // which triangle this edge indicates (local triangle ID)
};

template <typename CoordType>
std::vector<EdgeTrianglePair> get_candidate_edges_with_triangles_t(
    const query_point_t<CoordType>& qp,
    const std::vector<int>& possible_triangles,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi);

// Helper function to compute barycentric coordinates in triangle from edge barycentric coordinates
Eigen::Vector3<wmtk::Rational> edge_bc_to_triangle_bc(
    const Eigen::Vector2<wmtk::Rational>& edge_bc,
    const std::pair<int, int>& edge_vertices,
    int triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_before);

// Segment intersection function using rational arithmetic
bool intersectSegmentEdge_r(
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c,
    const Eigen::Vector2<wmtk::Rational>& d,
    Eigen::Vector2<wmtk::Rational>& barycentric_cd,
    Eigen::Vector2<wmtk::Rational>& barycentric_ab,
    bool debug_mode = false);

// Main segment handling function - templated
template <typename CoordType>
void handle_one_segment_t(
    query_curve_t<CoordType>& curve,
    int id,
    std::vector<query_point_t<CoordType>>& query_points,
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi,
    bool verbose);

// Backward compatibility version (double version)
void handle_one_segment(
    query_curve& curve,
    int id,
    std::vector<query_point>& query_points,
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi,
    bool verbose);

// Optimized version that takes pre-converted rational UV coordinates
void handle_one_segment_rational(
    query_curve& curve,
    int id,
    std::vector<query_point>& query_points,
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi,
    bool verbose);

// Curve handling functions for different operations - templated versions
template <typename CoordType>
void handle_collapse_edge_curves_t(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_curve_t<CoordType>>& curves,
    bool use_rational,
    bool verbose);

// Curve handling functions for different operations - templated versions
template <typename CoordType>
void handle_collapse_edge_curve_t(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve_t<CoordType>& curve,
    bool use_rational,
    bool verbose);

void handle_collapse_edge_curves_fast_rational(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    bool verbose);


template <typename CoordType>
void handle_non_collapse_operation_curves_t(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_curve_t<CoordType>>& curves,
    const std::string& operation_name,
    bool verbose);

template <typename CoordType>
void handle_non_collapse_operation_curve_t(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_curve_t<CoordType>& curve,
    const std::string& operation_name,
    bool verbose);


// Backward compatibility versions
template <typename CoordType>
void clean_up_curve_t(query_curve_t<CoordType>& curve);

template <typename CoordType>
bool is_curve_valid_t(const query_curve_t<CoordType>& curve);

template <typename CoordType>
int compute_intersections_between_two_curves_t(
    const query_curve_t<CoordType>& curve1,
    const query_curve_t<CoordType>& curve2,
    bool verbose = false);
int compute_intersections_between_two_curves(
    const query_curve& curve1,
    const query_curve& curve2,
    bool verbose = false);

template <typename CoordType>
int compute_curve_self_intersections_t(const query_curve_t<CoordType>& curve, bool verbose = true);
int compute_curve_self_intersections(const query_curve& curve, bool verbose = true);
