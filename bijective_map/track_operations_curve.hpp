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

// Helper function to get all possible triangle IDs for a point (returns local triangle IDs)
std::vector<int> get_possible_triangle_ids(
    const query_point& qp,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi);

// Helper function to transform barycentric coordinates from one triangle representation to another
std::pair<Eigen::Vector3d, Eigen::Vector3i> transform_bc_to_triangle(
    const query_point& qp,
    int target_triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before);

// Helper function to check if two points are in the same triangle and transform coordinates if
// needed
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

// Helper function to get candidate edges with their corresponding triangle indication
struct EdgeTrianglePair
{
    std::pair<int, int> edge; // vertex pair (local vertex IDs)
    int triangle_id; // which triangle this edge indicates (local triangle ID)
};

std::vector<EdgeTrianglePair> get_candidate_edges_with_triangles(
    const query_point& qp,
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
    Eigen::Vector2<wmtk::Rational>& barycentric,
    bool debug_mode = false);

// Main segment handling function
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

// Old version for comparison
void handle_one_segment_old(
    query_curve& curve,
    int id,
    std::vector<query_point>& query_points,
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi);

// Curve handling functions for different operations
void handle_collapse_edge_curve(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve& curve,
    bool use_rational,
    bool verbose);

void handle_non_collapse_operation_curve(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_curve& curve,
    const std::string& operation_name,
    bool verbose);


void clean_up_curve(query_curve& curve);