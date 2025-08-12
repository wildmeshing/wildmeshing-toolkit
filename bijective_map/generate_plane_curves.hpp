#pragma once

#include <Eigen/Dense>
#include <vector>
#include <map>
#include "track_operations_curve.hpp"

using namespace Eigen;

// Structure to represent directed edge information
struct EdgeInfo
{
    int v0, v1;     // Global vertex IDs (v0 -> v1 defines direction)
    int fl, fr;     // Left and right triangle IDs (-1 for boundary)
};

// Structure for intersection point
struct PlaneIntersection
{
    int triangle_id;       // Triangle where intersection occurs
    Vector3d barycentric;  // Barycentric coordinates
    int edge_v0, edge_v1;  // Edge vertices for connectivity
    int connected_intersection_id; // Index of connected intersection across edge (-1 if none)
};

/**
 * @brief Extract all directed edges from mesh with left/right triangle info
 * 
 * @param F Face matrix (m x 3)
 * @return Map from directed edge (v0,v1) to edge info with fl/fr
 */
std::map<std::pair<int,int>, EdgeInfo> extractMeshEdges(const MatrixXi& F);

/**
 * @brief Compute intersections between mesh edges and a plane
 * 
 * @param V Vertex matrix (n x 3)
 * @param F Face matrix (m x 3)
 * @param edges Preprocessed edge information
 * @param axis_value Plane position value
 * @param axis_index Which axis (0=x, 1=y, 2=z)
 * @return Vector of intersection points
 */
std::vector<PlaneIntersection> computePlaneIntersections(
    const MatrixXd& V,
    const MatrixXi& F,
    const std::map<std::pair<int,int>, EdgeInfo>& edges,
    double axis_value,
    int axis_index);

/**
 * @brief Build curves from intersection points using edge connectivity
 * 
 * @param intersections Vector of intersection points
 * @param edges Edge information for connectivity
 * @return Vector of curves, each curve is a vector of intersection indices
 */
std::vector<std::vector<int>> buildCurvesFromIntersections(
    const std::vector<PlaneIntersection>& intersections,
    const std::map<std::pair<int,int>, EdgeInfo>& edges);

/**
 * @brief Convert intersection curves to query_curves
 * 
 * @param intersections Vector of all intersection points
 * @param curves Vector of curves (indices into intersections)
 * @param V Vertex matrix
 * @param F Face matrix
 * @return Vector of query_curves
 */
std::vector<query_curve> convertToQueryCurves(
    const std::vector<PlaneIntersection>& intersections,
    const std::vector<std::vector<int>>& curves,
    const MatrixXd& V,
    const MatrixXi& F);

/**
 * @brief Directly build query_curves from plane intersections
 * Groups intersections by triangle and creates segments within each triangle
 * 
 * @param intersections Vector of all intersection points 
 * @param V Vertex matrix
 * @param F Face matrix
 * @return Vector of query_curves
 */
std::vector<query_curve> buildQueryCurvesFromIntersections(
    const std::vector<PlaneIntersection>& intersections,
    const MatrixXd& V,
    const MatrixXi& F);

/**
 * @brief Generate curves for all axes with N values each
 * 
 * @param V Vertex matrix (n x 3)
 * @param F Face matrix (m x 3)
 * @param N Number of values per axis
 * @return Vector of query_curves
 */
std::vector<query_curve> generatePlaneCurves(
    const MatrixXd& V,
    const MatrixXi& F,
    int N);