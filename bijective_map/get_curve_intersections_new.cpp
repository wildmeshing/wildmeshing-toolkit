#include "track_operations_curve.hpp"
#include <map>
#include <set>
#include <iomanip>

////////////////////////////////////////////////////////////
// CurveIntersectionPoint operator implementations
////////////////////////////////////////////////////////////

bool CurveIntersectionPoint::operator<(const CurveIntersectionPoint& other) const
{
    double rate = double(seg_order_id) + t;
    double other_rate = double(other.seg_order_id) + other.t;

    if (rate != other_rate)
        return rate < other_rate;
    else
        return other_curve_id < other.other_curve_id;
}

bool CurveIntersectionPoint::operator==(const CurveIntersectionPoint& other) const
{
    return seg_order_id == other.seg_order_id && t == t;
}

std::ostream& operator<<(std::ostream& os, const CurveIntersectionPoint& pt)
{
    os << "CurveIntersectionPoint{";
    os << "curve_id=" << pt.other_curve_id;
    os << ", type=";
    switch (pt.type) {
        case IntersectionType::POINT:
            os << "POINT";
            break;
        case IntersectionType::SEGMENT_LEFT:
            os << "SEGMENT_LEFT";
            break;
        case IntersectionType::SEGMENT_RIGHT:
            os << "SEGMENT_RIGHT";
            break;
    }
    os << ", seg_order=" << pt.seg_order_id;
    os << ", t=" << std::fixed << std::setprecision(6) << pt.t;
    os << "}";
    return os;
}

////////////////////////////////////////////////////////////
// Helper functions (local to this implementation)
////////////////////////////////////////////////////////////

// Check if a segment lies on a mesh edge and get the edge vertex IDs
template <typename CoordType>
bool is_segment_on_mesh_edge(
    const query_segment_t<CoordType>& seg,
    std::pair<int64_t, int64_t>& edge_vertices)
{
    const CoordType zero(0);

    // Find common zero positions between the two barycentric coordinates
    // If both endpoints share at least one zero position, the segment lies on an edge
    int common_zero_index = -1;
    for (int i = 0; i < 3; i++) {
        if (seg.bcs[0][i] == zero && seg.bcs[1][i] == zero) {
            common_zero_index = i;
            break;
        }
    }

    if (common_zero_index == -1) {
        return false; // No common zero, segment is not on an edge
    }

    // The segment lies on the edge formed by the other two vertices
    std::vector<int64_t> edge_vids;
    for (int i = 0; i < 3; i++) {
        if (i != common_zero_index) {
            edge_vids.push_back(seg.fv_ids[i]);
        }
    }

    // Order the edge vertices for consistent comparison
    if (edge_vids[0] < edge_vids[1]) {
        edge_vertices = {edge_vids[0], edge_vids[1]};
    } else {
        edge_vertices = {edge_vids[1], edge_vids[0]};
    }

    return true;
}

// CHECKED
// Helper function to get all edges that a point lies on
// Returns edges as pairs of vertex IDs
template <typename CoordType>
std::vector<std::pair<int64_t, int64_t>> get_edges_containing_point(
    const Eigen::Vector3<CoordType>& bc,
    const Eigen::Vector3i& fv_ids)
{
    const CoordType zero(0);
    std::vector<std::pair<int64_t, int64_t>> edges;

    // For each zero barycentric coordinate, the point lies on the edge
    // formed by the other two vertices
    for (int zero_idx = 0; zero_idx < 3; zero_idx++) {
        if (bc[zero_idx] == zero) {
            // Get the other two vertex indices
            std::vector<int64_t> edge_vids;
            for (int i = 0; i < 3; i++) {
                if (i != zero_idx) {
                    edge_vids.push_back(fv_ids[i]);
                }
            }

            // Order vertices consistently
            if (edge_vids[0] < edge_vids[1]) {
                edges.push_back({edge_vids[0], edge_vids[1]});
            } else {
                edges.push_back({edge_vids[1], edge_vids[0]});
            }
        }
    }

    return edges;
}

// CHECKED
// Build spatial cache: map from face_id to segments, and from edge to segments
template <typename CoordType>
void build_segment_spatial_cache(
    const query_curve_t<CoordType>& curve,
    std::map<int64_t, std::vector<int>>& face_to_segments,
    std::map<std::pair<int64_t, int64_t>, std::vector<int>>& edge_to_segments)
{
    face_to_segments.clear();
    edge_to_segments.clear();

    for (int seg_id = 0; seg_id < curve.segments.size(); seg_id++) {
        const auto& seg = curve.segments[seg_id];

        // Always cache by face_id
        face_to_segments[seg.f_id].push_back(seg_id);

        // Get all edges containing the first endpoint and cache
        auto edges_0 = get_edges_containing_point(seg.bcs[0], seg.fv_ids);
        for (const auto& edge : edges_0) {
            edge_to_segments[edge].push_back(seg_id);
        }

        // Get all edges containing the second endpoint and cache
        auto edges_1 = get_edges_containing_point(seg.bcs[1], seg.fv_ids);
        for (const auto& edge : edges_1) {
            edge_to_segments[edge].push_back(seg_id);
        }
    }
}

// CHECKED
// Get candidate segments from curve2 that could intersect with seg_on_curve1
template <typename CoordType>
std::pair<bool, std::vector<int>> get_candidate_segments(
    const query_segment_t<CoordType>& seg_on_curve1,
    const std::map<int64_t, std::vector<int>>& face_to_segments,
    const std::map<std::pair<int64_t, int64_t>, std::vector<int>>& edge_to_segments)
{
    std::set<int> candidates; // Use set to avoid duplicates

    // Check if seg_on_curve1 is on an edge
    std::pair<int64_t, int64_t> edge_vertices;
    bool is_on_edge = is_segment_on_mesh_edge(seg_on_curve1, edge_vertices);
    if (is_on_edge) {
        // seg_on_curve1 is on edge - get all segments on the same edge
        auto it = edge_to_segments.find(edge_vertices);
        if (it != edge_to_segments.end()) {
            for (int seg_id : it->second) {
                candidates.insert(seg_id);
            }
        }
    } else {
        // seg_on_curve1 is interior to a face - get all segments in the same face
        auto it = face_to_segments.find(seg_on_curve1.f_id);
        if (it != face_to_segments.end()) {
            for (int seg_id : it->second) {
                candidates.insert(seg_id);
            }
        }
    }

    return {is_on_edge, std::vector<int>(candidates.begin(), candidates.end())};
}


// Helper: Check if two points are the same based on bc and vertex IDs
template <typename CoordType>
bool are_points_equal(
    const Eigen::Vector3<CoordType>& bc1,
    const Eigen::Vector3i& fv_ids1,
    const Eigen::Vector3<CoordType>& bc2,
    const Eigen::Vector3i& fv_ids2)
{
    CoordType zero(0);

    // For each nonzero component in bc1, find corresponding vertex in bc2
    for (int i = 0; i < 3; i++) {
        if (bc1[i] == zero) continue;

        int64_t vid1 = fv_ids1[i];

        // Find this vertex in fv_ids2
        int idx2 = -1;
        for (int j = 0; j < 3; j++) {
            if (fv_ids2[j] == vid1) {
                idx2 = j;
                break;
            }
        }

        if (idx2 == -1) {
            return false; // Vertex not found in seg2
        }

        if (bc2[idx2] != bc1[i]) {
            return false; // Different barycentric coordinates
        }
    }

    return true;
}


// Unified function to check both point intersection and segment overlap
// This function handles all cases:
// 1. Different faces: Check if both on same edge (1D overlap), then check endpoint intersection
// 2. Same face: Use 2D barycentric coordinates for general intersection/overlap
template <typename CoordType>
SegmentIntersectionResult check_segment_intersection_and_overlap(
    const query_segment_t<CoordType>& seg1,
    const query_segment_t<CoordType>& seg2)
{
    SegmentIntersectionResult result;
    result.type = SegmentRelationType::NO_INTERSECTION;

    // Case 1: Segments in different faces
    if (seg1.f_id != seg2.f_id) {
        // First, check if both segments lie on mesh edges
        std::pair<int64_t, int64_t> edge1, edge2;
        bool seg1_on_edge = is_segment_on_mesh_edge(seg1, edge1);
        bool seg2_on_edge = is_segment_on_mesh_edge(seg2, edge2);

        // If both segments are on the same mesh edge, check 1D overlap
        // This handles the special case where segments cross face boundaries via a shared edge
        if (seg1_on_edge && seg2_on_edge && edge1 == edge2) {
            std::vector<int64_t> edge_vids = {edge1.first, edge1.second};

            // Extract barycentric coordinates at the first edge vertex for both segments
            wmtk::Rational s1_t0(0), s1_t1(0), s2_t0(0), s2_t1(0);

            for (int i = 0; i < 3; i++) {
                if (seg1.fv_ids[i] == edge_vids[0]) {
                    s1_t0 = wmtk::Rational(seg1.bcs[0][i]);
                    s1_t1 = wmtk::Rational(seg1.bcs[1][i]);
                }
            }

            for (int i = 0; i < 3; i++) {
                if (seg2.fv_ids[i] == edge_vids[0]) {
                    s2_t0 = wmtk::Rational(seg2.bcs[0][i]);
                    s2_t1 = wmtk::Rational(seg2.bcs[1][i]);
                }
            }

            // Map seg2 onto seg1's parameter space [0,1]
            wmtk::Rational s1_range = s1_t1 - s1_t0;
            if (s1_range != wmtk::Rational(0)) {
                wmtk::Rational t2a = (s2_t0 - s1_t0) / s1_range;
                wmtk::Rational t2b = (s2_t1 - s1_t0) / s1_range;

                if (t2a > t2b) std::swap(t2a, t2b);

                auto zero_r = wmtk::Rational(0);
                auto one_r = wmtk::Rational(1);
                auto overlap_start = std::max(zero_r, t2a);
                auto overlap_end = std::min(one_r, t2b);

                if (overlap_start < overlap_end) {
                    // Segments overlap on the edge
                    result.type = SegmentRelationType::SEGMENT_OVERLAP;
                    result.t_start = overlap_start;
                    result.t_end = overlap_end;
                    return result;
                } else if (overlap_start == overlap_end) {
                    // Segments touch at a single point
                    result.type = SegmentRelationType::POINT_INTERSECTION;
                    result.t_start = overlap_start;
                    if (t2b != t2a) {
                        result.u_start = (overlap_start - t2a) / (t2b - t2a);
                    } else {
                        result.u_start = wmtk::Rational(0);
                    }
                    return result;
                }
            }

            return result; // On same edge but no overlap
        }

        // Not on same edge - check if any endpoints coincide
        if (are_points_equal(seg1.bcs[0], seg1.fv_ids, seg2.bcs[0], seg2.fv_ids)) {
            result.type = SegmentRelationType::POINT_INTERSECTION;
            result.t_start = wmtk::Rational(0);
            result.u_start = wmtk::Rational(0);
            return result;
        }
        if (are_points_equal(seg1.bcs[0], seg1.fv_ids, seg2.bcs[1], seg2.fv_ids)) {
            result.type = SegmentRelationType::POINT_INTERSECTION;
            result.t_start = wmtk::Rational(0);
            result.u_start = wmtk::Rational(1);
            return result;
        }
        if (are_points_equal(seg1.bcs[1], seg1.fv_ids, seg2.bcs[0], seg2.fv_ids)) {
            result.type = SegmentRelationType::POINT_INTERSECTION;
            result.t_start = wmtk::Rational(1);
            result.u_start = wmtk::Rational(0);
            return result;
        }
        if (are_points_equal(seg1.bcs[1], seg1.fv_ids, seg2.bcs[1], seg2.fv_ids)) {
            result.type = SegmentRelationType::POINT_INTERSECTION;
            result.t_start = wmtk::Rational(1);
            result.u_start = wmtk::Rational(1);
            return result;
        }

        return result; // Different faces, no common edge, no common endpoints
    }

    // Case 2: Segments in the same face - use 2D barycentric intersection
    // Convert to 2D by using first two barycentric coordinates
    Eigen::Vector2<wmtk::Rational> s1_a(
        wmtk::Rational(seg1.bcs[0](0)),
        wmtk::Rational(seg1.bcs[0](1)));
    Eigen::Vector2<wmtk::Rational> s1_b(
        wmtk::Rational(seg1.bcs[1](0)),
        wmtk::Rational(seg1.bcs[1](1)));
    Eigen::Vector2<wmtk::Rational> s2_a(
        wmtk::Rational(seg2.bcs[0](0)),
        wmtk::Rational(seg2.bcs[0](1)));
    Eigen::Vector2<wmtk::Rational> s2_b(
        wmtk::Rational(seg2.bcs[1](0)),
        wmtk::Rational(seg2.bcs[1](1)));

    auto s1_dir = s1_b - s1_a;
    auto s2_dir = s2_b - s2_a;

    // Cross product to check parallelism
    auto cross = s1_dir(0) * s2_dir(1) - s1_dir(1) * s2_dir(0);

    if (cross == wmtk::Rational(0)) {
        // Segments are parallel - check collinearity
        auto s1_to_s2a = s2_a - s1_a;
        auto cross_collinear = s1_dir(0) * s1_to_s2a(1) - s1_dir(1) * s1_to_s2a(0);

        if (cross_collinear != wmtk::Rational(0)) {
            return result; // Parallel but not collinear - no intersection
        }

        // Collinear segments - check overlap
        auto s1_len_sq = s1_dir.squaredNorm();
        if (s1_len_sq == wmtk::Rational(0)) {
            return result; // Degenerate seg1
        }

        // Project seg2 endpoints onto seg1
        auto s1_to_s2b = s2_b - s1_a;
        auto t2a = s1_dir.dot(s1_to_s2a) / s1_len_sq;
        auto t2b = s1_dir.dot(s1_to_s2b) / s1_len_sq;

        if (t2a > t2b) {
            std::swap(t2a, t2b);
        }

        auto zero = wmtk::Rational(0);
        auto one = wmtk::Rational(1);

        auto overlap_start = std::max(zero, t2a);
        auto overlap_end = std::min(one, t2b);

        if (overlap_start >= overlap_end) {
            return result; // No overlap
        }

        result.type = SegmentRelationType::SEGMENT_OVERLAP;
        result.t_start = overlap_start;
        result.t_end = overlap_end;
        return result;
    }

    // Non-parallel segments - compute intersection using line-line intersection formula
    auto dx_diff = s2_a(0) - s1_a(0);
    auto dy_diff = s2_a(1) - s1_a(1);

    auto t = (dx_diff * s2_dir(1) - dy_diff * s2_dir(0)) / cross;
    auto u = (dx_diff * s1_dir(1) - dy_diff * s1_dir(0)) / cross;

    auto zero = wmtk::Rational(0);
    auto one = wmtk::Rational(1);

    // Check if intersection point lies within both segments
    if (t >= zero && t <= one && u >= zero && u <= one) {
        result.type = SegmentRelationType::POINT_INTERSECTION;
        result.t_start = t;
        result.u_start = u;
    }

    return result;
}


////////////////////////////////////////////////////////////
// Main function
////////////////////////////////////////////////////////////

// This version does not depend on TT, TTi
template <typename CoordType>
std::vector<CurveIntersectionPoint> compute_intersections_between_two_curve_new_t(
    const query_curve_t<CoordType>& curve1,
    const query_curve_t<CoordType>& curve2,
    int curve2_id,
    bool verbose)
{
    std::vector<CurveIntersectionPoint> intersections;

    // Step 1: Build spatial cache for curve2
    std::map<int64_t, std::vector<int>> face_to_segments;
    std::map<std::pair<int64_t, int64_t>, std::vector<int>> edge_to_segments;
    build_segment_spatial_cache(curve2, face_to_segments, edge_to_segments);

    if (verbose) {
        std::cout << "Built spatial cache for curve2 (id=" << curve2_id << "):" << std::endl;
        std::cout << "  Face cache size: " << face_to_segments.size() << std::endl;
        std::cout << "  Edge cache size: " << edge_to_segments.size() << std::endl;
    }

    // Step 2: Iterate through curve1 segments following the chain
    int current_seg1_id = 0;
    int seg1_order = 0;
    while (current_seg1_id != -1 && seg1_order < curve1.segments.size()) {
        const auto& seg1 = curve1.segments[current_seg1_id];

        auto [is_seg1_on_edge, candidate_seg2_ids] =
            get_candidate_segments(seg1, face_to_segments, edge_to_segments);

        if (verbose) {
            std::cout << "Segment order " << seg1_order << " (seg_id=" << current_seg1_id
                      << ") has " << candidate_seg2_ids.size() << " candidates" << std::endl;
            if (is_seg1_on_edge) {
                std::cout << "  Segment is on edge" << std::endl;
            } else {
                std::cout << "  Segment is on face" << std::endl;
            }
        }

        for (int seg2_id : candidate_seg2_ids) {
            const auto& seg2 = curve2.segments[seg2_id];

            auto intersection_result = check_segment_intersection_and_overlap(seg1, seg2);

            if (intersection_result.type == SegmentRelationType::SEGMENT_OVERLAP) {
                // add two end points
                CurveIntersectionPoint left_pt;
                left_pt.other_curve_id = curve2_id;
                left_pt.type = IntersectionType::SEGMENT_LEFT;
                left_pt.seg_order_id = seg1_order;
                left_pt.t = intersection_result.t_start.to_double();
                intersections.push_back(left_pt);

                CurveIntersectionPoint right_pt;
                right_pt.other_curve_id = curve2_id;
                right_pt.type = IntersectionType::SEGMENT_RIGHT;
                right_pt.seg_order_id = seg1_order;
                right_pt.t = intersection_result.t_end.to_double();
                intersections.push_back(right_pt);

                if (verbose) {
                    std::cout << "Segment overlap found:" << std::endl;
                    std::cout << "  curve1 seg_order=" << seg1_order
                              << " (seg_id=" << current_seg1_id << ")" << std::endl;
                    std::cout << "  curve2 seg_id=" << seg2_id << std::endl;
                    std::cout << "  overlap t=[" << intersection_result.t_start.to_double() << ", "
                              << intersection_result.t_end.to_double() << "]" << std::endl;
                }
            } else if (intersection_result.type == SegmentRelationType::POINT_INTERSECTION) {
                // add point intersection
                CurveIntersectionPoint pt;
                pt.other_curve_id = curve2_id;
                pt.type = IntersectionType::POINT;
                pt.seg_order_id = seg1_order;
                pt.t = intersection_result.t_start.to_double();
                intersections.push_back(pt);

                if (verbose) {
                    std::cout << "Point intersection found:" << std::endl;
                    std::cout << "  curve1 seg_order=" << seg1_order
                              << " (seg_id=" << current_seg1_id << ") at t=" << pt.t << std::endl;
                    std::cout << "  curve2 seg_id=" << seg2_id
                              << " at u=" << intersection_result.u_start.to_double()
                              << std::endl; // which is not important
                }
            }
        }


        // Move to next segment in curve1
        current_seg1_id = curve1.next_segment_ids[current_seg1_id];
        seg1_order++;

        // Check if we've returned to the start (closed curve)
        if (current_seg1_id == 0) {
            break;
        }
    }


    if (verbose) {
        std::cout << "\nTotal intersections found: " << intersections.size() << std::endl;
    }

    std::sort(intersections.begin(), intersections.end());
    // TODO: merge redundant points and segments
    if (verbose) {
        std::cout << "Sorted intersections:\n";
        for (const auto& intersection : intersections) {
            std::cout << "  " << intersection << '\n';
        }
    }


    return intersections;
}
