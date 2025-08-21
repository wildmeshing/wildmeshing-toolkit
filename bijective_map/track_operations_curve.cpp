#include "track_operations_curve.hpp"
#include <igl/Timer.h>
#include <igl/doublearea.h>
#ifdef USE_IGL_VIEWER
#include <igl/opengl/glfw/Viewer.h>
#endif

// Define log10 for wmtk::Rational to avoid Eigen NumTraits issues
namespace std {
inline double log10(const wmtk::Rational& r)
{
    return std::log10(r.to_double());
}
} // namespace std

// Helper function to get all possible triangle IDs for a point - templated version
template <typename CoordType>
std::vector<int> get_possible_triangle_ids_t(
    const query_point_t<CoordType>& qp,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi)
{
    std::vector<int> possible_triangle_ids;

    // Find the local face index
    auto it = std::find(id_map_before.begin(), id_map_before.end(), qp.f_id);
    if (it == id_map_before.end()) {
        return possible_triangle_ids; // Return empty if not found
    }
    int local_face_id = std::distance(id_map_before.begin(), it);

    // Always include the current triangle (return local ID)
    possible_triangle_ids.push_back(local_face_id);

    // Count how many coordinates are zero
    int zero_count = 0;
    int zero_indices[3];
    int zero_idx = 0;
    for (int i = 0; i < 3; i++) {
        bool is_zero;
        if constexpr (std::is_same_v<CoordType, double>) {
            is_zero = (qp.bc(i) == 0.0);
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            is_zero = (qp.bc(i) == wmtk::Rational(0));
        }
        if (is_zero) {
            zero_count++;
            zero_indices[zero_idx++] = i;
        }
    }

    if (zero_count == 2) {
        // Point is on vertex (two coordinates are 0)
        int vertex_idx = -1;
        for (int i = 0; i < 3; i++) {
            bool is_nonzero;
            if constexpr (std::is_same_v<CoordType, double>) {
                is_nonzero = (qp.bc(i) != 0.0);
            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                is_nonzero = (qp.bc(i) != wmtk::Rational(0));
            }
            if (is_nonzero) {
                vertex_idx = i;
                break;
            }
        }
        // Find all triangles containing this vertex (return local IDs)
        int global_vertex = qp.fv_ids[vertex_idx];
        for (int face_id = 0; face_id < F_before.rows(); face_id++) {
            if (face_id == local_face_id) continue; // Already added
            for (int local_v = 0; local_v < 3; local_v++) {
                if (v_id_map_before[F_before(face_id, local_v)] == global_vertex) {
                    possible_triangle_ids.push_back(face_id);
                    break;
                }
            }
        }
    } else if (zero_count == 1) {
        // Point is on edge (one coordinate is 0) - return local ID
        int edge_idx = (zero_indices[0] + 1) % 3;
        int adj_face = TT(local_face_id, edge_idx);
        if (adj_face != -1) {
            possible_triangle_ids.push_back(adj_face);
        }
    }
    // zero_count == 0 means interior point, only current triangle

    return possible_triangle_ids;
}

// Helper function to transform barycentric coordinates from one triangle representation to another
// - templated version
template <typename CoordType>
std::pair<Eigen::Vector3<CoordType>, Eigen::Vector3i> transform_bc_to_triangle_t(
    const query_point_t<CoordType>& qp,
    int target_triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before)
{
    Eigen::Vector3<CoordType> new_bc;
    if constexpr (std::is_same_v<CoordType, double>) {
        new_bc = Eigen::Vector3<CoordType>(0.0, 0.0, 0.0);
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        new_bc = Eigen::Vector3<CoordType>(wmtk::Rational(0), wmtk::Rational(0), wmtk::Rational(0));
    }
    Eigen::Vector3i new_fv_ids;

    // Get the vertex IDs of the target triangle
    for (int i = 0; i < 3; i++) {
        new_fv_ids[i] = v_id_map_before[F_before(target_triangle_id, i)];
    }

    // Find mapping from original vertices to new triangle vertices
    for (int i = 0; i < 3; i++) {
        bool is_nonzero;
        if constexpr (std::is_same_v<CoordType, double>) {
            is_nonzero = (qp.bc(i) != 0.0);
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            is_nonzero = (qp.bc(i) != wmtk::Rational(0));
        }
        if (is_nonzero) {
            int global_vertex = qp.fv_ids[i];
            // Find where this vertex appears in the target triangle
            for (int j = 0; j < 3; j++) {
                if (new_fv_ids[j] == global_vertex) {
                    new_bc(j) = qp.bc(i);
                    break;
                }
            }
        }
    }

    return std::make_pair(new_bc, new_fv_ids);
}
// Helper function to transform barycentric coordinates from one triangle representation to another
std::pair<Eigen::Vector3d, Eigen::Vector3i> transform_bc_to_triangle(
    const query_point& qp,
    int target_triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before)
{
    return transform_bc_to_triangle_t(
        qp,
        target_triangle_id,
        F_before,
        id_map_before,
        v_id_map_before);
}

// Helper function to check if two points are in the same triangle and transform coordinates if
// needed - templated version
template <typename CoordType>
SameTriangleResult_t<CoordType> check_and_transform_to_common_triangle_t(
    const query_point_t<CoordType>& qp1,
    const query_point_t<CoordType>& qp2,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi)
{
    SameTriangleResult_t<CoordType> result;
    result.in_same_triangle = false;

    auto triangles1 =
        get_possible_triangle_ids_t(qp1, F_before, id_map_before, v_id_map_before, TT, TTi);
    auto triangles2 =
        get_possible_triangle_ids_t(qp2, F_before, id_map_before, v_id_map_before, TT, TTi);

    // Find common triangle
    for (int t1 : triangles1) {
        for (int t2 : triangles2) {
            if (t1 == t2) {
                result.in_same_triangle = true;
                result.common_triangle_id = t1;

                // Transform both points to this common triangle representation
                auto transform1 =
                    transform_bc_to_triangle_t(qp1, t1, F_before, id_map_before, v_id_map_before);
                auto transform2 =
                    transform_bc_to_triangle_t(qp2, t1, F_before, id_map_before, v_id_map_before);

                result.transformed_qp1.f_id = id_map_before[t1];
                result.transformed_qp1.bc = transform1.first;
                result.transformed_qp1.fv_ids = transform1.second;

                result.transformed_qp2.f_id = id_map_before[t1];
                result.transformed_qp2.bc = transform2.first;
                result.transformed_qp2.fv_ids = transform2.second;

                return result;
            }
        }
    }

    return result;
}
// Helper function to check if two points are in the same triangle and transform coordinates if
// needed
SameTriangleResult check_and_transform_to_common_triangle(
    const query_point& qp1,
    const query_point& qp2,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi)
{
    auto result_t = check_and_transform_to_common_triangle_t(
        qp1,
        qp2,
        F_before,
        id_map_before,
        v_id_map_before,
        TT,
        TTi);

    SameTriangleResult result;
    result.in_same_triangle = result_t.in_same_triangle;
    result.common_triangle_id = result_t.common_triangle_id;
    result.transformed_qp1 = result_t.transformed_qp1;
    result.transformed_qp2 = result_t.transformed_qp2;

    return result;
}

// Helper function to get candidate edges with their corresponding triangle indication - templated
// version
template <typename CoordType>
std::vector<EdgeTrianglePair> get_candidate_edges_with_triangles_t(
    const query_point_t<CoordType>& qp,
    const std::vector<int>& possible_triangles,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi)
{
    std::vector<EdgeTrianglePair> candidate_edges;

    // Count how many coordinates are zero to determine point location
    int zero_count = 0;
    int zero_idx = -1;
    for (int i = 0; i < 3; i++) {
        bool is_zero;
        if constexpr (std::is_same_v<CoordType, double>) {
            is_zero = (qp.bc(i) == 0.0);
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            is_zero = (qp.bc(i) == wmtk::Rational(0));
        }
        if (is_zero) {
            zero_count++;
            zero_idx = i;
        }
    }

    if (zero_count == 0) {
        // Interior point: all three edges of current triangle indicate current triangle
        int current_triangle = possible_triangles[0];
        for (int i = 0; i < 3; i++) {
            int v0 = F_before(current_triangle, i); // Use local vertex ID
            int v1 = F_before(current_triangle, (i + 1) % 3); // Use local vertex ID
            candidate_edges.push_back({{v0, v1}, current_triangle});
        }
    } else if (zero_count == 1) {
        // Point on edge: each candidate edge indicates a different triangle
        // Current triangle edges (except the edge we're on)
        // If zero_idx = i, point is on edge ((i+1)%3, (i+2)%3), so exclude that edge
        int current_triangle = possible_triangles[0];
        int edge_on = (zero_idx + 1) % 3; // The edge we're on starts from vertex (zero_idx+1)%3
        for (int i = 0; i < 3; i++) {
            if (i != edge_on) { // Exclude the edge we're currently on
                int v0 = F_before(current_triangle, i); // Use local vertex ID
                int v1 = F_before(current_triangle, (i + 1) % 3); // Use local vertex ID
                candidate_edges.push_back({{v0, v1}, current_triangle});
            }
        }

        // Adjacent triangle edges (if exists)
        if (possible_triangles.size() > 1) {
            int adj_triangle = possible_triangles[1];
            int adj_edge = TTi(current_triangle, edge_on);
            for (int i = 0; i < 3; i++) {
                if (i != adj_edge) {
                    int v0 = F_before(adj_triangle, i); // Use local vertex ID
                    int v1 = F_before(adj_triangle, (i + 1) % 3); // Use local vertex ID
                    candidate_edges.push_back({{v0, v1}, adj_triangle});
                }
            }
        }
    } else if (zero_count == 2) {
        // Point on vertex: each edge from different triangles indicates that triangle
        for (int triangle_id : possible_triangles) {
            // Find the local vertex index in this triangle
            int local_vertex_idx = -1;
            int vertex_idx = -1;
            for (int i = 0; i < 3; i++) {
                bool is_nonzero;
                if constexpr (std::is_same_v<CoordType, double>) {
                    is_nonzero = (qp.bc(i) != 0.0);
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    is_nonzero = (qp.bc(i) != wmtk::Rational(0));
                }
                if (is_nonzero) {
                    vertex_idx = i;
                    break;
                }
            }
            int global_vertex = qp.fv_ids[vertex_idx];

            for (int local_v = 0; local_v < 3; local_v++) {
                if (v_id_map_before[F_before(triangle_id, local_v)] == global_vertex) {
                    local_vertex_idx = local_v;
                    break;
                }
            }

            if (local_vertex_idx != -1) {
                // Add the edge opposite to this vertex (use local vertex IDs)
                int v0 = F_before(triangle_id, (local_vertex_idx + 1) % 3); // Use local vertex ID
                int v1 = F_before(triangle_id, (local_vertex_idx + 2) % 3); // Use local vertex ID
                candidate_edges.push_back({{v0, v1}, triangle_id});
            }
        }
    }

    return candidate_edges;
}
// Helper function to get candidate edges with their corresponding triangle indication
std::vector<EdgeTrianglePair> get_candidate_edges_with_triangles(
    const query_point& qp,
    const std::vector<int>& possible_triangles,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXi& TT,
    const Eigen::MatrixXi& TTi)
{
    return get_candidate_edges_with_triangles_t(
        qp,
        possible_triangles,
        F_before,
        v_id_map_before,
        TT,
        TTi);
}

// Helper function to compute barycentric coordinates in triangle from edge barycentric coordinates
Eigen::Vector3<wmtk::Rational> edge_bc_to_triangle_bc(
    const Eigen::Vector2<wmtk::Rational>& edge_bc,
    const std::pair<int, int>& edge_vertices,
    int triangle_id,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_before)
{
    Eigen::Vector3<wmtk::Rational> triangle_bc(0, 0, 0);

    // Find which vertices of the triangle correspond to the edge vertices (now using local IDs)
    int edge_v0_local = -1, edge_v1_local = -1;
    for (int i = 0; i < 3; i++) {
        if (F_before(triangle_id, i) == edge_vertices.first) {
            edge_v0_local = i;
        } else if (F_before(triangle_id, i) == edge_vertices.second) {
            edge_v1_local = i;
        }
    }

    if (edge_v0_local != -1 && edge_v1_local != -1) {
        // Set the barycentric coordinates: edge_bc(0) goes to first vertex, edge_bc(1) goes to
        // second vertex
        triangle_bc(edge_v0_local) = edge_bc(0);
        triangle_bc(edge_v1_local) = edge_bc(1);
        // Third coordinate is 0 (point is on the edge)
    }

    return triangle_bc;
}

// Segment intersection function using rational arithmetic
bool intersectSegmentEdge_r(
    const Eigen::Vector2<wmtk::Rational>& a,
    const Eigen::Vector2<wmtk::Rational>& b,
    const Eigen::Vector2<wmtk::Rational>& c,
    const Eigen::Vector2<wmtk::Rational>& d,
    Eigen::Vector2<wmtk::Rational>& barycentric,
    bool debug_mode)
{
    Eigen::Vector2<wmtk::Rational> ab = b - a;
    Eigen::Vector2<wmtk::Rational> cd = d - c;
    Eigen::Vector2<wmtk::Rational> ac = c - a;

    wmtk::Rational denominator = (ab.x() * cd.y() - ab.y() * cd.x());
    if (denominator == wmtk::Rational(0)) {
        if (debug_mode) std::cout << "parallel" << std::endl;
        return false; // parallel
    }
    wmtk::Rational t = (ac.x() * cd.y() - ac.y() * cd.x()) / denominator;
    wmtk::Rational u = -(ab.x() * ac.y() - ab.y() * ac.x()) / denominator;
    if (debug_mode) std::cout << "t: " << t.to_double() << ", u: " << u.to_double() << std::endl;

    if (t >= 0 && t <= wmtk::Rational(1) && u >= 0 && u <= wmtk::Rational(1)) {
        // Calculate barycentric coordinates for intersection
        wmtk::Rational alpha = wmtk::Rational(1) - u;
        wmtk::Rational beta = u;
        barycentric = Eigen::Vector2<wmtk::Rational>(alpha, beta);
        if (debug_mode) std::cout << "intersection found" << std::endl;
        return true;
    }
    if (debug_mode) std::cout << "no intersection" << std::endl;
    return false;
}

// Main segment handling function - templated version
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
    bool verbose)
{
    // Profiling: total time of handling one segment
    igl::Timer seg_total_timer;
    seg_total_timer.start();


    query_segment_t<CoordType>& qs = curve.segments[id];
    // Check if two endpoints are in the same triangle considering all possible locations
    // Profiling: adjacency build time (only when needed)
    igl::Timer tt_build_timer;
    if (TT.rows() == 0) {
        tt_build_timer.start();
        igl::triangle_triangle_adjacency(F_before, TT, TTi);
    }

    // Profiling: initial same-triangle check
    igl::Timer same_tri_timer_initial;
    same_tri_timer_initial.start();
    auto same_triangle_result = check_and_transform_to_common_triangle_t(
        query_points[0],
        query_points[1],
        F_before,
        id_map_before,
        v_id_map_joint,
        TT,
        TTi);

    if (same_triangle_result.in_same_triangle) {
        // Two endpoints are on the same face --> no need to compute intersections
        qs.f_id = same_triangle_result.transformed_qp1.f_id;
        qs.bcs[0] = same_triangle_result.transformed_qp1.bc;
        qs.bcs[1] = same_triangle_result.transformed_qp2.bc;
        qs.fv_ids = same_triangle_result.transformed_qp1.fv_ids;
        if (false && verbose) {
            double t_total_ms = seg_total_timer.getElapsedTime() * 1000.0;
            double t_tt_ms = tt_build_timer.getElapsedTime() * 1000.0;
            double t_same_tri_ms = same_tri_timer_initial.getElapsedTime() * 1000.0;
            std::cout << "[handle_one_segment_rational] early-exit timings: total=" << t_total_ms
                      << " ms, TT/TTi=" << t_tt_ms << " ms, same-triangle-check=" << t_same_tri_ms
                      << " ms" << std::endl;
        }
        return;
    }

    // Get starting point and target point
    auto it_start = std::find(id_map_before.begin(), id_map_before.end(), query_points[0].f_id);
    auto it_end = std::find(id_map_before.begin(), id_map_before.end(), query_points[1].f_id);

    if (it_start == id_map_before.end() || it_end == id_map_before.end()) {
        std::cerr << "Error: Cannot find start or end face in id_map_before" << std::endl;
        throw std::runtime_error("Error: Cannot find start or end face in id_map_before");
        return;
    }

    int start_face_local = std::distance(id_map_before.begin(), it_start);
    int end_face_local = std::distance(id_map_before.begin(), it_end);

    // Compute start point a and end point b in UV space using rational arithmetic
    Eigen::Vector2<wmtk::Rational> a(0, 0), b(0, 0);
    Eigen::Vector3<wmtk::Rational> bc_start_r, bc_end_r;

    // Convert barycentric coordinates to rational and normalize
    if constexpr (std::is_same_v<CoordType, double>) {
        bc_start_r << wmtk::Rational(query_points[0].bc(0)), wmtk::Rational(query_points[0].bc(1)),
            wmtk::Rational(query_points[0].bc(2));
        bc_end_r << wmtk::Rational(query_points[1].bc(0)), wmtk::Rational(query_points[1].bc(1)),
            wmtk::Rational(query_points[1].bc(2));
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        bc_start_r << query_points[0].bc(0), query_points[0].bc(1), query_points[0].bc(2);
        bc_end_r << query_points[1].bc(0), query_points[1].bc(1), query_points[1].bc(2);
    }
    bc_start_r /= bc_start_r.sum();
    bc_end_r /= bc_end_r.sum();

    // Compute world coordinates
    for (int i = 0; i < 3; i++) {
        a += UV_joint_r.row(F_before(start_face_local, i)).transpose() * bc_start_r(i);
        b += UV_joint_r.row(F_before(end_face_local, i)).transpose() * bc_end_r(i);
    }

    // quiet

    // Cache for the last new segment
    int old_next_seg = curve.next_segment_ids[id];

    // Initialize current position and face
    Eigen::Vector2<wmtk::Rational> current_pos = a;
    int current_face = start_face_local;
    Eigen::Vector3<wmtk::Rational> current_bc = bc_start_r;

    // Set up first segment
    qs.f_id = query_points[0].f_id;
    qs.bcs[0] = query_points[0].bc;
    qs.fv_ids = query_points[0].fv_ids;

    std::vector<query_segment_t<CoordType>> new_segments;

    // Ray tracing loop
    int iteration = 0;
    double time_intersection_tests_total = 0; // detailed per-iteration intersection tests (verbose)
    double time_candidate_edges_total = 0; // always-on: time to build candidate edges
    double time_same_tri_checks_total = 0; // always-on: per-iteration same-triangle checks
    while (current_face != end_face_local) {
        iteration++;
        // quiet

        // Create query_point for current position
        query_point_t<CoordType> current_qp;
        current_qp.f_id = id_map_before[current_face];
        if constexpr (std::is_same_v<CoordType, double>) {
            current_qp.bc = Eigen::Vector3d(
                current_bc(0).to_double(),
                current_bc(1).to_double(),
                current_bc(2).to_double());
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            current_qp.bc = current_bc;
        }
        current_qp.fv_ids << v_id_map_joint[F_before(current_face, 0)],
            v_id_map_joint[F_before(current_face, 1)], v_id_map_joint[F_before(current_face, 2)];

        // Debug: Get possible triangles for both points
        auto current_possible_triangles = get_possible_triangle_ids_t(
            current_qp,
            F_before,
            id_map_before,
            v_id_map_joint,
            TT,
            TTi);
        auto target_possible_triangles = get_possible_triangle_ids_t(
            query_points[1],
            F_before,
            id_map_before,
            v_id_map_joint,
            TT,
            TTi);

        // quiet

        igl::Timer same_tri_timer_iter;
        same_tri_timer_iter.start();
        auto same_triangle_result = check_and_transform_to_common_triangle_t(
            current_qp,
            query_points[1],
            F_before,
            id_map_before,
            v_id_map_joint,
            TT,
            TTi);
        time_same_tri_checks_total += same_tri_timer_iter.getElapsedTime();

        // quiet

        if (same_triangle_result.in_same_triangle) {
            // Current point and target point are in the same triangle, finish directly
            query_segment_t<CoordType> final_seg;
            final_seg.f_id = same_triangle_result.transformed_qp1.f_id;
            final_seg.origin_segment_id = qs.origin_segment_id; // Preserve the original face ID
            final_seg.bcs[0] = same_triangle_result.transformed_qp1.bc;
            final_seg.bcs[1] = same_triangle_result.transformed_qp2.bc;
            final_seg.fv_ids = same_triangle_result.transformed_qp1.fv_ids;
            new_segments.push_back(final_seg);
            break; // Exit the while loop
        }

        // Get candidate edges with their corresponding triangles
        igl::Timer cand_edges_timer;
        cand_edges_timer.start();
        auto candidate_edges = get_candidate_edges_with_triangles_t(
            current_qp,
            current_possible_triangles,
            F_before,
            v_id_map_joint,
            TT,
            TTi);
        time_candidate_edges_total += cand_edges_timer.getElapsedTime();

        // Find next intersection using rational arithmetic with detailed profiling
        bool found_intersection = false;
        Eigen::Vector2<wmtk::Rational> next_intersection;
        Eigen::Vector2<wmtk::Rational> next_bc_edge;
        int next_face = -1;
        int selected_edge_idx = -1;

        int num_candidate_edges = candidate_edges.size();
        double time_edge_vertex_lookup = 0;
        double time_intersection_computation = 0;
        igl::Timer intersection_timer;
        intersection_timer.start();

        for (int i = 0; i < candidate_edges.size(); i++) {
            const auto& edge_tri_pair = candidate_edges[i];
            const auto& edge = edge_tri_pair.edge;
            int edge_triangle = edge_tri_pair.triangle_id;

            // Time vertex lookup (only if verbose)
            igl::Timer lookup_timer;
            if (verbose) lookup_timer.start();
            Eigen::Vector2<wmtk::Rational> edge_start = UV_joint_r.row(edge.first).transpose();
            Eigen::Vector2<wmtk::Rational> edge_end = UV_joint_r.row(edge.second).transpose();
            if (verbose) time_edge_vertex_lookup += lookup_timer.getElapsedTime();

            // Time intersection computation (only if verbose)
            igl::Timer compute_timer;
            if (verbose) compute_timer.start();
            Eigen::Vector2<wmtk::Rational> edge_bc;
            bool has_intersection =
                intersectSegmentEdge_r(current_pos, b, edge_start, edge_end, edge_bc, false);
            if (verbose) time_intersection_computation += compute_timer.getElapsedTime();

            // quiet
            if (has_intersection) {
                // quiet

                selected_edge_idx = i;
                next_intersection = edge_bc(0) * edge_start + edge_bc(1) * edge_end;
                next_bc_edge = edge_bc;
                next_face = edge_triangle; // Use the triangle indicated by this edge

                found_intersection = true;
                // quiet
                break;
            } else {
                // std::cout << "no intersection" << std::endl;
            }
        }

        double total_intersection_time = intersection_timer.getElapsedTime();
        time_intersection_tests_total += total_intersection_time;

        if (!found_intersection) {
            std::cerr << "Error: No intersection found in ray tracing" << std::endl;
            throw std::runtime_error("Error: No intersection found in ray tracing");
            break;
        }

        // Create new segment for the intersection
        query_segment_t<CoordType> new_seg;
        new_seg.f_id = id_map_before[next_face]; // Use the indicated triangle
        new_seg.origin_segment_id =
            qs.origin_segment_id; // Preserve the original segment ID from the parent segment
        new_seg.bcs[0] = transform_bc_to_triangle_t(
                             current_qp,
                             next_face,
                             F_before,
                             id_map_before,
                             v_id_map_joint)
                             .first;
        // new_seg.bcs[0] = Eigen::Vector3d(
        //     current_bc(0).to_double(),
        //     current_bc(1).to_double(),
        //     current_bc(2).to_double());

        // Compute barycentric coordinates for intersection point in the indicated triangle
        // using edge barycentric coordinates (now with local vertex IDs)

        Eigen::Vector3<wmtk::Rational> intersection_bc = edge_bc_to_triangle_bc(
            next_bc_edge,
            candidate_edges[selected_edge_idx].edge,
            next_face,
            F_before,
            v_id_map_joint);

        if constexpr (std::is_same_v<CoordType, double>) {
            new_seg.bcs[1] = Eigen::Vector3d(
                intersection_bc(0).to_double(),
                intersection_bc(1).to_double(),
                intersection_bc(2).to_double());
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            new_seg.bcs[1] = intersection_bc;
        }


        new_seg.fv_ids << v_id_map_joint[F_before(next_face, 0)],
            v_id_map_joint[F_before(next_face, 1)], v_id_map_joint[F_before(next_face, 2)];

        new_segments.push_back(new_seg);

        // quiet

        // Move to next face and update position
        current_pos = next_intersection;
        current_face = next_face;
        // Update current_bc to match the new segment's end coordinates
        if constexpr (std::is_same_v<CoordType, double>) {
            current_bc << wmtk::Rational(new_seg.bcs[1](0)), wmtk::Rational(new_seg.bcs[1](1)),
                wmtk::Rational(new_seg.bcs[1](2));
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            current_bc = new_seg.bcs[1];
        }

        // Print detailed intersection profiling for this iteration (only if verbose)
        // quiet
    }

    // Add final segment to target (only if we didn't exit early from the while loop)
    if (!new_segments.empty() && current_face == end_face_local) {
        throw std::runtime_error("Error: No intersection found in ray tracing");
        // query_segment final_seg;
        // final_seg.f_id = query_points[1].f_id;
        // final_seg.bcs[0] = new_segments.back().bcs[1]; // Start from last intersection
        // final_seg.bcs[1] = query_points[1].bc; // End at target point
        // final_seg.fv_ids = query_points[1].fv_ids;
        // new_segments.push_back(final_seg);
    }

    // Update curve with new segments
    if (!new_segments.empty()) {
        // qs.bcs[1] = new_segments[0].bcs[1]; // First segment ends at first intersection
        qs = new_segments[0];
        curve.next_segment_ids[id] = curve.segments.size(); // Point to first new segment

        // Add all new segments
        for (size_t i = 1; i < new_segments.size(); i++) {
            curve.segments.push_back(new_segments[i]);
            if (i < new_segments.size() - 1) {
                curve.next_segment_ids.push_back(
                    curve.segments.size()); // Point to next new segment
            } else {
                curve.next_segment_ids.push_back(
                    old_next_seg); // Last segment points to original next
            }
        }
    } else {
        throw std::runtime_error("Error: No intersections found in ray tracing, why?");
        // No intersections needed, direct connection
        // qs.bcs[1] = query_points[1].bc;
    }

    if (verbose) {
        std::cout << "New segments created in ray tracing:" << std::endl;
        for (size_t i = 0; i < new_segments.size(); ++i) {
            const auto& seg = new_segments[i];
            std::cout << "  Segment " << i << ": f_id=" << seg.f_id << ", start_bc=("
                      << seg.bcs[0](0) << ", " << seg.bcs[0](1) << ", " << seg.bcs[0](2) << ")"
                      << ", end_bc=(" << seg.bcs[1](0) << ", " << seg.bcs[1](1) << ", "
                      << seg.bcs[1](2) << ")" << ", fv_ids=(" << seg.fv_ids(0) << ", "
                      << seg.fv_ids(1) << ", " << seg.fv_ids(2) << ")" << std::endl;
        }
    }
    if (verbose && false) {
#ifdef USE_IGL_VIEWER
        igl::opengl::glfw::Viewer viewer;
        // Convert rational UV back to double for visualization
        Eigen::MatrixXd UV_joint_double(UV_joint_r.rows(), UV_joint_r.cols());
        for (int i = 0; i < UV_joint_r.rows(); i++) {
            for (int j = 0; j < UV_joint_r.cols(); j++) {
                UV_joint_double(i, j) = UV_joint_r(i, j).to_double();
            }
        }
        viewer.data().set_mesh(UV_joint_double, F_before);
        for (const auto& seg : new_segments) {
            Eigen::MatrixXd pts(2, 3);
            for (int i = 0; i < 2; i++) {
                Eigen::Vector3d p(0, 0, 0);
                for (int j = 0; j < 3; j++) {
                    int local_vid =
                        std::find(v_id_map_joint.begin(), v_id_map_joint.end(), seg.fv_ids(j)) -
                        v_id_map_joint.begin();
                    if constexpr (std::is_same_v<CoordType, double>) {
                        p += UV_joint_double.row(local_vid).transpose() * seg.bcs[i](j);
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        p += UV_joint_double.row(local_vid).transpose() * seg.bcs[i](j).to_double();
                    }
                }
                pts.row(i) = p;
            }
            viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
            viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
            viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
        }
        viewer.launch();
#endif
    }
    // Print profiling summary (only if verbose)
    if (false) {
        double t_total_ms = seg_total_timer.getElapsedTime() * 1000.0;
        double t_tt_ms = tt_build_timer.getElapsedTime() * 1000.0;
        double t_same_tri_init_ms = same_tri_timer_initial.getElapsedTime() * 1000.0;
        std::cout << "\n=== handle_one_segment_rational Timing ===" << std::endl;
        std::cout << "  Total: " << t_total_ms << " ms" << std::endl;
        std::cout << "  TT/TTi build: " << t_tt_ms << " ms" << std::endl;
        std::cout << "  Initial same-triangle check: " << t_same_tri_init_ms << " ms" << std::endl;
        std::cout << "  Ray-trace iterations: " << iteration << std::endl;
        std::cout << "    Candidate-edges total: " << time_candidate_edges_total * 1000.0 << " ms"
                  << std::endl;
        std::cout << "    Same-triangle checks total: " << time_same_tri_checks_total * 1000.0
                  << " ms" << std::endl;
        std::cout << "    Intersection tests total: " << time_intersection_tests_total * 1000.0
                  << " ms" << std::endl;
        if (iteration > 0) {
            std::cout << "    Intersection tests avg/iter: "
                      << (time_intersection_tests_total / iteration) * 1000.0 << " ms" << std::endl;
        }
    }
}


// Curve handling functions for different operations - template version
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
    bool verbose)
{
    // Profiling accumulators
    igl::Timer total_timer;
    total_timer.start();
    igl::Timer area_timer;
    igl::Timer convert_timer;
    double time_area_total = 0.0;
    double time_convert_rational = 0.0;
    double time_map_points_total = 0.0;
    double time_trace_segment_total = 0.0;


    Eigen::VectorXd dblarea_before, dblarea_after;
    area_timer.start();
    igl::doublearea(UV_joint, F_before, dblarea_before);
    if (dblarea_before.minCoeff() <= 0) {
        std::cout << "Error: dblarea_before is negative" << std::endl;
        throw std::runtime_error("Error: dblarea_before is negative");
    }
    igl::doublearea(UV_joint, F_after, dblarea_after);
    time_area_total += area_timer.getElapsedTime();
    if (dblarea_after.minCoeff() <= 0) {
        std::cout << "Error: dblarea_after is negative" << std::endl;
        throw std::runtime_error("Error: dblarea_after is negative");
    }

    int curve_length = curve.segments.size();
    Eigen::MatrixXi TT, TTi;

    // Convert UV_joint to rational once for all segments (optimization)
    Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
    convert_timer.start();
    for (int i = 0; i < UV_joint.rows(); i++) {
        for (int j = 0; j < UV_joint.cols(); j++) {
            UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
        }
    }
    time_convert_rational += convert_timer.getElapsedTime();
    int cnter = 0;
    for (int id = 0; id < curve_length; id++) {
        ////////////////////////////////////
        // map the two end points of one segment
        ////////////////////////////////////
        query_segment_t<CoordType>& qs = curve.segments[id];
        query_point_t<CoordType> qp0 = {qs.f_id, qs.bcs[0], qs.fv_ids};
        query_point_t<CoordType> qp1 = {qs.f_id, qs.bcs[1], qs.fv_ids};
        std::vector<query_point_t<CoordType>> query_points = {qp0, qp1};

        std::vector<query_point_t<CoordType>> query_points_copy = query_points;


        auto it = std::find(id_map_after.begin(), id_map_after.end(), qs.f_id);
        if (it == id_map_after.end()) {
            continue;
        }

        cnter++;

        // Map points timing
        igl::Timer map_timer;
        map_timer.start();
        // Build cache once per operation at curve level if desired
        std::vector<BarycentricPrecompute2D> bc_cache_collapse =
            build_barycentric_cache_2d_from_double(UV_joint, F_before);

        handle_collapse_edge_t(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            query_points,
            use_rational,
            &bc_cache_collapse);

        time_map_points_total += map_timer.getElapsedTime() * 1000;
        if constexpr (std::is_same_v<CoordType, double>) {
            if ((query_points[0].bc - query_points[1].bc).norm() == 0) {
                std::cout << "Query points for segment " << id << ":" << std::endl;
                for (size_t i = 0; i < query_points.size(); i++) {
                    std::cout << "  Point " << i << ": f_id=" << query_points[i].f_id << ", bc=("
                              << query_points[i].bc[0] << ", " << query_points[i].bc[1] << ", "
                              << query_points[i].bc[2] << ")" << std::endl;
                }
                // Print query_points_copy for debugging
                std::cout << "Query points before collapse mapping for segment " << id << ":"
                          << std::endl;
                for (size_t i = 0; i < query_points_copy.size(); i++) {
                    std::cout << "  Point " << i << ": f_id=" << query_points_copy[i].f_id
                              << ", bc=(" << query_points_copy[i].bc[0] << ", "
                              << query_points_copy[i].bc[1] << ", " << query_points_copy[i].bc[2]
                              << ")" << std::endl;
                }
            }
        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
            Eigen::Vector3d bc_diff = Eigen::Vector3d(
                (query_points[0].bc - query_points[1].bc)(0).to_double(),
                (query_points[0].bc - query_points[1].bc)(1).to_double(),
                (query_points[0].bc - query_points[1].bc)(2).to_double());
            if (bc_diff.norm() == 0) {
                std::cout << "Query points for segment " << id << ":" << std::endl;
                for (size_t i = 0; i < query_points.size(); i++) {
                    std::cout << "  Point " << i << ": f_id=" << query_points[i].f_id << ", bc=("
                              << query_points[i].bc[0].to_double() << ", "
                              << query_points[i].bc[1].to_double() << ", "
                              << query_points[i].bc[2].to_double() << ")" << std::endl;
                }
                // Print query_points_copy for debugging
                std::cout << "Query points before collapse mapping for segment " << id << ":"
                          << std::endl;
                for (size_t i = 0; i < query_points_copy.size(); i++) {
                    std::cout << "  Point " << i << ": f_id=" << query_points_copy[i].f_id
                              << ", bc=(" << query_points_copy[i].bc[0].to_double() << ", "
                              << query_points_copy[i].bc[1].to_double() << ", "
                              << query_points_copy[i].bc[2].to_double() << ")" << std::endl;
                }
            }
        }
        // Per-segment tracing timing
        igl::Timer trace_timer;
        trace_timer.start();
        handle_one_segment_t(
            curve,
            id,
            query_points,
            UV_joint_r,
            F_before,
            v_id_map_joint,
            id_map_before,
            TT,
            TTi,
            verbose);
        time_trace_segment_total += trace_timer.getElapsedTime() * 1000;
    }

    if (!is_curve_valid_t(curve)) {
        std::cout << "Error: curve is not valid after EdgeCollapse" << std::endl;
        throw std::runtime_error("Error: curve is not valid after EdgeCollapse");
    }

    // Suppress high-level summary to reduce noise
    // std::cout << "count: " << cnter << std::endl;
    // std::cout << "handle_collapse_edge_curve map points time: " << time_map_points_total << " ms"
    //           << std::endl;
    // std::cout << "handle_collapse_edge_curve map points time per segment: "
    //           << time_map_points_total / cnter << " ms" << std::endl;
    // std::cout << "handle_collapse_edge_curve trace segment time: " << time_trace_segment_total
    //           << " ms" << std::endl;
    // std::cout << "handle_collapse_edge_curve trace segment time per segment: "
    //           << time_trace_segment_total / cnter << " ms" << std::endl;
}

// Backward compatibility version
void handle_collapse_edge_curve(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve& curve,
    bool use_rational,
    bool verbose)
{
    return handle_collapse_edge_curve_t(
        UV_joint,
        F_before,
        F_after,
        v_id_map_joint,
        id_map_before,
        id_map_after,
        curve,
        use_rational,
        verbose);
}

// Template version of handle_non_collapse_operation_curve
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
    bool verbose)
{
    if (verbose) {
        std::cout << "Handling " << operation_name << " curve" << std::endl;
    }
    Eigen::VectorXd dblarea_before, dblarea_after;
    igl::doublearea(V_before, F_before, dblarea_before);
    if (dblarea_before.minCoeff() <= 0) {
        std::cout << "Error: dblarea_before is negative" << std::endl;
        throw std::runtime_error("Error: dblarea_before is negative");
    }
    igl::doublearea(V_after, F_after, dblarea_after);
    if (dblarea_after.minCoeff() <= 0) {
        std::cout << "Error: dblarea_after is negative" << std::endl;
        throw std::runtime_error("Error: dblarea_after is negative");
    }

    int curve_length = curve.segments.size();
    Eigen::MatrixXi TT, TTi;

    // Convert V_before to rational once for all segments (optimization)
    Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
    for (int i = 0; i < V_before.rows(); i++) {
        for (int j = 0; j < V_before.cols(); j++) {
            V_before_r(i, j) = wmtk::Rational(V_before(i, j));
        }
    }

    for (int id = 0; id < curve_length; id++) {
        query_segment_t<CoordType>& qs = curve.segments[id];
        query_point_t<CoordType> qp0 = {qs.f_id, qs.bcs[0], qs.fv_ids};
        query_point_t<CoordType> qp1 = {qs.f_id, qs.bcs[1], qs.fv_ids};
        std::vector<query_point_t<CoordType>> query_points = {qp0, qp1};

        auto it = std::find(id_map_after.begin(), id_map_after.end(), qs.f_id);
        if (it == id_map_after.end()) {
            continue;
        }

        std::vector<BarycentricPrecompute2D> bc_cache_non =
            build_barycentric_cache_2d_from_double(V_before, F_before);

        handle_non_collapse_operation_t(
            V_before,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after,
            F_after,
            id_map_after,
            v_id_map_after,
            query_points,
            operation_name,
            true,
            &bc_cache_non);
        if (verbose) {
            std::cout << "Mapped query points for segment " << id << ":" << std::endl;
            for (int i = 0; i < query_points.size(); i++) {
                std::cout << "  Point " << i << ":" << std::endl;
                std::cout << "    f_id: " << query_points[i].f_id << std::endl;
                if constexpr (std::is_same_v<CoordType, double>) {
                    std::cout << "    bcs: [" << query_points[i].bc.transpose() << "]" << std::endl;
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    std::cout << "    bcs: [" << query_points[i].bc(0).to_double() << ", "
                              << query_points[i].bc(1).to_double() << ", "
                              << query_points[i].bc(2).to_double() << "]" << std::endl;
                }
            }
        }
        handle_one_segment_t(
            curve,
            id,
            query_points,
            V_before_r,
            F_before,
            v_id_map_before,
            id_map_before,
            TT,
            TTi,
            verbose);
    }

    if (!is_curve_valid_t(curve)) {
        std::cout << "Error: curve is not valid after " << operation_name << std::endl;
        throw std::runtime_error("Error: curve is not valid after " + operation_name);
    }
}

// Backward compatibility version
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
    bool verbose)
{
    return handle_non_collapse_operation_curve_t(
        V_before,
        F_before,
        id_map_before,
        v_id_map_before,
        V_after,
        F_after,
        id_map_after,
        v_id_map_after,
        curve,
        operation_name,
        verbose);
}

// Template version of clean_up_curve
template <typename CoordType>
void clean_up_curve_t(query_curve_t<CoordType>& curve)
{
    // TODO: make this function work for loops
    if (curve.segments.empty()) {
        return;
    }

    std::vector<query_segment_t<CoordType>> new_segments;
    std::vector<int> new_next_segment_ids;
    std::vector<int> old_to_new_mapping(curve.segments.size(), -1);


    // Process segments in chain order starting from 0
    // int current_id = 0;
    // while (current_id != -1 && current_id < curve.segments.size()) {
    for (int current_id = 0; current_id < curve.segments.size(); current_id++) {
        if (old_to_new_mapping[current_id] != -1) {
            // This segment was already processed, skip
            // current_id = curve.next_segment_ids[current_id];
            continue;
        }

        query_segment_t<CoordType>& current_segment = curve.segments[current_id];

        // Convert to rational arithmetic for exact collinearity check (only first 2 components)
        Eigen::Vector2<wmtk::Rational> start_bc_r(
            wmtk::Rational(current_segment.bcs[0](0)),
            wmtk::Rational(current_segment.bcs[0](1)));

        Eigen::Vector2<wmtk::Rational> current_slope_r =
            Eigen::Vector2<wmtk::Rational>(
                wmtk::Rational(current_segment.bcs[1](0)),
                wmtk::Rational(current_segment.bcs[1](1))) -
            start_bc_r;

        // Find consecutive segments that can be merged
        std::vector<int> segments_to_merge = {current_id};
        int next_id = curve.next_segment_ids[current_id];

        while (next_id != -1 && next_id < curve.segments.size()) {
            if (next_id == current_id) {
                break; // loop detected
            }
            query_segment_t<CoordType>& next_segment = curve.segments[next_id];

            // Check if they can be merged (same origin_segment_id and f_id)
            if (current_segment.origin_segment_id != next_segment.origin_segment_id ||
                current_segment.f_id != next_segment.f_id) {
                break; // Different segments or faces, cannot merge
            }

            // Convert next segment to rational arithmetic (only first 2 components)
            Eigen::Vector2<wmtk::Rational> next_slope_r =
                Eigen::Vector2<wmtk::Rational>(
                    wmtk::Rational(next_segment.bcs[1](0)),
                    wmtk::Rational(next_segment.bcs[1](1))) -
                start_bc_r;

            // Check collinearity using cross product in 2D (exact test)
            // Two 2D vectors are collinear if their cross product is zero
            wmtk::Rational cross_product_2d =
                current_slope_r(0) * next_slope_r(1) - current_slope_r(1) * next_slope_r(0);
            bool is_collinear = (cross_product_2d == wmtk::Rational(0));


            // TODO: handle zero slope case
            {
                if (current_slope_r.norm() == 0) {
                    current_slope_r = next_slope_r;
                }
            }

            if (!is_collinear) {
                break; // not collinear, stop merging
            }

            segments_to_merge.push_back(next_id);
            next_id = curve.next_segment_ids[next_id];
        }

        // for debug
        // {
        //     if (segments_to_merge.size() > 1) {
        //         std::cout << "Merging segments: ";
        //         for (int idx : segments_to_merge) {
        //             const auto& seg = curve.segments[idx];
        //             std::cout << "\n  seg id: " << idx << ", f_id: " << seg.f_id
        //                       << ", origin_segment_id: " << seg.origin_segment_id << ", bcs[0]:
        //                       ["
        //                       << seg.bcs[0].transpose() << "]" << ", bcs[1]: ["
        //                       << seg.bcs[1].transpose() << "]" << ", fv_ids: [";
        //             for (int k = 0; k < seg.fv_ids.size(); ++k) {
        //                 std::cout << seg.fv_ids[k];
        //                 if (k + 1 < seg.fv_ids.size()) std::cout << ", ";
        //             }
        //             std::cout << "]";
        //         }
        //         std::cout << std::endl;
        //     }
        // }
        // Create merged segment
        query_segment_t<CoordType> merged_segment;
        merged_segment.f_id = current_segment.f_id;
        merged_segment.origin_segment_id = current_segment.origin_segment_id;
        merged_segment.bcs[0] = current_segment.bcs[0]; // First segment's bc0
        merged_segment.bcs[1] =
            curve.segments[segments_to_merge.back()].bcs[1]; // Last segment's bc1
        merged_segment.fv_ids = current_segment.fv_ids;

        // Add to new segments
        int new_segment_id = new_segments.size();
        new_segments.push_back(merged_segment);

        // Map all merged segments to the new segment
        for (int old_id : segments_to_merge) {
            old_to_new_mapping[old_id] = new_segment_id;
        }

        // Set next segment ID for the merged segment (this will be updated later)
        int final_next = curve.next_segment_ids[segments_to_merge.back()];
        new_next_segment_ids.push_back(final_next);

        // Move to the next unprocessed segment
        // current_id = final_next;
    }

    // Update next_segment_ids to point to new indices
    for (int i = 0; i < new_next_segment_ids.size(); i++) {
        if (new_next_segment_ids[i] != -1) {
            int old_next = new_next_segment_ids[i];
            if (old_next < old_to_new_mapping.size() && old_to_new_mapping[old_next] != -1) {
                new_next_segment_ids[i] = old_to_new_mapping[old_next];
            } else {
                // If the old_next was not mapped, it means it was merged into another segment
                // We need to find which segment it was merged into
                std::cout << "Warning: old_next " << old_next << " was not mapped, setting to -1"
                          << std::endl;
                new_next_segment_ids[i] = -1;
            }
        }
    }
    std::cout << "clean up curve, segments size: " << curve.segments.size() << " -> "
              << new_segments.size() << std::endl;
    // Replace the curve's segments and next_segment_ids
    curve.segments = std::move(new_segments);
    curve.next_segment_ids = std::move(new_next_segment_ids);
}
void clean_up_curve(query_curve& curve)
{
    return clean_up_curve_t(curve);
}

// Template version of is_curve_valid
template <typename CoordType>
bool is_curve_valid_t(const query_curve_t<CoordType>& curve)
{
    // TODO: make this function work for loops
    if (curve.segments.empty()) {
        std::cout << "Warning:curve is empty" << std::endl;
        return true;
    }

    // int cur_seg_id = 0;
    bool is_valid = true;
    // while (cur_seg_id != -1 && cur_seg_id < curve.segments.size()) {
    for (int cur_seg_id = 0; cur_seg_id < curve.segments.size(); cur_seg_id++) {
        int next_seg_id = curve.next_segment_ids[cur_seg_id];
        // std::cout << "cur_seg: " << cur_seg_id << " next_seg: " << next_seg_id << std::endl;
        if (next_seg_id == -1) {
            continue;
        }
        const auto& cur_seg = curve.segments[cur_seg_id];
        const auto& next_seg = curve.segments[next_seg_id];

        // std::cout << "cur_seg.f_id: " << cur_seg.f_id << " next_seg.f_id: " << next_seg.f_id
        //           << std::endl;
        if (cur_seg.f_id == next_seg.f_id) {
            auto bc_diff = next_seg.bcs[0] - cur_seg.bcs[1];
            bool is_invalid = false;

            if constexpr (std::is_same_v<CoordType, double>) {
                is_invalid = (bc_diff.norm() > 1e-8);
            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                // For rational, check if all components are exactly zero
                is_invalid =
                    (bc_diff(0) != wmtk::Rational(0) || bc_diff(1) != wmtk::Rational(0) ||
                     bc_diff(2) != wmtk::Rational(0));
            }

            if (is_invalid) {
                std::cout << "Error: bc_diff is too large" << std::endl;
                // std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose() << std::endl;
                // std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose() << std::endl;
                // std::cout << "bc_diff: " << bc_diff.transpose() << std::endl;
                is_valid = false;
                return is_valid;
            }
        } else {
            for (int i = 0; i < 3; i++) {
                bool is_nonzero;
                if constexpr (std::is_same_v<CoordType, double>) {
                    is_nonzero = (cur_seg.bcs[1](i) != 0.0);
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    is_nonzero = (cur_seg.bcs[1](i) != wmtk::Rational(0));
                }

                if (is_nonzero) {
                    int vid = cur_seg.fv_ids[i];

                    // find vid in next_seg.fv_ids
                    auto it = std::find(next_seg.fv_ids.begin(), next_seg.fv_ids.end(), vid);
                    if (it == next_seg.fv_ids.end()) {
                        std::cout << "Error: vid not found in next_seg.fv_ids" << std::endl;
                        // std::cout << "cur_seg.fv_ids: " << cur_seg.fv_ids.transpose() <<
                        // std::endl; std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose()
                        // << std::endl; std::cout << "next_seg.fv_ids: " <<
                        // next_seg.fv_ids.transpose()
                        //           << std::endl;
                        // std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose()
                        //           << std::endl;
                        is_valid = false;
                    } else {
                        int next_seg_vid_id = std::distance(next_seg.fv_ids.begin(), it);
                        auto diff = next_seg.bcs[0](next_seg_vid_id) - cur_seg.bcs[1](i);
                        bool is_diff_too_large = false;

                        if constexpr (std::is_same_v<CoordType, double>) {
                            is_diff_too_large = (abs(diff) > 1e-8);
                        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                            // For rational, check if exactly zero
                            is_diff_too_large = (diff != wmtk::Rational(0));
                        }

                        if (is_diff_too_large) {
                            std::cout << "Error: bc_diff is too large" << std::endl;
                            // std::cout << "cur_seg.fv_ids: " << cur_seg.fv_ids.transpose()
                            //           << std::endl;
                            // std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose()
                            //           << std::endl;
                            // std::cout << "next_seg.fv_ids: " << next_seg.fv_ids.transpose()
                            //           << std::endl;
                            // std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose()
                            //           << std::endl;
                            is_valid = false;
                        }
                    }
                }
            }
        }
        // cur_seg_id = next_seg_id;
    }
    return is_valid;
}
bool is_curve_valid(const query_curve& curve)
{
    return is_curve_valid_t(curve);
}


// function that computes all the intersections between two curves
template <typename CoordType>
int compute_intersections_between_two_curves_t(
    const query_curve_t<CoordType>& curve1,
    const query_curve_t<CoordType>& curve2,
    bool verbose)
{
    int intersections = 0;
    for (auto seg1 : curve1.segments) {
        for (auto seg2 : curve2.segments) {
            if (seg1.f_id != seg2.f_id) {
                continue;
            }

            Eigen::Vector2<wmtk::Rational> s1_a, s1_b, s2_a, s2_b;
            s1_a << wmtk::Rational(seg1.bcs[0](0)), wmtk::Rational(seg1.bcs[0](1));
            s1_b << wmtk::Rational(seg1.bcs[1](0)), wmtk::Rational(seg1.bcs[1](1));
            s2_a << wmtk::Rational(seg2.bcs[0](0)), wmtk::Rational(seg2.bcs[0](1));
            s2_b << wmtk::Rational(seg2.bcs[1](0)), wmtk::Rational(seg2.bcs[1](1));

            Eigen::Vector2<wmtk::Rational> bc_tmp;
            if (intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, false)) {
                intersectSegmentEdge_r(
                    s1_a,
                    s1_b,
                    s2_a,
                    s2_b,
                    bc_tmp,
                    true); // TODO: this line is only for debug
                intersections++;
                if (verbose) {
                    if constexpr (std::is_same_v<CoordType, double>) {
                        // std::cout << "Intersection found in face " << seg1.f_id << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id
                                  << ", bcs[0]=" << seg1.bcs[0].transpose()
                                  << ", bcs[1]=" << seg1.bcs[1].transpose() << std::endl;
                        std::cout << "seg2: f_id=" << seg2.f_id
                                  << ", bcs[0]=" << seg2.bcs[0].transpose()
                                  << ", bcs[1]=" << seg2.bcs[1].transpose() << std::endl;
                        std::cout << "intersection position at t = " << bc_tmp(0).to_double()
                                  << std::endl;
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << "Intersection found in face " << seg1.f_id << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg1.bcs[0].size(); k++) {
                            std::cout << seg1.bcs[0](k).to_double();
                            if (k < seg1.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg1.bcs[0].size(); k++) {
                            std::cout << seg1.bcs[0](k).to_double();
                            if (k < seg1.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg2: f_id=" << seg2.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k).to_double();
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k).to_double();
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "intersection position at t = " << bc_tmp(0).to_double()
                                  << std::endl;
                    }
                }
            }
        }
    }
    return intersections;
}
int compute_intersections_between_two_curves(
    const query_curve& curve1,
    const query_curve& curve2,
    bool verbose)
{
    return compute_intersections_between_two_curves_t(curve1, curve2, verbose);
}

// Template version of compute_curve_self_intersections
template <typename CoordType>
int compute_curve_self_intersections_t(const query_curve_t<CoordType>& curve, bool verbose)
{
    int intersections = 0;

    for (int i = 0; i < curve.segments.size(); i++) {
        for (int j = i + 1; j < curve.segments.size(); j++) {
            if (curve.next_segment_ids[i] == j || curve.next_segment_ids[j] == i) {
                continue;
            }

            const auto& seg1 = curve.segments[i];
            const auto& seg2 = curve.segments[j];
            if (seg1.f_id != seg2.f_id) {
                continue;
            }

            Eigen::Vector2<wmtk::Rational> s1_a, s1_b, s2_a, s2_b;
            s1_a << wmtk::Rational(seg1.bcs[0](0)), wmtk::Rational(seg1.bcs[0](1));
            s1_b << wmtk::Rational(seg1.bcs[1](0)), wmtk::Rational(seg1.bcs[1](1));
            s2_a << wmtk::Rational(seg2.bcs[0](0)), wmtk::Rational(seg2.bcs[0](1));
            s2_b << wmtk::Rational(seg2.bcs[1](0)), wmtk::Rational(seg2.bcs[1](1));

            Eigen::Vector2<wmtk::Rational> bc_tmp;
            if (intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, false)) {
                intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, true);
                intersections++;
                if (verbose) {
                    if constexpr (std::is_same_v<CoordType, double>) {
                        std::cout << "i=" << i << ", j=" << j << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id
                                  << ", bcs[0]=" << seg1.bcs[0].transpose()
                                  << ", bcs[1]=" << seg1.bcs[1].transpose() << std::endl;
                        std::cout << "seg1: origin_segment_id=" << seg1.origin_segment_id
                                  << std::endl;
                        std::cout << "seg1: next_segment_id=" << curve.next_segment_ids[i]
                                  << std::endl;

                        if (curve.next_segment_ids[i] != -1) {
                            const auto& next_seg1 = curve.segments[curve.next_segment_ids[i]];
                            std::cout << "seg1 next: f_id=" << next_seg1.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg1.bcs[0].size(); k++) {
                                std::cout << next_seg1.bcs[0](k);
                                if (k < next_seg1.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg1.bcs[1].size(); k++) {
                                std::cout << next_seg1.bcs[1](k);
                                if (k < next_seg1.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg1 next: origin_segment_id=" << next_seg1.origin_segment_id
                                << std::endl;
                        }
                        std::cout << "seg2: f_id=" << seg2.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k);
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg2.bcs[1].size(); k++) {
                            std::cout << seg2.bcs[1](k);
                            if (k < seg2.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg2: origin_segment_id=" << seg2.origin_segment_id
                                  << std::endl;
                        std::cout << "seg2: next_segment_id=" << curve.next_segment_ids[j]
                                  << std::endl;

                        if (curve.next_segment_ids[j] != -1) {
                            const auto& next_seg2 = curve.segments[curve.next_segment_ids[j]];
                            std::cout << "seg2 next: f_id=" << next_seg2.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg2.bcs[0].size(); k++) {
                                std::cout << next_seg2.bcs[0](k);
                                if (k < next_seg2.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg2.bcs[1].size(); k++) {
                                std::cout << next_seg2.bcs[1](k);
                                if (k < next_seg2.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg2 next: origin_segment_id=" << next_seg2.origin_segment_id
                                << std::endl;
                        }

                        std::cout << "intersection position at t = " << bc_tmp(0).to_double()
                                  << std::endl;
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << "i=" << i << ", j=" << j << std::endl;
                        std::cout << "seg1: f_id=" << seg1.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg1.bcs[0].size(); k++) {
                            std::cout << seg1.bcs[0](k).to_double();
                            if (k < seg1.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg1.bcs[1].size(); k++) {
                            std::cout << seg1.bcs[1](k).to_double();
                            if (k < seg1.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg1: origin_segment_id=" << seg1.origin_segment_id
                                  << std::endl;
                        std::cout << "seg1: next_segment_id=" << curve.next_segment_ids[i]
                                  << std::endl;

                        if (curve.next_segment_ids[i] != -1) {
                            const auto& next_seg1 = curve.segments[curve.next_segment_ids[i]];
                            std::cout << "seg1 next: f_id=" << next_seg1.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg1.bcs[0].size(); k++) {
                                std::cout << next_seg1.bcs[0](k).to_double();
                                if (k < next_seg1.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg1.bcs[1].size(); k++) {
                                std::cout << next_seg1.bcs[1](k).to_double();
                                if (k < next_seg1.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg1 next: origin_segment_id=" << next_seg1.origin_segment_id
                                << std::endl;
                        }
                        std::cout << "seg2: f_id=" << seg2.f_id << ", bcs[0]=[";
                        for (int k = 0; k < seg2.bcs[0].size(); k++) {
                            std::cout << seg2.bcs[0](k).to_double();
                            if (k < seg2.bcs[0].size() - 1) std::cout << ", ";
                        }
                        std::cout << "], bcs[1]=[";
                        for (int k = 0; k < seg2.bcs[1].size(); k++) {
                            std::cout << seg2.bcs[1](k).to_double();
                            if (k < seg2.bcs[1].size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        std::cout << "seg2: origin_segment_id=" << seg2.origin_segment_id
                                  << std::endl;
                        std::cout << "seg2: next_segment_id=" << curve.next_segment_ids[j]
                                  << std::endl;

                        if (curve.next_segment_ids[j] != -1) {
                            const auto& next_seg2 = curve.segments[curve.next_segment_ids[j]];
                            std::cout << "seg2 next: f_id=" << next_seg2.f_id << ", bcs[0]=[";
                            for (int k = 0; k < next_seg2.bcs[0].size(); k++) {
                                std::cout << next_seg2.bcs[0](k).to_double();
                                if (k < next_seg2.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "], bcs[1]=[";
                            for (int k = 0; k < next_seg2.bcs[1].size(); k++) {
                                std::cout << next_seg2.bcs[1](k).to_double();
                                if (k < next_seg2.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            std::cout
                                << "seg2 next: origin_segment_id=" << next_seg2.origin_segment_id
                                << std::endl;
                        }

                        std::cout << "intersection position at t = " << bc_tmp(0).to_double()
                                  << std::endl;
                    }
                }
            }
        }
    }

    return intersections;
}
int compute_curve_self_intersections(const query_curve& curve, bool verbose)
{
    return compute_curve_self_intersections_t(curve, verbose);
}

// Explicit template instantiation for compilation
template bool is_curve_valid_t<double>(const query_curve_t<double>& curve);
template bool is_curve_valid_t<wmtk::Rational>(const query_curve_t<wmtk::Rational>& curve);

template void clean_up_curve_t<double>(query_curve_t<double>& curve);
template void clean_up_curve_t<wmtk::Rational>(query_curve_t<wmtk::Rational>& curve);

template int compute_intersections_between_two_curves_t<double>(
    const query_curve_t<double>& curve1,
    const query_curve_t<double>& curve2,
    bool verbose);
template int compute_intersections_between_two_curves_t<wmtk::Rational>(
    const query_curve_t<wmtk::Rational>& curve1,
    const query_curve_t<wmtk::Rational>& curve2,
    bool verbose);

template int compute_curve_self_intersections_t<double>(
    const query_curve_t<double>& curve,
    bool verbose);
template int compute_curve_self_intersections_t<wmtk::Rational>(
    const query_curve_t<wmtk::Rational>& curve,
    bool verbose);

template void handle_collapse_edge_curve_t<double>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve_t<double>& curve,
    bool use_rational,
    bool verbose);
template void handle_collapse_edge_curve_t<wmtk::Rational>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve_t<wmtk::Rational>& curve,
    bool use_rational,
    bool verbose);

template void handle_non_collapse_operation_curve_t<double>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_curve_t<double>& curve,
    const std::string& operation_name,
    bool verbose);
template void handle_non_collapse_operation_curve_t<wmtk::Rational>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_curve_t<wmtk::Rational>& curve,
    const std::string& operation_name,
    bool verbose);