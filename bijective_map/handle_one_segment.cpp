#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <set>
#include "track_operations_curve.hpp"

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
    Eigen::Vector2<wmtk::Rational>& barycentric_cd,
    Eigen::Vector2<wmtk::Rational>& barycentric_ab,
    bool debug_mode)
{
    Eigen::Vector2<wmtk::Rational> ab = b - a;
    Eigen::Vector2<wmtk::Rational> cd = d - c;
    Eigen::Vector2<wmtk::Rational> ac = c - a;

    wmtk::Rational denominator = (ab.x() * cd.y() - ab.y() * cd.x());

    if (denominator == wmtk::Rational(0)) {
        if (debug_mode) std::cout << "Lines are parallel" << std::endl;
        return false; // parallel
    }

    wmtk::Rational t = (ac.x() * cd.y() - ac.y() * cd.x()) / denominator;
    wmtk::Rational u = -(ab.x() * ac.y() - ab.y() * ac.x()) / denominator;

    if (debug_mode) {
        std::cout << std::fixed << std::setprecision(15);
        std::cout << "t=" << t.to_double() << ", u=" << u.to_double() << std::endl;

        // Calculate and verify intersection points
        Eigen::Vector2<wmtk::Rational> pt_from_t = a + t * ab;
        Eigen::Vector2<wmtk::Rational> pt_from_u = c + u * cd;

        std::cout << "Point from t: (" << pt_from_t.x().to_double() << ", "
                  << pt_from_t.y().to_double() << ")" << std::endl;
        std::cout << "Point from u: (" << pt_from_u.x().to_double() << ", "
                  << pt_from_u.y().to_double() << ")" << std::endl;

        Eigen::Vector2<wmtk::Rational> diff = pt_from_t - pt_from_u;
        double diff_mag = std::sqrt(
            diff.x().to_double() * diff.x().to_double() +
            diff.y().to_double() * diff.y().to_double());
        std::cout << "Difference: " << diff_mag << std::endl;
    }

    if (t >= 0 && t <= wmtk::Rational(1) && u >= 0 && u <= wmtk::Rational(1)) {
        // Calculate barycentric coordinates for intersection on edge CD
        wmtk::Rational alpha_cd = wmtk::Rational(1) - u; // weight for point c
        wmtk::Rational beta_cd = u; // weight for point d
        barycentric_cd = Eigen::Vector2<wmtk::Rational>(alpha_cd, beta_cd);

        // Calculate barycentric coordinates for intersection on edge AB
        wmtk::Rational alpha_ab = wmtk::Rational(1) - t; // weight for point a
        wmtk::Rational beta_ab = t; // weight for point b
        barycentric_ab = Eigen::Vector2<wmtk::Rational>(alpha_ab, beta_ab);

        if (debug_mode) {
            std::cout << "INTERSECTION: CD(alpha=" << alpha_cd.to_double()
                      << ", beta=" << beta_cd.to_double() << "), AB(alpha=" << alpha_ab.to_double()
                      << ", beta=" << beta_ab.to_double() << ")" << std::endl;
        }
        return true;
    }

    if (debug_mode) std::cout << "No intersection - params out of range" << std::endl;
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
        if (verbose) {
            std::cout << "Early exit: Two endpoints are on the same face" << std::endl;
        }

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
        std::cout << "query_points[0]: " << query_points[0] << std::endl;
        std::cout << "query_points[1]: " << query_points[1] << std::endl;
        std::cout << "id_map_before contents:" << std::endl;
        for (size_t i = 0; i < id_map_before.size(); i++) {
            std::cout << "  [" << i << "] = " << id_map_before[i] << std::endl;
        }
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
            Eigen::Vector2<wmtk::Rational> edge_bc, placeholder_ab;
            bool has_intersection = intersectSegmentEdge_r(
                current_pos,
                b,
                edge_start,
                edge_end,
                edge_bc,
                placeholder_ab,
                false);
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
    if (true) {
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


// Explicit template instantiations
template void handle_one_segment_t<double>(
    query_curve_t<double>& curve,
    int id,
    std::vector<query_point_t<double>>& query_points,
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi,
    bool verbose);

template void handle_one_segment_t<wmtk::Rational>(
    query_curve_t<wmtk::Rational>& curve,
    int id,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi,
    bool verbose);
