
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <set>
#include "track_operations_curve.hpp"
#ifdef USE_IGL_VIEWER
#include <igl/opengl/glfw/Viewer.h>
#endif

// Template version of handle_non_collapse_operation_curves
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

    // Profiling accumulators
    igl::Timer total_timer;
    total_timer.start();
    igl::Timer convert_timer;
    double time_area_total = 0.0;
    double time_convert_rational = 0.0;
    double time_map_points_total = 0.0;
    double time_handle_segment_total = 0.0;

    int curve_length = curve.segments.size();
    Eigen::MatrixXi TT, TTi;

    // Convert V_before to rational once for all segments (optimization)
    convert_timer.start();
    Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
    for (int i = 0; i < V_before.rows(); i++) {
        for (int j = 0; j < V_before.cols(); j++) {
            V_before_r(i, j) = wmtk::Rational(V_before(i, j));
        }
    }
    time_convert_rational += convert_timer.getElapsedTime();

    int processed_segments = 0;
    for (int id = 0; id < curve_length; id++) {
        query_segment_t<CoordType>& qs = curve.segments[id];
        query_point_t<CoordType> qp0 = {qs.f_id, qs.bcs[0], qs.fv_ids};
        query_point_t<CoordType> qp1 = {qs.f_id, qs.bcs[1], qs.fv_ids};
        std::vector<query_point_t<CoordType>> query_points = {qp0, qp1};
        auto query_points_copy = query_points;

        auto it = std::find(id_map_after.begin(), id_map_after.end(), qs.f_id);
        if (it == id_map_after.end()) {
            continue;
        }
        processed_segments++;

        std::vector<BarycentricPrecompute2D> bc_cache_non =
            build_barycentric_cache_2d_from_double(V_before, F_before);

        // Map points timing
        igl::Timer map_timer;
        map_timer.start();
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
        time_map_points_total += map_timer.getElapsedTime() * 1000;
        if (verbose == true) {
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

                std::cout << " Point before mapping: \n f_id:" << query_points_copy[i].f_id;
                if constexpr (std::is_same_v<CoordType, double>) {
                    std::cout << " bcs: [" << query_points_copy[i].bc.transpose() << "]"
                              << std::endl;
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    std::cout << " bcs: [" << query_points_copy[i].bc(0).to_double() << ", "
                              << query_points_copy[i].bc(1).to_double() << ", "
                              << query_points_copy[i].bc(2).to_double() << "]" << std::endl;
                }
            }

            if ((query_points[0].f_id == query_points[1].f_id) &&
                (query_points[0].bc - query_points[1].bc).norm() == 0) {
                std::cout << "ERROR: map to same point" << std::endl;
            }
        }
        // Per-segment handling timing
        igl::Timer segment_timer;
        segment_timer.start();
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
        time_handle_segment_total += segment_timer.getElapsedTime() * 1000;
    }

    if (!is_curve_valid_t(curve)) {
        std::cout << "Error: curve is not valid after " << operation_name << std::endl;
        throw std::runtime_error("Error: curve is not valid after " + operation_name);
    }

    // Print timing summary similar to collapse edge function
    if (verbose) {
        double total_time_ms = total_timer.getElapsedTime() * 1000.0;
        std::cout << "\n=== handle_non_collapse_operation_curve_t Timing ===" << std::endl;
        std::cout << "  Operation: " << operation_name << std::endl;
        std::cout << "  Total segments: " << curve_length << std::endl;
        std::cout << "  Processed segments: " << processed_segments << std::endl;
        std::cout << "  Total time: " << total_time_ms << " ms" << std::endl;
        std::cout << "  Rational conversion: " << time_convert_rational * 1000.0 << " ms"
                  << std::endl;
        std::cout << "  Point mapping total: " << time_map_points_total << " ms" << std::endl;
        std::cout << "  Segment handling total: " << time_handle_segment_total << " ms"
                  << std::endl;
        if (processed_segments > 0) {
            std::cout << "  Point mapping per segment: "
                      << time_map_points_total / processed_segments << " ms" << std::endl;
            std::cout << "  Segment handling per segment: "
                      << time_handle_segment_total / processed_segments << " ms" << std::endl;
        }
    }
}


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
    bool verbose)
{
    if constexpr (std::is_same_v<CoordType, double>) {
        for (auto& curve : curves) {
            handle_non_collapse_operation_curve_t(
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
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        handle_non_collapse_operation_curves_fast_rational(
            V_before,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after,
            F_after,
            id_map_after,
            v_id_map_after,
            curves,
            operation_name,
            verbose);
    }
}

////////////////////////////////////////////////////////////
// Rational fast version of handle_non_collapse_operation_curves_t
////////////////////////////////////////////////////////////
void map_all_query_points_rational_non_collapse(
    const Eigen::MatrixX<wmtk::Rational>& V_before_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixX<wmtk::Rational>& V_after_r,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_r>& all_query_points,
    const std::vector<int>& all_query_seg_ids,
    const std::vector<int>& bc0_places,
    const query_curve_t<wmtk::Rational>& curve,
    const std::vector<BarycentricPrecompute2D>& bc_cache_non,
    std::vector<std::vector<int>>& all_curve_parts)
{
    // query points that are not on the boundary of the local patch
    std::vector<query_point_r> non_bd_qps;
    std::vector<int> non_bd_qps_ids;
    std::vector<int> bd_qps_ids;
    // Special handling for the qps that are on the boundary of the local patch
    {
        classify_boundary_and_interior_query_points(
            all_query_points,
            all_query_seg_ids,
            bc0_places,
            curve,
            non_bd_qps,
            non_bd_qps_ids,
            bd_qps_ids,
            all_curve_parts);
    }

    // map all the boundary qps
    {
        map_local_boundary_qps(
            F_before,
            v_id_map_before,
            id_map_before,
            all_query_points,
            bd_qps_ids);
    }

    // map all the non boundary qps
    {
        handle_non_collapse_operation_rational(
            V_before_r,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after_r,
            F_after,
            id_map_after,
            v_id_map_after,
            non_bd_qps,
            &bc_cache_non);

        for (int i = 0; i < non_bd_qps.size(); i++) {
            all_query_points[non_bd_qps_ids[i]] = non_bd_qps[i];
        }
    }

    // std::cout << "all_query_points after mapping: " << std::endl;
    // for (int i = 0; i < all_query_points.size(); i++) {
    //     std::cout << "  [" << i << "]: " << all_query_points[i] << std::endl;
    // }
}

void handle_non_collapse_operation_curve_rational(
    const Eigen::MatrixX<wmtk::Rational>& V_before_r,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixX<wmtk::Rational>& V_after_r,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_curve_t<wmtk::Rational>& curve,
    const std::vector<BarycentricPrecompute2D>& bc_cache_non, // Cache for faster bc compute
    std::vector<std::vector<int>>& all_curve_parts_after_mapping,
    bool verbose)
{
    // TODO: add timer here
    double time_trace_segment_total = 0.0;
    // get all query points that need to be mapped
    std::vector<query_point_r> all_query_points;
    std::vector<int> all_query_seg_ids;
    std::vector<int> bc0_places;

    // STEP1: get all query points that need to be mapped
    {
        get_all_query_points_for_one_curve_rational(
            id_map_after,
            curve,
            all_query_points,
            all_query_seg_ids,
            bc0_places);
    }

    // Print debug information about query points
    if (verbose) {
        std::cout << "Debug: all_query_points (" << all_query_points.size()
                  << " points):" << std::endl;
        for (size_t i = 0; i < all_query_points.size(); i++) {
            std::cout << "  [" << i << "]: " << all_query_points[i] << std::endl;
        }
        std::cout << "Debug: all_query_seg_ids size=" << all_query_seg_ids.size() << std::endl;
        if (all_query_seg_ids.size() > 0) {
            for (size_t i = 0; i < all_query_seg_ids.size(); i++) {
                std::cout << all_query_seg_ids[i] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
        std::cout << "Debug: bc0_places size=" << bc0_places.size() << std::endl;
        if (bc0_places.size() > 0) {
            for (size_t i = 0; i < bc0_places.size(); i++) {
                std::cout << bc0_places[i] << " ";
            }
            std::cout << std::endl;
        }
    }
    if (all_query_points.size() == 0) {
        if (verbose) {
            std::cout << "No query points to map" << std::endl;
        }
        return;
    }

    std::vector<std::vector<int>> all_curve_parts;
    // STEP2: map all query points
    {
        map_all_query_points_rational_non_collapse(
            V_before_r,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after_r,
            F_after,
            id_map_after,
            v_id_map_after,
            all_query_points,
            all_query_seg_ids,
            bc0_places,
            curve,
            bc_cache_non,
            all_curve_parts);
    }

    // STEP3: get intersections with mesh (handle one segment)
    const int num_segments_before_mapping = curve.segments.size();
    Eigen::MatrixXi TT, TTi;
    igl::triangle_triangle_adjacency(F_before, TT, TTi);
    {
        for (int i = 0; i < all_query_seg_ids.size(); i++) {
            int seg_id = all_query_seg_ids[i];
            std::vector<query_point_r> query_point_for_one_segment;
            query_point_for_one_segment.push_back(all_query_points[bc0_places[i]]);
            query_point_for_one_segment.push_back(all_query_points[bc0_places[i] + 1]);

            igl::Timer trace_timer;
            trace_timer.start();

            handle_one_segment_t(
                curve,
                seg_id,
                query_point_for_one_segment,
                V_before_r,
                F_before,
                v_id_map_before,
                id_map_before,
                TT,
                TTi,
                verbose);

            time_trace_segment_total += trace_timer.getElapsedTime() * 1000;
        }
        if (verbose) {
            std::cout << "Total time for tracing segments: " << time_trace_segment_total << " ms"
                      << std::endl;
            std::cout << "Tracing time per segment:"
                      << time_trace_segment_total / all_query_seg_ids.size() << " ms" << std::endl;
        }
    }
    // STEP4: update the new segments and then return
    {
        all_curve_parts_after_mapping.clear();
        all_curve_parts_after_mapping.resize(all_curve_parts.size());
        for (int i = 0; i < all_curve_parts.size(); i++) {
            for (int j = 0; j < all_curve_parts[i].size(); j++) {
                int cur_seg_id = all_curve_parts[i][j];
                all_curve_parts_after_mapping[i].push_back(cur_seg_id);
                while (curve.next_segment_ids[cur_seg_id] >=
                       num_segments_before_mapping) { // indicate new segment is created
                    cur_seg_id = curve.next_segment_ids[cur_seg_id];
                    all_curve_parts_after_mapping[i].push_back(cur_seg_id);
                }
            }
        }
    }
}

void handle_non_collapse_operation_curves_fast_rational(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::string& operation_name,
    bool verbose)
{
    verbose = false;
    std::cout << "handle " << operation_name << " curves fast rational" << std::endl;

    double convert_V_to_rational_time = 0.0;
    // convert V_before and V_after to rational
    Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
    Eigen::MatrixX<wmtk::Rational> V_after_r(V_after.rows(), V_after.cols());
    {
        igl::Timer convert_timer;
        convert_timer.start();
        for (int i = 0; i < V_before.rows(); i++) {
            for (int j = 0; j < V_before.cols(); j++) {
                V_before_r(i, j) = wmtk::Rational(V_before(i, j));
            }
        }
        for (int i = 0; i < V_after.rows(); i++) {
            for (int j = 0; j < V_after.cols(); j++) {
                V_after_r(i, j) = wmtk::Rational(V_after(i, j));
            }
        }
        convert_V_to_rational_time = convert_timer.getElapsedTime();
    }

    // build cache for bc compute
    double build_bc_cache_time = 0.0;
    std::vector<BarycentricPrecompute2D> bc_cache_non =
        build_barycentric_cache_2d(V_before_r, F_before);


    // optimized version of handle_non_collapse_operation curve in rational
    std::vector<std::vector<std::vector<int>>> all_curve_parts_after_mapping(curves.size());
    for (int i = 0; i < curves.size(); i++) {
        if (verbose) {
            std::cout << "handle curve " << i << std::endl;
        }
        auto& curve = curves[i];


        // implement some version that take cache in
        handle_non_collapse_operation_curve_rational(
            V_before_r,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after_r,
            F_after,
            id_map_after,
            v_id_map_after,
            curve,
            bc_cache_non,
            all_curve_parts_after_mapping[i],
            verbose);

        if (verbose) {
            std::cout << "handle curve " << i << " done" << std::endl << std::endl;
        }
    }


    {}
}

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

template void handle_non_collapse_operation_curves_t<double>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_curve_t<double>>& curves,
    const std::string& operation_name,
    bool verbose);
template void handle_non_collapse_operation_curves_t<wmtk::Rational>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::string& operation_name,
    bool verbose);