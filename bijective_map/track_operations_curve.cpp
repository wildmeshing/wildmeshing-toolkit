#include "track_operations_curve.hpp"
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <set>
#ifdef USE_IGL_VIEWER
#include <igl/opengl/glfw/Viewer.h>
#endif


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
    if (verbose) {
        std::cout << "handle_collapse_edge_curve_t" << std::endl;
    }
    // Profiling accumulators
    igl::Timer total_timer;
    total_timer.start();
    igl::Timer cache_timer;
    igl::Timer convert_timer;
    double time_convert_rational = 0.0;
    double time_map_points_total = 0.0;
    double time_trace_segment_total = 0.0;
    double time_map_points_time_double = 0.0;
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

    // cache_timer.start();
    // // Build cache once per operation at curve level if desired
    // std::vector<BarycentricPrecompute2D> bc_cache_collapse =
    //     build_barycentric_cache_2d(UV_joint_r, F_before);
    // double time_cache = cache_timer.getElapsedTime() * 1000;

    // TODO: For here we are going to do something different
    std::vector<query_point_t<CoordType>> all_query_points;
    std::vector<int> all_query_seg_ids;
    std::vector<int> places_in_all_query_points;
    {
        int id = 0;
        const int start_id = 0;
        bool last_segment_included = false;
        while (id != -1 && id < curve_length) {
            query_segment_t<CoordType> qs = curve.segments[id];

            auto it = std::find(id_map_after.begin(), id_map_after.end(), qs.f_id);
            if (it == id_map_after.end()) {
                id = curve.next_segment_ids[id];
                last_segment_included = false;
                if (id == start_id) {
                    break;
                }
                continue;
            }

            all_query_seg_ids.push_back(id);
            places_in_all_query_points.push_back(all_query_points.size() - 1);

            if (!last_segment_included) {
                places_in_all_query_points.back()++;
                all_query_points.push_back(query_point_t<CoordType>{qs.f_id, qs.bcs[0], qs.fv_ids});
            }
            all_query_points.push_back(query_point_t<CoordType>{qs.f_id, qs.bcs[1], qs.fv_ids});

            last_segment_included = true;

            id = curve.next_segment_ids[id];
            if (id == start_id) {
                break;
            }
        } // end while
    }

    // {
    //     std::cout << "Debug: all_query_points (" << all_query_points.size()
    //               << " points):" << std::endl;
    //     for (size_t i = 0; i < all_query_points.size(); i++) {
    //         const auto& qp = all_query_points[i];
    //         std::cout << "  [" << i << "]: " << qp << std::endl;
    //     }
    // }

    std::vector<query_point_t<CoordType>> all_query_points_need_mapping;
    std::vector<int> all_query_points_need_mapping_ids;
    // special handling for the edge case:
    {
        for (int i = 0; i < all_query_points.size(); i++) {
            auto& qp = all_query_points[i];
            int v0 = -1, v1 = -1;
            int v0_index = -1;
            for (int j = 0; j < 3; j++) {
                if (qp.bc(j) == 0) {
                    v0 = qp.fv_ids((j + 1) % 3);
                    v1 = qp.fv_ids((j + 2) % 3);
                    v0_index = (j + 1) % 3;
                    break;
                }
            }

            if (v0 == -1 || v0 == v_id_map_joint.front() || v0 == v_id_map_joint.back() ||
                v1 == v_id_map_joint.front() || v1 == v_id_map_joint.back()) {
                all_query_points_need_mapping.push_back(all_query_points[i]);
                all_query_points_need_mapping_ids.push_back(i);
                continue;
            }

            // look for the edge in F_before
            bool bc_updated = false;
            for (int f_id = 0; f_id < F_before.rows(); f_id++) {
                for (int k = 0; k < 3; k++) {
                    if (v_id_map_joint[F_before(f_id, k)] == v0 &&
                        v_id_map_joint[F_before(f_id, (k + 1) % 3)] == v1) {
                        // std::cout << "map edge point qp: " << qp << std::endl;
                        auto qp_copy = qp;
                        qp.bc(k) = qp_copy.bc(v0_index);
                        qp.bc((k + 1) % 3) = qp_copy.bc((v0_index + 1) % 3);
                        qp.bc((k + 2) % 3) = 0;
                        qp.f_id = id_map_before[f_id];
                        qp.fv_ids << v_id_map_joint[F_before(f_id, 0)],
                            v_id_map_joint[F_before(f_id, 1)], v_id_map_joint[F_before(f_id, 2)];
                        // std::cout << "mapped edge point qp: " << qp << std::endl;
                        bc_updated = true;
                        break;
                    }
                }
                if (bc_updated) {
                    break;
                }
            }

            if (!bc_updated) {
                all_query_points_need_mapping.push_back(all_query_points[i]);
                all_query_points_need_mapping_ids.push_back(i);
            }
        }
    }

    // use double first
    std::vector<query_point_t<double>> all_query_points_need_mapping_double;
    for (const auto& qp : all_query_points_need_mapping) {
        query_point_t<double> qp_double;
        qp_double = convert_query_point<CoordType, double>(qp);
        all_query_points_need_mapping_double.push_back(qp_double);
    }
    // std::vector<query_point_t<double>> all_query_points_double_copy;
    // {
    //     std::cout << "Debug: all_query_seg_ids (" << all_query_seg_ids.size()
    //               << " segments):" << std::endl;
    //     for (size_t i = 0; i < all_query_seg_ids.size(); i++) {
    //         std::cout << "  [" << i << "]: " << all_query_seg_ids[i] << std::endl;
    //     }
    //     std::cout << "Debug: places_in_all_query_points (" << places_in_all_query_points.size()
    //               << " places):" << std::endl;
    //     for (size_t i = 0; i < places_in_all_query_points.size(); i++) {
    //         std::cout << "  [" << i << "]: " << places_in_all_query_points[i] << std::endl;
    //     }
    // }
    igl::Timer map_timer_double;


    {
        map_timer_double.start();


        handle_collapse_edge_use_exact_predicate(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            all_query_points_need_mapping_double);
        // handle_collapse_edge(
        //     UV_joint,
        //     F_before,
        //     F_after,
        //     v_id_map_joint,
        //     id_map_before,
        //     id_map_after,
        //     all_query_points_double,
        //     false);

        // convert it back to CoordType
        // for (size_t i = 0; i < all_query_points_double.size(); i++) {
        //     all_query_points[i] =
        //         convert_query_point<double, CoordType>(all_query_points_double[i]);
        // }
        for (int i = 0; i < all_query_points_need_mapping_ids.size(); i++) {
            int id = all_query_points_need_mapping_ids[i];
            all_query_points[id] =
                convert_query_point<double, CoordType>(all_query_points_need_mapping_double[i]);
        }
        time_map_points_time_double = map_timer_double.getElapsedTime() * 1000;
    }
    // std::cout << "mapped " << all_query_points.size() << " points" << std::endl;
    // {
    //     std::cout << "Debug: all_query_points after mapping (" << all_query_points.size()
    //               << " points):" << std::endl;
    //     for (size_t i = 0; i < all_query_points.size(); i++) {
    //         const auto& qp = all_query_points[i];
    //         std::cout << "  [" << i << "]: " << qp << std::endl;
    //     }
    // }


    for (int i = 0; i < all_query_seg_ids.size(); i++) {
        int id = all_query_seg_ids[i];
        std::vector<query_point_t<CoordType>> query_points_for_one_segment;
        query_points_for_one_segment.push_back(all_query_points[places_in_all_query_points[i]]);
        query_points_for_one_segment.push_back(all_query_points[places_in_all_query_points[i] + 1]);

        // std::cout << "id: " << id << std::endl;
        // std::cout << "query_points_for_one_segment[0]: " << query_points_for_one_segment[0]
        //           << std::endl;
        // std::cout << "query_points_for_one_segment[1]: " << query_points_for_one_segment[1]
        //           << std::endl;

        igl::Timer trace_timer;
        trace_timer.start();
        handle_one_segment_t(
            curve,
            id,
            query_points_for_one_segment,
            UV_joint_r,
            F_before,
            v_id_map_joint,
            id_map_before,
            TT,
            TTi,
            false);
        time_trace_segment_total += trace_timer.getElapsedTime() * 1000;
    }

    if (!is_curve_valid_t(curve)) {
        std::cout << "Error: curve is not valid after handle_collapse_edge_curve_t" << std::endl;
        throw std::runtime_error("Error: curve is not valid after handle_collapse_edge_curve_t");
    }

    if (verbose) {
        std::cout << "handle_collapse_edge_curve_t finished" << std::endl;
    }

    if (verbose && all_query_points.size() > 0) {
        std::cout << "Time summary:" << std::endl;
        std::cout << "  Total time: " << total_timer.getElapsedTime() * 1000 << " ms" << std::endl;
        std::cout << "  Time to convert to rational: " << time_convert_rational << " ms"
                  << std::endl;
        std::cout << "  Time to map points: " << time_map_points_total << " ms" << std::endl;
        std::cout << "  Time to map points in double: " << time_map_points_time_double << " ms"
                  << std::endl;
        std::cout << " Average time to map points in double: "
                  << time_map_points_time_double / all_query_points.size() << " ms" << std::endl;
        std::cout << "  Time to trace segments: " << time_trace_segment_total << " ms" << std::endl;
        std::cout << " Average time to trace segments: "
                  << time_trace_segment_total / all_query_seg_ids.size() << " ms" << std::endl;
    }
    std::cout << std::endl;
}

// Curve handling functions for different operations - template version
// this version's rational version is guaranteed to be correct
template <typename CoordType>
void handle_collapse_edge_curve_t_correct(
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
    if (verbose) {
        std::cout << "handle_collapse_edge_curve_t old exact version" << std::endl;
    }
    // Profiling accumulators
    igl::Timer total_timer;
    total_timer.start();
    igl::Timer cache_timer;
    igl::Timer convert_timer;
    // double time_area_total = 0.0;
    double time_convert_rational = 0.0;
    double time_map_points_total = 0.0;
    double time_trace_segment_total = 0.0;
    double time_map_points_time_double = 0.0;
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

    cache_timer.start();
    // Build cache once per operation at curve level if desired
    std::vector<BarycentricPrecompute2D> bc_cache_collapse =
        build_barycentric_cache_2d(UV_joint_r, F_before);
    double time_cache = cache_timer.getElapsedTime() * 1000;

    for (int id = 0; id < curve_length; id++) {
        ////////////////////////////////////
        // map the two end points of one segment
        ////////////////////////////////////
        query_segment_t<CoordType>& qs = curve.segments[id];
        query_point_t<CoordType> qp0 = {qs.f_id, qs.bcs[0], qs.fv_ids};
        query_point_t<CoordType> qp1 = {qs.f_id, qs.bcs[1], qs.fv_ids};
        std::vector<query_point_t<CoordType>> query_points = {qp0, qp1};
        auto query_points_copy = query_points;
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qs.f_id);
        if (it == id_map_after.end()) {
            continue;
        }

        cnter++;

        // Map points timing
        igl::Timer map_timer;
        map_timer.start();

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
    if (verbose == true && cnter > 0) {
        std::cout << "count: " << cnter << std::endl;
        std::cout << "handle_collapse_edge_curve cache time: " << time_cache << " ms" << std::endl;
        std::cout << "handle_collapse_edge_curve map points time: " << time_map_points_total
                  << " ms" << std::endl;
        std::cout << "handle_collapse_edge_curve map points time per segment: "
                  << time_map_points_total / cnter << " ms" << std::endl;
        std::cout << "handle_collapse_edge_curve trace segment time: " << time_trace_segment_total
                  << " ms" << std::endl;
        std::cout << "handle_collapse_edge_curve trace segment time per segment: "
                  << time_trace_segment_total / cnter << " ms" << std::endl;
    }
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


            // handle zero slope case
            if (current_slope_r.norm() == 0) {
                current_slope_r = next_slope_r;
            }

            if (!is_collinear) {
                // std::cout << "Collinearity check failed for segment " << current_id << " and "
                //           << next_id << std::endl;
                // std::cout << "  current_slope_r: [" << current_slope_r(0).to_double() << ", "
                //           << current_slope_r(1).to_double() << "]" << std::endl;
                // std::cout << "  next_slope_r: [" << next_slope_r(0).to_double() << ", "
                //           << next_slope_r(1).to_double() << "]" << std::endl;
                // std::cout << "  cross_product_2d: " << std::fixed << std::setprecision(16)
                //           << cross_product_2d.to_double() << std::endl;

                break; // not collinear, stop merging
            }

            segments_to_merge.push_back(next_id);
            next_id = curve.next_segment_ids[next_id];
        }

        // for debug
        {
            if (segments_to_merge.size() > 1) {
                std::cout << "Merging segments: ";
                for (int idx : segments_to_merge) {
                    const auto& seg = curve.segments[idx];
                    std::cout << "\n  seg id: " << idx << ", f_id: " << seg.f_id
                              << ", origin_segment_id: " << seg.origin_segment_id << ", bcs[0]: [";
                    if constexpr (std::is_same_v<CoordType, double>) {
                        std::cout << std::fixed << std::setprecision(16) << seg.bcs[0].transpose();
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << std::fixed << std::setprecision(16)
                                  << seg.bcs[0](0).to_double() << ", " << seg.bcs[0](1).to_double()
                                  << ", " << seg.bcs[0](2).to_double();
                    }
                    std::cout << "]" << ", bcs[1]: [";
                    if constexpr (std::is_same_v<CoordType, double>) {
                        std::cout << std::fixed << std::setprecision(16) << seg.bcs[1].transpose();
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << std::fixed << std::setprecision(16)
                                  << seg.bcs[1](0).to_double() << ", " << seg.bcs[1](1).to_double()
                                  << ", " << seg.bcs[1](2).to_double();
                    }
                    std::cout << "]" << ", fv_ids: [";
                    for (int k = 0; k < seg.fv_ids.size(); ++k) {
                        std::cout << seg.fv_ids[k];
                        if (k + 1 < seg.fv_ids.size()) std::cout << ", ";
                    }
                    std::cout << "]";
                }
                std::cout << std::endl;
            }
        }
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
                if constexpr (std::is_same_v<CoordType, double>) {
                    std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose() << std::endl;
                    std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose() << std::endl;
                    std::cout << "bc_diff: " << bc_diff.transpose() << std::endl;
                } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                    std::cout << "cur_seg.bcs[1]: [";
                    for (int k = 0; k < cur_seg.bcs[1].size(); k++) {
                        std::cout << cur_seg.bcs[1](k).to_double();
                        if (k < cur_seg.bcs[1].size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;
                    std::cout << "next_seg.bcs[0]: [";
                    for (int k = 0; k < next_seg.bcs[0].size(); k++) {
                        std::cout << next_seg.bcs[0](k).to_double();
                        if (k < next_seg.bcs[0].size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;
                    std::cout << "bc_diff: [";
                    for (int k = 0; k < bc_diff.size(); k++) {
                        std::cout << bc_diff(k).to_double();
                        if (k < bc_diff.size() - 1) std::cout << ", ";
                    }
                    std::cout << "]" << std::endl;
                }
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
                        std::cout << "cur_seg.f_id: " << cur_seg.f_id << std::endl;
                        std::cout << "cur_seg.fv_ids: [";
                        for (int k = 0; k < cur_seg.fv_ids.size(); k++) {
                            std::cout << cur_seg.fv_ids[k];
                            if (k < cur_seg.fv_ids.size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        if constexpr (std::is_same_v<CoordType, double>) {
                            std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose()
                                      << std::endl;
                        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                            std::cout << "cur_seg.bcs[1]: [";
                            for (int k = 0; k < cur_seg.bcs[1].size(); k++) {
                                std::cout << cur_seg.bcs[1](k).to_double();
                                if (k < cur_seg.bcs[1].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                        }
                        std::cout << "next_seg.f_id: " << next_seg.f_id << std::endl;
                        std::cout << "next_seg.fv_ids: [";
                        for (int k = 0; k < next_seg.fv_ids.size(); k++) {
                            std::cout << next_seg.fv_ids[k];
                            if (k < next_seg.fv_ids.size() - 1) std::cout << ", ";
                        }
                        std::cout << "]" << std::endl;
                        if constexpr (std::is_same_v<CoordType, double>) {
                            std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose()
                                      << std::endl;
                        } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                            std::cout << "next_seg.bcs[0]: [";
                            for (int k = 0; k < next_seg.bcs[0].size(); k++) {
                                std::cout << next_seg.bcs[0](k).to_double();
                                if (k < next_seg.bcs[0].size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                        }
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

                            std::cout << "cur_seg.fv_ids: [";
                            for (int k = 0; k < cur_seg.fv_ids.size(); k++) {
                                std::cout << cur_seg.fv_ids[k];
                                if (k < cur_seg.fv_ids.size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            if constexpr (std::is_same_v<CoordType, double>) {
                                std::cout << "diff:" << diff << std::endl;
                                std::cout << "cur_seg.bcs[1]: " << cur_seg.bcs[1].transpose()
                                          << std::endl;
                            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                                std::cout << "diff:" << diff.to_double() << std::endl;
                                std::cout << "cur_seg.bcs[1]: [";
                                for (int k = 0; k < cur_seg.bcs[1].size(); k++) {
                                    std::cout << cur_seg.bcs[1](k).to_double();
                                    if (k < cur_seg.bcs[1].size() - 1) std::cout << ", ";
                                }
                                std::cout << "]" << std::endl;
                            }
                            std::cout << "next_seg.fv_ids: [";
                            for (int k = 0; k < next_seg.fv_ids.size(); k++) {
                                std::cout << next_seg.fv_ids[k];
                                if (k < next_seg.fv_ids.size() - 1) std::cout << ", ";
                            }
                            std::cout << "]" << std::endl;
                            if constexpr (std::is_same_v<CoordType, double>) {
                                std::cout << "next_seg.bcs[0]: " << next_seg.bcs[0].transpose()
                                          << std::endl;
                            } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                                std::cout << "next_seg.bcs[0]: [";
                                for (int k = 0; k < next_seg.bcs[0].size(); k++) {
                                    std::cout << next_seg.bcs[0](k).to_double();
                                    if (k < next_seg.bcs[0].size() - 1) std::cout << ", ";
                                }
                                std::cout << "]" << std::endl;
                            }
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


// function that computes all the intersections between two curves
template <typename CoordType>
int compute_intersections_between_two_curves_t(
    const query_curve_t<CoordType>& curve1,
    const query_curve_t<CoordType>& curve2,
    bool verbose)
{
    int intersections = 0;

    std::set<std::pair<int, int>> skip_seg_pairs;

    for (int seg1_id = 0; seg1_id < curve1.segments.size(); seg1_id++) {
        for (int seg2_id = 0; seg2_id < curve2.segments.size(); seg2_id++) {
            auto seg1 = curve1.segments[seg1_id];
            auto seg2 = curve2.segments[seg2_id];

            if (skip_seg_pairs.find(std::make_pair(seg1_id, seg2_id)) != skip_seg_pairs.end()) {
                continue;
            }

            if (seg1.f_id != seg2.f_id) {
                continue;
            }

            Eigen::Vector2<wmtk::Rational> s1_a, s1_b, s2_a, s2_b;
            s1_a << wmtk::Rational(seg1.bcs[0](0)), wmtk::Rational(seg1.bcs[0](1));
            s1_b << wmtk::Rational(seg1.bcs[1](0)), wmtk::Rational(seg1.bcs[1](1));
            s2_a << wmtk::Rational(seg2.bcs[0](0)), wmtk::Rational(seg2.bcs[0](1));
            s2_b << wmtk::Rational(seg2.bcs[1](0)), wmtk::Rational(seg2.bcs[1](1));

            Eigen::Vector2<wmtk::Rational> bc_on_seg2;
            Eigen::Vector2<wmtk::Rational> bc_on_seg1;
            if (intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_on_seg2, bc_on_seg1, false)) {
                // handle intersection on endpoints
                if (bc_on_seg2(0) == wmtk::Rational(0) || bc_on_seg2(0) == wmtk::Rational(1) ||
                    bc_on_seg1(0) == wmtk::Rational(0) || bc_on_seg1(0) == wmtk::Rational(1)) {
                    int seg1_nei = -1;
                    int seg2_nei = -1;
                    if (bc_on_seg2(0) == wmtk::Rational(1)) {
                        seg2_nei = std::find(
                                       curve2.next_segment_ids.begin(),
                                       curve2.next_segment_ids.end(),
                                       seg2_id) -
                                   curve2.next_segment_ids.begin();
                    } else if (bc_on_seg2(0) == wmtk::Rational(0)) {
                        seg2_nei = curve2.next_segment_ids[seg2_id];
                    }


                    if (bc_on_seg1(0) == wmtk::Rational(1)) {
                        seg1_nei = std::find(
                                       curve1.next_segment_ids.begin(),
                                       curve1.next_segment_ids.end(),
                                       seg1_id) -
                                   curve1.next_segment_ids.begin();
                    } else if (bc_on_seg1(0) == wmtk::Rational(0)) {
                        seg1_nei = curve1.next_segment_ids[seg1_id];
                    }
                    if (seg2_nei != -1) {
                        skip_seg_pairs.insert(std::make_pair(seg1_id, seg2_nei));
                        std::cout << "add pair: " << seg1_id << ", " << seg2_nei << std::endl;
                    }
                    if (seg1_nei != -1) {
                        skip_seg_pairs.insert(std::make_pair(seg1_nei, seg2_id));
                        skip_seg_pairs.insert(std::make_pair(seg1_nei, seg2_nei));
                        std::cout << "add pair: " << seg1_nei << ", " << seg2_nei << std::endl;
                        std::cout << "add pair: " << seg1_nei << ", " << seg2_id << std::endl;
                    }
                }

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
                        std::cout << "intersection position at t = " << bc_on_seg2(0).to_double()
                                  << std::endl;
                    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
                        std::cout << "seg1_id: " << seg1_id << ", seg2_id: " << seg2_id
                                  << std::endl;
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
                        std::cout << "intersection position at t = " << bc_on_seg2(0).to_double()
                                  << std::endl;
                        std::cout << std::endl;
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

            Eigen::Vector2<wmtk::Rational> bc_tmp, bc_tmp_seg1;
            if (intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, bc_tmp_seg1, false)) {
                // intersectSegmentEdge_r(s1_a, s1_b, s2_a, s2_b, bc_tmp, bc_tmp_seg1, true);
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
