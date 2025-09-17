
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <igl/parallel_for.h>
#include <set>
#include "track_operations_curve.hpp"
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
    bool verbose)
{
    if constexpr (std::is_same_v<CoordType, double>) {
        igl::parallel_for(curves.size(), [&](int i) {
            handle_collapse_edge_curve_t(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                curves[i],
                use_rational,
                verbose);
        });
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        handle_collapse_edge_curves_fast_rational(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            curves,
            verbose);
    }
}

void handle_collapse_edge_curves_fast_rational(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    bool verbose)
{
    // TODO: Implement optimized rational version
    // For now, just call the regular version for each curve
    for (auto& curve : curves) {
        handle_collapse_edge_curve_t(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            curve,
            true, // use_rational
            verbose);
    }
}


template void handle_collapse_edge_curves_t<double>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_curve_t<double>>& curves,
    bool use_rational,
    bool verbose);
template void handle_collapse_edge_curves_t<wmtk::Rational>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    bool use_rational,
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