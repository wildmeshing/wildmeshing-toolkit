
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

    // For here we are going to do something different
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

////////////////////////////////////////////////////////////
// Rational fast version of handle_collapse_edge_curve and handle_collapse_edge_curves
////////////////////////////////////////////////////////////

void map_all_query_points_rational_collapse(
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_r>& all_query_points,
    const std::vector<int>& all_query_seg_ids,
    const std::vector<int>& bc0_places,
    const query_curve_t<wmtk::Rational>& curve,
    const std::vector<BarycentricPrecompute2D>& bc_cache_collapse,
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
            v_id_map_joint,
            id_map_before,
            all_query_points,
            bd_qps_ids);
    }

    // map all the non boundary qps
    {
        // use a version that already convert UV_joint
        igl::Timer map_timer;
        map_timer.start();
        handle_collapse_edge_rational(
            UV_joint_r,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            non_bd_qps,
            &bc_cache_collapse);

        double time_map_points_total = map_timer.getElapsedTime() * 1000;
        std::cout << "handle_collapse_edge_rational time: " << time_map_points_total << " ms"
                  << std::endl;
        std::cout << "handle_collapse_edge_rational time per point: "
                  << time_map_points_total / non_bd_qps.size() << " ms" << std::endl;
        for (int i = 0; i < non_bd_qps.size(); i++) {
            all_query_points[non_bd_qps_ids[i]] = non_bd_qps[i];
        }
    }


    // std::cout << "all_query_points after mapping: " << std::endl;
    // for (int i = 0; i < all_query_points.size(); i++) {
    //     std::cout << "  [" << i << "]: " << all_query_points[i] << std::endl;
    // }
}

// This function is called by handle_collapse_edge_curves_fast_rational, only handle one
// curve in rational
void handle_collapse_edge_curve_rational(
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    query_curve_t<wmtk::Rational>& curve,
    const std::vector<BarycentricPrecompute2D>& bc_cache_collapse, // Cache for faster bc compute
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
        map_all_query_points_rational_collapse(
            UV_joint_r,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            all_query_points,
            all_query_seg_ids,
            bc0_places,
            curve,
            bc_cache_collapse,
            all_curve_parts);
    }

    // STEP3: get intersections with mesh (handle one segment)
    const int num_segments_before_mapping = curve.segments.size();
    {
        for (int i = 0; i < all_query_seg_ids.size(); i++) {
            int seg_id = all_query_seg_ids[i];
            std::vector<query_point_r> query_point_for_one_segment;

            query_point_for_one_segment.push_back(all_query_points[bc0_places[i]]);
            query_point_for_one_segment.push_back(all_query_points[bc0_places[i] + 1]);

            igl::Timer trace_timer;
            trace_timer.start();

            Eigen::MatrixXi TT, TTi;

            handle_one_segment_t(
                curve,
                seg_id,
                query_point_for_one_segment,
                UV_joint_r,
                F_before,
                v_id_map_joint,
                id_map_before,
                TT,
                TTi,
                verbose);

            time_trace_segment_total += trace_timer.getElapsedTime() * 1000;
        }
        if (verbose) {
            std::cout << "Total time for tracing segments: " << time_trace_segment_total << " ms"
                      << std::endl;
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
    verbose = false;
    std::cout << "handle collapse edge curves fast rational" << std::endl;


    double convert_UV_to_rational_time = 0.0;
    // convert things to rational
    Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
    {
        igl::Timer convert_timer;
        convert_timer.start();
        for (int i = 0; i < UV_joint.rows(); i++) {
            for (int j = 0; j < UV_joint.cols(); j++) {
                UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
            }
        }
        convert_UV_to_rational_time = convert_timer.getElapsedTime();
    }

    // build cache for bc compute
    double build_bc_cache_time = 0.0;
    std::vector<BarycentricPrecompute2D> bc_cache_collapse =
        build_barycentric_cache_2d(UV_joint_r, F_before);


    std::vector<std::vector<std::vector<int>>> all_curve_parts_after_mapping(curves.size());
    // Implement optimized rational version
    for (int i = 0; i < curves.size(); i++) {
        if (verbose) {
            std::cout << "handle curve " << i << std::endl;
        }
        auto& curve = curves[i];
        handle_collapse_edge_curve_rational(
            UV_joint_r,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            curve,
            bc_cache_collapse,
            all_curve_parts_after_mapping[i],
            verbose);
        if (verbose) {
            std::cout << "handle curve " << i << " done" << std::endl << std::endl;
        }
    }

    // TODO: get all new seg and convert them to double
    {
        // handle everything to double
        for (int curve_id = 0; curve_id < all_curve_parts_after_mapping.size(); curve_id++) {
            for (int part_id = 0; part_id < all_curve_parts_after_mapping[curve_id].size();
                 part_id++) {
                const auto& curve_part = all_curve_parts_after_mapping[curve_id][part_id];
                for (int i = 0; i < curve_part.size() - 1; i++) {
                    int seg_id = curve_part[i];
                    int next_seg_id = curve_part[i + 1];
                    for (int j = 0; j < 3; j++) {
                        curves[curve_id].segments[seg_id].bcs[1](j) =
                            wmtk::Rational(curves[curve_id].segments[seg_id].bcs[1](j).to_double());
                        curves[curve_id].segments[next_seg_id].bcs[0](j) = wmtk::Rational(
                            curves[curve_id].segments[next_seg_id].bcs[0](j).to_double());
                    }
                }
            }
        }
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