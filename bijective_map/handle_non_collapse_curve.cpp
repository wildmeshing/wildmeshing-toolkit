
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <set>
#include "track_operations_curve.hpp"
#ifdef USE_IGL_VIEWER
#include <igl/opengl/glfw/Viewer.h>
#endif

// Template version of handle_non_collapse_operation_curves
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
    if (verbose) {
        std::cout << "Handling " << operation_name << " curve" << std::endl;
    }

    for (auto& curve : curves) {
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
                        std::cout << "    bcs: [" << query_points[i].bc.transpose() << "]"
                                  << std::endl;
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
}


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
    std::vector<query_curve_t<CoordType>> curves{curve};
    handle_non_collapse_operation_curves_t(
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

    // Copy the modified curve back to the original parameter
    if (!curves.empty()) {
        curve = curves[0];
    }
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