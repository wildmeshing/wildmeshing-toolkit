#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "track_operations.hpp"
#include "track_operations_curve.hpp"

#include <wmtk/utils/orient.hpp>


// use 2d predicate to check if the point is in the triangle or on the edge
bool is_point_in_triangle_exact_predicate(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c,
    Eigen::Vector3d& barycentric,
    bool verbose = false)
{
    int res_abp = wmtk::utils::wmtk_orient2d(a, b, p);
    int res_bcp = wmtk::utils::wmtk_orient2d(b, c, p);
    int res_cap = wmtk::utils::wmtk_orient2d(c, a, p);

    if ((res_abp >= 0 && res_bcp >= 0 && res_cap >= 0) ||
        (res_abp <= 0 && res_bcp <= 0 && res_cap <= 0)) {
        if (res_abp == 0) {
            if (verbose) std::cout << "point is on edge ab" << std::endl;
            if (a(0) == b(0)) {
                double t = (p(1) - a(1)) / (b(1) - a(1));
                barycentric = Eigen::Vector3d(1 - t, t, 0);
            } else {
                double t = (p(0) - a(0)) / (b(0) - a(0));
                barycentric = Eigen::Vector3d(1 - t, t, 0);
            }
        } else if (res_bcp == 0) {
            if (verbose) std::cout << "point is on edge bc" << std::endl;
            if (b(0) == c(0)) {
                double t = (p(1) - b(1)) / (c(1) - b(1));
                barycentric = Eigen::Vector3d(0, 1 - t, t);
            } else {
                double t = (p(0) - b(0)) / (c(0) - b(0));
                barycentric = Eigen::Vector3d(0, 1 - t, t);
            }
        } else if (res_cap == 0) {
            if (verbose) std::cout << "point is on edge ca" << std::endl;
            if (c(0) == a(0)) {
                double t = (p(1) - c(1)) / (a(1) - c(1));
                barycentric = Eigen::Vector3d(t, 0, 1 - t);
            } else {
                double t = (p(0) - c(0)) / (a(0) - c(0));
                barycentric = Eigen::Vector3d(t, 0, 1 - t);
            }
        } else {
            if (verbose) std::cout << "point is inside the triangle" << std::endl;
            barycentric = ComputeBarycentricCoordinates2D(p, a, b, c);
        }
        return true;
    }
    if (verbose) std::cout << "point is not in the triangle" << std::endl;
    return false;
}

void handle_collapse_edge_use_exact_predicate(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points)
{
    // TODO: We want to a exact one with exact predicate check in double arithmetic

    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it == id_map_after.end()) continue;

        int local_index_in_f_after = std::distance(id_map_after.begin(), it);

        int offset_in_f_after = -1;
        for (int i = 0; i < 3; i++) {
            if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                offset_in_f_after = i;
                break;
            }
        }
        if (offset_in_f_after == -1) {
            throw std::runtime_error("FATAL ERROR in handle_collapse_edge_use_exact_predicate: "
                                     "Failed to find vertex ID mapping.");
        }


        // TODO: handle points that are on the edge
        /*
                {
                    int v0 = -1, v1 = -1;
                    int zero_index = -1;
                    for (int i = 0; i < 3; i++) {
                        if (qp.bc(i) == 0) {
                            v0 = F_after(local_index_in_f_after, (offset_in_f_after + i + 1) % 3);
                            v1 = F_after(local_index_in_f_after, (offset_in_f_after + i + 2) % 3);
                            zero_index = i;
                            break;
                        }
                    }

                    if (!(v0 == -1 || v1 == -1 || v0 == 0 || v1 == 0 || v0 == v_id_map_joint.size()
           - 1 || v1 == v_id_map_joint.size() - 1)) { bool bc_updated = false;

                        std::cout << "try to find edge " << v0 << "-" << v1 << std::endl;
                        // try to find the edge in F_before
                        for (int i = 0; i < F_before.rows(); i++) {
                            for (int j = 0; j < 3; j++) {
                                if (F_before(i, j) == v0 && F_before(i, (j + 1) % 3) == v1) {
                                    std::cout << "find edge " << v0 << "-" << v1 << " in triangle "
           << i
                                              << std::endl;
                                    std::cout << "qp before: " << qp << std::endl;
                                    auto bc = qp.bc;
                                    qp.bc(j) = bc((zero_index + 1) % 3);
                                    qp.bc((j + 1) % 3) = bc((zero_index + 2) % 3);
                                    qp.bc((j + 2) % 3) = 0;
                                    qp.f_id = id_map_before[i];
                                    qp.fv_ids << v_id_map_joint[F_before(i, 0)],
                                        v_id_map_joint[F_before(i, 1)], v_id_map_joint[F_before(i,
           2)]; std::cout << "qp after: " << qp << std::endl; bc_updated = true; break;
                                }
                            }
                            if (bc_updated) {
                                break;
                            }
                        }

                        if (bc_updated) {
                            continue;
                        }
                    }
                }

        */
        // actual copute part of the barycentric coordinates
        Eigen::Vector2d p(0, 0);
        for (int i = 0; i < 3; i++) {
            p += UV_joint.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3)) *
                 qp.bc(i);
        }

        bool bc_updated = false;
        int local_index_in_f_before = -1;
        Eigen::Vector3d barycentric;
        for (int i = 0; i < F_before.rows(); i++) {
            if (is_point_in_triangle_exact_predicate(
                    p,
                    UV_joint.row(F_before(i, 0)),
                    UV_joint.row(F_before(i, 1)),
                    UV_joint.row(F_before(i, 2)),
                    barycentric,
                    false)) {
                bc_updated = true;
                local_index_in_f_before = i;
                break;
            }
        }
        if (!bc_updated) {
            std::cout << "cant find the point in the triangle" << std::endl;
            std::cout << qp << std::endl;
            throw std::runtime_error("FATAL ERROR in handle_collapse_edge_use_exact_predicate: "
                                     "Failed to update barycentric coordinates.");
        }

        qp.f_id = id_map_before[local_index_in_f_before];
        for (int i = 0; i < 3; i++) {
            qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
        }
        qp.bc = barycentric;
        qp.bc /= qp.bc.sum();


        if (qp.bc.minCoeff() < 0 || qp.bc.maxCoeff() > 1) {
            throw std::runtime_error("FATAL ERROR in handle_collapse_edge_use_exact_predicate: "
                                     "Barycentric coordinates out of range.");
        }
    }
}


void handle_collapse_edge(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points,
    bool use_rational)
{
    if (use_rational) {
        handle_collapse_edge_r(
            UV_joint,
            F_before,
            F_after,
            v_id_map_joint,
            id_map_before,
            id_map_after,
            query_points);
        return;
    }
    // std::cout << "Handling EdgeCollapse" << std::endl;
    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        // find if qp is in the id_map_after
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // std::cout << "find qp: " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;

            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg
                    << "FATAL ERROR in handle_collapse_edge: Failed to find vertex ID mapping.\n";
                error_msg << "Query point vertex ID " << qp.fv_ids[0]
                          << " not found in F_after triangle.\n";
                error_msg << "F_after triangle vertices: ";
                for (int i = 0; i < 3; i++) {
                    error_msg << v_id_map_joint[F_after(local_index_in_f_after, i)];
                    if (i < 2) error_msg << ", ";
                }
                error_msg << "\nQuery point vertex IDs: ";
                for (int i = 0; i < 3; i++) {
                    error_msg << qp.fv_ids[i];
                    if (i < 2) error_msg << ", ";
                }
                error_msg << "\nThis indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
            }


            // compute the location of the qp
            Eigen::Vector2d p(0, 0);
            for (int i = 0; i < 3; i++) {
                p += UV_joint.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3)) *
                     qp.bc(i);
            }

            // std::cout << "p:\n" << p << std::endl;

            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;
            double bc_min_coef = 1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Vector3d bc;

                bc = ComputeBarycentricCoordinates2D(
                    p,
                    UV_joint.row(F_before(i, 0)),
                    UV_joint.row(F_before(i, 1)),
                    UV_joint.row(F_before(i, 2)));

                // std::cout << "bc candidate:" << bc << std::endl;
                // std::cout << "check with p:\n"
                //           << bc(0) * UV_joint.row(F_before(i, 0)) +
                //                  bc(1) * UV_joint.row(F_before(i, 1)) +
                //                  bc(2) * UV_joint.row(F_before(i, 2))
                //           << std::endl;
                if (-bc.minCoeff() < bc_min_coef) {
                    // std::cout << "good!" << std::endl;
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in handle_collapse_edge: Failed to update barycentric "
                             "coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
            }
            // else {
            //     std::cout << "bc updated\n" << std::endl;
            // }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
            }
            // avoid numerical issue
            qp.bc[0] = std::max(0.0, std::min(1.0, qp.bc[0]));
            qp.bc[1] = std::max(0.0, std::min(1.0, qp.bc[1]));
            qp.bc[2] = std::max(0.0, std::min(1.0, qp.bc[2]));
            qp.bc /= qp.bc.sum();

            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;
        }
    }
}

// Rational version of handle_collapse_edge
void handle_collapse_edge_r(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point>& query_points,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    struct SimpleTimer
    {
        std::chrono::high_resolution_clock::time_point t;
        void start() { t = std::chrono::high_resolution_clock::now(); }
        void stop() {}
        double getElapsedTime() const
        {
            return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - t)
                .count();
        }
    };

    SimpleTimer total_timer;
    total_timer.start();
    // first convert the UV_joint to Rational
    Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
    SimpleTimer convert_timer;
    convert_timer.start();
    for (int i = 0; i < UV_joint.rows(); i++) {
        for (int j = 0; j < UV_joint.cols(); j++) {
            UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
        }
    }
    double t_convert_ms = convert_timer.getElapsedTime() * 1000.0;

    // Precompute barycentric cache for F_before using UV_joint_r (2D) if not provided
    std::vector<BarycentricPrecompute2D> local_cache;
    const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
    if (bc_cache_ptr == nullptr) {
        std::cout << "???Building barycentric cache for F_before" << std::endl;
        local_cache = build_barycentric_cache_2d(UV_joint_r, F_before);
        bc_cache_ptr = &local_cache;
    }

    // Profiling accumulators
    double t_find_in_after_ms = 0.0;
    double t_find_offset_ms = 0.0;
    double t_compute_point_ms = 0.0;
    double t_bc_search_ms = 0.0;
    double t_bc_call_ms_total = 0.0;
    long long bc_call_count = 0;
    int num_processed_points = 0;

    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        // find if qp is in the id_map_after
        SimpleTimer t_find_after;
        t_find_after.start();
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        t_find_in_after_ms += t_find_after.getElapsedTime() * 1000.0;
        if (it != id_map_after.end()) {
            num_processed_points++;
            // std::cout << "find qp: " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl;
            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            {
                SimpleTimer t_offset;
                t_offset.start();
                for (int i = 0; i < 3; i++) {
                    if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                        offset_in_f_after = i;
                        break;
                    }
                }
                t_find_offset_ms += t_offset.getElapsedTime() * 1000.0;
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg
                    << "FATAL ERROR in handle_collapse_edge_r: Failed to find vertex ID mapping.\n";
                error_msg << "Query point vertex IDs: " << qp.fv_ids[0] << ", " << qp.fv_ids[1]
                          << ", " << qp.fv_ids[2] << "\n";
                error_msg << "Local index in F_after: " << local_index_in_f_after << "\n";
                error_msg << "This indicates a serious numerical or topological error in rational "
                             "computation.";
                throw std::runtime_error(error_msg.str());
            }

            // First convert barycentric coordinates to rational and normalize
            Eigen::Vector3<wmtk::Rational> bc_rational(
                wmtk::Rational(qp.bc(0)),
                wmtk::Rational(qp.bc(1)),
                wmtk::Rational(qp.bc(2)));
            bc_rational /= bc_rational.sum();
            // compute the location of the qp
            Eigen::Vector2<wmtk::Rational> p(0, 0);
            {
                SimpleTimer t_compute_p;
                t_compute_p.start();
                for (int i = 0; i < 3; i++) {
                    p = p +
                        UV_joint_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            bc_rational(i);
                }
                t_compute_point_ms += t_compute_p.getElapsedTime() * 1000.0;
            }
            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;

            // if computation is exact we don't need this anymore
            // double bc_min_coef = 1;
            bool bc_updated = false;
            {
                SimpleTimer t_bc_search;
                t_bc_search.start();
                for (int i = 0; i < F_before.rows(); i++) {
                    Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational;

                    SimpleTimer t_bc_call;
                    t_bc_call.start();
                    bc_rational = barycentric_from_cache((*bc_cache_ptr)[i], p);
                    t_bc_call_ms_total += t_bc_call.getElapsedTime() * 1000.0;
                    bc_call_count++;


                    if (bc_rational.minCoeff() >= 0 && bc_rational.maxCoeff() <= 1) {
                        local_index_in_f_before = i;
                        qp.bc(0) = bc_rational(0).to_double();
                        qp.bc(1) = bc_rational(1).to_double();
                        qp.bc(2) = bc_rational(2).to_double();
                        bc_updated = true;
                        break;
                    }
                }
                t_bc_search_ms += t_bc_search.getElapsedTime() * 1000.0;
            }

            if (!bc_updated) {
                std::cout << "Input barycentric coordinates (bc): ";
                std::cout << std::setprecision(16) << qp.bc(0) << ", " << qp.bc(1) << ", "
                          << qp.bc(2) << std::endl;
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational computation: Failed to update barycentric "
                             "coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error in exact "
                             "arithmetic.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
            }
        }
    }

    // Suppress detailed timing output to reduce noise
    // std::cout << "F_before.rows(): " << F_before.rows() << std::endl;
    // std::cout << "handle_collapse_edge_r barycentric call time: " << t_bc_call_ms_total << " ms"
    //           << std::endl;
    // std::cout << "handle_collapse_edge_r barycentric calls: " << bc_call_count << std::endl;
    // std::cout << "handle_collapse_edge_r barycentric search time: " << t_bc_search_ms << " ms"
    //           << std::endl;
}

// Template version of handle_collapse_edge
template <typename CoordType>
void handle_collapse_edge_t(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<CoordType>>& query_points,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    bool profile_verbose = false;

    if constexpr (std::is_same_v<CoordType, double>) {
        if (use_rational) {
            handle_collapse_edge_r(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                query_points,
                barycentric_cache);
        } else {
            handle_collapse_edge(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                query_points,
                false);
        }
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        // For rational query points, always use rational processing (ignore use_rational parameter)

        auto time_rational_start = std::chrono::high_resolution_clock::now();

        // Convert UV_joint to rational
        auto time_conversion_start = std::chrono::high_resolution_clock::now();
        Eigen::MatrixX<wmtk::Rational> UV_joint_r(UV_joint.rows(), UV_joint.cols());
        for (int i = 0; i < UV_joint.rows(); i++) {
            for (int j = 0; j < UV_joint.cols(); j++) {
                UV_joint_r(i, j) = wmtk::Rational(UV_joint(i, j));
            }
        }
        auto time_conversion_end = std::chrono::high_resolution_clock::now();
        auto conversion_time =
            std::chrono::duration<double, std::milli>(time_conversion_end - time_conversion_start);

        if (profile_verbose) {
            std::cout << "[PROFILE] Rational UV_joint conversion time: " << conversion_time.count()
                      << " ms" << std::endl;
        }

        // Precompute barycentric cache for F_before (2D) if not provided
        auto time_cache_start = std::chrono::high_resolution_clock::now();
        std::vector<BarycentricPrecompute2D> local_cache;
        const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
        if (bc_cache_ptr == nullptr) {
            local_cache = build_barycentric_cache_2d(UV_joint_r, F_before);
            bc_cache_ptr = &local_cache;
        }
        auto time_cache_end = std::chrono::high_resolution_clock::now();
        auto cache_time =
            std::chrono::duration<double, std::milli>(time_cache_end - time_cache_start);
        if (profile_verbose) {
            std::cout << "[PROFILE] Barycentric cache build time: " << cache_time.count() << " ms"
                      << std::endl;
        }
        auto time_loop_start = std::chrono::high_resolution_clock::now();
        double total_mapping_time = 0.0;
        double total_position_computation_time = 0.0;
        double total_barycentric_search_time = 0.0;
        double total_normalization_time = 0.0;
        int processed_points = 0;

        for (int id = 0; id < query_points.size(); id++) {
            query_point_t<CoordType>& qp = query_points[id];
            if (qp.f_id < 0) continue;

            auto time_point_start = std::chrono::high_resolution_clock::now();

            // find if qp is in the id_map_after
            auto time_mapping_start = std::chrono::high_resolution_clock::now();
            auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
            if (it != id_map_after.end()) {
                // find the index of qp in id_map_after
                int local_index_in_f_after = std::distance(id_map_after.begin(), it);
                auto time_mapping_end = std::chrono::high_resolution_clock::now();
                total_mapping_time +=
                    std::chrono::duration<double, std::milli>(time_mapping_end - time_mapping_start)
                        .count();

                // offset of the qp.fv_ids
                int offset_in_f_after = -1;
                for (int i = 0; i < 3; i++) {
                    if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                        offset_in_f_after = i;
                        break;
                    }
                }
                if (offset_in_f_after == -1) {
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template edge collapse"
                              << ": Failed to find vertex ID mapping.\n";
                    error_msg << "Query point face ID: " << qp.f_id << "\n";
                    error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                    error_msg << "This indicates a serious numerical or topological error in "
                                 "rational "
                                 "template edge collapse computation.";
                    throw std::runtime_error(error_msg.str());
                }

                // Normalize barycentric coordinates
                auto time_norm1_start = std::chrono::high_resolution_clock::now();
                wmtk::Rational bc_sum = qp.bc.sum();
                if (bc_sum != wmtk::Rational(0)) {
                    qp.bc /= bc_sum;
                }
                auto time_norm1_end = std::chrono::high_resolution_clock::now();
                total_normalization_time +=
                    std::chrono::duration<double, std::milli>(time_norm1_end - time_norm1_start)
                        .count();

                // compute the location of the qp using rational arithmetic
                auto time_pos_start = std::chrono::high_resolution_clock::now();
                Eigen::Vector2<wmtk::Rational> p = Eigen::Vector2<wmtk::Rational>::Zero();
                for (int i = 0; i < 3; i++) {
                    p = p +
                        UV_joint_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .head<2>()
                                .transpose() *
                            qp.bc(i);
                }
                auto time_pos_end = std::chrono::high_resolution_clock::now();
                total_position_computation_time +=
                    std::chrono::duration<double, std::milli>(time_pos_end - time_pos_start)
                        .count();

                // compute bc of the p in F_before using rational arithmetic
                auto time_bary_search_start = std::chrono::high_resolution_clock::now();
                int local_index_in_f_before = -1;
                bool bc_updated = false;

                for (int i = 0; i < F_before.rows(); i++) {
                    Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result =
                        barycentric_from_cache((*bc_cache_ptr)[i], p);

                    // Check if point is inside triangle (all barycentric coordinates >= 0)
                    // TODO: this could take time
                    if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                        local_index_in_f_before = i;
                        qp.bc = bc_rational_result;
                        bc_updated = true;
                        break;
                    }
                }
                auto time_bary_search_end = std::chrono::high_resolution_clock::now();
                total_barycentric_search_time += std::chrono::duration<double, std::milli>(
                                                     time_bary_search_end - time_bary_search_start)
                                                     .count();

                if (!bc_updated) {
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template edge collapse"
                              << ": Failed to update barycentric coordinates.\n";
                    error_msg << "Query point could not be located in any triangle after edge "
                                 "collapse.\n";
                    error_msg << "This indicates a serious numerical or topological error in exact "
                                 "template edge collapse arithmetic.";
                    throw std::runtime_error(error_msg.str());
                }

                // update qp
                qp.f_id = id_map_before[local_index_in_f_before];
                for (int i = 0; i < 3; i++) {
                    qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
                }

                // Normalize barycentric coordinates
                auto time_norm2_start = std::chrono::high_resolution_clock::now();
                wmtk::Rational bc_sum_final = qp.bc.sum();
                if (bc_sum_final != wmtk::Rational(0)) {
                    qp.bc /= bc_sum_final;
                }
                auto time_norm2_end = std::chrono::high_resolution_clock::now();
                total_normalization_time +=
                    std::chrono::duration<double, std::milli>(time_norm2_end - time_norm2_start)
                        .count();

                processed_points++;
            }
        }

        auto time_loop_end = std::chrono::high_resolution_clock::now();
        auto total_loop_time =
            std::chrono::duration<double, std::milli>(time_loop_end - time_loop_start);
        auto total_rational_time =
            std::chrono::duration<double, std::milli>(time_loop_end - time_rational_start);

        // Print detailed profiling results
        if (profile_verbose) {
            std::cout << "\n[PROFILE] Rational branch detailed timing breakdown:" << std::endl;
            std::cout << "[PROFILE] - Total rational processing time: "
                      << total_rational_time.count() << " ms" << std::endl;
            std::cout << "[PROFILE] - Main loop time: " << total_loop_time.count() << " ms"
                      << std::endl;
            std::cout << "[PROFILE] - Processed points: " << processed_points << std::endl;
            if (processed_points > 0) {
                std::cout << "[PROFILE] - Average time per point: "
                          << (total_loop_time.count() / processed_points) << " ms" << std::endl;
                std::cout << "[PROFILE] - ID mapping time: " << total_mapping_time << " ms ("
                          << (total_mapping_time / total_loop_time.count() * 100) << "%)"
                          << std::endl;
                std::cout << "[PROFILE] - Position computation time: "
                          << total_position_computation_time << " ms ("
                          << (total_position_computation_time / total_loop_time.count() * 100)
                          << "%)" << std::endl;
                std::cout << "[PROFILE] - Barycentric search time: "
                          << total_barycentric_search_time << " ms ("
                          << (total_barycentric_search_time / total_loop_time.count() * 100) << "%)"
                          << std::endl;
                std::cout << "[PROFILE] - Normalization time: " << total_normalization_time
                          << " ms (" << (total_normalization_time / total_loop_time.count() * 100)
                          << "%)" << std::endl;
            }
        }
    } else {
        static_assert(
            std::is_same_v<CoordType, double> || std::is_same_v<CoordType, wmtk::Rational>,
            "CoordType must be either double or wmtk::Rational");
    }
}


void handle_collapse_edge_rational(
    const Eigen::MatrixX<wmtk::Rational>& UV_joint_r,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    bool profile_verbose = false;
    // For rational query points, always use rational processing (ignore use_rational parameter)

    auto time_rational_start = std::chrono::high_resolution_clock::now();

    // Precompute barycentric cache for F_before (2D) if not provided
    auto time_cache_start = std::chrono::high_resolution_clock::now();
    std::vector<BarycentricPrecompute2D> local_cache;
    const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
    if (bc_cache_ptr == nullptr) {
        local_cache = build_barycentric_cache_2d(UV_joint_r, F_before);
        bc_cache_ptr = &local_cache;
    }
    auto time_cache_end = std::chrono::high_resolution_clock::now();
    auto cache_time = std::chrono::duration<double, std::milli>(time_cache_end - time_cache_start);
    if (profile_verbose) {
        std::cout << "[PROFILE] Barycentric cache build time: " << cache_time.count() << " ms"
                  << std::endl;
    }
    auto time_loop_start = std::chrono::high_resolution_clock::now();
    double total_mapping_time = 0.0;
    double total_position_computation_time = 0.0;
    double total_barycentric_search_time = 0.0;
    double total_normalization_time = 0.0;
    int processed_points = 0;

    for (int id = 0; id < query_points.size(); id++) {
        query_point_t<wmtk::Rational>& qp = query_points[id];
        if (qp.f_id < 0) continue;

        auto time_point_start = std::chrono::high_resolution_clock::now();

        // find if qp is in the id_map_after
        auto time_mapping_start = std::chrono::high_resolution_clock::now();
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            auto time_mapping_end = std::chrono::high_resolution_clock::now();
            total_mapping_time +=
                std::chrono::duration<double, std::milli>(time_mapping_end - time_mapping_start)
                    .count();

            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_joint[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational template edge collapse"
                          << ": Failed to find vertex ID mapping.\n";
                error_msg << "Query point face ID: " << qp.f_id << "\n";
                error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                error_msg << "This indicates a serious numerical or topological error in "
                             "rational "
                             "template edge collapse computation.";
                throw std::runtime_error(error_msg.str());
            }

            // Normalize barycentric coordinates
            auto time_norm1_start = std::chrono::high_resolution_clock::now();
            wmtk::Rational bc_sum = qp.bc.sum();
            if (bc_sum != wmtk::Rational(0)) {
                qp.bc /= bc_sum;
            }
            auto time_norm1_end = std::chrono::high_resolution_clock::now();
            total_normalization_time +=
                std::chrono::duration<double, std::milli>(time_norm1_end - time_norm1_start)
                    .count();

            // compute the location of the qp using rational arithmetic
            auto time_pos_start = std::chrono::high_resolution_clock::now();
            Eigen::Vector2<wmtk::Rational> p = Eigen::Vector2<wmtk::Rational>::Zero();
            for (int i = 0; i < 3; i++) {
                p = p + UV_joint_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .head<2>()
                                .transpose() *
                            qp.bc(i);
            }
            auto time_pos_end = std::chrono::high_resolution_clock::now();
            total_position_computation_time +=
                std::chrono::duration<double, std::milli>(time_pos_end - time_pos_start).count();

            // compute bc of the p in F_before using rational arithmetic
            auto time_bary_search_start = std::chrono::high_resolution_clock::now();
            int local_index_in_f_before = -1;
            bool bc_updated = false;

            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result =
                    barycentric_from_cache((*bc_cache_ptr)[i], p);

                // Check if point is inside triangle (all barycentric coordinates >= 0)
                // TODO: this could take time
                if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                    local_index_in_f_before = i;
                    qp.bc = bc_rational_result;
                    bc_updated = true;
                    break;
                }
            }
            auto time_bary_search_end = std::chrono::high_resolution_clock::now();
            total_barycentric_search_time += std::chrono::duration<double, std::milli>(
                                                 time_bary_search_end - time_bary_search_start)
                                                 .count();

            if (!bc_updated) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational template edge collapse"
                          << ": Failed to update barycentric coordinates.\n";
                error_msg << "Query point could not be located in any triangle after edge "
                             "collapse.\n";
                error_msg << "This indicates a serious numerical or topological error in exact "
                             "template edge collapse arithmetic.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_joint[F_before(local_index_in_f_before, i)];
            }

            // Normalize barycentric coordinates
            auto time_norm2_start = std::chrono::high_resolution_clock::now();
            wmtk::Rational bc_sum_final = qp.bc.sum();
            if (bc_sum_final != wmtk::Rational(0)) {
                qp.bc /= bc_sum_final;
            }
            auto time_norm2_end = std::chrono::high_resolution_clock::now();
            total_normalization_time +=
                std::chrono::duration<double, std::milli>(time_norm2_end - time_norm2_start)
                    .count();

            processed_points++;
        }
    }

    auto time_loop_end = std::chrono::high_resolution_clock::now();
    auto total_loop_time =
        std::chrono::duration<double, std::milli>(time_loop_end - time_loop_start);
    auto total_rational_time =
        std::chrono::duration<double, std::milli>(time_loop_end - time_rational_start);

    // Print detailed profiling results
    if (profile_verbose) {
        std::cout << "\n[PROFILE] Rational branch detailed timing breakdown:" << std::endl;
        std::cout << "[PROFILE] - Total rational processing time: " << total_rational_time.count()
                  << " ms" << std::endl;
        std::cout << "[PROFILE] - Main loop time: " << total_loop_time.count() << " ms"
                  << std::endl;
        std::cout << "[PROFILE] - Processed points: " << processed_points << std::endl;
        if (processed_points > 0) {
            std::cout << "[PROFILE] - Average time per point: "
                      << (total_loop_time.count() / processed_points) << " ms" << std::endl;
            std::cout << "[PROFILE] - ID mapping time: " << total_mapping_time << " ms ("
                      << (total_mapping_time / total_loop_time.count() * 100) << "%)" << std::endl;
            std::cout << "[PROFILE] - Position computation time: "
                      << total_position_computation_time << " ms ("
                      << (total_position_computation_time / total_loop_time.count() * 100) << "%)"
                      << std::endl;
            std::cout << "[PROFILE] - Barycentric search time: " << total_barycentric_search_time
                      << " ms (" << (total_barycentric_search_time / total_loop_time.count() * 100)
                      << "%)" << std::endl;
            std::cout << "[PROFILE] - Normalization time: " << total_normalization_time << " ms ("
                      << (total_normalization_time / total_loop_time.count() * 100) << "%)"
                      << std::endl;
        }
    }
}
// Explicit template instantiations
template void handle_collapse_edge_t<double>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<double>>& query_points,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);

template void handle_collapse_edge_t<wmtk::Rational>(
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_joint,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);