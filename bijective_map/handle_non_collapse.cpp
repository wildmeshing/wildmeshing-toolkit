#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "track_operations.hpp"
#include "track_operations_curve.hpp"
void handle_non_collapse_operation(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points,
    const std::string& operation_name,
    bool use_rational)
{
    if (use_rational) {
        handle_non_collapse_operation_r(
            V_before,
            F_before,
            id_map_before,
            v_id_map_before,
            V_after,
            F_after,
            id_map_after,
            v_id_map_after,
            query_points,
            operation_name);
        return;
    }
    // std::cout << "Handling " << operation_name << std::endl;
    // igl::parallel_for(query_points.size(), [&](int id) {
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
                if (v_id_map_after[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in " << operation_name
                          << ": Failed to find vertex ID mapping.\n";
                error_msg << "Query point face ID: " << qp.f_id << "\n";
                error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                error_msg << "This indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
            }

            // compute the location of the qp
            int V_cols = V_after.cols();
            Eigen::VectorXd p = Eigen::VectorXd::Zero(V_cols);
            for (int i = 0; i < 3; i++) {
                p += V_after.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3)) *
                     qp.bc(i);
            }


            // compute bc of the p in (V, F)_before
            int local_index_in_f_before = -1;
            double bc_min_coef = 1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Vector3d bc;
                if (V_cols == 2) {
                    bc = ComputeBarycentricCoordinates2D(
                        p,
                        V_before.row(F_before(i, 0)),
                        V_before.row(F_before(i, 1)),
                        V_before.row(F_before(i, 2)));
                } else // V_cols == 3
                {
                    throw std::runtime_error("FATAL ERROR: V_cols == 3 after local flattening");
                }
                if (-bc.minCoeff() < bc_min_coef) {
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in " << operation_name
                          << ": Failed to update barycentric coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }
            qp.bc[0] = std::max(0.0, std::min(1.0, qp.bc[0]));
            qp.bc[1] = std::max(0.0, std::min(1.0, qp.bc[1]));
            qp.bc[2] = std::max(0.0, std::min(1.0, qp.bc[2]));
            qp.bc /= qp.bc.sum();

            // std::cout << "qp-> " << qp.f_id << std::endl;
            // std::cout << "qp.bc: (" << qp.bc(0) << ", " << qp.bc(1) << ", " << qp.bc(2) << ")"
            //           << std::endl
            //           << std::endl;
        }
    }
    // });
}

// Rational version of handle_non_collapse_operation
void handle_non_collapse_operation_r(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points,
    const std::string& operation_name,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    // std::cout << "Handling " << operation_name << " Rational" << std::endl;

    // Convert V_before and V_after to rational
    Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
    Eigen::MatrixX<wmtk::Rational> V_after_r(V_after.rows(), V_after.cols());

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

    // Precompute barycentric cache for F_before (2D) if not provided
    std::vector<BarycentricPrecompute2D> local_cache;
    const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
    if (bc_cache_ptr == nullptr) {
        local_cache = build_barycentric_cache_2d(V_before_r, F_before);
        bc_cache_ptr = &local_cache;
    }

    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];
        if (qp.f_id < 0) continue;

        // find if qp is in the id_map_after
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
        if (it != id_map_after.end()) {
            // find the index of qp in id_map_after
            int local_index_in_f_after = std::distance(id_map_after.begin(), it);
            // offset of the qp.fv_ids
            int offset_in_f_after = -1;
            for (int i = 0; i < 3; i++) {
                if (v_id_map_after[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                    offset_in_f_after = i;
                    break;
                }
            }
            if (offset_in_f_after == -1) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational " << operation_name
                          << ": Failed to find vertex ID mapping.\n";
                error_msg << "Query point face ID: " << qp.f_id << "\n";
                error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                error_msg << "This indicates a serious numerical or topological error in rational "
                             "computation.";
                throw std::runtime_error(error_msg.str());
            }

            // Convert barycentric coordinates to rational and normalize
            Eigen::Vector3<wmtk::Rational> bc_rational(
                wmtk::Rational(qp.bc(0)),
                wmtk::Rational(qp.bc(1)),
                wmtk::Rational(qp.bc(2)));
            bc_rational /= bc_rational.sum();

            // compute the location of the qp using rational arithmetic
            int V_cols = V_after.cols();
            Eigen::VectorX<wmtk::Rational> p = Eigen::VectorX<wmtk::Rational>::Zero(V_cols);
            for (int i = 0; i < 3; i++) {
                p = p + V_after_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            bc_rational(i);
            }

            // compute bc of the p in (V, F)_before using rational arithmetic
            int local_index_in_f_before = -1;
            bool bc_updated = false;
            for (int i = 0; i < F_before.rows(); i++) {
                Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result;
                if (V_cols == 2) {
                    bc_rational_result = barycentric_from_cache((*bc_cache_ptr)[i], p.head<2>());
                } else {
                    throw std::runtime_error(
                        "FATAL ERROR: V_cols == 3 after local flattening in rational computation");
                }

                // Check if point is inside triangle (all barycentric coordinates >= 0)
                if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                    local_index_in_f_before = i;
                    qp.bc(0) = bc_rational_result(0).to_double();
                    qp.bc(1) = bc_rational_result(1).to_double();
                    qp.bc(2) = bc_rational_result(2).to_double();
                    bc_updated = true;
                    break;
                }
            }

            if (!bc_updated) {
                std::stringstream error_msg;
                error_msg << "FATAL ERROR in rational " << operation_name
                          << ": Failed to update barycentric coordinates.\n";
                error_msg
                    << "Query point could not be located in any triangle after mesh operation.\n";
                error_msg << "This indicates a serious numerical or topological error in exact "
                             "arithmetic.";
                throw std::runtime_error(error_msg.str());
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }

            // Normalize barycentric coordinates (they should already be normalized from rational
            // computation)
            qp.bc /= qp.bc.sum();
        }
    }
}


template <typename Matrix>
Matrix json_to_matrix(const json& js)
{
    int rows = js["rows"];
    int cols = js["values"][0].size();

    Matrix mat(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            mat(i, j) = js["values"][i][j];
        }
    }

    return mat;
}

void parse_consolidate_file(
    const json& operation_log,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps)
{
    face_ids_maps = operation_log["new2old"][2].get<std::vector<int64_t>>();
    vertex_ids_maps = operation_log["new2old"][0].get<std::vector<int64_t>>();
}

void parse_non_collapse_file(
    const json& operation_log,
    bool& is_skipped,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& F_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after)
{
    is_skipped = operation_log["is_skipped"].get<bool>();
    if (is_skipped) {
        return;
    }

    F_before = json_to_matrix<Eigen::MatrixXi>(operation_log["F_before"]);
    V_before = json_to_matrix<Eigen::MatrixXd>(operation_log["V_before"]);
    id_map_before = operation_log["F_id_map_before"].get<std::vector<int64_t>>();
    v_id_map_before = operation_log["V_id_map_before"].get<std::vector<int64_t>>();

    F_after = json_to_matrix<Eigen::MatrixXi>(operation_log["F_after"]);
    V_after = json_to_matrix<Eigen::MatrixXd>(operation_log["V_after"]);
    id_map_after = operation_log["F_id_map_after"].get<std::vector<int64_t>>();
    v_id_map_after = operation_log["V_id_map_after"].get<std::vector<int64_t>>();
}

void parse_edge_collapse_file(
    const json& operation_log,
    Eigen::MatrixXd& UV_joint,
    Eigen::MatrixXi& F_before,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& v_id_map_joint,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& id_map_after)
{
    UV_joint = json_to_matrix<Eigen::MatrixXd>(operation_log["UV_joint"]);
    F_before = json_to_matrix<Eigen::MatrixXi>(operation_log["F_before"]);
    F_after = json_to_matrix<Eigen::MatrixXi>(operation_log["F_after"]);

    v_id_map_joint = operation_log["v_id_map_joint"].get<std::vector<int64_t>>();
    id_map_before = operation_log["F_id_map_before"].get<std::vector<int64_t>>();
    id_map_after = operation_log["F_id_map_after"].get<std::vector<int64_t>>();
}


// Template version of handle_non_collapse_operation
template <typename CoordType>
void handle_non_collapse_operation_t(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<CoordType>>& query_points,
    const std::string& operation_name,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache)
{
    if constexpr (std::is_same_v<CoordType, double>) {
        // For double query points, use use_rational to decide between regular and rational
        // processing
        if (use_rational) {
            handle_non_collapse_operation_r(
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
                barycentric_cache);
        } else {
            handle_non_collapse_operation(
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
                false);
        }
    } else if constexpr (std::is_same_v<CoordType, wmtk::Rational>) {
        // For rational query points, always use rational processing (ignore use_rational parameter)

        // Convert V_before and V_after to rational
        Eigen::MatrixX<wmtk::Rational> V_before_r(V_before.rows(), V_before.cols());
        Eigen::MatrixX<wmtk::Rational> V_after_r(V_after.rows(), V_after.cols());

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

        // Precompute barycentric cache for F_before (2D) if not provided
        std::vector<BarycentricPrecompute2D> local_cache;
        const std::vector<BarycentricPrecompute2D>* bc_cache_ptr = barycentric_cache;
        if (bc_cache_ptr == nullptr) {
            local_cache = build_barycentric_cache_2d(V_before_r, F_before);
            bc_cache_ptr = &local_cache;
        }

        for (int id = 0; id < query_points.size(); id++) {
            query_point_t<CoordType>& qp = query_points[id];
            if (qp.f_id < 0) continue;

            // find if qp is in the id_map_after
            auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.f_id);
            if (it != id_map_after.end()) {
                // find the index of qp in id_map_after
                int local_index_in_f_after = std::distance(id_map_after.begin(), it);
                // offset of the qp.fv_ids
                int offset_in_f_after = -1;
                for (int i = 0; i < 3; i++) {
                    if (v_id_map_after[F_after(local_index_in_f_after, i)] == qp.fv_ids[0]) {
                        offset_in_f_after = i;
                        break;
                    }
                }
                if (offset_in_f_after == -1) {
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template " << operation_name
                              << ": Failed to find vertex ID mapping.\n";
                    error_msg << "Query point face ID: " << qp.f_id << "\n";
                    error_msg << "Expected vertex ID: " << qp.fv_ids[0] << "\n";
                    error_msg
                        << "This indicates a serious numerical or topological error in rational "
                           "template computation.";
                    throw std::runtime_error(error_msg.str());
                }

                // Normalize barycentric coordinates
                wmtk::Rational bc_sum = qp.bc.sum();
                if (bc_sum != wmtk::Rational(0)) {
                    qp.bc /= bc_sum;
                }

                // compute the location of the qp using rational arithmetic
                int V_cols = V_after.cols();
                Eigen::VectorX<wmtk::Rational> p = Eigen::VectorX<wmtk::Rational>::Zero(V_cols);
                for (int i = 0; i < 3; i++) {
                    p = p +
                        V_after_r.row(F_after(local_index_in_f_after, (i + offset_in_f_after) % 3))
                                .transpose() *
                            qp.bc(i);
                }

                // compute bc of the p in (V, F)_before using rational arithmetic
                int local_index_in_f_before = -1;
                bool bc_updated = false;
                for (int i = 0; i < F_before.rows(); i++) {
                    Eigen::Matrix<wmtk::Rational, 3, 1> bc_rational_result;
                    if (V_cols == 2) {
                        bc_rational_result =
                            barycentric_from_cache((*bc_cache_ptr)[i], p.head<2>());
                    } else {
                        throw std::runtime_error("FATAL ERROR: V_cols == 3 after local flattening "
                                                 "in rational template computation");
                    }

                    // Check if point is inside triangle (all barycentric coordinates >= 0)
                    if (bc_rational_result.minCoeff() >= 0 && bc_rational_result.maxCoeff() <= 1) {
                        local_index_in_f_before = i;
                        qp.bc = bc_rational_result;
                        bc_updated = true;
                        break;
                    }
                }

                if (!bc_updated) {
                    std::stringstream error_msg;
                    error_msg << "FATAL ERROR in rational template " << operation_name
                              << ": Failed to update barycentric coordinates.\n";
                    error_msg << "Query point could not be located in any triangle after mesh "
                                 "operation.\n";
                    error_msg << "This indicates a serious numerical or topological error in exact "
                                 "template arithmetic.";
                    throw std::runtime_error(error_msg.str());
                }

                // update qp
                qp.f_id = id_map_before[local_index_in_f_before];
                for (int i = 0; i < 3; i++) {
                    qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
                }

                // Normalize barycentric coordinates (they should already be normalized from
                // rational computation)
                wmtk::Rational bc_sum_final = qp.bc.sum();
                if (bc_sum_final != wmtk::Rational(0)) {
                    qp.bc /= bc_sum_final;
                }
            }
        }
    } else {
        static_assert(
            std::is_same_v<CoordType, double> || std::is_same_v<CoordType, wmtk::Rational>,
            "CoordType must be either double or wmtk::Rational");
    }
}


// Explicit template instantiations
template void handle_non_collapse_operation_t<double>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<double>>& query_points,
    const std::string& operation_name,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);

template void handle_non_collapse_operation_t<wmtk::Rational>(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_t<wmtk::Rational>>& query_points,
    const std::string& operation_name,
    bool use_rational,
    const std::vector<BarycentricPrecompute2D>* barycentric_cache);
