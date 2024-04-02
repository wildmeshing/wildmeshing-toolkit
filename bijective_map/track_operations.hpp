#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

// igl
#include <igl/parallel_for.h>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

struct query_point
{
    int64_t f_id; // face id
    Eigen::Vector3d bc; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

void handle_consolidate(
    const std::vector<int64_t>& face_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<query_point>& query_points)
{
    std::cout << "Handling Consolidate" << std::endl;
    igl::parallel_for(query_points.size(), [&](int id) {
        query_point& qp = query_points[id];
        if (face_ids_maps[qp.f_id] != qp.f_id) {
            qp.f_id = face_ids_maps[qp.f_id];
        }
        for (int i = 0; i < 3; i++) {
            if (vertex_ids_maps[qp.fv_ids[i]] != qp.fv_ids[i]) {
                qp.fv_ids[i] = vertex_ids_maps[qp.fv_ids[i]];
            }
        }
    });
}

// TODO: get a more acurate version of this
Eigen::Vector3d ComputeBarycentricCoordinates3D(
    const Eigen::Vector3d& p,
    const Eigen::Vector3d& a,
    const Eigen::Vector3d& b,
    const Eigen::Vector3d& c,
    double eps = 1e-3)
{
    Eigen::Vector3d v0 = b - a, v1 = c - a, v2 = p - a;
    Eigen::Vector3d n = v0.cross(v1);
    n.normalized();
    if (std::abs(n.dot(p - a)) > eps) {
        // std::cout << "not on the plane, " << std::abs(n.dot(p - a)) << std::endl;
        // not on this face
        return Eigen::Vector3d(-1, -1, -1);
    }
    double d00 = v0.dot(v0);
    double d01 = v0.dot(v1);
    double d11 = v1.dot(v1);
    double d20 = v2.dot(v0);
    double d21 = v2.dot(v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return Eigen::Vector3d(u, v, w);
}

Eigen::Vector3d ComputeBarycentricCoordinates2D(
    const Eigen::Vector2d& p,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b,
    const Eigen::Vector2d& c)
{
    Eigen::Vector2d v0 = b - a, v1 = c - a, v2 = p - a;
    double d00 = v0.dot(v0);
    double d01 = v0.dot(v1);
    double d11 = v1.dot(v1);
    double d20 = v2.dot(v0);
    double d21 = v2.dot(v1);
    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;
    return Eigen::Vector3d(u, v, w);
}


void handle_split_edge(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point>& query_points)
{
    std::cout << "Handling EdgeSplit" << std::endl;
    // igl::parallel_for(query_points.size(), [&](int id) {
    for (int id = 0; id < query_points.size(); id++) {
        query_point& qp = query_points[id];

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
                std::cout << "something is wrong!" << std::endl;
                continue;
                // return;
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
                    bc = ComputeBarycentricCoordinates3D(
                        p,
                        V_before.row(F_before(i, 0)),
                        V_before.row(F_before(i, 1)),
                        V_before.row(F_before(i, 2)));
                }

                if (-bc.minCoeff() < bc_min_coef) {
                    bc_min_coef = -bc.minCoeff();
                    local_index_in_f_before = i;
                    qp.bc = bc;
                    bc_updated = true;
                }
            }

            if (!bc_updated) {
                std::cout << "bc not updated\n" << std::endl;
                continue;
                // return;
            }

            // update qp
            qp.f_id = id_map_before[local_index_in_f_before];
            for (int i = 0; i < 3; i++) {
                qp.fv_ids[i] = v_id_map_before[F_before(local_index_in_f_before, i)];
            }
            // avoid numerical issue
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

void parse_consolidate_file(
    const json& operation_log,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps)
{
    face_ids_maps = operation_log["new2old"][2].get<std::vector<int64_t>>();
    vertex_ids_maps = operation_log["new2old"][0].get<std::vector<int64_t>>();
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

void parse_edge_split_file(
    const json& operation_log,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& F_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after)
{
    F_before = json_to_matrix<Eigen::MatrixXi>(operation_log["F_before"]);
    V_before = json_to_matrix<Eigen::MatrixXd>(operation_log["V_before"]);
    id_map_before = operation_log["F_id_map_before"].get<std::vector<int64_t>>();
    v_id_map_before = operation_log["V_id_map_before"].get<std::vector<int64_t>>();

    F_after = json_to_matrix<Eigen::MatrixXi>(operation_log["F_after"]);
    V_after = json_to_matrix<Eigen::MatrixXd>(operation_log["V_after"]);
    id_map_after = operation_log["F_id_map_after"].get<std::vector<int64_t>>();
    v_id_map_after = operation_log["V_id_map_after"].get<std::vector<int64_t>>();
}