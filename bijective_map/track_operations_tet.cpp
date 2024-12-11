#include "track_operations_tet.hpp"
#include "FindPointTetMesh.hpp"

// helper function to convert json to matrix
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

// point version
void handle_consolidate_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<query_point_tet>& query_points,
    bool forward)
{
    std::cout << "Handling Consolidate" << std::endl;
    if (!forward) {
        // backward
        igl::parallel_for(query_points.size(), [&](int id) {
            query_point_tet& qp = query_points[id];
            if (qp.t_id >= 0) {
                if (tet_ids_maps[qp.t_id] != qp.t_id) {
                    qp.t_id = tet_ids_maps[qp.t_id];
                }
                for (int i = 0; i < 4; i++) {
                    if (vertex_ids_maps[qp.tv_ids[i]] != qp.tv_ids[i]) {
                        qp.tv_ids[i] = vertex_ids_maps[qp.tv_ids[i]];
                    }
                }
            }
        });
    } else {
        // forward
        igl::parallel_for(query_points.size(), [&](int id) {
            query_point_tet& qp = query_points[id];
            if (qp.t_id >= 0) {
                auto it = std::find(tet_ids_maps.begin(), tet_ids_maps.end(), qp.t_id);
                if (it != tet_ids_maps.end()) {
                    qp.t_id = std::distance(tet_ids_maps.begin(), it);
                }
                for (int i = 0; i < 4; i++) {
                    auto it_v =
                        std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qp.tv_ids[i]);
                    if (it_v != vertex_ids_maps.end()) {
                        qp.tv_ids[i] = std::distance(vertex_ids_maps.begin(), it_v);
                    } else {
                        std::cout << "Error: vertex not found" << std::endl;
                    }
                }
            }
        });
    }
}

// curve version
void handle_consolidat_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    query_curve_tet& curve,
    bool forward)
{
    std::cout << "Handling Consolidate forward for curve" << std::endl;
    if (!forward) {
        // backward
        // TODO: maybe use igl::parallel_for
        for (int id = 0; id < curve.segments.size(); id++) {
            auto& qs = curve.segments[id];
            if (qs.t_id >= 0) {
                qs.t_id = tet_ids_maps[qs.t_id];
            }
            for (int j = 0; j < 4; j++) {
                qs.tv_ids[j] = vertex_ids_maps[qs.tv_ids[j]];
            }
        }
    } else {
        // forward
        for (int id = 0; id < curve.segments.size(); id++) {
            auto& qs = curve.segments[id];
            if (qs.t_id >= 0) {
                auto it = std::find(tet_ids_maps.begin(), tet_ids_maps.end(), qs.t_id);
                if (it != tet_ids_maps.end()) {
                    qs.t_id = std::distance(tet_ids_maps.begin(), it);
                }
                for (int j = 0; j < 4; j++) {
                    auto it_v =
                        std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qs.tv_ids[j]);
                    if (it_v != vertex_ids_maps.end()) {
                        qs.tv_ids[j] = std::distance(vertex_ids_maps.begin(), it_v);
                    } else {
                        std::cout << "Error: vertex not found" << std::endl;
                    }
                }
            }
        }
    }
}

void handle_local_mapping_tet(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& T_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& T_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    std::vector<query_point_tet>& query_points)
{
    std::cout << "Handling Local Mapping" << std::endl;
    for (int id = 0; id < query_points.size(); id++) {
        query_point_tet& qp = query_points[id];
        // TODO: maybe for here is not needed
        if (qp.t_id < 0) continue;
        auto it = std::find(id_map_after.begin(), id_map_after.end(), qp.t_id);
        if (it == id_map_after.end()) continue; // not found

        int local_index_in_t_after = std::distance(id_map_after.begin(), it);

        // get position here
        Eigen::Vector3d p(0, 0, 0);
        for (int i = 0; i < 4; i++) {
            int v_id = qp.tv_ids[i];
            auto it_v = std::find(v_id_map_after.begin(), v_id_map_after.end(), v_id);
            if (it_v == v_id_map_after.end()) {
                std::cout << "Error: vertex not found" << std::endl;
                continue;
            }

            int local_index_in_v_after = std::distance(v_id_map_after.begin(), it_v);
            p += V_after.row(local_index_in_v_after) * qp.bc(i);
        }

        // compute bc of the p in (V, T)_before
        auto result = findTetContainingPoint(V_before, T_before, p);
        auto [t_id_before, bc_before] = result;
        if (t_id_before == -1) {
            std::cout << "Error: Point not in T_before" << std::endl;
            continue;
        }

        // write out the change
        std::cout << "Change: " << qp.t_id << "->" << id_map_before[t_id_before] << std::endl;
        std::cout << "BC:" << qp.bc.transpose() << "->" << bc_before.transpose() << std::endl;

        // update the query point
        qp.t_id = id_map_before[t_id_before];
        for (int i = 0; i < 4; i++) {
            qp.tv_ids[i] = v_id_map_before[T_before(t_id_before, i)];
            qp.bc(i) = std::max(0.0, std::min(1.0, bc_before(i)));
        }
        qp.bc /= qp.bc.sum(); // normalize
    }
}

void parse_consolidate_file_tet(
    const json& operation_log,
    std::vector<int64_t>& tet_ids_maps,
    std::vector<int64_t>& vertex_ids_maps)
{
    tet_ids_maps = operation_log["new2old"][3].get<std::vector<int64_t>>();
    vertex_ids_maps = operation_log["new2old"][0].get<std::vector<int64_t>>();
}

void parse_non_collapse_file_tet(
    const json& operation_log,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& T_before,
    std::vector<int64_t>& id_map_before,
    std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& T_after,
    std::vector<int64_t>& id_map_after,
    std::vector<int64_t>& v_id_map_after)
{
    T_before = json_to_matrix<Eigen::MatrixXi>(operation_log["T_before"]);
    V_before = json_to_matrix<Eigen::MatrixXd>(operation_log["V_before"]);
    id_map_before = operation_log["T_id_map_before"].get<std::vector<int64_t>>();
    v_id_map_before = operation_log["V_id_map_before"].get<std::vector<int64_t>>();

    T_after = json_to_matrix<Eigen::MatrixXi>(operation_log["T_after"]);
    V_after = json_to_matrix<Eigen::MatrixXd>(operation_log["V_after"]);
    id_map_after = operation_log["T_id_map_after"].get<std::vector<int64_t>>();
    v_id_map_after = operation_log["V_id_map_after"].get<std::vector<int64_t>>();
}