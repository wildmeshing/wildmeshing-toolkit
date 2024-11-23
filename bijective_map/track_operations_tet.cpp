#include "track_operations_tet.hpp"


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