#include "track_operations_tet.hpp"
#include <igl/barycentric_coordinates.h>
#include <igl/tet_tet_adjacency.h>
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
void handle_consolidate_tet_curve(
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

Eigen::Vector3d barycentric_to_world_tet(
    const Eigen::Vector4d& bc,
    const Eigen::Matrix<double, 4, 3>& v)
{
    std::cout << "bc: " << bc.transpose() << std::endl;
    std::cout << "v: \n" << v << std::endl;
    return bc[0] * v.row(0) + bc[1] * v.row(1) + bc[2] * v.row(2) + bc[3] * v.row(3);
}

Eigen::Vector4d world_to_barycentric_tet(
    const Eigen::Vector3d& p,
    const Eigen::Matrix<double, 4, 3>& v)
{
    Eigen::MatrixXd p_mat = p.transpose();
    Eigen::MatrixXd v0_mat = v.row(0);
    Eigen::MatrixXd v1_mat = v.row(1);
    Eigen::MatrixXd v2_mat = v.row(2);
    Eigen::MatrixXd v3_mat = v.row(3);
    Eigen::MatrixXd bc_mat;
    igl::barycentric_coordinates(p_mat, v0_mat, v1_mat, v2_mat, v3_mat, bc_mat);
    Eigen::Vector4d bc = bc_mat.row(0);
    return bc;
}


void handle_one_segment_tet(
    query_curve_tet& curve,
    int id,
    std::vector<query_point_tet>& current_qps,
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const std::vector<int64_t>& id_map,
    const std::vector<int64_t>& v_id_map,
    Eigen::MatrixXi& TT,
    Eigen::MatrixXi& TTi,
    double eps)
{
    auto& seg = curve.segments[id];
    auto& p0 = current_qps[0];
    auto& p1 = current_qps[1];

    // TODO: get the exact version of this function
    auto is_same_tet = [&](query_point_tet& qp0, query_point_tet& qp1) {
        return qp0.t_id == qp1.t_id;
    };


    // libigl face ordering → vertex‑to‑face map for TT access
    // if bc[i] == 0  then v2f[i] is the face
    constexpr int v2f[4] = {2, 3, 1, 0};


    auto tet_vertices = [&](int64_t tid) {
        std::cout << "V.rows(): " << V.rows() << std::endl;
        std::cout << "tid = " << tid << std::endl;
        std::cout << "T.rows(): " << T.rows() << std::endl;

        Eigen::Matrix<double, 4, 3> v;
        v.row(0) = V.row(T(tid, 0));
        v.row(1) = V.row(T(tid, 1));
        v.row(2) = V.row(T(tid, 2));
        v.row(3) = V.row(T(tid, 3));
        return v;
    };


    if (TT.rows() == 0) {
        std::cout << "Getting TT and TTi" << std::endl;
        igl::tet_tet_adjacency(T, TT, TTi);
    }

    std::cout << "Getting p1_local_tid" << std::endl;
    int p1_local_tid = -1;
    {
        std::cout << "p1.t_id: " << p1.t_id << std::endl;
        std::cout << "id_map size: " << id_map.size() << std::endl;
        std::cout << "id_map contents: ";
        for (size_t i = 0; i < id_map.size(); ++i) {
            std::cout << id_map[i] << " ";
        }
        std::cout << std::endl;
        auto it = std::find(id_map.begin(), id_map.end(), p1.t_id);
        p1_local_tid = std::distance(id_map.begin(), it);
    }

    std::cout << "Getting p1_world" << std::endl;
    auto p1_tet_vertices = tet_vertices(p1_local_tid);
    const Eigen::Vector3d p1_world = barycentric_to_world_tet(p1.bc, p1_tet_vertices);

    if (is_same_tet(p0, p1)) {
        seg.t_id = p0.t_id;
        seg.bcs[0] = p0.bc;
        seg.bcs[1] = p1.bc;
        seg.tv_ids = p0.tv_ids;
        return;
    } else {
        std::cout << "Start splitting: " << std::endl;
        int old_next_seg = curve.next_segment_ids[id];
        auto it = std::find(id_map.begin(), id_map.end(), current_qps[0].t_id);
        int current_local_tid = std::distance(id_map.begin(), it);

        query_point_tet cur = p0;

        while (true) {
            const auto v_cur = tet_vertices(current_local_tid);
            const Eigen::Vector4d b0 = cur.bc;
            Eigen::Vector4d b1;

            // If p1 in same tet, use its barycentrics directly; else compute
            if (is_same_tet(cur, p1)) {
                query_segment_tet new_seg{cur.t_id, {b0, p1.bc}, p1.tv_ids};
                curve.segments.push_back(new_seg);
                curve.next_segment_ids[id] = curve.segments.size() - 1;
                curve.next_segment_ids.push_back(old_next_seg);
                break;
            } else {
                b1 = world_to_barycentric_tet(p1_world, v_cur);
            }

            std::cout << "b0: " << b0.transpose() << std::endl;
            std::cout << "b1: " << b1.transpose() << std::endl;

            double t_exit = 1.0;
            int exit_fid = -1;

            for (int i = 0; i < 4; i++) {
                std::cout << "i: " << i << std::endl;
                double denom = b1[i] - b0[i];
                if (abs(denom) < eps) {
                    std::cout << "ray parallel to the face" << std::endl;
                    continue;
                }
                double t = b0[i] / (b0[i] - b1[i]);
                std::cout << "t: " << t << std::endl;

                if (t < eps) continue; // on the wrong side

                Eigen::Vector4d bc_intersect = b0 + t * (b1 - b0);
                std::cout << "bc_intersect: " << bc_intersect.transpose() << std::endl;

                // check if all bc_intersect are non-negative and less than 1
                if ((bc_intersect.array() >= -eps).all() &&
                    (bc_intersect.array() <= 1.0 + eps).all()) {
                    t_exit = t;
                    exit_fid = v2f[i];
                    break;
                }
            }

            // TODO: for debug
            break;
        } // end of while
        // If p1 is inside current tet → finish
    } // end of else
}
void handle_local_mapping_tet_curve(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& T_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& T_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_curve_tet& curve)
{
    int curve_length = curve.segments.size();
    double eps = 1e-8;
    std::cout << "Handling Local Mapping for curve" << std::endl;
    Eigen::MatrixXi TT, TTi; // connectivity of T_before

    // TODO: can we do parallel here?
    for (int id = 0; id < curve_length; id++) {
        auto& seg = curve.segments[id];
        query_point_tet qp0 = {seg.t_id, seg.bcs[0], seg.tv_ids};
        query_point_tet qp1 = {seg.t_id, seg.bcs[1], seg.tv_ids};
        std::vector<query_point_tet> qps = {qp0, qp1};

        std::cout << "Handling local mapping for segment " << id << std::endl;
        handle_local_mapping_tet(
            V_before,
            T_before,
            id_map_before,
            v_id_map_before,
            V_after,
            T_after,
            id_map_after,
            v_id_map_after,
            qps);
        std::cout << "Handling one segment, get intersections." << std::endl;
        // TODO: implement this function
        handle_one_segment_tet(
            curve,
            id,
            qps,
            V_before,
            T_before,
            id_map_before,
            v_id_map_before,
            TT,
            TTi,
            eps);
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

void write_query_surface_tet_to_file(const query_surface_tet& surface, const std::string& filename)
{
    json j;
    j["num_triangles"] = surface.triangles.size();

    for (size_t i = 0; i < surface.triangles.size(); ++i) {
        const auto& tri = surface.triangles[i];
        json tri_json;
        tri_json["t_id"] = tri.t_id;

        // Store barycentric coordinates
        for (int j = 0; j < 3; ++j) {
            tri_json["bcs"][j] = {tri.bcs[j][0], tri.bcs[j][1], tri.bcs[j][2], tri.bcs[j][3]};
        }

        // Store tetrahedron vertex ids
        tri_json["tv_ids"] = {tri.tv_ids[0], tri.tv_ids[1], tri.tv_ids[2], tri.tv_ids[3]};

        j["triangles"].push_back(tri_json);
    }

    std::ofstream file(filename);
    if (file.is_open()) {
        file << j.dump(2);
        file.close();
    } else {
        std::cerr << "Failed to open file for writing: " << filename << std::endl;
    }
}

query_surface_tet read_query_surface_tet_from_file(const std::string& filename)
{
    query_surface_tet surface;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file for reading: " << filename << std::endl;
        return surface;
    }

    json j;
    file >> j;
    file.close();

    size_t num_triangles = j["num_triangles"];
    surface.triangles.resize(num_triangles);

    for (size_t i = 0; i < num_triangles; ++i) {
        const auto& tri_json = j["triangles"][i];
        auto& tri = surface.triangles[i];

        tri.t_id = tri_json["t_id"];

        // Read barycentric coordinates
        for (int j = 0; j < 3; ++j) {
            const auto& bc_array = tri_json["bcs"][j];
            tri.bcs[j] = Eigen::Vector4d(bc_array[0], bc_array[1], bc_array[2], bc_array[3]);
        }

        // Read tetrahedron vertex ids
        const auto& tv_array = tri_json["tv_ids"];
        tri.tv_ids = Eigen::Vector4i(tv_array[0], tv_array[1], tv_array[2], tv_array[3]);
    }

    return surface;
}