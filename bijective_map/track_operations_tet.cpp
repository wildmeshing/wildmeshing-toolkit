#include "track_operations_tet.hpp"
#include <igl/barycentric_coordinates.h>
#include <igl/tet_tet_adjacency.h>
#include "FindPointTetMesh.hpp"
#include "InteractiveAndRobustMeshBooleans/code/booleans.h"
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

// handle consolidate point version
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

// handle consolidate curve version
void handle_consolidate_tet_curve(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    query_curve_tet& curve,
    bool forward)
{
    std::cout << "Handling Consolidatefor curve" << std::endl;
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

// handle consolidate surface version
void handle_consolidate_tet_surface(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    query_surface_tet& surface,
    bool forward)
{
    std::cout << "Handling Consolidate for surface" << std::endl;
    if (!forward) {
        // backward
        for (auto& qt : surface.triangles) {
            if (qt.t_id >= 0) {
                qt.t_id = tet_ids_maps[qt.t_id];
            }
            for (int j = 0; j < 4; j++) {
                qt.tv_ids[j] = vertex_ids_maps[qt.tv_ids[j]];
            }
        }
    } else {
        // forward
        for (auto& qt : surface.triangles) {
            if (qt.t_id >= 0) {
                auto it = std::find(tet_ids_maps.begin(), tet_ids_maps.end(), qt.t_id);
                if (it != tet_ids_maps.end()) {
                    qt.t_id = std::distance(tet_ids_maps.begin(), it);
                }
                for (int j = 0; j < 4; j++) {
                    auto it_v =
                        std::find(vertex_ids_maps.begin(), vertex_ids_maps.end(), qt.tv_ids[j]);
                    if (it_v != vertex_ids_maps.end()) {
                        qt.tv_ids[j] = std::distance(vertex_ids_maps.begin(), it_v);
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


/**
 * @brief Helper function to get all possible representations of a vertex
 * @param local_t_id The local tetrahedron ID
 * @param local_bc The barycentric coordinates within the tetrahedron
 * @param T_local The local tetrahedron connectivity matrix
 * @return std::pair<std::vector<int>, std::vector<Eigen::Vector4d>> All possible tetrahedron IDs
 * and their corresponding barycentric coordinates
 */

std::pair<std::vector<int>, std::vector<Eigen::Vector4d>> get_all_possible_representations(
    const int local_t_id,
    const Eigen::Vector4d& local_bc,
    const Eigen::MatrixXi& T_local,
    const double eps = 1e-10)
{
    std::vector<int> all_possible_t_ids;
    std::vector<Eigen::Vector4d> all_possible_bcs;

    // add the original representation
    all_possible_t_ids.push_back(local_t_id);
    all_possible_bcs.push_back(local_bc);

    std::vector<int> non_zeros;
    for (int i = 0; i < 4; i++) {
        if (abs(local_bc(i)) >= eps) {
            non_zeros.push_back(i);
        }
    }

    std::cout << "T_local(local_t_id, non_zeros): ";
    for (int i = 0; i < non_zeros.size(); i++) {
        std::cout << T_local(local_t_id, non_zeros[i]) << " ";
    }
    std::cout << std::endl;

    if (non_zeros.size() < 4) {
        for (int t_id = 0; t_id < T_local.rows(); t_id++) {
            if (t_id == local_t_id) continue;

            bool contains_all = true;
            Eigen::Vector4d bc_tmp = Eigen::Vector4d::Zero();
            for (int i = 0; i < non_zeros.size(); i++) {
                int v_idx = T_local(local_t_id, non_zeros[i]);
                bool found = false;

                for (int j = 0; j < 4; j++) {
                    if (T_local(t_id, j) == v_idx) {
                        found = true;
                        bc_tmp(j) = local_bc(non_zeros[i]);
                        break;
                    }
                }

                if (!found) {
                    contains_all = false;
                    break;
                }
            }

            if (contains_all) {
                all_possible_t_ids.push_back(t_id);
                all_possible_bcs.push_back(bc_tmp);
            }
        }
    }

    return {all_possible_t_ids, all_possible_bcs};
}


void handle_local_mapping_tet_surface(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& T_before,
    const std::vector<int64_t>& id_map_before,
    const std::vector<int64_t>& v_id_map_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& T_after,
    const std::vector<int64_t>& id_map_after,
    const std::vector<int64_t>& v_id_map_after,
    query_surface_tet& surface)
{
    // TODO: what can we do with this eps?
    double eps = 1e-10;


    std::cout << "Handling Local Mapping for surface" << std::endl;

    // prepare for the arrangement
    std::vector<double> T_before_coords;
    std::vector<uint> T_before_tris;
    std::vector<uint> T_before_labels;
    for (int i = 0; i < V_before.rows(); i++) {
        for (int j = 0; j < V_before.cols(); j++) {
            T_before_coords.push_back(V_before(i, j));
        }
    }
    for (int t_id = 0; t_id < T_before.rows(); t_id++) {
        auto tet = T_before.row(t_id);

        // Extract the four faces of the tetrahedron
        std::vector<std::vector<int>> faces = {
            {tet[0], tet[1], tet[2]},
            {tet[0], tet[1], tet[3]},
            {tet[0], tet[2], tet[3]},
            {tet[1], tet[2], tet[3]}};

        for (const auto& tri : faces) {
            for (const auto& vertex_id : tri) {
                T_before_tris.push_back(static_cast<uint>(vertex_id));
            }
            // Set label based on tetrahedron ID
            T_before_labels.push_back(static_cast<uint>(t_id));
        }
    }

    int current_surface_triangle_size = surface.triangles.size();
    for (int id = 0; id < current_surface_triangle_size; id++) {
        auto& qt = surface.triangles[id];
        if (qt.t_id >= 0) {
            auto it = std::find(id_map_after.begin(), id_map_after.end(), qt.t_id);
            if (it == id_map_after.end()) continue; // not found in local patch
        }
        query_point_tet qp0 = {qt.t_id, qt.bcs[0], qt.tv_ids};
        query_point_tet qp1 = {qt.t_id, qt.bcs[1], qt.tv_ids};
        query_point_tet qp2 = {qt.t_id, qt.bcs[2], qt.tv_ids};
        std::vector<query_point_tet> qps = {qp0, qp1, qp2};

        // std::cout << "Before local mapping:" << std::endl;
        // for (int i = 0; i < qps.size(); i++) {
        //     std::cout << "Point " << i << ": t_id=" << qps[i].t_id
        //               << ", bc=" << qps[i].bc.transpose() << std::endl;
        // }

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

        // Clip barycentric coordinates by epsilon to avoid numerical issues
        for (auto& qp : qps) {
            for (int i = 0; i < 4; i++) {
                if (qp.bc[i] < 1e-10) {
                    qp.bc[i] = 0.0;
                } else if (qp.bc[i] > 1.0 - 1e-10) {
                    qp.bc[i] = 1.0;
                }
            }
            // Renormalize to ensure sum equals 1
            double sum = qp.bc.sum();
            if (sum > 0) {
                qp.bc /= sum;
            }
        }
        std::cout << "After local mapping:" << std::endl;
        for (int i = 0; i < qps.size(); i++) {
            std::cout << "Point " << i << ": t_id=" << qps[i].t_id
                      << ", bc=" << qps[i].bc.transpose() << std::endl;
        }

        std::cout << "Handling one triangle, get intersections." << std::endl;

        // Get local t_ids for each point
        std::vector<int> local_t_ids;
        for (const auto& qp : qps) {
            auto it = std::find(id_map_before.begin(), id_map_before.end(), qp.t_id);
            if (it != id_map_before.end()) {
                int local_id = std::distance(id_map_before.begin(), it);
                local_t_ids.push_back(local_id);
            } else {
                std::cout << "Error: t_id " << qp.t_id << " not found in id_map_before"
                          << std::endl;
                exit(1);
            }
        }
        std::vector<std::vector<int>> all_possible_t_ids_per_point;
        std::vector<std::vector<Eigen::Vector4d>> all_possible_bcs_per_point;

        for (int i = 0; i < qps.size(); i++) {
            auto [t_ids, bcs] =
                get_all_possible_representations(local_t_ids[i], qps[i].bc, T_before, 1e-10);
            all_possible_t_ids_per_point.push_back(t_ids);
            all_possible_bcs_per_point.push_back(bcs);
        }

        // Check if there is a common t_id across all possible representations
        bool found_common_tet = false;
        int common_t_id = -1;

        // Get the first point's possible t_ids as the starting set
        std::set<int> common_t_ids(
            all_possible_t_ids_per_point[0].begin(),
            all_possible_t_ids_per_point[0].end());

        // Intersect with each subsequent point's possible t_ids
        for (int i = 1; i < all_possible_t_ids_per_point.size(); i++) {
            std::set<int> current_t_ids(
                all_possible_t_ids_per_point[i].begin(),
                all_possible_t_ids_per_point[i].end());

            std::set<int> intersection;
            std::set_intersection(
                common_t_ids.begin(),
                common_t_ids.end(),
                current_t_ids.begin(),
                current_t_ids.end(),
                std::inserter(intersection, intersection.begin()));

            common_t_ids = intersection;

            if (common_t_ids.empty()) {
                break;
            }
        }

        if (!common_t_ids.empty()) {
            found_common_tet = true;
            common_t_id = *common_t_ids.begin(); // Take the first common t_id
            std::cout << "Found common tetrahedron: " << common_t_id << std::endl;

            // Set the tetrahedron ID and vertex IDs
            qt.t_id = id_map_before[common_t_id];
            for (int i = 0; i < 4; i++) {
                qt.tv_ids[i] = v_id_map_before[T_before(common_t_id, i)];
            }

            // Get corresponding barycentric coordinates for each point in the common tetrahedron
            for (int i = 0; i < all_possible_t_ids_per_point.size(); i++) {
                for (int j = 0; j < all_possible_t_ids_per_point[i].size(); j++) {
                    if (all_possible_t_ids_per_point[i][j] == common_t_id) {
                        qt.bcs[i] = all_possible_bcs_per_point[i][j];
                        break;
                    }
                }
            }
        } else {
            // std::cout << "No common tetrahedron found across all representations" << std::endl;
            // }
            // // TODO: handle one triangle
            // Check if all 3 points in qps have the same t_id
            // if (qps[0].t_id == qps[1].t_id && qps[1].t_id == qps[2].t_id) {
            //     std::cout << "All points are in one tetrahedron " << qps[0].t_id << std::endl;
            //     // update the query triangle
            //     qt.t_id = qps[0].t_id;
            //     qt.bcs[0] = qps[0].bc;
            //     qt.bcs[1] = qps[1].bc;
            //     qt.bcs[2] = qps[2].bc;
            //     qt.tv_ids = qps[0].tv_ids;
            // }
            // else
            // {
            std::cout << "Not in one tetrahedron" << std::endl;
            std::cout << "Computing arrangement" << std::endl;


            std::vector<double> in_coords = T_before_coords;
            std::vector<uint> in_tris = T_before_tris;
            std::vector<uint> in_labels = T_before_labels;

            // Compute real positions for each query point
            for (int i = 0; i < qps.size(); i++) {
                // Get tetrahedron vertices
                Eigen::Vector4i tet = T_before.row(local_t_ids[i]);
                Eigen::Vector4d bc = qps[i].bc;

                // Compute position using barycentric coordinates
                Eigen::Vector3d pos = Eigen::Vector3d::Zero();
                for (int j = 0; j < 4; j++) {
                    pos += bc[j] * V_before.row(tet[j]).transpose();
                }
                std::cout << "Position " << i << ": (" << pos[0] << ", " << pos[1] << ", " << pos[2]
                          << ")" << std::endl;

                // Add position to in_coords
                in_coords.push_back(pos[0]);
                in_coords.push_back(pos[1]);
                in_coords.push_back(pos[2]);
            }

            // Add triangle to in_tris using base index at end of original vertices
            uint base_idx = V_before.rows();
            in_tris.push_back(base_idx);
            in_tris.push_back(base_idx + 1);
            in_tris.push_back(base_idx + 2);

            // Add label for the triangle
            in_labels.push_back(T_before.rows());


            // init the necessary data structures
            point_arena arena;
            std::vector<genericPoint*> arr_verts;
            std::vector<uint> arr_in_tris, arr_out_tris;
            std::vector<std::bitset<NBIT>> arr_in_labels;
            std::vector<DuplTriInfo> dupl_triangles;
            Labels labels;
            cinolib::Octree octree;
            std::vector<uint> vertex_id_map; // map from original vertices to arranged vertices

            // arrangement, last parameter is false to avoid parallelization
            vertex_id_map = customArrangementPipeline(
                in_coords,
                in_tris,
                in_labels,
                arr_in_tris,
                arr_in_labels,
                arena,
                arr_verts,
                arr_out_tris,
                labels,
                octree,
                dupl_triangles,
                false);

            // create FastTrimesh
            FastTrimesh tm(arr_verts, arr_out_tris, true);
            // Prepare output data
            std::vector<double> out_coords;
            std::vector<uint> out_tris;
            std::vector<std::bitset<NBIT>> out_labels;
            {
                tm.resetTrianglesInfo();
                uint num_tris = 0;

                // All triangles
                for (uint t_id = 0; t_id < tm.numTris(); t_id++) {
                    tm.setTriInfo(t_id, 1);
                    num_tris++;
                }

                getFinalMeshInOder(tm, labels, num_tris, out_coords, out_tris, out_labels);
            }

            // get barycentric coordinates of the output triangles
            std::vector<int> out_tri_ids;
            std::vector<std::set<int>> vertex_to_labels(tm.numVerts());

            for (int i = 0; i < 3; i++) {
                std::cout << "i = " << i << std::endl;

                // Add the primary tetrahedron for this point
                vertex_to_labels[vertex_id_map[V_before.rows() + i]].insert(local_t_ids[i]);

                // Add all possible representations computed earlier
                for (int t_id : all_possible_t_ids_per_point[i]) {
                    vertex_to_labels[vertex_id_map[V_before.rows() + i]].insert(t_id);
                }
            }

            std::cout << "V_before.rows() = " << V_before.rows() << std::endl;
            std::cout << "T_before.rows() = " << T_before.rows() << std::endl;

            for (uint t_id = 0; t_id < tm.numTris(); t_id++) {
                uint v0 = tm.tri(t_id)[0];
                uint v1 = tm.tri(t_id)[1];
                uint v2 = tm.tri(t_id)[2];
                uint v3 = tm.tri(t_id)[3];

                for (uint label_id = 0; label_id < labels.num; label_id++) {
                    if (labels.surface[t_id][label_id]) {
                        vertex_to_labels[v0].insert(label_id);
                        vertex_to_labels[v1].insert(label_id);
                        vertex_to_labels[v2].insert(label_id);
                    }
                }

                if (labels.surface[t_id][labels.num - 1]) {
                    out_tri_ids.push_back(t_id);
                }
            }

            {
                std::cout << "\n=== All Vertices and IDs ===" << std::endl;
                for (uint v_id = 0; v_id < tm.numVerts(); v_id++) {
                    if (!vertex_to_labels[v_id].empty()) {
                        std::cout << "Vertex " << v_id << ": ";
                        for (auto label : vertex_to_labels[v_id]) {
                            std::cout << label << " ";
                        }
                        std::cout << "pos: (" << out_coords[3 * v_id] << ", "
                                  << out_coords[3 * v_id + 1] << ", " << out_coords[3 * v_id + 2]
                                  << ")";
                        std::cout << std::endl;
                    }
                }

                for (int i = 0; i < out_tri_ids.size(); i++) {
                    int triangle_id = out_tri_ids[i];
                    uint v0 = tm.tri(triangle_id)[0];
                    uint v1 = tm.tri(triangle_id)[1];
                    uint v2 = tm.tri(triangle_id)[2];
                    std::cout << "Triangle " << triangle_id << ": vertices [" << v0 << ", " << v1
                              << ", " << v2 << "]" << std::endl;
                }
            }

            for (int i = 0; i < out_tri_ids.size(); i++) {
                int triangle_id = out_tri_ids[i];
                std::cout << "checking triangle id: " << triangle_id << std::endl;
                uint v0_idx = tm.tri(triangle_id)[0];
                uint v1_idx = tm.tri(triangle_id)[1];
                uint v2_idx = tm.tri(triangle_id)[2];

                {
                    std::cout << "vertex_to_labels for triangle vertices:" << std::endl;
                    std::cout << "v0 (" << v0_idx << "): ";
                    for (auto label : vertex_to_labels[v0_idx]) {
                        std::cout << label << " ";
                    }
                    std::cout << std::endl;
                    std::cout << "v1 (" << v1_idx << "): ";
                    for (auto label : vertex_to_labels[v1_idx]) {
                        std::cout << label << " ";
                    }
                    std::cout << std::endl;
                    std::cout << "v2 (" << v2_idx << "): ";
                    for (auto label : vertex_to_labels[v2_idx]) {
                        std::cout << label << " ";
                    }
                    std::cout << std::endl;
                }

                int containing_tet_id = -1;
                for (int tet_id = 0; tet_id < T_before.rows(); tet_id++) {
                    if (vertex_to_labels[v0_idx].count(tet_id) &&
                        vertex_to_labels[v1_idx].count(tet_id) &&
                        vertex_to_labels[v2_idx].count(tet_id)) {
                        std::cout << "triangle " << triangle_id << " is in tet " << tet_id
                                  << std::endl;
                        containing_tet_id = tet_id;
                        break;
                    }
                }
                if (containing_tet_id == -1) {
                    std::cout << "Error: triangle " << triangle_id << " is not in any tet"
                              << std::endl;

                    std::ofstream obj_file("debug_triangles.obj");
                    obj_file << "# Debug triangles that couldn't be found in any tet" << std::endl;

                    for (int i = 0; i < out_tri_ids.size(); i++) {
                        int triangle_id = out_tri_ids[i];
                        uint v0_idx = tm.tri(triangle_id)[0];
                        uint v1_idx = tm.tri(triangle_id)[1];
                        uint v2_idx = tm.tri(triangle_id)[2];

                        // Write vertex positions
                        obj_file << "v " << out_coords[v0_idx * 3] << " "
                                 << out_coords[v0_idx * 3 + 1] << " " << out_coords[v0_idx * 3 + 2]
                                 << std::endl;
                        obj_file << "v " << out_coords[v1_idx * 3] << " "
                                 << out_coords[v1_idx * 3 + 1] << " " << out_coords[v1_idx * 3 + 2]
                                 << std::endl;
                        obj_file << "v " << out_coords[v2_idx * 3] << " "
                                 << out_coords[v2_idx * 3 + 1] << " " << out_coords[v2_idx * 3 + 2]
                                 << std::endl;
                    }

                    // Write face indices (1-indexed in OBJ format)
                    for (int i = 0; i < out_tri_ids.size(); i++) {
                        obj_file << "f " << (i * 3 + 1) << " " << (i * 3 + 2) << " " << (i * 3 + 3)
                                 << std::endl;
                    }

                    obj_file.close();
                    std::cout << "Debug triangles written to debug_triangles.obj" << std::endl;

                    exit(1);
                }

                query_triangle_tet q_tri;
                q_tri.t_id = id_map_before[containing_tet_id];
                Eigen::Matrix<double, 4, 3> tet_Vs;
                tet_Vs.row(0) = V_before.row(T_before(containing_tet_id, 0));
                tet_Vs.row(1) = V_before.row(T_before(containing_tet_id, 1));
                tet_Vs.row(2) = V_before.row(T_before(containing_tet_id, 2));
                tet_Vs.row(3) = V_before.row(T_before(containing_tet_id, 3));
                for (int j = 0; j < 4; j++) {
                    q_tri.tv_ids[j] = v_id_map_before[T_before(containing_tet_id, j)];
                }
                auto v0_world = Eigen::Vector3d(
                    out_coords[v0_idx * 3],
                    out_coords[v0_idx * 3 + 1],
                    out_coords[v0_idx * 3 + 2]);
                auto v1_world = Eigen::Vector3d(
                    out_coords[v1_idx * 3],
                    out_coords[v1_idx * 3 + 1],
                    out_coords[v1_idx * 3 + 2]);
                auto v2_world = Eigen::Vector3d(
                    out_coords[v2_idx * 3],
                    out_coords[v2_idx * 3 + 1],
                    out_coords[v2_idx * 3 + 2]);
                q_tri.bcs[0] = world_to_barycentric_tet(v0_world, tet_Vs);
                q_tri.bcs[1] = world_to_barycentric_tet(v1_world, tet_Vs);
                q_tri.bcs[2] = world_to_barycentric_tet(v2_world, tet_Vs);
                for (int j = 0; j < 3; j++) {
                    for (int k = 0; k < 4; k++) {
                        if (std::abs(q_tri.bcs[j](k)) < 1e-15) {
                            q_tri.bcs[j](k) = 0.0;
                        }
                    }
                }
                if (i == 0) {
                    qt = q_tri;
                } else {
                    surface.triangles.push_back(q_tri);
                }
            }
        }
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