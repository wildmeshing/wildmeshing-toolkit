#pragma once

#ifdef WMTK_ENABLE_COLLAPSE_PATCH_DUMP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "track_operations_curve.hpp"

namespace wmtk::bijective_map::collapse_debug {

struct EdgeMeshDebugData
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi E;
    Eigen::VectorXi curve_ids;

    bool empty() const { return E.rows() == 0 || V.rows() == 0; }
};

int get_patch_dump_threshold();

std::unordered_map<int64_t, int> build_face_lookup(const std::vector<int64_t>& id_map);

int count_segments_in_patch(
    const std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::unordered_map<int64_t, int>& face_lookup);

EdgeMeshDebugData build_edge_mesh_debug_data(
    const std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::unordered_map<int64_t, int>& face_lookup,
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_local,
    const std::vector<int64_t>& v_id_map_joint,
    bool use_local_face_vertices);

} // namespace wmtk::bijective_map::collapse_debug

#endif // WMTK_ENABLE_COLLAPSE_PATCH_DUMP

