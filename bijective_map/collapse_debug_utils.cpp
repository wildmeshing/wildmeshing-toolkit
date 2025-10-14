

#include "collapse_debug_utils.hpp"

#ifdef WMTK_ENABLE_COLLAPSE_PATCH_DUMP

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace wmtk::bijective_map::collapse_debug {

int get_patch_dump_threshold()
{
    constexpr int kDefaultThreshold = 200;
    if (const char* env = std::getenv("WMTK_COLLAPSE_PATCH_DUMP_THRESHOLD")) {
        try {
            int value = std::stoi(env);
            if (value > 0) {
                return value;
            }
        } catch (...) {
        }
    }
    return kDefaultThreshold;
}

std::unordered_map<int64_t, int> build_face_lookup(const std::vector<int64_t>& id_map)
{
    std::unordered_map<int64_t, int> lookup;
    lookup.reserve(id_map.size());
    for (int i = 0; i < int(id_map.size()); ++i) {
        lookup.emplace(id_map[i], i);
    }
    return lookup;
}

int count_segments_in_patch(
    const std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::unordered_map<int64_t, int>& face_lookup)
{
    int total = 0;
    for (const auto& curve : curves) {
        for (const auto& seg : curve.segments) {
            if (face_lookup.find(seg.f_id) != face_lookup.end()) {
                ++total;
            }
        }
    }
    return total;
}

EdgeMeshDebugData build_edge_mesh_debug_data(
    const std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::unordered_map<int64_t, int>& face_lookup,
    const Eigen::MatrixXd& UV_joint,
    const Eigen::MatrixXi& F_local,
    const std::vector<int64_t>& v_id_map_joint,
    bool use_local_face_vertices)
{
    std::vector<Eigen::Vector2d> vertices;
    std::vector<Eigen::Vector2i> edges;
    std::vector<int> curve_attributes;

    std::unordered_map<int64_t, int> vertex_lookup;
    vertex_lookup.reserve(v_id_map_joint.size());
    for (int i = 0; i < int(v_id_map_joint.size()); ++i) {
        vertex_lookup.emplace(v_id_map_joint[i], i);
    }

    for (int curve_id = 0; curve_id < int(curves.size()); ++curve_id) {
        const auto& curve = curves[curve_id];
        for (const auto& seg : curve.segments) {
            auto it = face_lookup.find(seg.f_id);
            if (it == face_lookup.end()) continue;

            const int local_face_id = it->second;

            Eigen::Vector2d tri_pts[3];
            if (use_local_face_vertices) {
                for (int k = 0; k < 3; ++k) {
                    const int local_vid = F_local(local_face_id, k);
                    if (local_vid < 0 || local_vid >= UV_joint.rows()) {
                        throw std::runtime_error("Error: local vertex id out of range");
                    }
                    tri_pts[k] = UV_joint.row(local_vid).head<2>();
                }
            } else {
                for (int k = 0; k < 3; ++k) {
                    const int64_t global_vid = seg.fv_ids[k];
                    auto vit = vertex_lookup.find(global_vid);
                    if (vit == vertex_lookup.end()) {
                        throw std::runtime_error("Error: vertex not found in vertex_lookup");
                    }
                    tri_pts[k] = UV_joint.row(vit->second).head<2>();
                }
            }

            Eigen::Vector3d bc0;
            Eigen::Vector3d bc1;
            for (int k = 0; k < 3; ++k) {
                bc0[k] = seg.bcs[0](k).to_double();
                bc1[k] = seg.bcs[1](k).to_double();
            }

            Eigen::Vector2d p0 = bc0[0] * tri_pts[0] + bc0[1] * tri_pts[1] + bc0[2] * tri_pts[2];
            Eigen::Vector2d p1 = bc1[0] * tri_pts[0] + bc1[1] * tri_pts[1] + bc1[2] * tri_pts[2];


            const int base_index = static_cast<int>(vertices.size());
            vertices.push_back(p0);
            vertices.push_back(p1);
            edges.emplace_back(base_index, base_index + 1);
            curve_attributes.push_back(curve_id);
        }
    }

    EdgeMeshDebugData data;
    const int num_vertices = static_cast<int>(vertices.size());
    const int num_edges = static_cast<int>(edges.size());

    data.V.resize(num_vertices, 2);
    for (int i = 0; i < num_vertices; ++i) {
        data.V.row(i) = vertices[i];
    }

    data.E.resize(num_edges, 2);
    for (int i = 0; i < num_edges; ++i) {
        data.E.row(i) = edges[i];
    }

    data.curve_ids.resize(num_edges);
    for (int i = 0; i < num_edges; ++i) {
        data.curve_ids(i) = curve_attributes[i];
    }

    return data;
}

} // namespace wmtk::bijective_map::collapse_debug

#endif // WMTK_ENABLE_COLLAPSE_PATCH_DUMP
