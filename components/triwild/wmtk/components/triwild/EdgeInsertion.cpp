#include <wmtk/utils/Rational.hpp>
#include "TriWildMesh.h"

#include <bitset>
#include <wmtk/utils/Delaunay.hpp>
#include "wmtk/utils/Logger.hpp"

#include <tbb/concurrent_priority_queue.h>
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

#include <random>
#include <unordered_set>

namespace wmtk::components::triwild {

// void TriWildMesh::init_from_delaunay_box_mesh(const std::vector<Eigen::Vector3d>& vertices)
// {
//     ///points for delaunay
//     std::vector<wmtk::delaunay::Point3D> points(vertices.size());
//     // add points from surface
//     for (int i = 0; i < vertices.size(); i++) {
//         for (int j = 0; j < 3; j++) points[i][j] = vertices[i][j];
//     }

//     // bbox
//     double delta = m_params.diag_l / 15.0;
//     Vector3d box_min(m_params.min[0] - delta, m_params.min[1] - delta, m_params.min[2] - delta);
//     Vector3d box_max(m_params.max[0] + delta, m_params.max[1] + delta, m_params.max[2] + delta);

//     // add corners of domain
//     for (int i = 0; i < 8; i++) {
//         Vector3d p;
//         std::bitset<sizeof(int) * 8> a(i);
//         for (int j = 0; j < 3; j++) {
//             if (a.test(j)) {
//                 p[j] = box_max[j];
//             } else {
//                 p[j] = box_min[j];
//             }
//         }
//         points.push_back({{p[0], p[1], p[2]}});
//     }

//     const double voxel_resolution = m_params.diag_l / 20.0;
//     std::array<int, 3> N; // number of grid points per dimension
//     std::array<double, 3> h; // distance between grid points per dimension
//     for (int i = 0; i < 3; i++) {
//         const double D = box_max[i] - box_min[i];
//         N[i] = (D / voxel_resolution) + 1;
//         h[i] = D / N[i];
//     }

//     std::array<std::vector<double>, 3> ds;
//     for (int i = 0; i < 3; i++) {
//         ds[i].push_back(box_min[i]);
//         for (int j = 0; j < N[i] - 1; j++) {
//             ds[i].push_back(box_min[i] + h[i] * (j + 1));
//         }
//         ds[i].push_back(box_max[i]);
//     }

//     const double min_dis = voxel_resolution * voxel_resolution / 4;
//     //    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
//     for (int i = 0; i < ds[0].size(); i++) {
//         for (int j = 0; j < ds[1].size(); j++) {
//             for (int k = 0; k < ds[2].size(); k++) {
//                 if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1) &&
//                     (k == 0 || k == ds[2].size() - 1)) {
//                     continue;
//                 }
//                 const Vector3d p(ds[0][i], ds[1][j], ds[2][k]);

//                 Eigen::Vector3d n;
//                 const double sqd = m_envelope.nearest_point(p, n);

//                 if (sqd < min_dis) {
//                     continue;
//                 }
//                 points.push_back({{ds[0][i], ds[1][j], ds[2][k]}});
//             }
//         }
//     }

//     m_params.box_min = box_min;
//     m_params.box_max = box_max;

//     ///delaunay
//     auto [unused_points, tets] = delaunay::delaunay3D(points);
//     logger().info("after delauney tets.size() {}  points.size() {}", tets.size(), points.size());

//     // conn
//     init(points.size(), tets);
//     logger().info("init finished");
//     // attr
//     m_vertex_attribute.m_attributes.resize(points.size());
//     m_tet_attribute.m_attributes.resize(tets.size());
//     m_face_attribute.m_attributes.resize(tets.size() * 4);
//     for (int i = 0; i < vert_capacity(); i++) {
//         m_vertex_attribute[i].m_pos = Vector3r(points[i][0], points[i][1], points[i][2]);
//         m_vertex_attribute[i].m_posf = Vector3d(points[i][0], points[i][1], points[i][2]);
//     }
//     logger().info("attribute vectors created");
// }

} // namespace wmtk::components::triwild