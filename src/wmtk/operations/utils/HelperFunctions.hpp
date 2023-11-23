#pragma once
#include <Eigen/Core>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

namespace wmtk::operations::utils {

Eigen::Vector3d nearest_point_to_edge(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& edge_tuple,
    const Tuple& vertex_tuple);

Eigen::Vector3d nearest_point_to_face(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& face_tuple,
    const Tuple& vertex_tuple);

bool is_invert(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& vertex_tuple,
    PrimitiveType type,
    Eigen::Vector3d original_pos = Eigen::Vector3d(0, 0, 0));

void optimize_position(
    Mesh& mesh,
    const MeshAttributeHandle<double> pos_handle,
    const Tuple& vertex_tuple,
    Eigen::Vector3d target_pos,
    Eigen::Vector3d last_best_pos,
    PrimitiveType type);

void push_offset(
    Mesh& mesh,
    const MeshAttributeHandle<double>& pos_handle,
    const Tuple& vertex_tuple,
    const Eigen::Vector3d& projection_pos,
    double len,
    PrimitiveType type,
    Eigen::Vector3d original_pos = Eigen::Vector3d(0, 0, 0));

} // namespace wmtk::operations::utils
