#pragma once

#include <wmtk/Mesh.hpp>

#include <string>
namespace wmtk {
    class TriMesh;
}

namespace wmtk::mesh_utils {
template <typename Mat>
inline attribute::MeshAttributeHandle set_matrix_attribute(
    const Mat& data,
    const std::string& name,
    const PrimitiveType& type,
    Mesh& mesh)
{
    attribute::MeshAttributeHandle handle =
        mesh.template register_attribute<typename Mat::Scalar>(name, type, data.cols());


    auto thandle = handle.as<typename Mat::Scalar>();

    auto accessor = mesh.create_accessor(thandle);
    const auto tuples = mesh.get_all(type);
    for (size_t i = 0; i < tuples.size(); ++i) {
        const auto& t = tuples[i];
        accessor.vector_attribute(t) = data.row(i).transpose();
    }

    return handle;
}

/**
 * @brief compute area vector of face
 */
Eigen::Vector3d
compute_face_normal_area_weighted(const TriMesh& m, const attribute::Accessor<double>& pos, const Tuple& f);

/**
 * @brief compute the normalized face normal based on the vertex positions
 */
Eigen::Vector3d compute_face_normal(const TriMesh& m, const attribute::Accessor<double>& pos, const Tuple& f);

/**
 * @brief compute the normalized vertex normal from the incident area weighted face normals
 */
Eigen::Vector3d
compute_vertex_normal(const TriMesh& m, const attribute::Accessor<double>& pos, const Tuple& v);

} // namespace wmtk::mesh_utils
