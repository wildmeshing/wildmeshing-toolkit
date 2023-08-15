#pragma once

#include <wmtk/Mesh.hpp>
#include <wmtk/SimplicialComplex.hpp>

#include <string>

namespace wmtk::mesh_utils {
template <typename Mat>
inline MeshAttributeHandle<typename Mat::Scalar> set_matrix_attribute(
    const Mat& data,
    const std::string& name,
    const PrimitiveType& type,
    Mesh& mesh)
{
    MeshAttributeHandle<typename Mat::Scalar> handle =
        mesh.register_attribute<typename Mat::Scalar>(name, type, data.cols());

    auto accessor = mesh.create_accessor(handle);
    const auto tuples = mesh.get_all(type);
    for (size_t i = 0; i < tuples.size(); ++i) {
        const auto& t = tuples[i];
        accessor.vector_attribute(t) = data.row(i).transpose();
    }

    return handle;
}

inline Eigen::Vector3d
compute_face_normal_area_weighted(const Mesh& m, const Accessor<double>& pos, const Tuple& f)
{
    const Tuple v0 = f;
    const Tuple v1 = m.switch_vertex(f);
    const Tuple v2 = m.switch_vertex(m.switch_edge(f));

    const Eigen::Vector3d p0 = pos.vector_attribute(v0);
    const Eigen::Vector3d p1 = pos.vector_attribute(v1);
    const Eigen::Vector3d p2 = pos.vector_attribute(v2);
    return ((p0 - p2).cross(p1 - p2));
}

inline Eigen::Vector3d
compute_face_normal(const Mesh& m, const Accessor<double>& pos, const Tuple& f)
{
    return compute_face_normal_area_weighted(m, pos, f).normalized();
}

inline Eigen::Vector3d
compute_vertex_normal(const Mesh& m, const Accessor<double>& pos, const Tuple& v)
{
    const SimplicialComplex closed_star = SimplicialComplex::closed_star(m, Simplex::vertex(v));

    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (const Simplex& f : closed_star.get_faces()) {
        Tuple t = f.tuple();
        if (!m.is_ccw(t)) {
            t = m.switch_vertex(t);
        }
        n += compute_face_normal_area_weighted(m, pos, t);
    }
    n.normalize();
    return n;
}

} // namespace wmtk::mesh_utils