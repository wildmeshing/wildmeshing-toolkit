#include "mesh_utils.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/closed_star.hpp>

namespace wmtk::mesh_utils {
Eigen::Vector3d
compute_face_normal_area_weighted(const TriMesh& m, const Accessor<double>& pos, const Tuple& f)
{
    const Tuple v0 = f;
    const Tuple v1 = m.switch_vertex(f);
    const Tuple v2 = m.switch_vertex(m.switch_edge(f));

    const Eigen::Vector3d p0 = pos.vector_attribute(v0);
    const Eigen::Vector3d p1 = pos.vector_attribute(v1);
    const Eigen::Vector3d p2 = pos.vector_attribute(v2);
    return ((p0 - p2).cross(p1 - p2));
}

Eigen::Vector3d compute_face_normal(const TriMesh& m, const Accessor<double>& pos, const Tuple& f)
{
    return compute_face_normal_area_weighted(m, pos, f).normalized();
}

Eigen::Vector3d compute_vertex_normal(const TriMesh& m, const Accessor<double>& pos, const Tuple& v)
{
    const simplex::SimplexCollection closed_star =
        simplex::closed_star(m, simplex::Simplex::vertex(v));

    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (const simplex::Simplex& f : closed_star.simplex_vector(PrimitiveType::Face)) {
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
