#include "VertexTangentialLaplacianSmooth.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "VertexLaplacianSmooth.hpp"


namespace wmtk::operations {
VertexTangentialLaplacianSmooth::VertexTangentialLaplacianSmooth(
    attribute::MeshAttributeHandle& handle,
    const double damping_factor)
    : VertexLaplacianSmooth(handle)
    , m_damping_factor(damping_factor)
{}

bool VertexTangentialLaplacianSmooth::operator()(Mesh& mesh, const simplex::Simplex& simplex)
{
    auto accessor = mesh.create_accessor<double>(m_attibute_handle);
    const Eigen::Vector3d p = accessor.vector_attribute(simplex.tuple());

    if (!VertexLaplacianSmooth::operator()(mesh, simplex)) return false;

    const Tuple tup = simplex.tuple();


    assert(mesh.is_valid_slow(tup));
    const Eigen::Vector3d g = accessor.vector_attribute(tup); // center of gravity

    if (mesh.is_boundary_vertex(tup)) {
        //
        Tuple t0 = tup;
        while (!mesh.is_boundary_edge(t0)) {
            t0 = mesh.switch_edge(mesh.switch_face(t0));
        }
        const Tuple v0 = mesh.switch_vertex(t0);

        Tuple t1 = mesh.switch_edge(tup);
        while (!mesh.is_boundary_edge(t1)) {
            t1 = mesh.switch_edge(mesh.switch_face(t1));
        }
        const Tuple v1 = mesh.switch_vertex(t1);

        const Eigen::Vector3d p0 = accessor.vector_attribute(v0);
        const Eigen::Vector3d p1 = accessor.vector_attribute(v1);

        const Eigen::Vector3d tang = (p1 - p0).normalized();
        if (tang.squaredNorm() < 1e-10) {
            return false;
        }

        accessor.vector_attribute(tup) = p + m_damping_factor * tang * tang.transpose() * (g - p);

    } else {
        const Eigen::Vector3d n =
            mesh_utils::compute_vertex_normal(static_cast<TriMesh&>(mesh), accessor, tup);

        if (n.squaredNorm() < 1e-10) {
            return false;
        }

        // following Botsch&Kobbelt - Remeshing for Multiresolution Modeling
        accessor.vector_attribute(tup) =
            p + m_damping_factor * (Eigen::Matrix3d::Identity() - n * n.transpose()) * (g - p);
    }

    return true;
}


} // namespace wmtk::operations
