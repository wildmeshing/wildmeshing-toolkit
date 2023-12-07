#include "VertexTangentialLaplacianSmooth.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "VertexLaplacianSmooth.hpp"

namespace wmtk::operations {
void OperationSettings<tri_mesh::VertexTangentialLaplacianSmooth>::initialize_invariants(
    const TriMesh& m)
{
    smooth_settings.initialize_invariants(m);
}
} // namespace wmtk::operations

namespace wmtk::operations::tri_mesh {
VertexTangentialLaplacianSmooth::VertexTangentialLaplacianSmooth(
    Mesh& m,
    const Simplex& t,
    const OperationSettings<VertexTangentialLaplacianSmooth>& settings)
    : VertexLaplacianSmooth(m, t, settings)
    , m_settings{settings}
{}
std::string VertexTangentialLaplacianSmooth::name() const
{
    return "tri_mesh_vertex_tangential_smooth";
}

bool VertexTangentialLaplacianSmooth::execute()
{
    const Eigen::Vector3d p = m_pos_accessor.vector_attribute(input_tuple().tuple());

    if (!tri_mesh::VertexLaplacianSmooth::execute()) {
        return false;
    }
    const Tuple tup = tri_mesh::VertexAttributesUpdateBase::return_tuple();


    assert(mesh().is_valid_slow(tup));
    const Eigen::Vector3d g = m_pos_accessor.vector_attribute(tup); // center of gravity

    if (mesh().is_boundary_vertex(tup)) {
        //
        Tuple t0 = tup;
        while (!mesh().is_boundary_edge(t0)) {
            t0 = mesh().switch_edge(mesh().switch_face(t0));
        }
        const Tuple v0 = mesh().switch_vertex(t0);

        Tuple t1 = mesh().switch_edge(tup);
        while (!mesh().is_boundary_edge(t1)) {
            t1 = mesh().switch_edge(mesh().switch_face(t1));
        }
        const Tuple v1 = mesh().switch_vertex(t1);

        const Eigen::Vector3d p0 = m_pos_accessor.vector_attribute(v0);
        const Eigen::Vector3d p1 = m_pos_accessor.vector_attribute(v1);

        const Eigen::Vector3d tang = (p1 - p0).normalized();
        if (tang.squaredNorm() < 1e-10) {
            return false;
        }

        m_pos_accessor.vector_attribute(tup) =
            p + m_settings.damping_factor * tang * tang.transpose() * (g - p);

    } else {
        const Eigen::Vector3d n = mesh_utils::compute_vertex_normal(mesh(), m_pos_accessor, tup);

        if (n.squaredNorm() < 1e-10) {
            return false;
        }

        // following Botsch&Kobbelt - Remeshing for Multiresolution Modeling
        m_pos_accessor.vector_attribute(tup) =
            p +
            m_settings.damping_factor * (Eigen::Matrix3d::Identity() - n * n.transpose()) * (g - p);
    }

    return true;
}


} // namespace wmtk::operations::tri_mesh
