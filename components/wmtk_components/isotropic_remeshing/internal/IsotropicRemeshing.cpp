#include "IsotropicRemeshing.hpp"

#include <wmtk/SimplicialComplex.hpp>

namespace wmtk {
namespace components {
namespace internal {

IsotropicRemeshing::IsotropicRemeshing(TriMesh* mesh, const double length, const bool lock_boundary)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
    , m_lock_boundary{lock_boundary}
{
    if (!m_lock_boundary) {
        throw std::runtime_error("free boundary is not implemented yet");
    }
}

void IsotropicRemeshing::remeshing(const long iterations)
{
    for (long i = 0; i < iterations; ++i) {
        split_long_edges();
        collapse_short_edges();
        flip_edges_for_valence();
        smooth_vertices();
    }
}

void IsotropicRemeshing::split_long_edges()
{
    throw "Very bad implementation. DO NOT USE!";
    // This is not how it's supposed to be done.
    // Do not copy!!!

    const std::vector<Tuple> edges = m_mesh->get_all(wmtk::PrimitiveType::Edge);

    MeshAttributeHandle<double> pts_attr =
        m_mesh->get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
    auto pts_accessor = m_mesh->create_const_accessor(pts_attr);

    for (const Tuple& e : edges) {
        if (!m_mesh->is_valid(e)) {
            continue;
        }

        const Tuple v0 = e;
        const Tuple v1 = m_mesh->switch_tuple(e, PrimitiveType::Vertex);
        const Eigen::Vector3d p0 = pts_accessor.vector_attribute(v0);
        const Eigen::Vector3d p1 = pts_accessor.vector_attribute(v1);
        const double l = (p1 - p0).norm();

        if (l > m_length_max) {
            m_mesh->split_edge(e);
        }
    }
}

void IsotropicRemeshing::collapse_short_edges()
{
    throw "Very bad implementation. DO NOT USE!";
    // This is not how it's supposed to be done.
    // Do not copy!!!

    const std::vector<Tuple> edges = m_mesh->get_all(wmtk::PrimitiveType::Edge);

    MeshAttributeHandle<double> pts_attr =
        m_mesh->get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
    auto pts_accessor = m_mesh->create_const_accessor(pts_attr);

    for (const Tuple& e : edges) {
        if (!m_mesh->is_valid(e)) {
            continue;
        }

        const Tuple v0 = e;
        const Tuple v1 = m_mesh->switch_tuple(e, PrimitiveType::Vertex);
        const Eigen::Vector3d p0 = pts_accessor.vector_attribute(v0);
        const Eigen::Vector3d p1 = pts_accessor.vector_attribute(v1);
        const double l = (p1 - p0).norm();

        if (l > m_length_min) {
            m_mesh->collapse_edge(e);
        }
    }
}

void IsotropicRemeshing::flip_edges_for_valence()
{
    throw "implementation missing";
}

void IsotropicRemeshing::smooth_vertices()
{
    throw "Very bad implementation. DO NOT USE!";
    // TODO this implementation of smoothing is NOT correct for isotropic remeshing!
    // TODO operators should be used here instead of modifying points directly

    const std::vector<Tuple> vertices = m_mesh->get_all(wmtk::PrimitiveType::Vertex);
    MeshAttributeHandle<double> pts_attr =
        m_mesh->get_attribute_handle<double>("position", wmtk::PrimitiveType::Vertex);
    auto pts_accessor = m_mesh->create_accessor(pts_attr);

    for (const Tuple& v : vertices) {
        const Eigen::Vector3d p = pts_accessor.vector_attribute(v);

        // get neighbors
        std::vector<Simplex> one_ring = SimplicialComplex::vertex_one_ring(v, *m_mesh);
        Eigen::Vector3d p_mid(0, 0, 0);
        for (const Simplex& neigh : one_ring) {
            p_mid += pts_accessor.vector_attribute(neigh.tuple());
        }
        p_mid /= one_ring.size();

        pts_accessor.vector_attribute(v) = p_mid;
    }
}


} // namespace internal
} // namespace components
} // namespace wmtk