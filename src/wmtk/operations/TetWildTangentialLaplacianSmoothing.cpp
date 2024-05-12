#include "TetWildTangentialLaplacianSmoothing.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/Accessor.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include <wmtk/simplex/link.hpp>

namespace wmtk::operations {

TetWildTangentialLaplacianSmoothing::TetWildTangentialLaplacianSmoothing(
    Mesh& m,
    const TypedAttributeHandle<Rational>& coordinate,
    double damping_factor)
    : AttributesUpdate(m)
    , m_coordinate_handle(coordinate)
    , m_damping_factor(damping_factor)
{
    assert(mesh().top_simplex_type() == PrimitiveType::Surface);
}

std::vector<simplex::Simplex> TetWildTangentialLaplacianSmoothing::execute(
    const simplex::Simplex& simplex)
{
    // normal laplacian smoothing
    auto accessor = mesh().create_accessor(m_coordinate_handle);
    const auto old_pos = accessor.const_vector_attribute(simplex.tuple()).cast<double>();

    const auto one_ring = simplex::link(mesh(), simplex).simplex_vector(PrimitiveType::Vertex);

    Vector3d new_pos(0, 0, 0);
    for (const auto& v : one_ring) {
        new_pos = new_pos + accessor.const_vector_attribute(v).cast<double>();
    }

    new_pos = new_pos / one_ring.size();

    // tangential
    if (mesh().is_boundary(PrimitiveType::Vertex, simplex.tuple())) {
        Tuple t0 = simplex.tuple();
        while (!mesh().is_boundary(PrimitiveType::Edge, t0)) {
            t0 = mesh().switch_tuples(t0, {PrimitiveType::Triangle, PrimitiveType::Edge});
        }

        const Tuple v0 = mesh().switch_tuple(t0, PrimitiveType::Vertex);

        Tuple t1 = mesh().switch_tuple(simplex.tuple(), PrimitiveType::Edge);
        while (!mesh().is_boundary(PrimitiveType::Edge, t1)) {
            t1 = mesh().switch_tuples(t1, {PrimitiveType::Triangle, PrimitiveType::Edge});
        }
        const Tuple v1 = mesh().switch_tuple(t1, PrimitiveType::Vertex);

        const Vector3d p0 = accessor.const_vector_attribute(v0).cast<double>();
        const Vector3d p1 = accessor.const_vector_attribute(v1).cast<double>();

        const Vector3d tang = (p1 - p0).normalized();

        new_pos = old_pos + m_damping_factor * tang * tang.transpose() * (new_pos - old_pos);
    } else {
        Tuple t = simplex.tuple();

        Vector3d n(0, 0, 0);

        for (int64_t i = 0; i < one_ring.size(); ++i) {
            const Vector3d p0 = accessor.const_vector_attribute(t).cast<double>();
            const Vector3d p1 =
                accessor.const_vector_attribute(mesh().switch_tuple(t, PrimitiveType::Vertex))
                    .cast<double>();
            const Vector3d p2 =
                accessor
                    .const_vector_attribute(
                        mesh().switch_tuples(t, {PrimitiveType::Edge, PrimitiveType::Vertex}))
                    .cast<double>();

            Vector3d weighted_ni = (p0 - p2).cross(p1 - p2);

            n = n + weighted_ni;

            t = mesh().switch_tuples(t, {PrimitiveType::Triangle, PrimitiveType::Edge});
        }

        n.normalized();
        new_pos = old_pos + m_damping_factor * (Eigen::Matrix3d::Identity() - n * n.transpose()) *
                                (new_pos - old_pos);
    }

    accessor.vector_attribute(simplex) = Vector3r(
        Rational(new_pos[0], true),
        Rational(new_pos[1], true),
        Rational(new_pos[2], true));

    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations