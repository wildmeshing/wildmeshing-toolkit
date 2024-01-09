#include "EnvelopeInvariant.hpp"

#include <wmtk/Mesh.hpp>

#include <fastenvelope/FastEnvelope.h>


namespace wmtk::invariants {
EnvelopeInvariant::EnvelopeInvariant(
    const Mesh& m,
    const TypedAttributeHandle<double>& coordinate,
    const Mesh& envelope_mesh,
    const TypedAttributeHandle<double>& envelope_mesh_coordinate,
    double envelope_size)
    : Invariant(m)
    , m_coordinate_handle(coordinate)
{
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;

    ConstAccessor<double> accessor = envelope_mesh.create_accessor(envelope_mesh_coordinate);


    if (envelope_mesh.top_simplex_type() == PrimitiveType::Face) {
        int64_t count = 0;
        assert(accessor.dimension() == 3);

        const std::vector<Tuple>& facest = m.get_all(wmtk::PrimitiveType::Face);
        for (const auto& f : facest) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(envelope_mesh.switch_vertex(f));
            Eigen::Vector3d p2 = accessor.const_vector_attribute(
                envelope_mesh.switch_vertex(envelope_mesh.switch_edge(f)));

            faces.emplace_back(count, count + 1, count + 2);
            vertices.push_back(p0);
            vertices.push_back(p1);
            vertices.push_back(p2);

            count += 3;
        }
    } else {
        throw std::runtime_error("Envelope works only for tri meshes");
    }


    m_envelope = std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);
}

bool EnvelopeInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinate_handle);
    assert(accessor.dimension() == 3);

    if (mesh().top_simplex_type() == PrimitiveType::Face) {
        std::array<Eigen::Vector3d, 3> triangle;

        for (const Tuple& tuple : top_dimension_tuples_after) {
            triangle[0] = accessor.const_vector_attribute(tuple);
            triangle[1] = accessor.const_vector_attribute(mesh().switch_vertex(tuple));
            triangle[2] =
                accessor.const_vector_attribute(mesh().switch_vertex(mesh().switch_edge(tuple)));

            if (m_envelope->is_outside(triangle)) return false;
        }

        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Edge) {
        for (const Tuple& tuple : top_dimension_tuples_after) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(tuple);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(mesh().switch_vertex(tuple));

            if (m_envelope->is_outside(p0, p1)) return false;
        }

        return true;
    } else if (mesh().top_simplex_type() == PrimitiveType::Vertex) {
        for (const Tuple& tuple : top_dimension_tuples_after) {
            Eigen::Vector3d p = accessor.const_vector_attribute(tuple);

            if (m_envelope->is_outside(p)) return false;
        }

        return true;
    } else {
        throw std::runtime_error("Invalid mesh type");
    }
    return true;
}


} // namespace wmtk::invariants