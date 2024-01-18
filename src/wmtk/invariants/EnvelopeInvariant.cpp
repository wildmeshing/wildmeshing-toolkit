#include "EnvelopeInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>


#include <fastenvelope/FastEnvelope.h>
#include <SimpleBVH/BVH.hpp>


namespace wmtk::invariants {

EnvelopeInvariant::EnvelopeInvariant(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    double envelope_size,
    const attribute::MeshAttributeHandle& coordinate)
    : Invariant(coordinate.mesh())
    , m_coordinate_handle(coordinate.as<double>())
    , m_envelope_size(envelope_size)
{
    const auto& envelope_mesh = envelope_mesh_coordinate.mesh();
    ConstAccessor<double> accessor =
        envelope_mesh.create_accessor(envelope_mesh_coordinate.as<double>());


    if (envelope_mesh.top_simplex_type() == PrimitiveType::Face) {
        std::vector<Eigen::Vector3d> vertices;
        std::vector<Eigen::Vector3i> faces;

        int count = 0;
        assert(accessor.dimension() == 3);

        const std::vector<Tuple>& facest = envelope_mesh.get_all(wmtk::PrimitiveType::Face);
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

        m_envelope = std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);

    } else if (envelope_mesh.top_simplex_type() == PrimitiveType::Edge) {
        logger().warn("Envelope for edge mesh is using sampling");

        int64_t count = 0;
        int64_t index = 0;

        const std::vector<Tuple>& edgest = envelope_mesh.get_all(wmtk::PrimitiveType::Edge);

        Eigen::MatrixXd vertices(2 * edgest.size(), accessor.dimension());
        Eigen::MatrixXi edges(edgest.size(), 2);

        for (const auto& e : edgest) {
            auto p0 = accessor.const_vector_attribute(e);
            auto p1 = accessor.const_vector_attribute(envelope_mesh.switch_vertex(e));

            edges.row(index) << count, count + 1;
            vertices.row(2 * index) = p0;
            vertices.row(2 * index + 1) = p1;

            count += 2;
            ++index;
        }

        m_bvh = std::make_shared<SimpleBVH::BVH>();
        m_bvh->init(vertices, edges, 1e-10);
    } else {
        throw std::runtime_error("Envelope works only for tri/edges meshes");
    }
}

bool EnvelopeInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (top_dimension_tuples_after.empty()) return true;

    ConstAccessor<double> accessor = mesh().create_accessor(m_coordinate_handle);
    const auto type = mesh().top_simplex_type();

    if (m_envelope) {
        assert(accessor.dimension() == 3);

        std::vector<Tuple> faces;

        if (type == PrimitiveType::Face) {
            std::array<Eigen::Vector3d, 3> triangle;

            for (const Tuple& tuple : top_dimension_tuples_after) {
                faces = faces_single_dimension_tuples(
                    mesh(),
                    simplex::Simplex(type, tuple),
                    PrimitiveType::Vertex);

                triangle[0] = accessor.const_vector_attribute(faces[0]);
                triangle[1] = accessor.const_vector_attribute(faces[1]);
                triangle[2] = accessor.const_vector_attribute(faces[2]);

                if (m_envelope->is_outside(triangle)) return false;
            }

            return true;
        } else if (type == PrimitiveType::Edge) {
            for (const Tuple& tuple : top_dimension_tuples_after) {
                faces = faces_single_dimension_tuples(
                    mesh(),
                    simplex::Simplex(type, tuple),
                    PrimitiveType::Vertex);

                Eigen::Vector3d p0 = accessor.const_vector_attribute(faces[0]);
                Eigen::Vector3d p1 = accessor.const_vector_attribute(faces[1]);

                if (m_envelope->is_outside(p0, p1)) return false;
            }

            return true;
        } else if (type == PrimitiveType::Vertex) {
            for (const Tuple& tuple : top_dimension_tuples_after) {
                Eigen::Vector3d p = accessor.const_vector_attribute(tuple);

                if (m_envelope->is_outside(p)) return false;
            }

            return true;
        } else {
            throw std::runtime_error("Invalid mesh type");
        }
        return true;
    } else {
        assert(m_bvh);

        SimpleBVH::VectorMax3d nearest_point;
        double sq_dist;

        const double d = m_envelope_size;
        const double real_envelope = m_envelope_size - d / sqrt(accessor.dimension());
        const double real_envelope_2 = real_envelope * real_envelope;

        if (type == PrimitiveType::Edge) {
            std::vector<SimpleBVH::VectorMax3d> pts;

            for (const Tuple& tuple : top_dimension_tuples_after) {
                SimpleBVH::VectorMax3d p0 = accessor.const_vector_attribute(tuple);
                SimpleBVH::VectorMax3d p1 =
                    accessor.const_vector_attribute(mesh().switch_vertex(tuple));

                const int64_t N = (p0 - p1).norm() / d + 1;
                pts.reserve(pts.size() + N);

                for (int64_t n = 0; n <= N; n++) {
                    auto tmp = p0 * (double(n) / N) + p1 * (N - double(n)) / N;
                    pts.push_back(tmp);
                }
            }

            auto current_point = pts[0];

            int prev_facet = m_bvh->nearest_facet(current_point, nearest_point, sq_dist);
            if (sq_dist > real_envelope_2) return false;

            for (const auto& v : pts) {
                sq_dist = (v - nearest_point).squaredNorm();
                m_bvh->nearest_facet_with_hint(v, prev_facet, nearest_point, sq_dist);
                if (sq_dist > real_envelope_2) return false;
            }

            return true;
        } else if (type == PrimitiveType::Vertex) {
            for (const Tuple& tuple : top_dimension_tuples_after) {
                Eigen::Vector3d p = accessor.const_vector_attribute(tuple);
                m_bvh->nearest_facet(p, nearest_point, sq_dist);
                if (sq_dist > m_envelope_size * m_envelope_size) return false;
            }

            return true;
        } else {
            throw std::runtime_error("Invalid mesh type");
        }
        return true;
    }
}


} // namespace wmtk::invariants