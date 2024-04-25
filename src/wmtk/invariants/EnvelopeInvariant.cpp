#include "EnvelopeInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>


#include <fastenvelope/FastEnvelope.h>
#include <SimpleBVH/BVH.hpp>


namespace wmtk {

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
} // namespace wmtk
namespace wmtk::invariants {

EnvelopeInvariant::EnvelopeInvariant(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    double envelope_size,
    const attribute::MeshAttributeHandle& coordinate)
    : Invariant(coordinate.mesh())
    , m_coordinate_handle(coordinate)
    , m_envelope_size(envelope_size)
{
    const auto& envelope_mesh = envelope_mesh_coordinate.mesh();

    assert(envelope_mesh_coordinate.holds<Rational>() || envelope_mesh_coordinate.holds<double>());

    if (envelope_mesh_coordinate.holds<Rational>()) {
        // for rational
        const attribute::Accessor<Rational> accessor =
            envelope_mesh.create_const_accessor(envelope_mesh_coordinate.as<Rational>());


        if (envelope_mesh.top_simplex_type() == PrimitiveType::Triangle) {
            std::vector<Eigen::Vector3d> vertices;
            std::vector<Eigen::Vector3i> faces;

            int count = 0;
            assert(accessor.dimension() == 3);

            const std::vector<Tuple>& facest = envelope_mesh.get_all(wmtk::PrimitiveType::Triangle);
            for (const auto& f : facest) {
                Eigen::Vector3d p0 = accessor.const_vector_attribute(f).cast<double>();
                Eigen::Vector3d p1 =
                    accessor.const_vector_attribute(envelope_mesh.switch_tuple(f, PV))
                        .cast<double>();
                Eigen::Vector3d p2 =
                    accessor.const_vector_attribute(envelope_mesh.switch_tuples(f, {PE, PV}))
                        .cast<double>();

                faces.emplace_back(count, count + 1, count + 2);
                vertices.push_back(p0);
                vertices.push_back(p1);
                vertices.push_back(p2);

                count += 3;
            }

            m_envelope =
                std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);

        } else if (envelope_mesh.top_simplex_type() == PrimitiveType::Edge) {
            logger().warn("Envelope for edge mesh is using sampling");

            int64_t count = 0;
            int64_t index = 0;

            const std::vector<Tuple>& edgest = envelope_mesh.get_all(wmtk::PrimitiveType::Edge);

            Eigen::MatrixXd vertices(2 * edgest.size(), accessor.dimension());
            Eigen::MatrixXi edges(edgest.size(), 2);

            for (const auto& e : edgest) {
                auto p0 = accessor.const_vector_attribute(e).cast<double>();
                auto p1 = accessor.const_vector_attribute(envelope_mesh.switch_tuple(e, PV))
                              .cast<double>();

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
    } else if (envelope_mesh_coordinate.holds<double>()) {
        // for double
        const attribute::Accessor<double> accessor =
            envelope_mesh.create_const_accessor(envelope_mesh_coordinate.as<double>());


        if (envelope_mesh.top_simplex_type() == PrimitiveType::Triangle) {
            std::vector<Eigen::Vector3d> vertices;
            std::vector<Eigen::Vector3i> faces;

            int count = 0;
            assert(accessor.dimension() == 3);

            const std::vector<Tuple>& facest = envelope_mesh.get_all(wmtk::PrimitiveType::Triangle);
            for (const auto& f : facest) {
                Eigen::Vector3d p0 = accessor.const_vector_attribute(f);
                Eigen::Vector3d p1 =
                    accessor.const_vector_attribute(envelope_mesh.switch_tuple(f, PV));
                Eigen::Vector3d p2 =
                    accessor.const_vector_attribute(envelope_mesh.switch_tuples(f, {PE, PV}));

                faces.emplace_back(count, count + 1, count + 2);
                vertices.push_back(p0);
                vertices.push_back(p1);
                vertices.push_back(p2);

                count += 3;
            }

            m_envelope =
                std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);

        } else if (envelope_mesh.top_simplex_type() == PrimitiveType::Edge) {
            logger().warn("Envelope for edge mesh is using sampling");

            int64_t count = 0;
            int64_t index = 0;

            const std::vector<Tuple>& edgest = envelope_mesh.get_all(wmtk::PrimitiveType::Edge);

            Eigen::MatrixXd vertices(2 * edgest.size(), accessor.dimension());
            Eigen::MatrixXi edges(edgest.size(), 2);

            for (const auto& e : edgest) {
                auto p0 = accessor.const_vector_attribute(e);
                auto p1 = accessor.const_vector_attribute(envelope_mesh.switch_tuple(e, PV));

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
    } else {
        throw std::runtime_error("Envelope mesh handle type invlid");
    }
}

bool EnvelopeInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (top_dimension_tuples_after.empty()) return true;

    assert(m_coordinate_handle.holds<Rational>() || m_coordinate_handle.holds<double>());

    if (m_coordinate_handle.holds<Rational>()) {
        const attribute::Accessor<Rational> accessor =
            mesh().create_const_accessor(m_coordinate_handle.as<Rational>());
        const auto type = mesh().top_simplex_type();

        if (m_envelope) {
            assert(accessor.dimension() == 3);

            std::vector<Tuple> faces;

            if (type == PrimitiveType::Triangle) {
                std::array<Eigen::Vector3d, 3> triangle;

                for (const Tuple& tuple : top_dimension_tuples_after) {
                    faces = faces_single_dimension_tuples(
                        mesh(),
                        simplex::Simplex(type, tuple),
                        PrimitiveType::Vertex);

                    triangle[0] = accessor.const_vector_attribute(faces[0]).cast<double>();
                    triangle[1] = accessor.const_vector_attribute(faces[1]).cast<double>();
                    triangle[2] = accessor.const_vector_attribute(faces[2]).cast<double>();

                    if (m_envelope->is_outside(triangle)) return false;
                }

                return true;
            } else if (type == PrimitiveType::Edge) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    faces = faces_single_dimension_tuples(
                        mesh(),
                        simplex::Simplex(type, tuple),
                        PrimitiveType::Vertex);

                    Eigen::Vector3d p0 = accessor.const_vector_attribute(faces[0]).cast<double>();
                    Eigen::Vector3d p1 = accessor.const_vector_attribute(faces[1]).cast<double>();

                    if (m_envelope->is_outside(p0, p1)) return false;
                }

                return true;
            } else if (type == PrimitiveType::Vertex) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple).cast<double>();

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
                    SimpleBVH::VectorMax3d p0 =
                        accessor.const_vector_attribute(tuple).cast<double>();
                    SimpleBVH::VectorMax3d p1 =
                        accessor.const_vector_attribute(mesh().switch_tuple(tuple, PV))
                            .cast<double>();

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
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple).cast<double>();
                    m_bvh->nearest_facet(p, nearest_point, sq_dist);
                    if (sq_dist > m_envelope_size * m_envelope_size) return false;
                }

                return true;
            } else {
                throw std::runtime_error("Invalid mesh type");
            }
            return true;
        }
    } else if (m_coordinate_handle.holds<double>()) {
        const attribute::Accessor<double> accessor =
            mesh().create_const_accessor(m_coordinate_handle.as<double>());
        const auto type = mesh().top_simplex_type();

        if (m_envelope) {
            assert(accessor.dimension() == 3);

            std::vector<Tuple> faces;

            if (type == PrimitiveType::Triangle) {
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
                    SimpleBVH::VectorMax3d p0 =
                        accessor.const_vector_attribute(tuple).cast<double>();
                    SimpleBVH::VectorMax3d p1 =
                        accessor.const_vector_attribute(mesh().switch_tuple(tuple, PV))
                            .cast<double>();

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
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple).cast<double>();
                    m_bvh->nearest_facet(p, nearest_point, sq_dist);
                    if (sq_dist > m_envelope_size * m_envelope_size) return false;
                }

                return true;
            } else {
                throw std::runtime_error("Invalid mesh type");
            }
            return true;
        }
    } else {
        throw std::runtime_error("Envelope mesh handle type invlid");
    }
}


} // namespace wmtk::invariants
