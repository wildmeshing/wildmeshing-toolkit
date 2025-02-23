#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnull-dereference"
#endif
#include <Eigen/Dense>
#if defined(__GNUG__) && !defined(__clang__)
#pragma GCC diagnostic pop
#endif
#include "EnvelopeInvariant.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/Logger.hpp>


#include <fastenvelope/FastEnvelope.h>
#include <SimpleBVH/BVH.hpp>


namespace wmtk {
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PV = PrimitiveType::Vertex;

template <typename T>
std::shared_ptr<fastEnvelope::FastEnvelope> init_fast_envelope(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    const double envelope_size)
{
    const auto& envelope_mesh = envelope_mesh_coordinate.mesh();

    const attribute::Accessor<T> accessor =
        envelope_mesh.create_const_accessor(envelope_mesh_coordinate.as<T>());

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;

    int count = 0;
    assert(accessor.dimension() == 3);

    const std::vector<Tuple>& facest = envelope_mesh.get_all(PF);
    for (const auto& f : facest) {
        if constexpr (std::is_same<T, Rational>()) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f).cast<double>();
            Eigen::Vector3d p1 =
                accessor.const_vector_attribute(envelope_mesh.switch_tuple(f, PV)).cast<double>();
            Eigen::Vector3d p2 =
                accessor.const_vector_attribute(envelope_mesh.switch_tuples(f, {PE, PV}))
                    .cast<double>();

            vertices.push_back(p0);
            vertices.push_back(p1);
            vertices.push_back(p2);
        } else {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(envelope_mesh.switch_tuple(f, PV));
            Eigen::Vector3d p2 =
                accessor.const_vector_attribute(envelope_mesh.switch_tuples(f, {PE, PV}));

            vertices.push_back(p0);
            vertices.push_back(p1);
            vertices.push_back(p2);
        }
        faces.emplace_back(count, count + 1, count + 2);

        count += 3;
    }

    return std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);
}

template <typename T>
std::shared_ptr<SimpleBVH::BVH> init_bvh(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate)
{
    const auto& envelope_mesh = envelope_mesh_coordinate.mesh();

    const attribute::Accessor<T> accessor =
        envelope_mesh.create_const_accessor(envelope_mesh_coordinate.as<T>());

    logger().warn("Envelope for edge mesh is using sampling");

    int64_t count = 0;
    int64_t index = 0;

    const std::vector<Tuple>& edgest = envelope_mesh.get_all(PE);

    Eigen::MatrixXd vertices(2 * edgest.size(), accessor.dimension());
    Eigen::MatrixXi edges(edgest.size(), 2);

    for (const Tuple& e : edgest) {
        if constexpr (std::is_same<T, Rational>()) {
            const auto p0 = accessor.const_vector_attribute(e).cast<double>();
            const auto p1 =
                accessor.const_vector_attribute(envelope_mesh.switch_tuple(e, PV)).cast<double>();

            vertices.row(2 * index) = p0;
            vertices.row(2 * index + 1) = p1;
        } else {
            const auto p0 = accessor.const_vector_attribute(e);
            const auto p1 = accessor.const_vector_attribute(envelope_mesh.switch_tuple(e, PV));

            vertices.row(2 * index) = p0;
            vertices.row(2 * index + 1) = p1;
        }
        edges.row(index) << count, count + 1;

        count += 2;
        ++index;
    }

    std::shared_ptr<SimpleBVH::BVH> bvh = std::make_shared<SimpleBVH::BVH>();
    bvh->init(vertices, edges, 1e-10);
    return bvh;
}

template <typename T>
std::shared_ptr<fastEnvelope::FastEnvelope> init_fast_envelope(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    const double envelope_size,
    const submesh::SubMesh& sub)
{
    const Mesh& m = envelope_mesh_coordinate.mesh();

    const attribute::Accessor<T> accessor =
        m.create_const_accessor(envelope_mesh_coordinate.as<T>());

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;

    int count = 0;
    assert(accessor.dimension() == 3);

    const std::vector<Tuple>& facest = sub.get_all(PF);
    for (const auto& f : facest) {
        if constexpr (std::is_same<T, Rational>()) {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f).cast<double>();
            Eigen::Vector3d p1 =
                accessor.const_vector_attribute(sub.switch_tuple(f, PV)).cast<double>();
            Eigen::Vector3d p2 =
                accessor.const_vector_attribute(sub.switch_tuples(f, {PE, PV})).cast<double>();

            vertices.push_back(p0);
            vertices.push_back(p1);
            vertices.push_back(p2);
        } else {
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f);
            Eigen::Vector3d p1 = accessor.const_vector_attribute(sub.switch_tuple(f, PV));
            Eigen::Vector3d p2 = accessor.const_vector_attribute(sub.switch_tuples(f, {PE, PV}));

            vertices.push_back(p0);
            vertices.push_back(p1);
            vertices.push_back(p2);
        }
        faces.emplace_back(count, count + 1, count + 2);

        count += 3;
    }

    return std::make_shared<fastEnvelope::FastEnvelope>(vertices, faces, envelope_size);
}

template <typename T>
std::shared_ptr<SimpleBVH::BVH> init_bvh(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    const submesh::SubMesh& sub)
{
    const Mesh& m = envelope_mesh_coordinate.mesh();

    const attribute::Accessor<T> accessor =
        m.create_const_accessor(envelope_mesh_coordinate.as<T>());

    logger().warn("Envelope for edge mesh is using sampling");

    int64_t count = 0;
    int64_t index = 0;

    const std::vector<Tuple>& edgest = sub.get_all(PE);

    Eigen::MatrixXd vertices(2 * edgest.size(), accessor.dimension());
    Eigen::MatrixXi edges(edgest.size(), 2);

    for (const Tuple& e : edgest) {
        if constexpr (std::is_same<T, Rational>()) {
            const auto p0 = accessor.const_vector_attribute(e).cast<double>();
            const auto p1 = accessor.const_vector_attribute(sub.switch_tuple(e, PV)).cast<double>();

            vertices.row(2 * index) = p0;
            vertices.row(2 * index + 1) = p1;
        } else {
            const auto p0 = accessor.const_vector_attribute(e);
            const auto p1 = accessor.const_vector_attribute(sub.switch_tuple(e, PV));

            vertices.row(2 * index) = p0;
            vertices.row(2 * index + 1) = p1;
        }
        edges.row(index) << count, count + 1;

        count += 2;
        ++index;
    }

    std::shared_ptr<SimpleBVH::BVH> bvh = std::make_shared<SimpleBVH::BVH>();
    bvh->init(vertices, edges, 1e-10);
    return bvh;
}

} // namespace wmtk
namespace wmtk::invariants {

EnvelopeInvariant::EnvelopeInvariant(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    double envelope_size,
    const attribute::MeshAttributeHandle& coordinate)
    : Invariant(coordinate.mesh(), false, false, true)
    , m_coordinate_handle(coordinate)
    , m_envelope_size(envelope_size)
{
    const auto& envelope_mesh = envelope_mesh_coordinate.mesh();

    // log_assert(
    //     envelope_mesh_coordinate.holds<Rational>() || envelope_mesh_coordinate.holds<double>(),
    //     "Envelope mesh handle type invalid");

    if (!(envelope_mesh_coordinate.holds<Rational>() || envelope_mesh_coordinate.holds<double>())) {
        log_and_throw_error("Envelope mesh handle type invalid");
    }

    // log_assert(
    //     envelope_mesh.top_simplex_type() == PF || envelope_mesh.top_simplex_type() == PE,
    //    "Envelope works only for tri/edges meshes");

    if (!(envelope_mesh.top_simplex_type() == PF || envelope_mesh.top_simplex_type() == PE)) {
        log_and_throw_error("Envelope works only for tri/edges meshes");
    }

    if (envelope_mesh_coordinate.holds<Rational>()) {
        if (envelope_mesh.top_simplex_type() == PF) {
            m_envelope = init_fast_envelope<Rational>(envelope_mesh_coordinate, envelope_size);
        } else if (envelope_mesh.top_simplex_type() == PE) {
            m_bvh = init_bvh<Rational>(envelope_mesh_coordinate);
        }
    } else if (envelope_mesh_coordinate.holds<double>()) {
        if (envelope_mesh.top_simplex_type() == PF) {
            m_envelope = init_fast_envelope<double>(envelope_mesh_coordinate, envelope_size);
        } else if (envelope_mesh.top_simplex_type() == PE) {
            m_bvh = init_bvh<double>(envelope_mesh_coordinate);
        }
    }
}

EnvelopeInvariant::EnvelopeInvariant(
    const attribute::MeshAttributeHandle& envelope_mesh_coordinate,
    double envelope_size,
    const submesh::SubMesh& sub)
    : Invariant(envelope_mesh_coordinate.mesh(), false, false, true)
    , m_coordinate_handle(envelope_mesh_coordinate)
    , m_envelope_size(envelope_size)
    , m_submesh(&sub)
{
    // log_assert(
    //     envelope_mesh_coordinate.holds<Rational>() || envelope_mesh_coordinate.holds<double>(),
    //     "Envelope mesh handle type invalid");

    if (!(envelope_mesh_coordinate.holds<Rational>() || envelope_mesh_coordinate.holds<double>())) {
        log_and_throw_error("Envelope mesh handle type invalid");
    }

    // log_assert(
    //     envelope_mesh.top_simplex_type() == PF || envelope_mesh.top_simplex_type() == PE,
    //    "Envelope works only for tri/edges meshes");

    if (!(sub.top_simplex_type() == PF || sub.top_simplex_type() == PE)) {
        log_and_throw_error("Envelope works only for tri/edges meshes");
    }

    if (envelope_mesh_coordinate.holds<Rational>()) {
        if (sub.top_simplex_type() == PF) {
            m_envelope = init_fast_envelope<Rational>(envelope_mesh_coordinate, envelope_size, sub);
        } else if (sub.top_simplex_type() == PE) {
            m_bvh = init_bvh<Rational>(envelope_mesh_coordinate, sub);
        }
    } else if (envelope_mesh_coordinate.holds<double>()) {
        if (sub.top_simplex_type() == PF) {
            m_envelope = init_fast_envelope<double>(envelope_mesh_coordinate, envelope_size, sub);
        } else if (sub.top_simplex_type() == PE) {
            m_bvh = init_bvh<double>(envelope_mesh_coordinate, sub);
        }
    }
}

bool EnvelopeInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    /*
    Modification for submesh:
    - Get faces of top_dimension_tuples_before that are of type sub.top_simplex_type()
    - Filter by sub.contains(tuple, pt_top)
    */

    if (top_dimension_tuples_after.empty()) return true;

    assert(m_coordinate_handle.holds<Rational>() || m_coordinate_handle.holds<double>());

    if (m_coordinate_handle.holds<Rational>()) {
        const attribute::Accessor<Rational> accessor =
            mesh().create_const_accessor(m_coordinate_handle.as<Rational>());
        const PrimitiveType type = mesh().top_simplex_type();

        if (m_envelope) {
            assert(accessor.dimension() == 3);

            std::vector<Tuple> faces;

            if (type == PrimitiveType::Triangle) {
                std::array<Eigen::Vector3d, 3> triangle;

                for (const Tuple& tuple : top_dimension_tuples_after) {
                    faces = faces_single_dimension_tuples(
                        mesh(),
                        simplex::Simplex(mesh(), type, tuple),
                        PrimitiveType::Vertex);

                    triangle[0] = accessor.const_vector_attribute(faces[0]).cast<double>();
                    triangle[1] = accessor.const_vector_attribute(faces[1]).cast<double>();
                    triangle[2] = accessor.const_vector_attribute(faces[2]).cast<double>();

                    if (m_envelope->is_outside(triangle)) {
                        wmtk::logger().debug("fail envelope check 1");
                        return false;
                    }
                }

                return true;
            } else if (type == PrimitiveType::Edge) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    faces = faces_single_dimension_tuples(
                        mesh(),
                        simplex::Simplex(mesh(), type, tuple),
                        PrimitiveType::Vertex);

                    Eigen::Vector3d p0 = accessor.const_vector_attribute(faces[0]).cast<double>();
                    Eigen::Vector3d p1 = accessor.const_vector_attribute(faces[1]).cast<double>();

                    if (m_envelope->is_outside(p0, p1)) {
                        wmtk::logger().debug("fail envelope check 2");
                        return false;
                    }
                }

                return true;
            } else if (type == PrimitiveType::Vertex) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple).cast<double>();

                    if (m_envelope->is_outside(p)) {
                        wmtk::logger().debug("fail envelope check 3");
                        return false;
                    }
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
                    const auto p0 = accessor.const_vector_attribute(tuple).cast<double>().eval();
                    const auto p1 = accessor.const_vector_attribute(mesh().switch_tuple(tuple, PV))
                                        .cast<double>()
                                        .eval();

                    const int64_t N = (p0 - p1).norm() / d + 1;
                    pts.reserve(pts.size() + N);

                    for (int64_t n = 0; n <= N; n++) {
                        auto tmp = p0 * (double(n) / N) + p1 * (N - double(n)) / N;
                        pts.push_back(tmp);
                    }
                }

                auto current_point = pts[0];

                int prev_facet = m_bvh->nearest_facet(current_point, nearest_point, sq_dist);
                if (sq_dist > real_envelope_2) {
                    wmtk::logger().debug("fail envelope check 4");
                    return false;
                }

                for (const auto& v : pts) {
                    sq_dist = (v - nearest_point).squaredNorm();
                    m_bvh->nearest_facet_with_hint(v, prev_facet, nearest_point, sq_dist);
                    if (sq_dist > real_envelope_2) {
                        wmtk::logger().debug("fail envelope check 5");
                        return false;
                    }
                }

                return true;
            } else if (type == PrimitiveType::Vertex) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple).cast<double>();
                    m_bvh->nearest_facet(p, nearest_point, sq_dist);
                    if (sq_dist > m_envelope_size * m_envelope_size) {
                        wmtk::logger().debug("fail envelope check 6");
                        return false;
                    }
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
                        simplex::Simplex(mesh(), type, tuple),
                        PrimitiveType::Vertex);

                    triangle[0] = accessor.const_vector_attribute(faces[0]);
                    triangle[1] = accessor.const_vector_attribute(faces[1]);
                    triangle[2] = accessor.const_vector_attribute(faces[2]);

                    if (m_envelope->is_outside(triangle)) {
                        wmtk::logger().debug("fail envelope check 7");
                        return false;
                    }
                }

                return true;
            } else if (type == PrimitiveType::Edge) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    faces = faces_single_dimension_tuples(
                        mesh(),
                        simplex::Simplex(mesh(), type, tuple),
                        PrimitiveType::Vertex);

                    Eigen::Vector3d p0 = accessor.const_vector_attribute(faces[0]);
                    Eigen::Vector3d p1 = accessor.const_vector_attribute(faces[1]);

                    if (m_envelope->is_outside(p0, p1)) {
                        wmtk::logger().debug("fail envelope check 8");
                        return false;
                    }
                }

                return true;
            } else if (type == PrimitiveType::Vertex) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple);

                    if (m_envelope->is_outside(p)) {
                        wmtk::logger().debug("fail envelope check 9");
                        return false;
                    }
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
                if (sq_dist > real_envelope_2) {
                    wmtk::logger().debug("fail envelope check 10");
                    return false;
                }

                for (const auto& v : pts) {
                    sq_dist = (v - nearest_point).squaredNorm();
                    m_bvh->nearest_facet_with_hint(v, prev_facet, nearest_point, sq_dist);
                    if (sq_dist > real_envelope_2) {
                        wmtk::logger().debug("fail envelope check 11");
                        return false;
                    }
                }

                return true;
            } else if (type == PrimitiveType::Vertex) {
                for (const Tuple& tuple : top_dimension_tuples_after) {
                    Eigen::Vector3d p = accessor.const_vector_attribute(tuple).cast<double>();
                    m_bvh->nearest_facet(p, nearest_point, sq_dist);
                    if (sq_dist > m_envelope_size * m_envelope_size) {
                        wmtk::logger().debug("fail envelope check 12");
                        return false;
                    }
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
