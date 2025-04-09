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
#include <wmtk/simplex/IdSimplexCollection.hpp>
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
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f).template cast<double>();
            Eigen::Vector3d p1 = accessor.const_vector_attribute(envelope_mesh.switch_tuple(f, PV))
                                     .template cast<double>();
            Eigen::Vector3d p2 =
                accessor.const_vector_attribute(envelope_mesh.switch_tuples(f, {PE, PV}))
                    .template cast<double>();

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
            const auto p0 = accessor.const_vector_attribute(e).template cast<double>();
            const auto p1 = accessor.const_vector_attribute(envelope_mesh.switch_tuple(e, PV))
                                .template cast<double>();

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
            Eigen::Vector3d p0 = accessor.const_vector_attribute(f).template cast<double>();
            Eigen::Vector3d p1 =
                accessor.const_vector_attribute(sub.switch_tuple(f, PV)).template cast<double>();
            Eigen::Vector3d p2 = accessor.const_vector_attribute(sub.switch_tuples(f, {PE, PV}))
                                     .template cast<double>();

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
            const auto p0 = accessor.const_vector_attribute(e).template cast<double>();
            const auto p1 =
                accessor.const_vector_attribute(sub.switch_tuple(e, PV)).template cast<double>();

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
    const attribute::MeshAttributeHandle& pt_attribute,
    double envelope_size,
    const submesh::SubMesh& sub)
    : Invariant(pt_attribute.mesh(), false, false, true)
    , m_coordinate_handle(pt_attribute)
    , m_submesh(&sub)
    , m_envelope_size(envelope_size)
{
    // log_assert(
    //     envelope_mesh_coordinate.holds<Rational>() || envelope_mesh_coordinate.holds<double>(),
    //     "Envelope mesh handle type invalid");

    if (!(pt_attribute.holds<Rational>() || pt_attribute.holds<double>())) {
        log_and_throw_error("Envelope mesh handle type invalid");
    }

    // log_assert(
    //     envelope_mesh.top_simplex_type() == PF || envelope_mesh.top_simplex_type() == PE,
    //    "Envelope works only for tri/edges meshes");

    if (!(sub.top_simplex_type() == PF || sub.top_simplex_type() == PE)) {
        log_and_throw_error("Envelope works only for tri/edges meshes");
    }

    if (pt_attribute.holds<Rational>()) {
        if (sub.top_simplex_type() == PF) {
            m_envelope = init_fast_envelope<Rational>(pt_attribute, envelope_size, sub);
        } else if (sub.top_simplex_type() == PE) {
            m_bvh = init_bvh<Rational>(pt_attribute, sub);
        }
    } else if (pt_attribute.holds<double>()) {
        if (sub.top_simplex_type() == PF) {
            m_envelope = init_fast_envelope<double>(pt_attribute, envelope_size, sub);
        } else if (sub.top_simplex_type() == PE) {
            m_bvh = init_bvh<double>(pt_attribute, sub);
        }
    }
}

bool EnvelopeInvariant::after(
    const std::vector<Tuple>& top_dimension_tuples_before,
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    if (top_dimension_tuples_after.empty()) {
        return true;
    }

    assert(m_coordinate_handle.holds<Rational>() || m_coordinate_handle.holds<double>());
    assert(m_envelope || m_bvh);

    /*
    Modification for submesh:
    - Get faces of top_dimension_tuples_after that are of type sub.top_simplex_type()
    - Filter by sub.contains(tuple, pt_top)
    */

    if (m_submesh) {
        const submesh::SubMesh& sub = *m_submesh;

        const PrimitiveType pt_top = sub.top_simplex_type();

        simplex::IdSimplexCollection tops(mesh());
        for (const Tuple& t : top_dimension_tuples_after) {
            if (mesh().top_simplex_type() == pt_top) {
                if (sub.contains(t, pt_top)) {
                    tops.add(pt_top, t);
                }
                continue;
            }

            const auto faces = simplex::faces_single_dimension_tuples(
                mesh(),
                simplex::Simplex(mesh().top_simplex_type(), t),
                pt_top);

            for (const Tuple& f : faces) {
                if (sub.contains(f, pt_top)) {
                    tops.add(pt_top, f);
                }
            }
        }
        tops.sort_and_clean();

        if (tops.size() == 0) {
            return true;
        }

        if (m_envelope) {
            return after_with_envelope(tops.simplex_vector_tuples());
        }
        if (m_bvh) {
            return after_with_bvh(tops.simplex_vector_tuples());
        }
        assert(false); // this code should be unreachable
        return false;
    }

    if (m_envelope) {
        return after_with_envelope(top_dimension_tuples_after);
    }
    if (m_bvh) {
        return after_with_bvh(top_dimension_tuples_after);
    }
    assert(false); // this code should be unreachable
    return false;
}


bool EnvelopeInvariant::after_with_envelope(
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    assert(m_envelope);
    assert(m_coordinate_handle.dimension() == 3);

    const PrimitiveType pt_top =
        m_submesh ? m_submesh->top_simplex_type() : mesh().top_simplex_type();

    if (pt_top == PrimitiveType::Triangle) {
        return after_with_envelope_triangle(top_dimension_tuples_after);
    }
    if (pt_top == PrimitiveType::Edge) {
        return after_with_envelope_edge(top_dimension_tuples_after);
    }
    if (pt_top == PrimitiveType::Vertex) {
        return after_with_envelope_vertex(top_dimension_tuples_after);
    }
    log_and_throw_error("Invalid mesh type");
}

bool EnvelopeInvariant::after_with_envelope_triangle(
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    assert(m_envelope);

    const bool res = std::visit(
        [&](auto&& tah) noexcept {
            using HandleType = typename std::decay_t<decltype(tah)>;
            using AttributeType = typename HandleType::Type;

            const auto accessor = mesh().create_const_accessor(tah);

            std::array<Eigen::Vector3d, 3> triangle;

            for (const Tuple& tuple : top_dimension_tuples_after) {
                const auto faces = faces_single_dimension_tuples(
                    mesh(),
                    simplex::Simplex(PrimitiveType::Triangle, tuple),
                    PrimitiveType::Vertex);

                if constexpr (std::is_same_v<AttributeType, double>) {
                    triangle[0] = accessor.const_vector_attribute(faces[0]);
                    triangle[1] = accessor.const_vector_attribute(faces[1]);
                    triangle[2] = accessor.const_vector_attribute(faces[2]);
                }
                if constexpr (std::is_same_v<AttributeType, Rational>) {
                    triangle[0] = accessor.const_vector_attribute(faces[0]).template cast<double>();
                    triangle[1] = accessor.const_vector_attribute(faces[1]).template cast<double>();
                    triangle[2] = accessor.const_vector_attribute(faces[2]).template cast<double>();
                }

                if (m_envelope->is_outside(triangle)) {
                    wmtk::logger().debug("fail envelope check 1");
                    return false;
                }
            }
            return true;
        },
        m_coordinate_handle.handle());

    return res;
}

bool EnvelopeInvariant::after_with_envelope_edge(
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    assert(m_envelope);
    assert(mesh().top_simplex_type() == PrimitiveType::Edge);

    const bool res = std::visit(
        [&](auto&& tah) noexcept {
            using HandleType = typename std::decay_t<decltype(tah)>;
            using AttributeType = typename HandleType::Type;

            const auto accessor = mesh().create_const_accessor(tah);

            std::array<Eigen::Vector3d, 3> triangle;
            const PrimitiveType pt_top = mesh().top_simplex_type();

            for (const Tuple& tuple : top_dimension_tuples_after) {
                const auto faces = faces_single_dimension_tuples(
                    mesh(),
                    simplex::Simplex(mesh(), pt_top, tuple),
                    PrimitiveType::Vertex);

                Eigen::Vector3d p0;
                Eigen::Vector3d p1;

                if constexpr (std::is_same_v<AttributeType, double>) {
                    p0 = accessor.const_vector_attribute(faces[0]);
                    p1 = accessor.const_vector_attribute(faces[1]);
                } else if constexpr (std::is_same_v<AttributeType, Rational>) {
                    p0 = accessor.const_vector_attribute(faces[0]).template cast<double>();
                    p1 = accessor.const_vector_attribute(faces[1]).template cast<double>();
                } else {
                    log_and_throw_error("Unknown attribute type");
                }

                if (m_envelope->is_outside(p0, p1)) {
                    wmtk::logger().debug("fail envelope check 2");
                    return false;
                }
            }

            return true;
        },
        m_coordinate_handle.handle());

    return res;
}

bool EnvelopeInvariant::after_with_envelope_vertex(
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    assert(m_envelope);
    assert(mesh().top_simplex_type() == PrimitiveType::Vertex);

    const bool res = std::visit(
        [&](auto&& tah) noexcept {
            using HandleType = typename std::decay_t<decltype(tah)>;
            using AttributeType = typename HandleType::Type;

            const auto accessor = mesh().create_const_accessor(tah);

            for (const Tuple& tuple : top_dimension_tuples_after) {
                Eigen::Vector3d p;
                if constexpr (std::is_same_v<AttributeType, double>) {
                    p = accessor.const_vector_attribute(tuple);
                } else if constexpr (std::is_same_v<AttributeType, Rational>) {
                    p = accessor.const_vector_attribute(tuple).template cast<double>();
                } else {
                    log_and_throw_error("Unknown attribute type");
                }

                if (m_envelope->is_outside(p)) {
                    wmtk::logger().debug("fail envelope check 3");
                    return false;
                }
            }

            return true;
        },
        m_coordinate_handle.handle());

    return res;
}

bool EnvelopeInvariant::after_with_bvh(const std::vector<Tuple>& top_dimension_tuples_after) const
{
    assert(m_bvh);

    const PrimitiveType pt_top =
        m_submesh ? m_submesh->top_simplex_type() : mesh().top_simplex_type();

    if (pt_top == PrimitiveType::Edge) {
        return after_with_bvh_edge(top_dimension_tuples_after);
    }
    if (pt_top == PrimitiveType::Vertex) {
        return after_with_bvh_vertex(top_dimension_tuples_after);
    }

    log_and_throw_error("Invalid mesh type");
}

bool EnvelopeInvariant::after_with_bvh_edge(
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    SimpleBVH::VectorMax3d nearest_point;
    double sq_dist;

    const double d = m_envelope_size;
    const double real_envelope = m_envelope_size - d / sqrt(m_coordinate_handle.dimension());
    const double real_envelope_2 = real_envelope * real_envelope;

    std::vector<SimpleBVH::VectorMax3d> pts;

    std::visit(
        [&](auto&& tah) noexcept {
            using HandleType = typename std::decay_t<decltype(tah)>;
            using AttributeType = typename HandleType::Type;

            const auto accessor = mesh().create_const_accessor(tah);

            for (const Tuple& tuple : top_dimension_tuples_after) {
                SimpleBVH::VectorMax3d p0;
                SimpleBVH::VectorMax3d p1;
                if constexpr (std::is_same_v<AttributeType, double>) {
                    p0 = accessor.const_vector_attribute(tuple);
                    p1 = accessor.const_vector_attribute(mesh().switch_tuple(tuple, PV));
                } else if constexpr (std::is_same_v<AttributeType, Rational>) {
                    p0 = accessor.const_vector_attribute(tuple).template cast<double>();
                    p1 = accessor.const_vector_attribute(mesh().switch_tuple(tuple, PV))
                             .template cast<double>();
                } else {
                    log_and_throw_error("Unknown attribute type");
                }

                const int64_t N = (p0 - p1).norm() / d + 1;
                pts.reserve(pts.size() + N);

                for (int64_t n = 0; n <= N; n++) {
                    auto tmp = p0 * (double(n) / N) + p1 * (N - double(n)) / N;
                    pts.push_back(tmp);
                }
            }
        },
        m_coordinate_handle.handle());

    SimpleBVH::VectorMax3d current_point = pts[0];

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
}

bool EnvelopeInvariant::after_with_bvh_vertex(
    const std::vector<Tuple>& top_dimension_tuples_after) const
{
    assert(m_bvh);
    assert(mesh().top_simplex_type() == PrimitiveType::Vertex);

    SimpleBVH::VectorMax3d nearest_point;
    double sq_dist;

    for (const Tuple& tuple : top_dimension_tuples_after) {
        SimpleBVH::VectorMax3d p = std::visit(
            [&](auto&& tah) noexcept {
                using HandleType = typename std::decay_t<decltype(tah)>;
                using AttributeType = typename HandleType::Type;

                const auto accessor = mesh().create_const_accessor(tah);

                SimpleBVH::VectorMax3d _p;
                if constexpr (std::is_same_v<AttributeType, double>) {
                    _p = accessor.const_vector_attribute(tuple);
                } else if constexpr (std::is_same_v<AttributeType, Rational>) {
                    _p = accessor.const_vector_attribute(tuple).template cast<double>();
                } else {
                    log_and_throw_error("Unknown attribute type");
                }
                return _p;
            },
            m_coordinate_handle.handle());

        m_bvh->nearest_facet(p, nearest_point, sq_dist);
        if (sq_dist > m_envelope_size * m_envelope_size) {
            wmtk::logger().debug("fail envelope check 6");
            return false;
        }
    }

    return true;
}

} // namespace wmtk::invariants
