#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "predicates.h"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/TetMesh_examples.hpp"

#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/utils/orient.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace wmtk;
using namespace wmtk::tests_3d;
using namespace wmtk::invariants;

TEST_CASE("tet_inversion_invariant", "[invariants][3D]")
{
    igl_predicates::exactinit();
    DEBUG_TetMesh m = single_tet();
    auto position_handle = m.register_attribute<double>("vertices", PrimitiveType::Vertex, 3);
    auto position_accessor = m.create_accessor<double>(position_handle);

    Tuple v0 = m.tuple_from_id(PrimitiveType::Vertex, 0);
    Tuple v1 = m.tuple_from_id(PrimitiveType::Vertex, 1);
    Tuple v2 = m.tuple_from_id(PrimitiveType::Vertex, 2);
    Tuple v3 = m.tuple_from_id(PrimitiveType::Vertex, 3);

    position_accessor.vector_attribute(v0) = Eigen::Vector3d(-1, -1, 0);
    position_accessor.vector_attribute(v1) = Eigen::Vector3d(1, 1, 0);
    position_accessor.vector_attribute(v2) = Eigen::Vector3d(1, -1, 0);
    position_accessor.vector_attribute(v3) = Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d p0 = position_accessor.vector_attribute(v0);
    Eigen::Vector3d p1 = position_accessor.vector_attribute(v1);
    Eigen::Vector3d p2 = position_accessor.vector_attribute(v2);
    Eigen::Vector3d p3 = position_accessor.vector_attribute(v3);

    // CHECK(m.is_ccw(v0));
    // Eigen::Vector3d p0 = position_accessor.vector_attribute(v0);
    // Eigen::Vector3d p1 = position_accessor.vector_attribute(m.switch_vertex(v0));
    // Eigen::Vector3d p2 = position_accessor.vector_attribute(m.switch_vertex(m.switch_edge(v0)));
    // Eigen::Vector3d p3 =
    //     position_accessor.vector_attribute(m.switch_vertex(m.switch_edge(m.switch_face(v0))));

    const SimplexInversionInvariant inv(m, position_handle.as<double>());
    Tuple t = v0;


    std::cout << igl_predicates::orient3d(p0.data(), p1.data(), p2.data(), p3.data()) << std::endl;
    CHECK(igl_predicates::orient3d(p0.data(), p1.data(), p2.data(), p3.data()) > 0);
    CHECK(wmtk::utils::wmtk_orient3d(p0, p1, p2, p3) > 0);

    for (const auto& t : m.get_all(PrimitiveType::Triangle)) {
        CHECK(inv.after({}, {t}));
        CHECK(inv.after({}, {m.switch_vertex(t)}));
        CHECK(inv.after({}, {m.switch_edge(t)}));
        CHECK(inv.after({}, {m.switch_vertex(m.switch_edge(t))}));
        CHECK(inv.after({}, {m.switch_edge(m.switch_vertex(t))}));
        CHECK(inv.after({}, {m.switch_vertex(m.switch_edge(m.switch_vertex(t)))}));
    }

    position_accessor.vector_attribute(v3) = Eigen::Vector3d(0, 0, -1);
    p3 = position_accessor.vector_attribute(v3);

    // std::cout << orient3d(p0.data(), p1.data(), p2.data(), p3.data()) << std::endl;
    // CHECK(orient3d(p0.data(), p1.data(), p2.data(), p3.data()) < 0);
    // CHECK(wmtk::utils::wmtk_orient3d(p0, p1, p2, p3) < 0);


    for (const auto& t : m.get_all(PrimitiveType::Triangle)) {
        CHECK_FALSE(inv.after({}, {t}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(m.switch_vertex(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(m.switch_vertex(t)))}));
    }
}

TEST_CASE("tet_rational_inversion_invariant", "[invariants][3D]")
{
    igl_predicates::exactinit();
    DEBUG_TetMesh m = single_tet();
    auto position_handle = m.register_attribute<Rational>("vertices", PrimitiveType::Vertex, 3);
    auto position_accessor = m.create_accessor<Rational>(position_handle);

    Tuple v0 = m.tuple_from_id(PrimitiveType::Vertex, 0);
    Tuple v1 = m.tuple_from_id(PrimitiveType::Vertex, 1);
    Tuple v2 = m.tuple_from_id(PrimitiveType::Vertex, 2);
    Tuple v3 = m.tuple_from_id(PrimitiveType::Vertex, 3);

    position_accessor.vector_attribute(v0) = Eigen::Vector3<Rational>(-1, -1, 0);
    position_accessor.vector_attribute(v1) = Eigen::Vector3<Rational>(1, 1, 0);
    position_accessor.vector_attribute(v2) = Eigen::Vector3<Rational>(1, -1, 0);
    position_accessor.vector_attribute(v3) = Eigen::Vector3<Rational>(0, 0, 1);

    Eigen::Vector3<Rational> p0 = position_accessor.vector_attribute(v0);
    Eigen::Vector3<Rational> p1 = position_accessor.vector_attribute(v1);
    Eigen::Vector3<Rational> p2 = position_accessor.vector_attribute(v2);
    Eigen::Vector3<Rational> p3 = position_accessor.vector_attribute(v3);

    CHECK(wmtk::utils::wmtk_orient3d(p0, p1, p2, p3) > 0);
    const SimplexInversionInvariant inv(m, position_handle.as<Rational>());
    Tuple t = v0;

    for (const auto& t : m.get_all(PrimitiveType::Triangle)) {
        CHECK(inv.after({}, {t}));
        CHECK(inv.after({}, {m.switch_vertex(t)}));
        CHECK(inv.after({}, {m.switch_edge(t)}));
        CHECK(inv.after({}, {m.switch_vertex(m.switch_edge(t))}));
        CHECK(inv.after({}, {m.switch_edge(m.switch_vertex(t))}));
        CHECK(inv.after({}, {m.switch_vertex(m.switch_edge(m.switch_vertex(t)))}));
    }


    position_accessor.vector_attribute(v3) = Eigen::Vector3<Rational>(0, 0, -1);
    p3 = position_accessor.vector_attribute(v3);


    for (const auto& t : m.get_all(PrimitiveType::Triangle)) {
        CHECK_FALSE(inv.after({}, {t}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(m.switch_vertex(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(m.switch_vertex(t)))}));
    }
}

TEST_CASE("orient3d", "[invariants][3D]")
{
    const Eigen::Vector3<Rational> v0 = Eigen::Vector3<Rational>(-1, -1, 0);
    const Eigen::Vector3<Rational> v1 = Eigen::Vector3<Rational>(1, 1, 0);
    const Eigen::Vector3<Rational> v2 = Eigen::Vector3<Rational>(1, -1, 0);
    const Eigen::Vector3<Rational> v3 = Eigen::Vector3<Rational>(0, 0, 1);

    const Eigen::Vector3d v0d = v0.cast<double>();
    const Eigen::Vector3d v1d = v1.cast<double>();
    const Eigen::Vector3d v2d = v2.cast<double>();
    const Eigen::Vector3d v3d = v3.cast<double>();


    REQUIRE(utils::wmtk_orient3d(v0, v1, v2, v3) == utils::wmtk_orient3d(v0d, v1d, v2d, v3d));
    REQUIRE(utils::wmtk_orient3d(v0, v2, v1, v3) == utils::wmtk_orient3d(v0d, v2d, v1d, v3d));
}
