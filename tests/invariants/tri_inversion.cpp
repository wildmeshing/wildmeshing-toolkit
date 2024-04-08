#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>

#include <wmtk/invariants/internal/simplex_inversion_predicates.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
#include "predicates.h"

#include <wmtk/invariants/SimplexInversionInvariant.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::invariants;

TEST_CASE("tri_inversion_invariant", "[invariants][2D]")
{
    exactinit();
    DEBUG_TriMesh m = single_triangle();
    auto position_handle = m.register_attribute<double>("vertices", PrimitiveType::Vertex, 2);
    auto position_accessor = m.create_accessor<double>(position_handle);

    const Tuple v0 = m.tuple_from_id(PrimitiveType::Vertex, 0);
    const Tuple v1 = m.tuple_from_id(PrimitiveType::Vertex, 1);
    const Tuple v2 = m.tuple_from_id(PrimitiveType::Vertex, 2);

    const Tuple t = v0;

    position_accessor.vector_attribute(v0) = Eigen::Vector2d(-1, 0);
    position_accessor.vector_attribute(v1) = Eigen::Vector2d(1, 0);
    position_accessor.vector_attribute(v2) = Eigen::Vector2d(0, 1);

    Eigen::Vector2d p0 = position_accessor.vector_attribute(v0);
    Eigen::Vector2d p1 = position_accessor.vector_attribute(v1);
    Eigen::Vector2d p2 = position_accessor.vector_attribute(v2);

    // CHECK(m.is_ccw(v0));
    // Eigen::Vector2d p0 = position_accessor.vector_attribute(v0);
    // Eigen::Vector2d p1 = position_accessor.vector_attribute(m.switch_vertex(v0));
    // Eigen::Vector2d p2 = position_accessor.vector_attribute(m.switch_vertex(m.switch_edge(v0)));
    // Eigen::Vector2d p3 =
    //     position_accessor.vector_attribute(m.switch_vertex(m.switch_edge(m.switch_face(v0))));

    const SimplexInversionInvariant inv(m, position_handle.as<double>());

    std::cout << orient2d(p0.data(), p1.data(), p2.data()) << std::endl;
    CHECK(orient2d(p0.data(), p1.data(), p2.data()) > 0);
    // check that our orientation invariant using the orient2d predicate does the expected thing
    CHECK(wmtk::invariants::internal::triangle_orientation<double, true>(p0, p1, p2));
    // check for the imprecise predicate having the expected sign
    CHECK(wmtk::invariants::internal::triangle_orientation<double, false>(p0, p1, p2));
    // check that the actual invariant invokes this the right way
    CHECK(inv.after({}, {t}));

    position_accessor.vector_attribute(v2) = Eigen::Vector2d(0, -1);

    CHECK(orient2d(p0.data(), p2.data(), p1.data()) < 0);
    // check that our orientation invariant using the orient2d predicate does the expected thing
    CHECK_FALSE(wmtk::invariants::internal::triangle_orientation<double, true>(p0, p2, p1));
    // check for the imprecise predicate having the expected sign
    CHECK_FALSE(wmtk::invariants::internal::triangle_orientation<double, false>(p0, p2, p1));
    // check that the actual invariant invokes this the right way
    CHECK_FALSE(inv.after({}, {t}));


    for (const auto& t : m.get_all(PrimitiveType::Triangle)) {
        CHECK_FALSE(inv.after({}, {t}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(m.switch_vertex(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(m.switch_vertex(t)))}));
    }
}

TEST_CASE("tri_hybrid_inversion_invariant", "[invariants][2D]")
{
    exactinit();
    DEBUG_TriMesh m = single_triangle();
    auto position_handle = m.register_attribute<wmtk::Rational>("vertices", PrimitiveType::Vertex, 2);
    auto position_accessor = m.create_accessor<wmtk::Rational>(position_handle);

    Tuple v0 = m.tuple_from_id(PrimitiveType::Vertex, 0);
    Tuple v1 = m.tuple_from_id(PrimitiveType::Vertex, 1);
    Tuple v2 = m.tuple_from_id(PrimitiveType::Vertex, 2);

    position_accessor.vector_attribute(v0) = Eigen::Vector2d(-1, 0).unaryExpr([](double v) { return wmtk::Rational(v); });
    position_accessor.vector_attribute(v1) = Eigen::Vector2d(1, 0) .unaryExpr([](double v) { return wmtk::Rational(v); });
    position_accessor.vector_attribute(v2) = Eigen::Vector2d(0, 1) .unaryExpr([](double v) { return wmtk::Rational(v); });



    auto hybrid_attr =
        wmtk::attribute::utils::HybridRationalAttribute<>::register_attribute_from_rational(
            static_cast<wmtk::TriMesh&>(m),
            position_handle.as<wmtk::Rational>());

    const SimplexInversionInvariant inv(wmtk::attribute::MeshAttributeHandle(m,hybrid_attr));

    // check based on pure rational values
    CHECK(inv.after({}, {v0}));

    // change the rational value and see if it still works
    position_accessor.vector_attribute(v2) = Eigen::Vector2d(0, -1) .unaryExpr([](double v) { return wmtk::Rational(v); });
    CHECK_FALSE(inv.after({}, {v0}));

    wmtk::attribute::utils::HybridRationalAccessor acc(static_cast<wmtk::TriMesh&>(m), hybrid_attr);

    // try rounding v2 to see if the invariant reads the rounded v2 value now even though the others are not rounded

    acc.round(v2);

    auto double_handle = hybrid_attr.get_double();

    auto double_acc = m.create_accessor(double_handle);

    double_acc.vector_attribute(v2) = Eigen::Vector2d(0, 1);
    CHECK(inv.after({}, {v0}));
    double_acc.vector_attribute(v2) = Eigen::Vector2d(0, -1);
    CHECK_FALSE(inv.after({}, {v0}));
}
