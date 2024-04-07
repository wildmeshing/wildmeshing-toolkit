#include <catch2/catch_test_macros.hpp>
#include <wmtk/invariants/internal/simplex_inversion_predicates.hpp>

#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "predicates.h"

#include <wmtk/invariants/SimplexInversionInvariant.hpp>

#include <catch2/catch_test_macros.hpp>

using namespace wmtk;
using namespace wmtk::tests_3d;
using namespace wmtk::invariants;

TEST_CASE("tet_inversion_invariant", "[invariants][3D]")
{
    exactinit();
    DEBUG_TetMesh m = single_tet();
    auto position_handle = m.register_attribute<double>("vertices", PrimitiveType::Vertex, 3);
    auto position_accessor = m.create_accessor<double>(position_handle);

    const Tuple v0 = m.tuple_from_id(PrimitiveType::Vertex, 0);
    const Tuple v1 = m.tuple_from_id(PrimitiveType::Vertex, 1);
    const Tuple v2 = m.tuple_from_id(PrimitiveType::Vertex, 2);
    const Tuple v3 = m.tuple_from_id(PrimitiveType::Vertex, 3);

    position_accessor.vector_attribute(v0) = Eigen::Vector3d(-1, -1, 0);
    position_accessor.vector_attribute(v1) = Eigen::Vector3d(1, 1, 0);
    position_accessor.vector_attribute(v2) = Eigen::Vector3d(1, -1, 0);
    position_accessor.vector_attribute(v3) = Eigen::Vector3d(0, 0, 1);

    Eigen::Vector3d p0 = position_accessor.vector_attribute(v0);
    Eigen::Vector3d p1 = position_accessor.vector_attribute(v1);
    Eigen::Vector3d p2 = position_accessor.vector_attribute(v2);
    Eigen::Vector3d p3 = position_accessor.vector_attribute(v3);

    const SimplexInversionInvariant inv(m, position_handle.as<double>());
    const Tuple t = v0;

    // CHECK(m.is_ccw(v0));
    // Eigen::Vector3d p0 = position_accessor.vector_attribute(v0);
    // Eigen::Vector3d p1 = position_accessor.vector_attribute(m.switch_vertex(v0));
    // Eigen::Vector3d p2 = position_accessor.vector_attribute(m.switch_vertex(m.switch_edge(v0)));
    // Eigen::Vector3d p3 =
    //     position_accessor.vector_attribute(m.switch_vertex(m.switch_edge(m.switch_face(v0))));


    std::cout << orient3d(p0.data(), p1.data(), p2.data(), p3.data()) << std::endl;
    CHECK(orient3d(p0.data(), p1.data(), p2.data(), p3.data()) > 0);
    // check that our orientation invariant using the orient2d predicate does the expected thing
    CHECK(wmtk::invariants::internal::tetrahedron_orientation<double, true>(p0, p1, p2, p3));
    // check for the imprecise predicate having the expected sign
    CHECK(wmtk::invariants::internal::tetrahedron_orientation<double, false>(p0, p1, p2, p3));
    // check that the actual invariant invokes this the right way
    CHECK(inv.after({}, {t}));

    position_accessor.vector_attribute(v3) = Eigen::Vector3d(0, 0, -1);
    p3 = position_accessor.vector_attribute(v3);

    std::cout << orient3d(p0.data(), p1.data(), p2.data(), p3.data()) << std::endl;
    CHECK(orient3d(p0.data(), p1.data(), p2.data(), p3.data()) < 0);
    CHECK(!wmtk::invariants::internal::tetrahedron_orientation<double, true>(p0, p1, p2, p3));
    // check for the imprecise predicate having the expected sign
    CHECK(!wmtk::invariants::internal::tetrahedron_orientation<double, false>(p0, p1, p2, p3));
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
