#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "predicates.h"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/TetMesh_examples.hpp"

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
    auto position_accessor = m.create_accessor(position_handle);

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


    std::cout << orient3d(p0.data(), p1.data(), p2.data(), p3.data()) << std::endl;
    CHECK(orient3d(p0.data(), p1.data(), p2.data(), p3.data()) > 0);

    position_accessor.vector_attribute(v3) = Eigen::Vector3d(0, 0, -1);
    p3 = position_accessor.vector_attribute(v3);

    std::cout << orient3d(p0.data(), p1.data(), p2.data(), p3.data()) << std::endl;
    CHECK(orient3d(p0.data(), p1.data(), p2.data(), p3.data()) < 0);

    const SimplexInversionInvariant inv(m, position_handle);
    Tuple t = v0;

    for (const auto& t : m.get_all(PrimitiveType::Face)) {
        CHECK_FALSE(inv.after({}, {t}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(t)}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_edge(m.switch_vertex(t))}));
        CHECK_FALSE(inv.after({}, {m.switch_vertex(m.switch_edge(m.switch_vertex(t)))}));
    }
}
