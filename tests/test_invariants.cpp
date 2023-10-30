#include <catch2/catch_test_macros.hpp>

#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

#include <wmtk/invariants/MinIncidentValenceInvariant.hpp>

using namespace wmtk;
using namespace wmtk::invariants;
using namespace wmtk::tests;

TEST_CASE("MinIncidentValenceInvariant", "[invariants][2D]")
{
    SECTION("single_triangle")
    {
        const DEBUG_TriMesh m = single_triangle();
        const MinIncidentValenceInvariant inv(m, 3);

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            CHECK_FALSE(inv.before(t));
            CHECK_FALSE(inv.after(PrimitiveType::Edge, {t}));
        }
    }
    SECTION("one_ear")
    {
        const DEBUG_TriMesh m = one_ear();
        const MinIncidentValenceInvariant inv(m, 3);

        const Simplex e_mid = Simplex::edge(m.edge_tuple_between_v1_v2(0, 1, 0));

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            const Simplex e = Simplex::edge(t);
            if (m.simplices_are_equal(e, e_mid)) {
                CHECK(inv.before(t));
                CHECK(inv.after(PrimitiveType::Edge, {t}));
            } else {
                CHECK_FALSE(inv.before(t));
                CHECK_FALSE(inv.after(PrimitiveType::Edge, {t}));
            }
        }

        CHECK_FALSE(inv.after(PrimitiveType::Edge, m.get_all(PrimitiveType::Edge)));
    }
    SECTION("edge_region")
    {
        const DEBUG_TriMesh m = edge_region();
        const MinIncidentValenceInvariant inv(m, 3);

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            CHECK(inv.before(t));
            CHECK(inv.after(PrimitiveType::Edge, {t}));
        }

        CHECK(inv.after(PrimitiveType::Edge, m.get_all(PrimitiveType::Edge)));
    }
}