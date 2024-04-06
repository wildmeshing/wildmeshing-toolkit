
#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/attribute/TypedAttributeHandle.hpp>
#include <wmtk/invariants/MinIncidentValenceInvariant.hpp>
#include <wmtk/invariants/MultiMeshTopologyInvariant.hpp>
#include <wmtk/invariants/TetMeshSubstructureTopologyPreservingInvariant.hpp>
#include <wmtk/invariants/TriMeshSubstructureTopologyPreservingInvariant.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

using namespace wmtk;
using namespace wmtk::simplex;
using namespace wmtk::invariants;
using namespace wmtk::tests;

TEST_CASE("MinIncidentValenceInvariant", "[invariants][2D]")
{
    SECTION("single_triangle")
    {
        const DEBUG_TriMesh m = single_triangle();
        const MinIncidentValenceInvariant inv(m, 3);

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            CHECK_FALSE(inv.before(Simplex::edge(t)));
            CHECK_FALSE(inv.after({}, {t}));
        }
    }
    SECTION("one_ear")
    {
        const DEBUG_TriMesh m = one_ear();
        const MinIncidentValenceInvariant inv(m, 3);

        const Simplex e_mid = Simplex::edge(m.edge_tuple_between_v1_v2(0, 1, 0));

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            const Simplex e = Simplex::edge(t);
            if (simplex::utils::SimplexComparisons::equal(m, e, e_mid)) {
                CHECK(inv.before(simplex::Simplex::edge(t)));
            } else {
                CHECK_FALSE(inv.before(simplex::Simplex::edge(t)));
            }
        }

        CHECK_FALSE(inv.after({}, m.get_all(PrimitiveType::Triangle)));
    }
    SECTION("edge_region")
    {
        const DEBUG_TriMesh m = edge_region();
        const MinIncidentValenceInvariant inv(m, 3);

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            CHECK(inv.before(Simplex::edge(t)));
        }

        CHECK(inv.after({}, m.get_all(PrimitiveType::Triangle)));
    }
}

TEST_CASE("MultiMeshEdgeTopologyInvariant", "[invariants][2D]")
{
    DEBUG_TriMesh mesh = single_triangle();
    auto tag_handle = mesh.register_attribute<int64_t>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    auto tag_accessor = mesh.create_accessor<int64_t>(tag_handle);
    Tuple e0 = mesh.edge_tuple_between_v1_v2(1, 2, 0);
    Tuple e1 = mesh.edge_tuple_between_v1_v2(0, 2, 0);
    Tuple e2 = mesh.edge_tuple_between_v1_v2(0, 1, 0);

    tag_accessor.scalar_attribute(e0) = 1;
    tag_accessor.scalar_attribute(e1) = 1;
    tag_accessor.scalar_attribute(e2) = 0;

    std::shared_ptr<Mesh> child_ptr =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            mesh,
            "is_boundary",
            1,
            PrimitiveType::Edge);

    const EdgeMesh& child = dynamic_cast<const EdgeMesh&>(*child_ptr);

    std::cout << "child mesh #v: " << child.capacity(PrimitiveType::Vertex) << std::endl;

    const MultiMeshEdgeTopologyInvariant inv(mesh, child);

    auto e2_v1_map = mesh.map_to_child_tuples(child, Simplex(PrimitiveType::Vertex, e2));
    auto e2_v2_map =
        mesh.map_to_child_tuples(child, Simplex(PrimitiveType::Vertex, mesh.switch_vertex(e2)));

    std::cout << e2_v1_map.size() << std::endl;
    std::cout << e2_v2_map.size() << std::endl;


    CHECK_FALSE(inv.before(Simplex::edge(e2)));
    CHECK(inv.before(Simplex::edge(e0)));
    CHECK(inv.before(Simplex::edge(e1)));

    std::cout << inv.before(Simplex::edge(e2)) << std::endl;
    std::cout << inv.before(Simplex::edge(e0)) << std::endl;
    std::cout << inv.before(Simplex::edge(e1)) << std::endl;
}
