#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TriMesh_examples.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/invariants/MinIncidentValenceInvariant.hpp>
#include <wmtk/invariants/MultiMeshTopologyInvariant.hpp>
#include <wmtk/invariants/SubstructureTopologyPreservingInvariant.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

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
            if (simplex::utils::SimplexComparisons::equal(m, e, e_mid)) {
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

TEST_CASE("MultiMeshEdgeTopologyInvariant", "[invariants][2D]")
{
    DEBUG_TriMesh mesh = single_triangle();
    auto tag_handle = mesh.register_attribute<long>("is_boundary", wmtk::PrimitiveType::Edge, 1);
    auto tag_accessor = mesh.create_accessor(tag_handle);
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


    CHECK_FALSE(inv.before(e2));
    CHECK(inv.before(e0));
    CHECK(inv.before(e1));

    std::cout << inv.before(e2) << std::endl;
    std::cout << inv.before(e0) << std::endl;
    std::cout << inv.before(e1) << std::endl;
}

TEST_CASE("SubstructureTopologyPreservingInvariant_tri", "[invariants]")
{
    DEBUG_TriMesh m = embedded_diamond();

    const MeshAttributeHandle<long> edge_tag_handle =
        m.register_attribute<long>("edge_tag", PrimitiveType::Edge, 1);

    const MeshAttributeHandle<long> face_tag_handle =
        m.register_attribute<long>("face_tag", PrimitiveType::Face, 1);

    const long tag_val = 1;

    SubstructureTopologyPreservingInvariant inv(m, face_tag_handle, edge_tag_handle, tag_val);

    SECTION("6-7")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(6, 7, 5)) = tag_val;

            // tag boundary
            for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
                if (m.is_boundary_edge(t)) {
                    edge_tag_acc.scalar_attribute(t) = tag_val;
                }
            }
        }

        CHECK_FALSE(inv.before(m.edge_tuple_between_v1_v2(6, 7, 5)));
        CHECK_FALSE(inv.before(m.edge_tuple_between_v1_v2(5, 6, 3)));
        CHECK(inv.before(m.edge_tuple_between_v1_v2(6, 3, 4)));
    }
}