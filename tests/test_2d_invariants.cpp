#include <catch2/catch_test_macros.hpp>

#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"

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

TEST_CASE("SubstructureTopologyPreservingInvariant_tri", "[invariants]")
{
    DEBUG_TriMesh m = embedded_diamond();

    const attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", PrimitiveType::Edge, 1);

    const int64_t tag_val = 1;

    TriMeshSubstructureTopologyPreservingInvariant inv(m, edge_tag_handle.as<int64_t>(), tag_val);

    SECTION("6-7")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(6, 7, 5)) = tag_val;

            // tag boundary
            for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
                if (m.is_boundary_edge(t)) {
                    edge_tag_acc.scalar_attribute(t) = tag_val;
                }
            }
        }

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(6, 7, 5))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(5, 6, 3))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(6, 3, 4))));
    }
    SECTION("3-6-7-3")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(6, 7, 5)) = tag_val;
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(3, 6, 5)) = tag_val;
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(7, 3, 5)) = tag_val;

            // tag boundary
            for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
                if (m.is_boundary_edge(t)) {
                    edge_tag_acc.scalar_attribute(t) = tag_val;
                }
            }
        }

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(6, 7, 5))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(3, 6, 5))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(7, 3, 5))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(10, 6, 9))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(12, 13, 14))));
    }
    SECTION("5-6-7-8")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(5, 6, 3)) = tag_val;
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(6, 7, 5)) = tag_val;
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(7, 8, 7)) = tag_val;

            // tag boundary
            for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
                if (m.is_boundary_edge(t)) {
                    edge_tag_acc.scalar_attribute(t) = tag_val;
                }
            }
        }

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(6, 2, 3))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(6, 7, 5))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(5, 6, 3))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(3, 6, 5))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 5, 3))));
    }
}

TEST_CASE("SubstructureTopologyPreservingInvariant_tet", "[invariants]")
{
    using namespace tests_3d;

    DEBUG_TetMesh m = six_cycle_tets();

    const attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", PrimitiveType::Edge, 1);

    const attribute::MeshAttributeHandle face_tag_handle =
        m.register_attribute<int64_t>("face_tag", PrimitiveType::Triangle, 1);

    const int64_t tag_val = 1;

    TetMeshSubstructureTopologyPreservingInvariant inv(
        m,
        face_tag_handle.as<int64_t>(),
        edge_tag_handle.as<int64_t>(),
        tag_val);

    SECTION("2-3")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(2, 3, 0)) = tag_val;

            //// tag boundary
            // for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            //     if (m.is_boundary_edge(t)) {
            //         edge_tag_acc.scalar_attribute(t) = tag_val;
            //     }
            // }
        }

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 3, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 2, 0))));
    }
    SECTION("0-2-7")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(0, 2, 0)) = tag_val;
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(2, 7, 4)) = tag_val;
        }

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            CHECK(inv.before(Simplex::edge(t)));
        }
    }
    SECTION("0-2-3")
    {
        // mark edge(s)
        {
            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(0, 2, 0)) = tag_val;
            edge_tag_acc.scalar_attribute(m.edge_tuple_between_v1_v2(2, 3, 0)) = tag_val;
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 2, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 3, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(6, 2, 4))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(3, 0, 0))));
    }
    SECTION("f023-f273")
    {
        // mark face(s)
        {
            auto face_tag_acc = m.create_accessor<int64_t>(face_tag_handle);
            face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_val;
            face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 7, 3)) = tag_val;

            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            // tag edges at the substructure's boundary
            for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
                int64_t n_tagged_faces = 0;
                for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                         m,
                         Simplex::edge(e),
                         PrimitiveType::Triangle)) {
                    if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                        ++n_tagged_faces;
                    }
                }
                if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                    edge_tag_acc.scalar_attribute(e) = tag_val;
                }
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 2, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 3, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 7, 4))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(3, 7, 4))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 4, 1))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 3, 0))));
    }
    SECTION("f023-f234")
    {
        // mark face(s)
        {
            auto face_tag_acc = m.create_accessor<int64_t>(face_tag_handle);
            face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_val;
            face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 4)) = tag_val;

            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            // tag edges at the substructure's boundary
            for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
                int64_t n_tagged_faces = 0;
                for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                         m,
                         Simplex::edge(e),
                         PrimitiveType::Triangle)) {
                    if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                        ++n_tagged_faces;
                    }
                }
                if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                    edge_tag_acc.scalar_attribute(e) = tag_val;
                }
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 2, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 3, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 7, 4))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(4, 5, 2))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 3, 0))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 4, 1))));
    }
    SECTION("f023")
    {
        // mark face(s)
        {
            auto face_tag_acc = m.create_accessor<int64_t>(face_tag_handle);
            face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_val;

            auto edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
            // tag edges at the substructure's boundary
            for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
                int64_t n_tagged_faces = 0;
                for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                         m,
                         Simplex::edge(e),
                         PrimitiveType::Triangle)) {
                    if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                        ++n_tagged_faces;
                    }
                }
                if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                    edge_tag_acc.scalar_attribute(e) = tag_val;
                }
            }
        }

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 2, 0))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 3, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 7, 4))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(4, 5, 2))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 3, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 4, 1))));
    }
    SECTION("f_boundaries")
    {
        // mark face(s)
        {
            auto face_tag_acc = m.create_accessor<int64_t>(face_tag_handle);
            for (const Tuple& t : m.get_all(PrimitiveType::Triangle)) {
                if (m.is_boundary_face(t)) {
                    face_tag_acc.scalar_attribute(t) = 1;
                }
            }
        }

        for (const Tuple& t : m.get_all(PrimitiveType::Edge)) {
            const Simplex e = Simplex::edge(t);
            if (m.is_boundary_edge(t)) {
                CHECK(inv.before(e));
            } else {
                CHECK_FALSE(inv.before(e));
            }
        }
    }
}

TEST_CASE("SubstructureTopologyPreservingInvariant_in_2_by_3_by_1_tet", "[invariants][2d][3d]")
{
    using namespace tests_3d;

    DEBUG_TetMesh m = two_by_three_grids_tets();

    const attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", PrimitiveType::Edge, 1);

    const attribute::MeshAttributeHandle face_tag_handle =
        m.register_attribute<int64_t>("face_tag", PrimitiveType::Triangle, 1);

    const int64_t tag_val = 1;

    TetMeshSubstructureTopologyPreservingInvariant inv(
        m,
        face_tag_handle.as<int64_t>(),
        edge_tag_handle.as<int64_t>(),
        tag_val);

    wmtk::attribute::Accessor<int64_t> edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
    wmtk::attribute::Accessor<int64_t> face_tag_acc = m.create_accessor<int64_t>(face_tag_handle);

    SECTION("bottom_plane_plus_face_1-7-18")
    {
        // shark_fin
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 4)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 2, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 7, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 7)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 8)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(8, 5, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 10, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 6, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(6, 7, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 11, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 6, 18)) = tag_val;

        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            int64_t n_tagged_faces = 0;
            for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                     m,
                     Simplex::edge(e),
                     PrimitiveType::Triangle)) {
                if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                    ++n_tagged_faces;
                }
            }
            if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                edge_tag_acc.scalar_attribute(e) = tag_val;
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 1, 0))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 4, 2))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(5, 6, 8))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(17, 5, 7))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(4, 5, 2))));

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 7, 10))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(7, 10, 26))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(7, 6, 11))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(18, 6, 8))));
    }

    SECTION("half_bottom_plane_plus_face_1-7-18")
    {
        // shark_fin
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 2, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 7, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 7)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 10, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 6, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(6, 7, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 11, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 6, 18)) = tag_val;

        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            int64_t n_tagged_faces = 0;
            for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                     m,
                     Simplex::edge(e),
                     PrimitiveType::Triangle)) {
                if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                    ++n_tagged_faces;
                }
            }
            if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                edge_tag_acc.scalar_attribute(e) = tag_val;
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(3, 7, 10))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(11, 7, 26))));

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 7, 10))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(7, 10, 26))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(7, 6, 11))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(18, 6, 8))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(5, 6, 8))));

        int failed_num = 0;
        for (const Tuple& edge : m.get_all(PrimitiveType::Edge)) {
            if (!inv.before(Simplex::edge(edge))) {
                ++failed_num;
            }
        }
        CHECK(failed_num == 13);
    }
    SECTION("two_layers")
    {
        // from left to right
        // ________ layer1
        // ________ layer2
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 4)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 2, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 7, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 7)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 8)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(8, 5, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 10, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 6, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(6, 7, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 11, 10)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(12, 13, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 17, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 18, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 14, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(14, 15, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(15, 19, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(16, 17, 21)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(16, 21, 20)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(17, 18, 21)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(18, 22, 21)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(18, 23, 22)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(18, 19, 23)) = tag_val;

        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            int64_t n_tagged_faces = 0;
            for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                     m,
                     Simplex::edge(e),
                     PrimitiveType::Triangle)) {
                if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                    ++n_tagged_faces;
                }
            }
            if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                edge_tag_acc.scalar_attribute(e) = tag_val;
            }
        }
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(17, 18, 7))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(19, 23, 28))));

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(13, 2, 6))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(18, 5, 8))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 13, 4))));

        int success_num = 0;
        for (const Tuple& edge : m.get_all(PrimitiveType::Edge)) {
            if (inv.before(Simplex::edge(edge))) {
                ++success_num;
            }
        }
        CHECK(success_num == (23 - 2) * 2);
    }
    SECTION("2by1_cube")
    {
        //    16_ _17_ _18
        // 12/_ _13_ _14/ |
        // | 04_ _05_ _ |06
        // 00/_ _01_ _02/

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 4, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 16, 12)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 13, 12)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 4)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(12, 13, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 17, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 17, 16)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 14, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 14, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 18, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 6, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 18, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 6, 5)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 6, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 18, 14)) = tag_val;

        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            int64_t n_tagged_faces = 0;
            for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                     m,
                     Simplex::edge(e),
                     PrimitiveType::Triangle)) {
                if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                    ++n_tagged_faces;
                }
            }
            if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                edge_tag_acc.scalar_attribute(e) = tag_val;
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(12, 13, 1))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 13, 1))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(5, 13, 9))));

        int failed_num = 0;
        for (const Tuple& edge : m.get_all(PrimitiveType::Edge)) {
            if (!inv.before(Simplex::edge(edge))) {
                ++failed_num;
            }
        }
        CHECK(failed_num == 11);
    }

    SECTION("3by1_cube")
    {
        //    16_ _..._ _19
        // 12/_ _..._ _15/ |
        // | 04_ _..._ _ |07
        // 00/_ _..._ _03/

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 4, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 16, 12)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 13, 12)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 4)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(12, 13, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 17, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 17, 16)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 14, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 14, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 18, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 6, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 18, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 5)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 6, 5)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 15)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 15, 14)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(14, 15, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(15, 19, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(6, 7, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 19, 18)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 7)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 7, 6)) = tag_val;

        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 7, 15)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 19, 15)) = tag_val;

        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            int64_t n_tagged_faces = 0;
            for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                     m,
                     Simplex::edge(e),
                     PrimitiveType::Triangle)) {
                if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                    ++n_tagged_faces;
                }
            }
            if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                edge_tag_acc.scalar_attribute(e) = tag_val;
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(2, 13, 5))));

        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(0, 13, 1))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(5, 13, 9))));

        int failed_num = 0;
        for (const Tuple& edge : m.get_all(PrimitiveType::Edge)) {
            if (!inv.before(Simplex::edge(edge))) {
                ++failed_num;
            }
        }
        CHECK(failed_num == 12);
    }
}

TEST_CASE("SubstructureTopologyPreservingInvariant_in_2_by_2_by_2_tet", "[invariants][2d][3d]")
{
    using namespace tests_3d;

    DEBUG_TetMesh m = two_by_two_by_two_grids_tets();

    const attribute::MeshAttributeHandle edge_tag_handle =
        m.register_attribute<int64_t>("edge_tag", PrimitiveType::Edge, 1);

    const attribute::MeshAttributeHandle face_tag_handle =
        m.register_attribute<int64_t>("face_tag", PrimitiveType::Triangle, 1);

    const int64_t tag_val = 1;

    TetMeshSubstructureTopologyPreservingInvariant inv(
        m,
        face_tag_handle.as<int64_t>(),
        edge_tag_handle.as<int64_t>(),
        tag_val);

    wmtk::attribute::Accessor<int64_t> edge_tag_acc = m.create_accessor<int64_t>(edge_tag_handle);
    wmtk::attribute::Accessor<int64_t> face_tag_acc = m.create_accessor<int64_t>(face_tag_handle);
    SECTION("one_bar_on_another")
    {
        //    _ _ _ _      24_25_26
        //  /_ _/_ _/|     21_22_23
        //  |   |   ||     |  |  ||
        //  |_ _|_ _|/     12_13_14
        // /_ _/||        /  /|7
        // |   ||/       9_10 4
        // |_ _|/        0_01/

        // lower bar (without the intersection faces)
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 10, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(9, 10, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(9, 13, 12)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 3)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 4, 3)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 12, 9)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 4, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 13, 10)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 4, 7)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 7, 6)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 6, 15)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 15, 12)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(4, 7, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 16, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(6, 7, 15)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(7, 16, 15)) = tag_val;

        // higher bar (without the intersection faces)
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(21, 22, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(21, 13, 12)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(21, 22, 25)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(21, 25, 24)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(24, 25, 15)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(15, 16, 25)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(22, 23, 13)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 14, 23)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(22, 23, 25)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(23, 26, 25)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(25, 26, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(16, 17, 25)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 14, 17)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 17, 16)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(14, 17, 23)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(17, 26, 23)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(12, 15, 21)) = tag_val;
        face_tag_acc.scalar_attribute(m.face_tuple_from_vids(21, 15, 24)) = tag_val;

        for (const Tuple& e : m.get_all(PrimitiveType::Edge)) {
            int64_t n_tagged_faces = 0;
            for (const Tuple& f : simplex::cofaces_single_dimension_tuples(
                     m,
                     Simplex::edge(e),
                     PrimitiveType::Triangle)) {
                if (face_tag_acc.const_scalar_attribute(f) == tag_val) {
                    ++n_tagged_faces;
                }
            }
            if (n_tagged_faces != 0 && n_tagged_faces != 2) {
                edge_tag_acc.scalar_attribute(e) = tag_val;
            }
        }

        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(10, 13, 7))));
        CHECK(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(12, 13, 30))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(1, 9, 0))));
        CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(13, 15, 12))));

        // // LOOK HERE!
        // // I don't know if this should be fail, since this vertex should be order-3?
        // CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(12, 13, 30))));

        // // tag the intersection face
        // face_tag_acc.scalar_attribute(m.face_tuple_from_vids(12, 13, 15)) = tag_val;
        // face_tag_acc.scalar_attribute(m.face_tuple_from_vids(13, 16, 15)) = tag_val;

        // CHECK_FALSE(inv.before(Simplex::edge(m.edge_tuple_between_v1_v2(12, 13, 30))));
    }
}