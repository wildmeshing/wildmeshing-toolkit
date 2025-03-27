#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Triangle;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("test_extract_child_point_mesh", "[multimesh][extract_childmesh]")
{
    SECTION("single_triangle")
    {
        DEBUG_TriMesh parent = single_triangle();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Vertex, 1)
                .as<int64_t>();
        auto tag_accessor = parent.create_accessor(tag_handle);

        for (const Tuple& t : parent.get_all(PV)) {
            tag_accessor.scalar_attribute(t) = 1;
        }

        REQUIRE_THROWS(wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child",
            1,
            PV));
    }
}

TEST_CASE("test_extract_child_edge_mesh", "[multimesh][extract_childmesh]")
{
    SECTION("single_triangle")
    {
        DEBUG_TriMesh parent = single_triangle();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Edge, 1)
                .as<int64_t>();
        auto tag_accessor = parent.create_accessor(tag_handle);

        for (const Tuple& t : parent.get_all(PE)) {
            tag_accessor.scalar_attribute(t) = 1;
        }

        std::shared_ptr<Mesh> child_ptr =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PE);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 1);

        const auto& child = *(p_mul_manager.children()[0].mesh);
        CHECK(child.get_all(PE).size() == 3);
        CHECK(child.get_all(PV).size() == 3);
    }
    SECTION("two_neighbors")
    {
        DEBUG_TriMesh parent = two_neighbors();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Edge, 1)
                .as<int64_t>();
        auto tag_accessor = parent.create_accessor(tag_handle);

        const Tuple& e01 = parent.edge_tuple_with_vs_and_t(0, 1, 0);
        const Tuple& e02 = parent.edge_tuple_with_vs_and_t(0, 2, 0);
        const Tuple& e12 = parent.edge_tuple_with_vs_and_t(1, 2, 0);
        const Tuple& e03 = parent.edge_tuple_with_vs_and_t(0, 3, 1);
        const Tuple& e13 = parent.edge_tuple_with_vs_and_t(1, 3, 1);
        const Tuple& e04 = parent.edge_tuple_with_vs_and_t(0, 4, 2);
        const Tuple& e24 = parent.edge_tuple_with_vs_and_t(2, 4, 2);

        tag_accessor.scalar_attribute(e01) = 1;
        tag_accessor.scalar_attribute(e02) = 1;
        tag_accessor.scalar_attribute(e12) = 1;
        tag_accessor.scalar_attribute(e03) = 2;
        tag_accessor.scalar_attribute(e13) = 2;
        tag_accessor.scalar_attribute(e04) = 3;
        tag_accessor.scalar_attribute(e24) = 3;


        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PE);
        std::shared_ptr<Mesh> child_ptr1 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PE);
        std::shared_ptr<Mesh> child_ptr2 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                3,
                PE);
        std::shared_ptr<Mesh> free_child =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PE,
                true);


        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 4);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        const auto& child2 = *(p_mul_manager.children()[2].mesh);
        CHECK(child0.get_all(PE).size() == 3);
        CHECK(child0.get_all(PV).size() == 3);
        CHECK(child1.get_all(PE).size() == 2);
        CHECK(child1.get_all(PV).size() == 3);
        CHECK(child2.get_all(PE).size() == 2);
        CHECK(child2.get_all(PV).size() == 3);

        REQUIRE(p_mul_manager.children()[3].mesh == free_child);
        CHECK(free_child->get_all(PE).size() == 3);
        CHECK(free_child->get_all(PV).size() == 6);
    }
}

TEST_CASE("test_extract_child_face_mesh", "[multimesh][extract_childmesh]")
{
    SECTION("single_triangle")
    {
        DEBUG_TriMesh parent = single_triangle();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Triangle, 1)
                .as<int64_t>();
        auto tag_accessor = parent.create_accessor(tag_handle);

        for (const Tuple& t : parent.get_all(PF)) {
            tag_accessor.scalar_attribute(t) = 1;
        }

        std::shared_ptr<Mesh> child_ptr =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PF);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 1);

        const auto& child = *(p_mul_manager.children()[0].mesh);
        CHECK(child.get_all(PF).size() == 1);
        CHECK(child.get_all(PE).size() == 3);
        CHECK(child.get_all(PV).size() == 3);
    }
    SECTION("two_neighbors")
    {
        DEBUG_TriMesh parent = two_neighbors();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Triangle, 1)
                .as<int64_t>();
        auto tag_accessor = parent.create_accessor(tag_handle);

        const auto face_tuples = parent.get_all(PF);
        REQUIRE(face_tuples.size() == 3);

        tag_accessor.scalar_attribute(face_tuples[0]) = 1;
        tag_accessor.scalar_attribute(face_tuples[1]) = 1;
        tag_accessor.scalar_attribute(face_tuples[2]) = 2;


        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PF);
        std::shared_ptr<Mesh> child_ptr1 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PF);


        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 2);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        CHECK(child0.get_all(PF).size() == 2);
        CHECK(child0.get_all(PE).size() == 5);
        CHECK(child0.get_all(PV).size() == 4);
        CHECK(child1.get_all(PF).size() == 1);
        CHECK(child1.get_all(PE).size() == 3);
        CHECK(child1.get_all(PV).size() == 3);
    }
}

TEST_CASE("test_extract_child_face_mesh_3d", "[multimesh][extract_childmesh]")
{
    SECTION("single_tet")
    {
        DEBUG_TetMesh parent = single_tet();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Triangle, 1)
                .as<int64_t>();
        ;
        auto tag_accessor = parent.create_accessor(tag_handle);

        const auto face_tuples = parent.get_all(PF);
        REQUIRE(face_tuples.size() == 4);

        tag_accessor.scalar_attribute(face_tuples[0]) = 1;
        tag_accessor.scalar_attribute(face_tuples[1]) = 1;
        tag_accessor.scalar_attribute(face_tuples[2]) = 2;
        tag_accessor.scalar_attribute(face_tuples[3]) = 2;

        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PF);
        std::shared_ptr<Mesh> child_ptr1 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PF);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 2);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        CHECK(child0.get_all(PF).size() == 2);
        CHECK(child0.get_all(PE).size() == 5);
        CHECK(child0.get_all(PV).size() == 4);
        CHECK(child1.get_all(PF).size() == 2);
        CHECK(child1.get_all(PE).size() == 5);
        CHECK(child1.get_all(PV).size() == 4);
    }

    SECTION("six_cycle_tets")
    {
        DEBUG_TetMesh parent = six_cycle_tets();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Triangle, 1)
                .as<int64_t>();
        ;
        auto tag_accessor = parent.create_accessor(tag_handle);

        const auto face_tuples = parent.get_all(PF);

        for (const auto& f : face_tuples) {
            if (parent.is_boundary(PF, f)) {
                tag_accessor.scalar_attribute(f) = 1;
            }
        }

        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PF);
        std::shared_ptr<Mesh> free_child =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PF,
                true);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 2);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);

        CHECK(child0.get_all(PF).size() == 12);
        CHECK(child0.get_all(PE).size() == 18);
        CHECK(child0.get_all(PV).size() == 8);
        CHECK(free_child->get_all(PF).size() == 12);
        CHECK(free_child->get_all(PE).size() == 36);
        CHECK(free_child->get_all(PV).size() == 36);
    }
}

TEST_CASE("test_extract_child_edge_mesh_3d", "[multimesh][extract_childmesh]")
{
    SECTION("single_tet")
    {
        DEBUG_TetMesh parent = single_tet();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Edge, 1)
                .as<int64_t>();
        ;
        auto tag_accessor = parent.create_accessor(tag_handle);

        const auto edge_tuples = parent.get_all(PE);
        REQUIRE(edge_tuples.size() == 6);

        tag_accessor.scalar_attribute(edge_tuples[0]) = 1; // 0-1
        tag_accessor.scalar_attribute(edge_tuples[1]) = 2; // 0-2
        tag_accessor.scalar_attribute(edge_tuples[2]) = 2; // 0-3
        tag_accessor.scalar_attribute(edge_tuples[3]) = 1; // 1-2
        tag_accessor.scalar_attribute(edge_tuples[4]) = 2; // 1-3
        tag_accessor.scalar_attribute(edge_tuples[5]) = 1; // 2-3

        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PE);
        std::shared_ptr<Mesh> child_ptr1 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PE);

        std::shared_ptr<Mesh> free_ptr =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PE,
                true);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 3);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        CHECK(child0.get_all(PE).size() == 3);
        CHECK(child0.get_all(PV).size() == 4);
        CHECK(child1.get_all(PE).size() == 3);
        CHECK(child1.get_all(PV).size() == 4);
        CHECK(free_ptr->get_all(PE).size() == 3);
        CHECK(free_ptr->get_all(PV).size() == 6);
    }

    SECTION("six_cycle_tet")
    {
        DEBUG_TetMesh parent = six_cycle_tets();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Edge, 1)
                .as<int64_t>();
        ;
        auto tag_accessor = parent.create_accessor(tag_handle);

        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(0, 4)) = 1;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(4, 5)) = 1;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(5, 7)) = 1;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(7, 6)) = 1;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(6, 1)) = 1;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(1, 0)) = 1;

        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(0, 2)) = 2;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(2, 3)) = 2;
        tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(3, 7)) = 2;

        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PE);
        std::shared_ptr<Mesh> child_ptr1 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PE);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 2);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        CHECK(child0.get_all(PE).size() == 6);
        CHECK(child0.get_all(PV).size() == 6);
        CHECK(child1.get_all(PE).size() == 3);
        CHECK(child1.get_all(PV).size() == 4);
    }
}

TEST_CASE("test_extract_child_tet_mesh_3d", "[multimesh][extract_childmesh]")
{
    SECTION("six_cycle_tet")
    {
        DEBUG_TetMesh parent = six_cycle_tets();
        auto tag_handle = parent.register_attribute<int64_t>("is_child", PT, 1).as<int64_t>();
        ;
        auto tag_accessor = parent.create_accessor(tag_handle);

        tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 1, 2, 3)) = 1;
        tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 2, 3, 4)) = 1;

        tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(4, 2, 3, 5)) = 2;
        tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(5, 2, 3, 7)) = 2;
        tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(7, 2, 3, 6)) = 2;
        tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(6, 2, 3, 1)) = 2;

        std::shared_ptr<Mesh> child_ptr0 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                1,
                PT);
        std::shared_ptr<Mesh> child_ptr1 =
            wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
                parent,
                "is_child",
                2,
                PT);

        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 2);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        CHECK(child0.get_all(PT).size() == 2);
        CHECK(child0.get_all(PF).size() == 7);
        CHECK(child0.get_all(PE).size() == 9);
        CHECK(child0.get_all(PV).size() == 5);
        CHECK(child1.get_all(PT).size() == 4);
        CHECK(child1.get_all(PF).size() == 13);
        CHECK(child1.get_all(PE).size() == 15);
        CHECK(child1.get_all(PV).size() == 7);
    }
}
