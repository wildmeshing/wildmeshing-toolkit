#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/DEBUG_Tuple.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
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

        const Tuple& e01 = parent.edge_tuple_between_v1_v2(0, 1, 0);
        const Tuple& e02 = parent.edge_tuple_between_v1_v2(0, 2, 0);
        const Tuple& e12 = parent.edge_tuple_between_v1_v2(1, 2, 0);
        const Tuple& e03 = parent.edge_tuple_between_v1_v2(0, 3, 1);
        const Tuple& e13 = parent.edge_tuple_between_v1_v2(1, 3, 1);
        const Tuple& e04 = parent.edge_tuple_between_v1_v2(0, 4, 2);
        const Tuple& e24 = parent.edge_tuple_between_v1_v2(2, 4, 2);

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


        const auto& p_mul_manager = parent.multi_mesh_manager();
        REQUIRE(p_mul_manager.children().size() == 3);

        const auto& child0 = *(p_mul_manager.children()[0].mesh);
        const auto& child1 = *(p_mul_manager.children()[1].mesh);
        const auto& child2 = *(p_mul_manager.children()[2].mesh);
        CHECK(child0.get_all(PE).size() == 3);
        CHECK(child0.get_all(PV).size() == 3);
        CHECK(child1.get_all(PE).size() == 2);
        CHECK(child1.get_all(PV).size() == 3);
        CHECK(child2.get_all(PE).size() == 2);
        CHECK(child2.get_all(PV).size() == 3);
    }
}

TEST_CASE("test_extract_child_face_mesh", "[multimesh][extract_childmesh]")
{
    SECTION("single_triangle")
    {
        DEBUG_TriMesh parent = single_triangle();
        auto tag_handle =
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Face, 1)
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
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Face, 1)
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
            parent.register_attribute<int64_t>("is_child", wmtk::PrimitiveType::Face, 1)
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
}
