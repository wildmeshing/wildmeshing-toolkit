#include <catch2/catch_test_macros.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TetMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/DEBUG_Tuple.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::tests_3d;
using namespace wmtk::simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

TEST_CASE("test_split_multi_mesh_2D_3D", "[multimesh][2D][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();

    auto child_0_tag_handle = parent.register_attribute<int64_t>("is_child_0", PF, 1).as<int64_t>();
    auto child_1_tag_handle = parent.register_attribute<int64_t>("is_child_1", PF, 1).as<int64_t>();

    auto child_0_tag_accessor = parent.create_accessor(child_0_tag_handle);
    auto child_1_tag_accessor = parent.create_accessor(child_1_tag_handle);

    for (const auto& f : parent.get_all(PF)) {
        if (parent.is_boundary(f, PF)) {
            child_0_tag_accessor.scalar_attribute(f) = 1;
        }
    }

    std::shared_ptr<Mesh> child_ptr_0 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_0",
            1,
            PF);

    const auto& parent_mmmanager = parent.multi_mesh_manager();

    auto child_0_handle = parent.get_attribute_handle<int64_t>("is_child_0", PF);
    auto child_1_handle = parent.get_attribute_handle<int64_t>("is_child_1", PF);

    operations::EdgeSplit split(parent);
    split.set_new_attribute_strategy(child_0_handle);
    split.set_new_attribute_strategy(child_1_handle);

    DEBUG_TriMesh& child0 = static_cast<DEBUG_TriMesh&>(*child_ptr_0);
    // DEBUG_TriMesh& child1 = static_cast<DEBUG_TriMesh&>(*child_ptr_1);


    SECTION("split_middle_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_f : child0.get_all(PF)) {
            child_to_parent[child0.id(child0_f, PF)] =
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(2, 3);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 12);

        for (const auto& child0_f : child0.get_all(PF)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT) >
                5); // all parent tets should be new

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_f, PF)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_f, PF)];
            }
            wmtk::logger().debug(
                "child 0 face {} maps to parent tet {} -> {} after split",
                child0.id(child0_f, PF),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT));
        }
    }

    SECTION("split_out_most_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_f : child0.get_all(PF)) {
            child_to_parent[child0.id(child0_f, PF)] =
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT);
        }

        Tuple edge = parent.edge_tuple_from_vids(0, 1);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 7);
        CHECK(child0.get_all(PF).size() == 14);

        for (const auto& child0_f : child0.get_all(PF)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT) >
                0); // tet 0 is splitted

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_f, PF)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_f, PF)];
            }
            wmtk::logger().debug(
                "child 0 face {} maps to parent tet {} -> {} after split",
                child0.id(child0_f, PF),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT));
        }
    }

    SECTION("split_fan_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_f : child0.get_all(PF)) {
            child_to_parent[child0.id(child0_f, PF)] =
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT);
        }

        Tuple edge = parent.edge_tuple_from_vids(0, 2);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 8);
        CHECK(child0.get_all(PF).size() == 14);

        for (const auto& child0_f : child0.get_all(PF)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT) >
                1); // tet 0 and 1 are splitted

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_f, PF)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_f, PF)];
            }
            wmtk::logger().debug(
                "child 0 face {} maps to parent tet {} -> {} after split",
                child0.id(child0_f, PF),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT));
        }
    }
}

TEST_CASE("test_split_multi_mesh_1D_3D", "[multimesh][1D][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();

    auto child_0_tag_handle = parent.register_attribute<int64_t>("is_child_0", PE, 1).as<int64_t>();

    auto child_0_tag_accessor = parent.create_accessor(child_0_tag_handle);

    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(0, 4)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(4, 5)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(5, 7)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(7, 6)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(6, 1)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(1, 0)) = 1;

    std::shared_ptr<Mesh> child_ptr_0 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_0",
            1,
            PE);

    const auto& parent_mmmanager = parent.multi_mesh_manager();

    auto child_0_handle = parent.get_attribute_handle<int64_t>("is_child_0", PE);

    operations::EdgeSplit split(parent);
    split.set_new_attribute_strategy(child_0_handle);

    DEBUG_EdgeMesh& child0 = static_cast<DEBUG_EdgeMesh&>(*child_ptr_0);

    SECTION("split_middle_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_e : child0.get_all(PE)) {
            child_to_parent[child0.id(child0_e, PE)] =
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(2, 3);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 12);

        for (const auto& child0_e : child0.get_all(PE)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT) >
                5); // all parent tets should be new

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_e, PE)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_e, PE)];
            }
            wmtk::logger().debug(
                "child 0 edge {} maps to parent tet {} -> {} after split",
                child0.id(child0_e, PE),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT));
        }
    }

    SECTION("split_out_most_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_e : child0.get_all(PE)) {
            child_to_parent[child0.id(child0_e, PE)] =
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(0, 1);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 7);
        CHECK(child0.get_all(PE).size() == 7);

        for (const auto& child0_e : child0.get_all(PE)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT) >
                0); // all parent tets should be new

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_e, PE)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_e, PE)];
            }
            wmtk::logger().debug(
                "child 0 edge {} maps to parent tet {} -> {} after split",
                child0.id(child0_e, PE),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT));
        }
    }

    SECTION("split_fan_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_e : child0.get_all(PE)) {
            child_to_parent[child0.id(child0_e, PE)] =
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(0, 2);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 8);
        CHECK(child0.get_all(PE).size() == 6);

        for (const auto& child0_e : child0.get_all(PE)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT) >
                1); // all parent tets should be new

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_e, PE)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_e, PE)];
            }
            wmtk::logger().info(
                "child 0 edge {} maps to parent tet {} -> {} after split",
                child0.id(child0_e, PE),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT));
        }
    }
}

TEST_CASE("test_collapse_multi_mesh_2D_3D", "[multimesh][2D][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();

    auto child_0_tag_handle = parent.register_attribute<int64_t>("is_child_0", PF, 1).as<int64_t>();
    auto child_1_tag_handle = parent.register_attribute<int64_t>("is_child_1", PF, 1).as<int64_t>();
    auto child_2_tag_handle = parent.register_attribute<int64_t>("is_child_2", PF, 1).as<int64_t>();
    auto child_3_tag_handle = parent.register_attribute<int64_t>("is_child_3", PF, 1).as<int64_t>();

    auto child_0_tag_accessor = parent.create_accessor(child_0_tag_handle);
    auto child_1_tag_accessor = parent.create_accessor(child_1_tag_handle);
    auto child_2_tag_accessor = parent.create_accessor(child_2_tag_handle);
    auto child_3_tag_accessor = parent.create_accessor(child_3_tag_handle);

    // child 0 one side surface
    child_0_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(0, 1, 2)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(0, 2, 4)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 4, 5)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 5, 7)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 7, 6)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 1, 6)) = 1;

    // child 1 full surface
    for (const auto& f : parent.get_all(PF)) {
        if (parent.is_boundary(f, PF)) {
            child_1_tag_accessor.scalar_attribute(f) = 1;
        }
    }

    // child 2 interior faces
    child_2_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(0, 2, 3)) = 1;
    child_2_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 3, 5)) = 1;

    // child 3 2-tri face
    child_3_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 5, 7)) = 1;
    child_3_tag_accessor.scalar_attribute(parent.face_tuple_from_vids(2, 6, 7)) = 1;

    std::shared_ptr<Mesh> child_ptr_0 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_0",
            1,
            PF);

    std::shared_ptr<Mesh> child_ptr_1 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_1",
            1,
            PF);

    std::shared_ptr<Mesh> child_ptr_2 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_2",
            1,
            PF);

    std::shared_ptr<Mesh> child_ptr_3 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_3",
            1,
            PF);

    const auto& parent_mmmanager = parent.multi_mesh_manager();

    auto child_0_handle = parent.get_attribute_handle<int64_t>("is_child_0", PF);
    auto child_1_handle = parent.get_attribute_handle<int64_t>("is_child_1", PF);
    auto child_2_handle = parent.get_attribute_handle<int64_t>("is_child_2", PF);
    auto child_3_handle = parent.get_attribute_handle<int64_t>("is_child_3", PF);

    operations::EdgeCollapse collapse(parent);
    collapse.set_new_attribute_strategy(child_0_handle);
    collapse.set_new_attribute_strategy(child_1_handle);
    collapse.set_new_attribute_strategy(child_2_handle);
    collapse.set_new_attribute_strategy(child_3_handle);
    collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));

    DEBUG_TriMesh& child0 = static_cast<DEBUG_TriMesh&>(*child_ptr_0);
    DEBUG_TriMesh& child1 = static_cast<DEBUG_TriMesh&>(*child_ptr_1);
    DEBUG_TriMesh& child2 = static_cast<DEBUG_TriMesh&>(*child_ptr_2);
    DEBUG_TriMesh& child3 = static_cast<DEBUG_TriMesh&>(*child_ptr_3);

    SECTION("collapse_outer_edge")
    {
        std::map<int64_t, int64_t> child0_to_parent;
        for (const auto& child0_f : child0.get_all(PF)) {
            child0_to_parent[child0.id(child0_f, PF)] =
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT);
        }

        std::map<int64_t, int64_t> child1_to_parent;
        for (const auto& child1_f : child1.get_all(PF)) {
            child1_to_parent[child1.id(child1_f, PF)] =
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(0, 1);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 5);
        CHECK(child0.get_all(PF).size() == 5);
        CHECK(child1.get_all(PF).size() == 10);
        CHECK(child2.get_all(PF).size() == 2);
        CHECK(child3.get_all(PF).size() == 2);

        for (const auto& child0_f : child0.get_all(PF)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT) >
                0); // tet 0 is collapsed

            int64_t parent_old = -1;
            if (child0_to_parent.find(child0.id(child0_f, PF)) != child0_to_parent.end()) {
                parent_old = child0_to_parent[child0.id(child0_f, PF)];
            }
            wmtk::logger().debug(
                "child 0 face {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_f, PF),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT));
        }

        for (const auto& child1_f : child1.get_all(PF)) {
            CHECK(
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT) >
                0); // tet 0 is collapsed

            int64_t parent_old = -1;
            if (child1_to_parent.find(child1.id(child1_f, PF)) != child1_to_parent.end()) {
                parent_old = child1_to_parent[child1.id(child1_f, PF)];
            }
            wmtk::logger().debug(
                "child 1 face {} maps to parent tet {} -> {} after collapse",
                child1.id(child1_f, PF),
                parent_old,
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT));
        }
    }

    SECTION("collapse_front_fan_edge")
    {
        std::map<int64_t, int64_t> child0_to_parent;
        for (const auto& child0_f : child0.get_all(PF)) {
            child0_to_parent[child0.id(child0_f, PF)] =
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT);
        }

        std::map<int64_t, int64_t> child1_to_parent;
        for (const auto& child1_f : child1.get_all(PF)) {
            child1_to_parent[child1.id(child1_f, PF)] =
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(0, 2);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 4);
        CHECK(child0.get_all(PF).size() == 4);
        CHECK(child1.get_all(PF).size() == 10);
        CHECK(child2.get_all(PF).size() == 1);
        CHECK(child3.get_all(PF).size() == 2);

        for (const auto& child0_f : child0.get_all(PF)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT) >
                1); // tet 0 and 1 is collapsed

            int64_t parent_old = -1;
            if (child0_to_parent.find(child0.id(child0_f, PF)) != child0_to_parent.end()) {
                parent_old = child0_to_parent[child0.id(child0_f, PF)];
            }
            wmtk::logger().debug(
                "child 0 face {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_f, PF),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT));
        }

        for (const auto& child1_f : child1.get_all(PF)) {
            CHECK(
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT) >
                1); // tet 0 and 1 is collapsed

            int64_t parent_old = -1;
            if (child1_to_parent.find(child1.id(child1_f, PF)) != child1_to_parent.end()) {
                parent_old = child1_to_parent[child1.id(child1_f, PF)];
            }
            wmtk::logger().debug(
                "child 1 face {} maps to parent tet {} -> {} after collapse",
                child1.id(child1_f, PF),
                parent_old,
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT));
        }
    }

    SECTION("collapse_back_fan_edge")
    {
        std::map<int64_t, int64_t> child0_to_parent;
        for (const auto& child0_f : child0.get_all(PF)) {
            child0_to_parent[child0.id(child0_f, PF)] =
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT);
        }

        std::map<int64_t, int64_t> child1_to_parent;
        for (const auto& child1_f : child1.get_all(PF)) {
            child1_to_parent[child1.id(child1_f, PF)] =
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(0, 3);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 4);
        CHECK(child0.get_all(PF).size() == 6);
        CHECK(child1.get_all(PF).size() == 10);
        CHECK(child2.get_all(PF).size() == 1);
        CHECK(child3.get_all(PF).size() == 2);

        for (const auto& child0_f : child0.get_all(PF)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT) >
                1); // tet 0 and 1 is collapsed

            int64_t parent_old = -1;
            if (child0_to_parent.find(child0.id(child0_f, PF)) != child0_to_parent.end()) {
                parent_old = child0_to_parent[child0.id(child0_f, PF)];
            }
            wmtk::logger().debug(
                "child 0 face {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_f, PF),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::face(child0_f)), PT));
        }

        for (const auto& child1_f : child1.get_all(PF)) {
            CHECK(
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT) >
                1); // tet 0 and 1 is collapsed

            int64_t parent_old = -1;
            if (child1_to_parent.find(child1.id(child1_f, PF)) != child1_to_parent.end()) {
                parent_old = child1_to_parent[child1.id(child1_f, PF)];
            }
            wmtk::logger().debug(
                "child 1 face {} maps to parent tet {} -> {} after collapse",
                child1.id(child1_f, PF),
                parent_old,
                parent.id(child1.map_to_parent_tuple(Simplex::face(child1_f)), PT));
        }
    }

    SECTION("collapse_middle_edge")
    {
        Tuple edge = parent.edge_tuple_from_vids(2, 3);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(collapse(Simplex::edge(edge)).empty()); // should fail
    }

    SECTION("collapse_degenerate_face_edge")
    {
        Tuple edge = parent.edge_tuple_from_vids(2, 7);
        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(collapse(Simplex::edge(edge)).empty()); // should fail
    }
}

TEST_CASE("test_collapse_multi_mesh_1D_3D", "[multimesh][2D][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();

    auto child_0_tag_handle = parent.register_attribute<int64_t>("is_child_0", PE, 1).as<int64_t>();
    auto child_1_tag_handle = parent.register_attribute<int64_t>("is_child_1", PE, 1).as<int64_t>();


    auto child_0_tag_accessor = parent.create_accessor(child_0_tag_handle);
    auto child_1_tag_accessor = parent.create_accessor(child_1_tag_handle);

    // ring
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(0, 4)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(4, 5)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(5, 7)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(7, 6)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(6, 1)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(1, 0)) = 1;

    // one edge
    child_1_tag_accessor.scalar_attribute(parent.edge_tuple_from_vids(2, 7)) = 1;

    std::shared_ptr<Mesh> child_ptr_0 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_0",
            1,
            PE);

    std::shared_ptr<Mesh> child_ptr_1 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_1",
            1,
            PE);

    const auto& parent_mmmanager = parent.multi_mesh_manager();

    auto child_0_handle = parent.get_attribute_handle<int64_t>("is_child_0", PE);
    auto child_1_handle = parent.get_attribute_handle<int64_t>("is_child_1", PE);

    operations::EdgeCollapse collapse(parent);

    collapse.set_new_attribute_strategy(child_0_handle);
    collapse.set_new_attribute_strategy(child_1_handle);
    collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));

    DEBUG_EdgeMesh& child0 = static_cast<DEBUG_EdgeMesh&>(*child_ptr_0);
    DEBUG_EdgeMesh& child1 = static_cast<DEBUG_EdgeMesh&>(*child_ptr_1);

    SECTION("collapse_outer_edge")
    {
        std::map<int64_t, int64_t> child0_to_parent;
        for (const auto& child0_e : child0.get_all(PE)) {
            child0_to_parent[child0.id(child0_e, PE)] =
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT);
        }

        Tuple edge = parent.edge_tuple_from_vids(0, 1);

        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 5);
        CHECK(child0.get_all(PE).size() == 5);
        CHECK(child1.get_all(PE).size() == 1);

        for (const auto& child0_e : child0.get_all(PE)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT) >
                0); // tet 0 is collapsed

            int64_t parent_old = -1;
            if (child0_to_parent.find(child0.id(child0_e, PE)) != child0_to_parent.end()) {
                parent_old = child0_to_parent[child0.id(child0_e, PE)];
            }
            wmtk::logger().debug(
                "child 0 edge {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_e, PE),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT));
        }
    }

    SECTION("collapse_fan_edge")
    {
        std::map<int64_t, int64_t> child0_to_parent;
        for (const auto& child0_e : child0.get_all(PE)) {
            child0_to_parent[child0.id(child0_e, PE)] =
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT);
        }

        Tuple edge = parent.edge_tuple_from_vids(0, 2);

        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 4);
        CHECK(child0.get_all(PE).size() == 6);
        CHECK(child1.get_all(PE).size() == 1);

        for (const auto& child0_e : child0.get_all(PE)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT) >
                1); // tet 0 and 1 is collapsed

            int64_t parent_old = -1;
            if (child0_to_parent.find(child0.id(child0_e, PE)) != child0_to_parent.end()) {
                parent_old = child0_to_parent[child0.id(child0_e, PE)];
            }
            wmtk::logger().info(
                "child 0 edge {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_e, PE),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT));
        }
    }

    SECTION("collapse_degenerate_edge")
    {
        // std::map<int64_t, int64_t> child0_to_parent;
        // for (const auto& child0_e : child0.get_all(PE)) {
        //     child0_to_parent[child0.id(child0_e, PE)] =
        //         parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT);
        // }

        Tuple edge = parent.edge_tuple_from_vids(2, 7);

        REQUIRE(parent.is_valid_slow(edge));
        REQUIRE(collapse(Simplex::edge(edge)).empty());

        // CHECK(parent.get_all(PT).size() == 4);
        // CHECK(child0.get_all(PE).size() == 6);
        // CHECK(child1.get_all(PE).size() == 1);

        // for (const auto& child0_e : child0.get_all(PE)) {
        //     CHECK(
        //         parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT) >
        //         1); // tet 0 and 1 is collapsed

        //     int64_t parent_old = -1;
        //     if (child0_to_parent.find(child0.id(child0_e, PE)) != child0_to_parent.end()) {
        //         parent_old = child0_to_parent[child0.id(child0_e, PE)];
        //     }
        //     wmtk::logger().info(
        //         "child 0 edge {} maps to parent tet {} -> {} after collapse",
        //         child0.id(child0_e, PE),
        //         parent_old,
        //         parent.id(child0.map_to_parent_tuple(Simplex::edge(child0_e)), PT));
        // }
    }
}

TEST_CASE("test_multi_mesh_navigation_3D", "[multimesh][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();
    std::shared_ptr<DEBUG_TetMesh> child0_ptr = std::make_shared<DEBUG_TetMesh>(two_ears());
    std::shared_ptr<DEBUG_TetMesh> child1_ptr = std::make_shared<DEBUG_TetMesh>(single_tet());

    auto& child0 = *child0_ptr;
    auto& child1 = *child1_ptr;

    auto child0_map = multimesh::same_simplex_dimension_surjection(parent, child0, {0, 1, 2});
    auto child1_map = multimesh::same_simplex_dimension_surjection(parent, child1, {0});

    parent.register_child_mesh(child0_ptr, child0_map);
    parent.register_child_mesh(child1_ptr, child1_map);

    auto get_single_child_tuple = [&](const auto& mesh, const auto& tuple) -> Tuple {
        auto tups = parent.map_to_child_tuples(mesh, Simplex(PT, tuple));
        REQUIRE(tups.size() == 1);
        return tups[0];
    };

    // check edges
    Tuple edge = parent.edge_tuple_between_v1_v2(0, 1, 2, 0);
    Tuple edge_child0 = get_single_child_tuple(child0, edge);
    Tuple edge_child1 = get_single_child_tuple(child1, edge);

    CHECK(edge_child0 == child0.edge_tuple_between_v1_v2(0, 1, 2, 0));
    CHECK(edge_child1 == child1.edge_tuple_between_v1_v2(0, 1, 2, 0));

    for (PrimitiveType pt : {PV, PE, PF}) {
        CHECK(
            child0.switch_tuple(edge_child0, pt) ==
            get_single_child_tuple(child0, parent.switch_tuple(edge, pt)));
        CHECK(
            child1.switch_tuple(edge_child1, pt) ==
            get_single_child_tuple(child1, parent.switch_tuple(edge, pt)));
    }
}

TEST_CASE("test_split_multi_mesh_3D_3D", "[multimesh][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();

    auto child_0_tag_handle = parent.register_attribute<int64_t>("is_child_0", PT, 1).as<int64_t>();

    auto child_0_tag_accessor = parent.create_accessor(child_0_tag_handle);

    child_0_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 1, 2, 3)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 2, 3, 4)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(2, 5, 3, 4)) = 1;

    std::shared_ptr<Mesh> child_ptr_0 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_0",
            1,
            PT);

    const auto& parent_mmmanager = parent.multi_mesh_manager();

    auto child_0_handle = parent.get_attribute_handle<int64_t>("is_child_0", PT);

    operations::EdgeSplit split(parent);
    split.set_new_attribute_strategy(child_0_handle);

    DEBUG_TetMesh& child0 = static_cast<DEBUG_TetMesh&>(*child_ptr_0);

    CHECK(child0.get_all(PT).size() == 3);
    CHECK(child0.get_all(PF).size() == 10);
    CHECK(child0.get_all(PE).size() == 12);
    CHECK(child0.get_all(PV).size() == 6);

    // std::cout << "tv0: " << std::endl << child0.tv_from_tid(0) << std::endl;
    // std::cout << "tv1: " << std::endl << child0.tv_from_tid(1) << std::endl;
    // std::cout << "tv2: " << std::endl << child0.tv_from_tid(2) << std::endl;

    // std::cout << "parent tv0: " << std::endl << parent.tv_from_tid(0) << std::endl;
    // std::cout << "parent tv1: " << std::endl << parent.tv_from_tid(1) << std::endl;
    // std::cout << "parent tv2: " << std::endl << parent.tv_from_tid(2) << std::endl;
    // std::cout << "parent tv3: " << std::endl << parent.tv_from_tid(3) << std::endl;
    // std::cout << "parent tv4: " << std::endl << parent.tv_from_tid(4) << std::endl;
    // std::cout << "parent tv5: " << std::endl << parent.tv_from_tid(5) << std::endl;


    SECTION("split_middle_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_t : child0.get_all(PT)) {
            child_to_parent[child0.id(child0_t, PT)] =
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(2, 3);

        REQUIRE(parent.is_valid_slow(edge));

        auto child_simplices = parent.map_to_child(child0, Simplex::edge(edge));

        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 12);
        CHECK(child0.get_all(PT).size() == 6);

        for (const auto& child0_t : child0.get_all(PT)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT) >
                5); // all parent tets should be new

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_t, PT)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_t, PT)];
            }
            wmtk::logger().debug(
                "child 0 tet {} maps to parent tet {} -> {} after split",
                child0.id(child0_t, PT),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT));
        }
    }

    SECTION("split_outer_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_t : child0.get_all(PT)) {
            child_to_parent[child0.id(child0_t, PT)] =
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT);
        }

        Tuple edge = parent.edge_tuple_from_vids(0, 1);

        REQUIRE(parent.is_valid_slow(edge));

        auto child_simplices = parent.map_to_child(child0, Simplex::edge(edge));

        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 7);
        CHECK(child0.get_all(PT).size() == 4);

        for (const auto& child0_t : child0.get_all(PT)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT) >
                0); // tet 0 is splitted
            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_t, PT)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_t, PT)];
            }
            wmtk::logger().debug(
                "child 0 tet {} maps to parent tet {} -> {} after split",
                child0.id(child0_t, PT),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT));
        }
    }

    SECTION("split_fan_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_t : child0.get_all(PT)) {
            child_to_parent[child0.id(child0_t, PT)] =
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT);
        }

        Tuple edge = parent.edge_tuple_from_vids(0, 2);

        REQUIRE(parent.is_valid_slow(edge));

        auto child_simplices = parent.map_to_child(child0, Simplex::edge(edge));

        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 8);
        CHECK(child0.get_all(PT).size() == 5);

        for (const auto& child0_t : child0.get_all(PT)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT) >
                1); // tet 0 and 1 are splitted
            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_t, PT)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_t, PT)];
            }
            wmtk::logger().debug(
                "child 0 tet {} maps to parent tet {} -> {} after split",
                child0.id(child0_t, PT),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT));
        }
    }

    SECTION("split_unrelavant_edge")
    {
        Tuple edge = parent.edge_tuple_from_vids(2, 6);

        REQUIRE(parent.is_valid_slow(edge));

        auto child_simplices = parent.map_to_child(child0, Simplex::edge(edge));

        REQUIRE(!split(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 8);
        CHECK(child0.get_all(PT).size() == 3);
    }
}

TEST_CASE("test_collapse_multi_mesh_3D_3D", "[multimesh][3D]")
{
    DEBUG_TetMesh parent = six_cycle_tets();

    auto child_0_tag_handle = parent.register_attribute<int64_t>("is_child_0", PT, 1).as<int64_t>();
    auto child_1_tag_handle = parent.register_attribute<int64_t>("is_child_1", PT, 1).as<int64_t>();

    auto child_0_tag_accessor = parent.create_accessor(child_0_tag_handle);
    auto child_1_tag_accessor = parent.create_accessor(child_1_tag_handle);

    child_0_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 1, 2, 3)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 2, 3, 4)) = 1;
    child_0_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(2, 5, 3, 4)) = 1;

    child_1_tag_accessor.scalar_attribute(parent.tet_tuple_from_vids(0, 2, 3, 4)) = 1;

    std::shared_ptr<Mesh> child_ptr_0 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_0",
            1,
            PT);

    std::shared_ptr<Mesh> child_ptr_1 =
        wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            parent,
            "is_child_1",
            1,
            PT);


    const auto& parent_mmmanager = parent.multi_mesh_manager();

    auto child_0_handle = parent.get_attribute_handle<int64_t>("is_child_0", PT);
    auto child_1_handle = parent.get_attribute_handle<int64_t>("is_child_1", PT);


    operations::EdgeCollapse collapse(parent);
    collapse.set_new_attribute_strategy(child_0_handle);
    collapse.set_new_attribute_strategy(child_1_handle);
    collapse.add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(parent));

    DEBUG_TetMesh& child0 = static_cast<DEBUG_TetMesh&>(*child_ptr_0);
    DEBUG_TetMesh& child1 = static_cast<DEBUG_TetMesh&>(*child_ptr_1);


    SECTION("collapse_outer_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_t : child0.get_all(PT)) {
            child_to_parent[child0.id(child0_t, PT)] =
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(0, 1);

        REQUIRE(parent.is_valid_slow(edge));

        auto child_simplices = parent.map_to_child(child0, Simplex::edge(edge));

        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 5);
        CHECK(child0.get_all(PT).size() == 2);
        CHECK(child1.get_all(PT).size() == 1);

        for (const auto& child0_t : child0.get_all(PT)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT) >
                0); // tet 0 is collapsed

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_t, PT)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_t, PT)];
            }
            wmtk::logger().debug(
                "child 0 tet {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_t, PT),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT));
        }
    }

    SECTION("collapse_fan_edge")
    {
        std::map<int64_t, int64_t> child_to_parent;
        for (const auto& child0_t : child0.get_all(PT)) {
            child_to_parent[child0.id(child0_t, PT)] =
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT);
        }
        Tuple edge = parent.edge_tuple_from_vids(1, 2);

        REQUIRE(parent.is_valid_slow(edge));

        auto child_simplices = parent.map_to_child(child0, Simplex::edge(edge));

        REQUIRE(!collapse(Simplex::edge(edge)).empty());

        CHECK(parent.get_all(PT).size() == 4);
        CHECK(child0.get_all(PT).size() == 2);
        CHECK(child1.get_all(PT).size() == 1);

        for (const auto& child0_t : child0.get_all(PT)) {
            CHECK(
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT) !=
                3); // tet 3 is collapsed

            int64_t parent_old = -1;
            if (child_to_parent.find(child0.id(child0_t, PT)) != child_to_parent.end()) {
                parent_old = child_to_parent[child0.id(child0_t, PT)];
            }
            wmtk::logger().debug(
                "child 0 tet {} maps to parent tet {} -> {} after collapse",
                child0.id(child0_t, PT),
                parent_old,
                parent.id(child0.map_to_parent_tuple(Simplex::tetrahedron(child0_t)), PT));
        }
    }

    SECTION("collapse_middle_edge")
    {
        Tuple edge = parent.edge_tuple_from_vids(2, 3);

        REQUIRE(parent.is_valid_slow(edge));

        REQUIRE(collapse(Simplex::edge(edge)).empty());
    }

    SECTION("collapse_child_degenerate_edge")
    {
        Tuple edge = parent.edge_tuple_from_vids(0, 4);

        REQUIRE(parent.is_valid_slow(edge));

        REQUIRE(collapse(Simplex::edge(edge)).empty());
    }
}