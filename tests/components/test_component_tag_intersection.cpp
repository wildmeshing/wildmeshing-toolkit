#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk_components/tag_intersection/internal/TagIntersectionOptions.hpp>
#include <wmtk_components/tag_intersection/tag_intersection.hpp>
#include "wmtk/../../tests/tools/DEBUG_TriMesh.hpp"
#include "wmtk/../../tests/tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;
using namespace tests;

TEST_CASE("component_tag_intersection_options", "[components][tag_intersection]")
{
    using namespace components::internal;

    json o = {
        {"type", "tag_intersection"},
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"input_tags", {{"tag1", PrimitiveType::Face, 1}, {"tag2", PrimitiveType::Face, 2}}},
        {"output_tags",
         {{"edge_interface", PrimitiveType::Edge, 42},
          {"vertex_interface", PrimitiveType::Vertex, 42}}}};

    CHECK_NOTHROW(o.get<TagIntersectionOptions>());

    o["type"] = "something else";
    CHECK_THROWS(o.get<TagIntersectionOptions>());
}

TEST_CASE("component_tag_intersection_tri", "[components][tag_intersection]")
{
    tests::DEBUG_TriMesh m = tests::edge_region_with_position();

    SECTION("two_face_tags_edgeintersection")
    {
        auto tag1 = m.register_attribute<long>("tag1", PrimitiveType::Face, 1, 0);
        auto tag2 = m.register_attribute<long>("tag2", PrimitiveType::Face, 1, 0);

        {
            auto tag1_acc = m.create_accessor(tag1);
            auto tag2_acc = m.create_accessor(tag2);
            tag1_acc.scalar_attribute(m.face_tuple_from_vids(1, 4, 5)) = 1;
            tag2_acc.scalar_attribute(m.face_tuple_from_vids(4, 8, 5)) = 1;
        }

        auto v_otag = m.register_attribute<long>("v_otag", PrimitiveType::Vertex, 1, 0);
        auto e_otag = m.register_attribute<long>("e_otag", PrimitiveType::Edge, 1, 0);

        std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {
            {tag1, 1},
            {tag2, 1}};
        std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags = {
            {v_otag, 1},
            {e_otag, 1}};

        components::tag_intersection_tri(m, input_tags, output_tags);

        auto v_otag_acc = m.create_accessor(v_otag);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 4)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(8, 4)) == 0);

        auto e_otag_acc = m.create_accessor(e_otag);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
    }

    SECTION("two_face_tags_vertexintersection")
    {
        auto tag1 = m.register_attribute<long>("tag1", PrimitiveType::Face, 1, 0);
        auto tag2 = m.register_attribute<long>("tag2", PrimitiveType::Face, 1, 0);

        {
            auto tag1_acc = m.create_accessor(tag1);
            auto tag2_acc = m.create_accessor(tag2);
            tag1_acc.scalar_attribute(m.face_tuple_from_vids(3, 4, 7)) = 1;
            tag2_acc.scalar_attribute(m.face_tuple_from_vids(4, 1, 5)) = 1;
        }

        auto v_otag = m.register_attribute<long>("v_otag", PrimitiveType::Vertex, 1, 0);
        auto e_otag = m.register_attribute<long>("e_otag", PrimitiveType::Edge, 1, 0);

        std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {
            {tag1, 1},
            {tag2, 1}};
        std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags = {
            {v_otag, 1},
            {e_otag, 1}};

        components::tag_intersection_tri(m, input_tags, output_tags);

        auto v_otag_acc = m.create_accessor(v_otag);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 3)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(3, 4)) == 0);

        auto e_otag_acc = m.create_accessor(e_otag);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 1)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 7)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 3)) == 0);
    }

    SECTION("four_blocks_tags_edgeintersection")
    {
        auto tag1 = m.register_attribute<long>("tag1", PrimitiveType::Face, 1, 0);
        auto tag2 = m.register_attribute<long>("tag2", PrimitiveType::Face, 1, 0);
        auto tag3 = m.register_attribute<long>("tag3", PrimitiveType::Face, 1, 0);
        auto tag4 = m.register_attribute<long>("tag4", PrimitiveType::Face, 1, 0);

        {
            auto tag1_acc = m.create_accessor(tag1);
            auto tag2_acc = m.create_accessor(tag2);
            auto tag3_acc = m.create_accessor(tag3);
            auto tag4_acc = m.create_accessor(tag4);
            tag1_acc.scalar_attribute(m.face_tuple_from_vids(3, 4, 7)) = 1;
            tag2_acc.scalar_attribute(m.face_tuple_from_vids(4, 1, 5)) = 1;
            tag3_acc.scalar_attribute(m.face_tuple_from_vids(4, 7, 8)) = 1;
            tag3_acc.scalar_attribute(m.face_tuple_from_vids(8, 5, 4)) = 1;
            tag4_acc.scalar_attribute(m.face_tuple_from_vids(3, 4, 0)) = 1;
            tag4_acc.scalar_attribute(m.face_tuple_from_vids(4, 1, 0)) = 1;
        }

        auto v_otag = m.register_attribute<long>("v_otag", PrimitiveType::Vertex, 1, 0);
        auto e_otag = m.register_attribute<long>("e_otag", PrimitiveType::Edge, 1, 0);

        std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {
            {tag1, 1},
            {tag2, 1},
            {tag3, 1},
            {tag4, 1}};
        std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags = {
            {v_otag, 1},
            {e_otag, 1}};

        components::tag_intersection_tri(m, input_tags, output_tags);

        auto v_otag_acc = m.create_accessor(v_otag);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 1)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(7, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(3, 4)) == 0);

        auto e_otag_acc = m.create_accessor(e_otag);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(3, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(7, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(0, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(8, 4)) == 0);
    }
    SECTION("two_seperated_vertices_and_two_intersected_vertices")
    {
        auto tag1 = m.register_attribute<long>("tag1", PrimitiveType::Vertex, 1, 0);
        auto tag2 = m.register_attribute<long>("tag2", PrimitiveType::Vertex, 1, 0);

        {
            auto tag1_acc = m.create_accessor(tag1);
            auto tag2_acc = m.create_accessor(tag2);
            tag1_acc.scalar_attribute(m.edge_tuple_from_vids(4, 5)) = 1;
            tag1_acc.scalar_attribute(m.edge_tuple_from_vids(1, 2)) = 1;
            tag2_acc.scalar_attribute(m.edge_tuple_from_vids(5, 4)) = 1;
            tag2_acc.scalar_attribute(m.edge_tuple_from_vids(1, 0)) = 1;
        }

        auto v_otag = m.register_attribute<long>("v_otag", PrimitiveType::Vertex, 1, 0);
        auto e_otag = m.register_attribute<long>("e_otag", PrimitiveType::Edge, 1, 0);

        std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {
            {tag1, 1},
            {tag2, 1}};
        std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags = {
            {v_otag, 1},
            {e_otag, 1}};

        components::tag_intersection_tri(m, input_tags, output_tags);

        auto v_otag_acc = m.create_accessor(v_otag);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 2)) == 1);

        auto e_otag_acc = m.create_accessor(e_otag);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 5)) == 0);
    }
    SECTION("two_seperated_edges_and_two_intersected_edges")
    {
        auto tag1 = m.register_attribute<long>("tag1", PrimitiveType::Edge, 1, 0);
        auto tag2 = m.register_attribute<long>("tag2", PrimitiveType::Edge, 1, 0);

        {
            auto tag1_acc = m.create_accessor(tag1);
            auto tag2_acc = m.create_accessor(tag2);
            tag1_acc.scalar_attribute(m.edge_tuple_from_vids(0, 4)) = 1;
            tag1_acc.scalar_attribute(m.edge_tuple_from_vids(4, 5)) = 1;
            tag2_acc.scalar_attribute(m.edge_tuple_from_vids(2, 5)) = 1;
            tag2_acc.scalar_attribute(m.edge_tuple_from_vids(5, 4)) = 1;
        }

        auto v_otag = m.register_attribute<long>("v_otag", PrimitiveType::Vertex, 1, 0);
        auto e_otag = m.register_attribute<long>("e_otag", PrimitiveType::Edge, 1, 0);

        std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {
            {tag1, 1},
            {tag2, 1}};
        std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags = {
            {v_otag, 1},
            {e_otag, 1}};

        components::tag_intersection_tri(m, input_tags, output_tags);

        auto v_otag_acc = m.create_accessor(v_otag);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 4)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(0, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(2, 5)) == 0);

        auto e_otag_acc = m.create_accessor(e_otag);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(0, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(2, 5)) == 0);
    }
    SECTION("two_overlap_blocks")
    {
        auto tag1 = m.register_attribute<long>("tag1", PrimitiveType::Face, 1, 0);
        auto tag2 = m.register_attribute<long>("tag2", PrimitiveType::Face, 1, 0);

        {
            auto tag1_acc = m.create_accessor(tag1);
            auto tag2_acc = m.create_accessor(tag2);
            tag1_acc.scalar_attribute(m.face_tuple_from_vids(4, 1, 0)) = 1;
            tag1_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 1)) = 1;
            tag2_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 1)) = 1;
            tag2_acc.scalar_attribute(m.face_tuple_from_vids(8, 5, 4)) = 1;
        }

        auto v_otag = m.register_attribute<long>("v_otag", PrimitiveType::Vertex, 1, 0);
        auto e_otag = m.register_attribute<long>("e_otag", PrimitiveType::Edge, 1, 0);
        auto f_otag = m.register_attribute<long>("f_otag", PrimitiveType::Face, 1, 0);

        std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {
            {tag1, 1},
            {tag2, 1}};
        std::vector<std::tuple<MeshAttributeHandle<long>, long>> output_tags = {
            {v_otag, 1},
            {e_otag, 1},
            {f_otag, 1}};

        components::tag_intersection_tri(m, input_tags, output_tags);

        auto v_otag_acc = m.create_accessor(v_otag);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 1)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 4)) == 1);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(0, 4)) == 0);
        CHECK(v_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(8, 5)) == 0);

        auto e_otag_acc = m.create_accessor(e_otag);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(1, 4)) == 1);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(4, 5)) == 1);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(5, 1)) == 1);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(0, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(0, 1)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(8, 4)) == 0);
        CHECK(e_otag_acc.const_scalar_attribute(m.edge_tuple_from_vids(8, 5)) == 0);

        auto f_otag_acc = m.create_accessor(f_otag);
        CHECK(f_otag_acc.const_scalar_attribute(m.face_tuple_from_vids(4, 5, 1)) == 1);
        CHECK(f_otag_acc.const_scalar_attribute(m.face_tuple_from_vids(0, 4, 1)) == 0);
        CHECK(f_otag_acc.const_scalar_attribute(m.face_tuple_from_vids(8, 5, 4)) == 0);
    }
}