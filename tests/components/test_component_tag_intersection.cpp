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
    tests::DEBUG_TriMesh m = tests::edge_region();

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

    std::vector<std::tuple<MeshAttributeHandle<long>, long>> input_tags = {{tag1, 1}, {tag2, 1}};
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