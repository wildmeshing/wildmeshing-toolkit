#include <catch2/catch_test_macros.hpp>

#include <wmtk/Types.hpp>
#include <wmtk/multimesh/utils/create_tag.hpp>

#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::multimesh::utils;
TEST_CASE("test_tag_initiation")
{
    DEBUG_TriMesh parent = hex_plus_two();
    std::set<int64_t> critical_vids = {0, 2, 3, 6, 7, 9};
    wmtk::multimesh::utils::internal::TupleTag tuple_tag(parent, critical_vids);
    tuple_tag.initialize();

    std::vector<Tuple> e_tuples = parent.get_all(PrimitiveType::Edge);
    for (const Tuple& e : e_tuples) {
        if (parent.is_boundary(PrimitiveType::Edge, e)) {
            REQUIRE(tuple_tag.get_edge_tag(e) == -1);
            REQUIRE(tuple_tag.m_edge_tag_acc.const_scalar_attribute(e) == -1);
        }
    }
}

TEST_CASE("test_create_tags")
{
    DEBUG_TriMesh parent = edge_region();
    std::set<int64_t> critical_vids = {0, 2, 3, 6, 7, 9};
    auto tags = create_tags(parent, critical_vids);
    REQUIRE(tags.size() == 6);
    // get attribute handle
    attribute::TypedAttributeHandle<int64_t> edge_tag_handle =
        parent.get_attribute_handle<int64_t>("edge_tag", PrimitiveType::Edge).as<int64_t>();
    const wmtk::attribute::Accessor edge_tag_accessor = parent.create_const_accessor(edge_tag_handle);

    attribute::TypedAttributeHandle<int64_t> vertex_tag_handle =
        parent.get_attribute_handle<int64_t>("vertex_tag", PrimitiveType::Vertex).as<int64_t>();
    const wmtk::attribute::Accessor vertex_tag_accessor = parent.create_const_accessor(vertex_tag_handle);

    std::vector<Tuple> e_tuples = parent.get_all(PrimitiveType::Edge);
    for (const Tuple& e : e_tuples) {
        if (parent.is_boundary(PrimitiveType::Edge, e)) {
            REQUIRE(edge_tag_accessor.const_scalar_attribute(e) > -1);
        }
    }

    Tuple v1 = parent.edge_tuple_with_vs_and_t(1, 0, 1);
    REQUIRE(parent.id(v1, PrimitiveType::Vertex) == 1);
    Tuple v2 = parent.edge_tuple_with_vs_and_t(8, 7, 6);
    REQUIRE(parent.id(v2, PrimitiveType::Vertex) == 8);

    REQUIRE(vertex_tag_accessor.const_scalar_attribute(v1) == 1);
    REQUIRE(vertex_tag_accessor.const_scalar_attribute(v2) == 8);

    Tuple e1 = parent.edge_tuple_from_vids(0, 1);
    Tuple e2 = parent.edge_tuple_from_vids(1, 2);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e1) ==
        edge_tag_accessor.const_scalar_attribute(e2));

    Tuple e3 = parent.edge_tuple_from_vids(0, 3);
    REQUIRE(edge_tag_accessor.const_scalar_attribute(e3) > 9);

    Tuple e4 = parent.edge_tuple_from_vids(2, 6);
    REQUIRE(edge_tag_accessor.const_scalar_attribute(e4) > 9);

    Tuple e5 = parent.edge_tuple_from_vids(3, 7);
    REQUIRE(edge_tag_accessor.const_scalar_attribute(e5) > 9);

    Tuple e6 = parent.edge_tuple_from_vids(8, 7);
    Tuple e7 = parent.edge_tuple_from_vids(8, 9);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e6) ==
        edge_tag_accessor.const_scalar_attribute(e7));
}

TEST_CASE("create_tags_2")
{
    DEBUG_TriMesh parent = embedded_diamond();
    std::set<int64_t> critical_vids = {0, 1, 5, 8, 13, 12};
    auto tags = create_tags(parent, critical_vids);
    REQUIRE(tags.size() == 6);
    // get attribute handle
    attribute::TypedAttributeHandle<int64_t> edge_tag_handle =
        parent.get_attribute_handle<int64_t>("edge_tag", PrimitiveType::Edge).as<int64_t>();
    const wmtk::attribute::Accessor edge_tag_accessor = parent.create_const_accessor(edge_tag_handle);


    // get attribute handle
    attribute::TypedAttributeHandle<int64_t> vertex_tag_handle =
        parent.get_attribute_handle<int64_t>("vertex_tag", PrimitiveType::Vertex).as<int64_t>();
    const wmtk::attribute::Accessor vertex_tag_accessor = parent.create_const_accessor(vertex_tag_handle);

    std::vector<Tuple> e_tuples = parent.get_all(PrimitiveType::Edge);

    for (const Tuple& e : e_tuples) {
        if (parent.is_boundary(PrimitiveType::Edge, e)) {
            REQUIRE(edge_tag_accessor.const_scalar_attribute(e) > -1);
        }
    }

    Tuple v1 = parent.edge_tuple_with_vs_and_t(2, 0, 0);
    REQUIRE(parent.id(v1, PrimitiveType::Vertex) == 2);
    Tuple v2 = parent.edge_tuple_with_vs_and_t(4, 1, 2);
    REQUIRE(parent.id(v2, PrimitiveType::Vertex) == 4);

    REQUIRE(vertex_tag_accessor.const_scalar_attribute(v1) == 2);
    REQUIRE(vertex_tag_accessor.const_scalar_attribute(v2) == 4);

    Tuple e1 = parent.edge_tuple_from_vids(0, 2);
    Tuple e2 = parent.edge_tuple_from_vids(5, 2);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e1) ==
        edge_tag_accessor.const_scalar_attribute(e2));

    Tuple e3 = parent.edge_tuple_from_vids(0, 1);
    REQUIRE(edge_tag_accessor.const_scalar_attribute(e3) > 13);

    Tuple e4 = parent.edge_tuple_from_vids(12, 13);
    REQUIRE(edge_tag_accessor.const_scalar_attribute(e4) > 13);

    Tuple e6 = parent.edge_tuple_from_vids(11, 8);
    Tuple e7 = parent.edge_tuple_from_vids(11, 13);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e6) ==
        edge_tag_accessor.const_scalar_attribute(e7));

    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e6) !=
        edge_tag_accessor.const_scalar_attribute(e3));

    Tuple e8 = parent.edge_tuple_from_vids(4, 8);
    Tuple e9 = parent.edge_tuple_from_vids(1, 4);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e8) ==
        edge_tag_accessor.const_scalar_attribute(e9));

    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e8) !=
        edge_tag_accessor.const_scalar_attribute(e7));
}

TEST_CASE("no_critical_point")
{
    DEBUG_TriMesh parent = embedded_diamond();
    std::set<int64_t> critical_vids = {};
    auto tags = create_tags(parent, critical_vids);
    REQUIRE(tags.size() == 1);

    attribute::TypedAttributeHandle<int64_t> edge_tag_handle =
        parent.get_attribute_handle<int64_t>("edge_tag", PrimitiveType::Edge).as<int64_t>();
    const wmtk::attribute::Accessor edge_tag_accessor = parent.create_const_accessor(edge_tag_handle);
    std::vector<Tuple> e_tuples = parent.get_all(PrimitiveType::Edge);
    for (const Tuple& e : e_tuples) {
        if (parent.is_boundary(PrimitiveType::Edge, e)) {
            REQUIRE(edge_tag_accessor.const_scalar_attribute(e) == 0);
        }
    }

    Tuple e1 = parent.edge_tuple_from_vids(0, 2);
    Tuple e2 = parent.edge_tuple_from_vids(5, 2);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e1) ==
        edge_tag_accessor.const_scalar_attribute(e2));

    Tuple e3 = parent.edge_tuple_from_vids(12, 13);
    Tuple e4 = parent.edge_tuple_from_vids(13, 11);
    REQUIRE(
        edge_tag_accessor.const_scalar_attribute(e3) ==
        edge_tag_accessor.const_scalar_attribute(e4));
}

TEST_CASE("one_critical_point")
{
    DEBUG_TriMesh parent = embedded_diamond();
    std::set<int64_t> critical_vids = {4};
    auto tags = create_tags(parent, critical_vids);
    REQUIRE(tags.size() == 1);
    attribute::TypedAttributeHandle<int64_t> edge_tag_handle =
        parent.get_attribute_handle<int64_t>("edge_tag", PrimitiveType::Edge).as<int64_t>();
    const wmtk::attribute::Accessor edge_tag_accessor = parent.create_const_accessor(edge_tag_handle);

    std::vector<Tuple> e_tuples = parent.get_all(PrimitiveType::Edge);
    for (const Tuple& e : e_tuples) {
        if (parent.is_boundary(PrimitiveType::Edge, e)) {
            REQUIRE(edge_tag_accessor.const_scalar_attribute(e) == 0);
        }
    }
}
