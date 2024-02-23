#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/AttributeScopeStack.hpp>
#include <wmtk/attribute/TupleAccessor.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "../tools/DEBUG_PointMesh.hpp"


using namespace wmtk::tests;
namespace {} // namespace

TEST_CASE("test_single_tuple_accessor", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 2, false, -1);
    REQUIRE(m.get_attribute_dimension(int64_t_handle) == 2);
    auto int64_t_acc = m.create_accessor(int64_t_handle);

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    wmtk::attribute::TupleAccessor<wmtk::PointMesh> tuple_accessor(m, int64_t_handle);

    REQUIRE(int64_t_acc.reserved_size() == size);
    REQUIRE(tuple_accessor.dimension() == 1);

    // test default initialization to 0
    for (const wmtk::Tuple& tup : vertices) {
        int64_t gid = m.id(tup);
        auto v = int64_t_acc.const_vector_attribute(tup);
        CHECK(v.x() == -1);
        CHECK(v.y() == -1);
        const wmtk::Tuple& t = tuple_accessor.const_scalar_attribute(tup);
        wmtk::Tuple& t_ref = tuple_accessor.scalar_attribute(tup);
        CHECK(t == t_ref);
        CHECK(t.is_null());
        t_ref = wmtk::Tuple(gid, gid + 1, gid + 2, gid + 3, gid + 4);
        {
            const wmtk::Tuple t2 = tuple_accessor.const_scalar_attribute(tup);
            CHECK(wmtk::utils::TupleInspector::local_vid(t2) == gid);
            CHECK(wmtk::utils::TupleInspector::local_eid(t2) == gid + 1);
            CHECK(wmtk::utils::TupleInspector::local_fid(t2) == gid + 2);
            CHECK(wmtk::utils::TupleInspector::global_cid(t2) == gid + 3);
            CHECK(wmtk::utils::TupleInspector::hash(t2) == gid + 4);
        }
    }
    for (const wmtk::Tuple& tup : vertices) {
        int64_t gid = m.id(tup);
        auto v = int64_t_acc.const_vector_attribute(tup);
        const wmtk::Tuple t = tuple_accessor.const_scalar_attribute(tup);
        CHECK(!t.is_null());
        CHECK(wmtk::utils::TupleInspector::local_vid(t) == gid);
        CHECK(wmtk::utils::TupleInspector::local_eid(t) == gid + 1);
        CHECK(wmtk::utils::TupleInspector::local_fid(t) == gid + 2);
        CHECK(wmtk::utils::TupleInspector::global_cid(t) == gid + 3);
        CHECK(wmtk::utils::TupleInspector::hash(t) == gid + 4);
    }
}

TEST_CASE("test_multi_tuple_accessor", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 4, false, -1);
    REQUIRE(m.get_attribute_dimension(int64_t_handle) == 4);
    auto int64_t_acc = m.create_accessor(int64_t_handle);

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    wmtk::attribute::TupleAccessor<wmtk::PointMesh> tuple_accessor(m, int64_t_handle);

    REQUIRE(int64_t_acc.reserved_size() == size);
    REQUIRE(tuple_accessor.dimension() == 2);

    // test default initialization to 0
    for (const wmtk::Tuple& tup : vertices) {
        auto v = int64_t_acc.const_vector_attribute(tup);
        CHECK(v.x() == -1);
        CHECK(v.y() == -1);
        auto t = tuple_accessor.vector_attribute(tup);
        REQUIRE(t.size() == 2);
        REQUIRE(t.rows() == 2);
        REQUIRE(t.cols() == 1);
        // CHECK(t == t_ref);
        CHECK(t(0).is_null());
        CHECK(t(1).is_null());
        int64_t gid = m.id(tup);
        t(0) = wmtk::Tuple(gid, gid + 1, gid + 2, gid + 3, gid + 4);
        t(1) = wmtk::Tuple(gid, gid + 5, gid + 6, gid + 7, gid + 8);
    }
    for (const wmtk::Tuple& tup : vertices) {
        auto v = int64_t_acc.const_vector_attribute(tup);
        const auto t = tuple_accessor.const_vector_attribute(tup);
        REQUIRE(t.size() == 2);
        REQUIRE(t.rows() == 2);
        REQUIRE(t.cols() == 1);
        // CHECK(t == t_ref);
        CHECK(!t(0).is_null());
        CHECK(!t(1).is_null());
        int64_t gid = m.id(tup);
        CHECK(t(0) == wmtk::Tuple(gid, gid + 1, gid + 2, gid + 3, gid + 4));
        CHECK(t(1) == wmtk::Tuple(gid, gid + 5, gid + 6, gid + 7, gid + 8));
    }
}
