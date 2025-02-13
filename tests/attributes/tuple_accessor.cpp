#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/TupleAccessor.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_PointMesh.hpp"


using namespace wmtk::tests;
namespace {} // namespace

TEST_CASE("tuple_to_int64_t_storage", "[accessor]")
{
    std::array basic_data = {
        wmtk::Tuple(0, 0, 0, 0),
        wmtk::Tuple(1, 0, 0, 0),
        wmtk::Tuple(0, 1, 0, 0),
        wmtk::Tuple(0, 0, 1, 0),
        wmtk::Tuple(0, 0, 0, 1),
        wmtk::Tuple(0, 0, 0, 0),
        wmtk::Tuple(-1, -1, -1, -1),
        wmtk::Tuple(0, 0, 0, 0),
        wmtk::Tuple(0, 20, 30, 40),
        wmtk::Tuple(-20, 0, -3, 16)};
    for (const auto& t : basic_data) {
        wmtk::Vector<int64_t, 2> idat = wmtk::multimesh::utils::tuple_to_vector(t);
        const int64_t* iptr = reinterpret_cast<const int64_t*>(&t);
        std::cout << idat.transpose() << " === " << iptr[0] << " " << iptr[1] << std::endl;
        CHECK(idat(0) == iptr[0]);
        CHECK(idat(1) == iptr[1]);
    }
}

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
        t_ref = wmtk::Tuple(gid, gid + 1, gid + 2, gid + 3);
        {
            const wmtk::Tuple t2 = tuple_accessor.const_scalar_attribute(tup);
            CHECK(t2.local_vid() == gid);
            CHECK(t2.local_eid() == gid + 1);
            CHECK(t2.local_fid() == gid + 2);
            CHECK(t2.global_cid() == gid + 3);
        }
    }
    for (const wmtk::Tuple& tup : vertices) {
        int64_t gid = m.id(tup);
        auto v = int64_t_acc.const_vector_attribute(tup);
        const wmtk::Tuple t = tuple_accessor.const_scalar_attribute(tup);
        CHECK(!t.is_null());
        CHECK(t.local_vid() == gid);
        CHECK(t.local_eid() == gid + 1);
        CHECK(t.local_fid() == gid + 2);
        CHECK(t.global_cid() == gid + 3);
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
        t(0) = wmtk::Tuple(gid, gid + 1, gid + 2, gid + 3);
        t(1) = wmtk::Tuple(gid, gid + 4, gid + 5, gid + 6);
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
        CHECK(t(0) == wmtk::Tuple(gid, gid + 1, gid + 2, gid + 3));
        CHECK(t(1) == wmtk::Tuple(gid, gid + 4, gid + 5, gid + 6));
    }
}
TEST_CASE("test_multi_tuple_accessor_gid", "[accessor]")
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
        auto v = int64_t_acc.vector_attribute(tup);
        CHECK((v.array() == -1).all());
        auto t = tuple_accessor.vector_attribute(tup);
        REQUIRE(t.size() == 2);
        REQUIRE(t.rows() == 2);
        REQUIRE(t.cols() == 1);
        // CHECK(t == t_ref);
        CHECK(t(0).is_null());
        CHECK(t(1).is_null());
        int64_t gid = m.id(tup);


        for (int j = 0; j < tuple_accessor.dimension(); ++j) {
            int64_t value = gid + 2 * j;
            v(2 * j) = value;
            CHECK(t(j).global_cid() == value);
        }
    }
}
