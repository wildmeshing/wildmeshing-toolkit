#include <spdlog/spdlog.h>
#include <numeric>
#include "tools/DEBUG_PointMesh.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace wmtk::tests;
namespace {


template <typename VectorAcc>
void populate(DEBUG_PointMesh& m, VectorAcc& va, bool for_zeros = false)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    size_t dimension = va.dimension();
    Eigen::Matrix<typename VectorAcc::Scalar, Eigen::Dynamic, 1> x;
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        auto v = va.vector_attribute(tup);
        if (for_zeros) {
            v.setZero();
        } else {
            std::iota(v.begin(), v.end(), dimension * id);
        }
    }
}
template <typename VectorAcc>
void check(DEBUG_PointMesh& m, VectorAcc& va, bool for_zeros = false)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    size_t dimension = va.dimension();
    Eigen::Matrix<typename VectorAcc::Scalar, Eigen::Dynamic, 1> x;
    bool is_scalar = va.dimension() == 1;
    x.resize(va.dimension());
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        if (for_zeros) {
            CHECK((va.const_vector_attribute(tup).array() == 0).all());
            if (is_scalar) {
                CHECK(va.const_scalar_attribute(tup) == 0);
            }
        } else {
            auto v = va.vector_attribute(tup);
            std::iota(x.begin(), x.end(), dimension * id);
            CHECK(v == x);
            if (is_scalar) {
                CHECK(va.const_scalar_attribute(tup) == id);
            }
        }
    }
}
} // namespace

TEST_CASE("test_accessor_basic")
{
    long size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto char_handle = m.register_attribute<char>("char", wmtk::PrimitiveType::Vertex, 1);
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);


    auto char_acc = m.create_accessor(char_handle);
    auto long_acc = m.create_accessor(long_handle);
    auto double_acc = m.create_accessor(double_handle);

    auto char_bacc = m.create_base_accessor(char_handle);
    auto long_bacc = m.create_base_accessor(long_handle);
    auto double_bacc = m.create_base_accessor(double_handle);

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    // check characteristics are all right
    REQUIRE(char_acc.reserved_size() == size);
    REQUIRE(long_acc.reserved_size() == size);
    REQUIRE(double_acc.reserved_size() == size);
    REQUIRE(char_acc.dimension() == 1);
    REQUIRE(long_acc.dimension() == 1);
    REQUIRE(double_acc.dimension() == 3);

    // test default initialization to 0
    for (const wmtk::Tuple& tup : vertices) {
        CHECK(char_acc.const_scalar_attribute(tup) == 0);
        CHECK(long_acc.const_scalar_attribute(tup) == 0);
        CHECK((double_acc.const_vector_attribute(tup).array() == 0).all());
    }

    // use global set to force all values
    // NOTE that the ugly static casts are in this unit test because we want
    // accessing the low level accessor data to be ugly.
    // Please keep set_attribute hidden from teh public unless some ugly
    // notation like these static asts exists
    {
        std::vector<char> d(size);
        std::iota(d.begin(), d.end(), char(0));
        char_bacc.set_attribute(d);
    }
    {
        std::vector<long> d(size);
        std::iota(d.begin(), d.end(), long(0));
        long_bacc.set_attribute(d);
    }
    {
        std::vector<double> d(3 * size);
        std::iota(d.begin(), d.end(), double(0));
        double_bacc.set_attribute(d);
    }
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        CHECK(char_acc.const_scalar_attribute(tup) == char(id));
        CHECK(long_acc.const_scalar_attribute(tup) == id);
        Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
        CHECK((double_acc.const_vector_attribute(tup) == x));
    }

    { // check const accessors
        auto char_cacc = m.create_const_accessor(char_handle);
        auto long_cacc = m.create_const_accessor(long_handle);
        auto double_cacc = m.create_const_accessor(double_handle);
        for (const wmtk::Tuple& tup : vertices) {
            long id = m.id(tup);
            CHECK(char_cacc.const_scalar_attribute(tup) == char(id));
            CHECK(long_cacc.const_scalar_attribute(tup) == id);
            Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
            CHECK((double_cacc.const_vector_attribute(tup) == x));
        }
    }
}

TEST_CASE("test_accessor_caching")
{
    long size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);

    auto immediate_long_acc = m.create_base_accessor(long_handle);
    auto immediate_double_acc = m.create_base_accessor(double_handle);

    std::vector<long*> long_ptrs;
    std::vector<double*> double_ptrs;
    for (long j = 0; j < m.capacity(wmtk::PrimitiveType::Vertex); ++j) {
        long_ptrs.emplace_back(&immediate_long_acc.vector_attribute(j)(0));
        double_ptrs.emplace_back(&immediate_double_acc.vector_attribute(j)(0));
    }

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    REQUIRE(long(vertices.size()) == size);

    {
        spdlog::info("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();
        {
            // make sure base accessors are not affected
            for (long j = 0; j < m.capacity(wmtk::PrimitiveType::Vertex); ++j) {
                CHECK(&immediate_long_acc.vector_attribute(j)(0) == long_ptrs[j]);
                CHECK(&immediate_double_acc.vector_attribute(j)(0) == double_ptrs[j]);
            }
        }
        auto long_acc = m.create_accessor(long_handle);
        auto double_acc = m.create_accessor(double_handle);
        {
            auto depth_opt = long_acc.stack_depth();
            REQUIRE(depth_opt.has_value());
            REQUIRE(depth_opt.value() == 1);

            // make sure base accessors are not affected
            for (const wmtk::Tuple& tup : vertices) {
                long id = m.id(tup);
                REQUIRE(long_acc.vector_attribute(tup).size() == 1);
                REQUIRE(double_acc.vector_attribute(tup).size() == 3);
                CHECK(&long_acc.vector_attribute(tup)(0) != long_ptrs[id]);
                CHECK(&double_acc.vector_attribute(tup)(0) != double_ptrs[id]);
            }
        }

        // check characteristics are all right
        REQUIRE(long_acc.reserved_size() == size);
        REQUIRE(double_acc.reserved_size() == size);
        REQUIRE(long_acc.dimension() == 1);
        REQUIRE(double_acc.dimension() == 3);

        // use global set to force all values

        populate(m, long_acc, false);
        populate(m, double_acc, false);
        check(m, long_acc, false);
        check(m, double_acc, false);

        for (const wmtk::Tuple& tup : vertices) {
            long id = m.id(tup);
            CHECK(immediate_long_acc.const_scalar_attribute(id) == 0);
            CHECK((immediate_double_acc.const_vector_attribute(id).array() == 0).all());
        }
    }
    // test that the accessors above unbuffered when they finished scope
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        CHECK(immediate_long_acc.const_scalar_attribute(id) == id);
        Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
        CHECK((immediate_double_acc.const_vector_attribute(id) == x));
    }
}

TEST_CASE("test_accessor_caching_scope_fails")
{
    long size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto long_acc = m.create_accessor(long_handle);
    auto double_acc = m.create_accessor(double_handle);
    {
        spdlog::info("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();

        populate(m, long_acc, false);
        populate(m, double_acc, false);
        check(m, long_acc, false);
        check(m, double_acc, false);

        scope.mark_failed();
    }
    check(m, long_acc, true);
    check(m, double_acc, true);
}
TEST_CASE("test_accessor_caching_scope_success_fails")
{
    long size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto long_acc = m.create_accessor(long_handle);
    auto double_acc = m.create_accessor(double_handle);
    {
        spdlog::info("Creating a scope");
        // TODO: create scope

        auto scope = m.create_scope();
        populate(m, long_acc, false);
        populate(m, double_acc, false);
        check(m, long_acc, false);
        check(m, double_acc, false);
        {
            auto scope2 = m.create_scope();
            populate(m, long_acc, true);
            populate(m, double_acc, true);
            check(m, long_acc, true);
            check(m, double_acc, true);

            scope2.mark_failed();
        }
    }
    check(m, long_acc, false);
    check(m, double_acc, false);
}
TEST_CASE("test_accessor_caching_scope_fails_success")
{
    long size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);
    auto long_acc = m.create_accessor(long_handle);
    auto double_acc = m.create_accessor(double_handle);
    populate(m, long_acc, true);
    populate(m, double_acc, true);
    check(m, long_acc, true);
    check(m, double_acc, true);
    {
        spdlog::info("Creating a scope");
        // TODO: create scope
        auto scope = m.create_scope();

        populate(m, long_acc, false);
        populate(m, double_acc, false);
        check(m, long_acc, false);
        check(m, double_acc, false);
        {
            auto scope2 = m.create_scope();
            populate(m, long_acc, true);
            populate(m, double_acc, true);
            check(m, long_acc, true);
            check(m, double_acc, true);
        }
        scope.mark_failed();
    }
    check(m, long_acc, true);
    check(m, double_acc, true);
}
