#include <numeric>
#include <wmtk/PointMesh.hpp>

#include <catch2/catch_test_macros.hpp>

namespace {
class DEBUG_PointMesh : public wmtk::PointMesh
{
public:
    using PointMesh::PointMesh;
    long id(const wmtk::Tuple& tup) const
    {
        return PointMesh::id(tup, wmtk::PrimitiveType::Vertex);
    }
};


template <typename VectorAcc>
void populate(DEBUG_PointMesh& m, VectorAcc& va)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    size_t stride = va.stride();
    Eigen::Matrix<typename VectorAcc::T, Eigen::Dynamic, 1> x;
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        auto v = va.vector_attribute(tup);
        std::iota(v.begin(), v.end(), stride * id);
    }
}
template <typename VectorAcc>
void check(DEBUG_PointMesh& m, VectorAcc& va)
{
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);
    size_t stride = va.stride();
    Eigen::Matrix<typename VectorAcc::T, Eigen::Dynamic, 1> x;
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        auto v = va.vector_attribute(tup);
        std::iota(x.begin(), x.end(), stride * id);
        CHECK(v == x);
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

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    // check characteristics are all right
    REQUIRE(char_acc.size() == size);
    REQUIRE(long_acc.size() == size);
    REQUIRE(double_acc.size() == size);
    REQUIRE(char_acc.stride() == 1);
    REQUIRE(long_acc.stride() == 1);
    REQUIRE(double_acc.stride() == 3);

    // test default initialization to 0
    for (const wmtk::Tuple& tup : vertices) {
        CHECK(char_acc.const_scalar_attribute(tup) == 0);
        CHECK(long_acc.const_scalar_attribute(tup) == 0);
        CHECK((double_acc.const_vector_attribute(tup).array() == 0).all());
    }

    // use global set to force all values

    {
        std::vector<char> d(size);
        std::iota(d.begin(), d.end(), char(0));
        char_acc.set_attribute(d);
    }
    {
        std::vector<long> d(size);
        std::iota(d.begin(), d.end(), long(0));
        long_acc.set_attribute(d);
    }
    {
        std::vector<double> d(3 * size);
        std::iota(d.begin(), d.end(), double(0));
        double_acc.set_attribute(d);
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

    auto immediate_long_acc = m.create_accessor(long_handle);
    auto immediate_double_acc = m.create_accessor(double_handle);


    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);


    {
        // TODO: create scope
        auto long_acc = m.create_accessor(long_handle);
        auto double_acc = m.create_accessor(double_handle);

        // check characteristics are all right
        REQUIRE(long_acc.size() == size);
        REQUIRE(double_acc.size() == size);
        REQUIRE(long_acc.stride() == 1);
        REQUIRE(double_acc.stride() == 3);

        // use global set to force all values

        for (const wmtk::Tuple& tup : vertices) {
            long id = m.id(tup);
            long_acc.scalar_attribute(tup) = id;
            Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
            double_acc.vector_attribute(tup) = x;
        }
        for (const wmtk::Tuple& tup : vertices) {
            long id = m.id(tup);
            CHECK(long_acc.const_scalar_attribute(tup) == id);
            Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
            CHECK((double_acc.const_vector_attribute(tup) == x));

            CHECK(immediate_long_acc.const_scalar_attribute(tup) == 0);
            CHECK((immediate_double_acc.const_vector_attribute(tup).array() == 0).all());
        }
    }
    // test that the accessors above unbuffered when they finished scope
    for (const wmtk::Tuple& tup : vertices) {
        long id = m.id(tup);
        CHECK(immediate_long_acc.const_scalar_attribute(tup) == id);
        Eigen::Vector3d x(3 * id, 3 * id + 1, 3 * id + 2);
        CHECK((immediate_double_acc.const_vector_attribute(tup) == x));
    }
}

TEST_CASE("test_accessor_caching_scope")
{
    long size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);
}
