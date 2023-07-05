#include <wmtk/PointMesh.hpp>
#include <numeric>

#include <catch2/catch_test_macros.hpp>

namespace {
    class DEBUG_PointMesh: public wmtk::PointMesh {
        public:

            using PointMesh::PointMesh;
            long id(const wmtk::Tuple& tup) const {
                return PointMesh::id(tup, wmtk::PrimitiveType::Vertex);
            }
    };
}

TEST_CASE("test_attribute")
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
    for(const wmtk::Tuple& tup: vertices) {
        CHECK(char_acc.const_scalar_attribute(tup) == 0);
        CHECK(long_acc.const_scalar_attribute(tup) == 0);
        CHECK((double_acc.const_vector_attribute(tup).array() == 0).all());
    }

    // use global set to force all values

    {
        std::vector<char> d(size);
        std::iota(d.begin(),d.end(),char(0));
        char_acc.set_attribute(d);
    }
    {
        std::vector<long> d(size);
        std::iota(d.begin(),d.end(),long(0));
        long_acc.set_attribute(d);
    }
    {
        std::vector<double> d(3 * size);
        std::iota(d.begin(),d.end(),double(0));
        double_acc.set_attribute(d);
    }
    for(const wmtk::Tuple& tup: vertices) {

        long id = m.id(tup);
        CHECK(char_acc.const_scalar_attribute(tup) == char(id));
        CHECK(long_acc.const_scalar_attribute(tup) == id);
        Eigen::Vector3d x(3*id,3*id+1,3*id+2);
        CHECK((double_acc.const_vector_attribute(tup) == x));
    }
    
}

