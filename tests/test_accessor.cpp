#include <wmtk/PointMesh.hpp>

#include <catch2/catch_test_macros.hpp>


TEST_CASE("test_attribute")
{
    long size = 20;
    wmtk::PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto long_handle = m.register_attribute<long>("long", wmtk::PrimitiveType::Vertex, 1);
    auto double_handle = m.register_attribute<double>("double", wmtk::PrimitiveType::Vertex, 3);


    auto long_acc = m.create_accessor(long_handle);
    auto double_acc = m.create_accessor(double_handle);

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    REQUIRE(long_acc.size() == size);
    REQUIRE(double_acc.size() == size);

    
}

