#include <numeric>

#include <catch2/catch_test_macros.hpp>
#include <wmtk/attribute/Attribute.hpp>
#include <wmtk/attribute/internal/CompoundAccessor.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_PointMesh.hpp"


using namespace wmtk::tests;

TEST_CASE("test_single_compound_accessor", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);
    REQUIRE(size == m.capacity(wmtk::PrimitiveType::Vertex));
    auto int64_t_handle =
        m.register_attribute_typed<int64_t>("int64_t", wmtk::PrimitiveType::Vertex, 2, false, -1);
    REQUIRE(m.get_attribute_dimension(int64_t_handle) == 2);
    auto double_handle =
        m.register_attribute_typed<double>("double", wmtk::PrimitiveType::Vertex, 1, false, 0);
    auto int64_t_acc = m.create_accessor(int64_t_handle);
    auto double_acc = m.create_accessor(double_handle);

    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);

    wmtk::attribute::internal::CompoundAccessor<2, wmtk::PointMesh, int64_t, double> caccessor(
        int64_t_acc,
        double_acc);

    REQUIRE(int64_t_acc.reserved_size() == size);
    REQUIRE(double_acc.reserved_size() == size);

    // test default initialization to 0
    for (const wmtk::Tuple& tup : vertices) {
        int64_t gid = m.id(tup);
        auto iv = int64_t_acc.vector_attribute(tup);
        auto dv = double_acc.vector_attribute(tup);

        CHECK(iv(0) == -1);
        CHECK(iv(1) == -1);
        CHECK(dv(0) == 0);

        auto [iv2, dv2] = caccessor.const_value(tup);
        CHECK(iv == iv2);
        CHECK(dv == dv2);

        iv(0) = 2;
        iv(1) = 3;
        dv(0) = 0.1;
        CHECK(iv == iv2);
        CHECK(dv == dv2);
    }
}
