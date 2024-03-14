#include <catch2/catch_test_macros.hpp>
#include <numeric>
#include <wmtk/attribute/utils/HybridRationalAttribute.hpp>
#include "../tools/DEBUG_PointMesh.hpp"


using namespace wmtk::tests;

TEST_CASE("test_single_hybrid_rational_attribute", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);

    auto hybrid_handle = wmtk::attribute::utils::HybridRationalAttribute<>::register_attribute(
        m,
        "scalar",
        wmtk::PrimitiveType::Vertex);

    auto rational_handle = m.get_attribute_handle_typed<wmtk::Rational>(
        "[hybrid_rational](rational)scalar",
        wmtk::PrimitiveType::Vertex);
    auto char_handle = m.get_attribute_handle_typed<char>(
        "[hybrid_rational](char)scalar",
        wmtk::PrimitiveType::Vertex);
    auto double_handle = m.get_attribute_handle_typed<double>(
        "[hybrid_rational](double)scalar",
        wmtk::PrimitiveType::Vertex);

    using ATType = std::decay_t<decltype(hybrid_handle)>;
    using ATTupleType = ATType::TupleType;
    static_assert(std::tuple_size<ATTupleType>() == 3);
    static_assert(std::is_same_v<
                  std::tuple_element_t<0, ATTupleType>,
                  wmtk::attribute::TypedAttributeHandle<char>>);
    static_assert(std::is_same_v<
                  std::tuple_element_t<1, ATTupleType>,
                  wmtk::attribute::TypedAttributeHandle<wmtk::Rational>>);
    static_assert(std::is_same_v<
                  std::tuple_element_t<2, ATTupleType>,
                  wmtk::attribute::TypedAttributeHandle<double>>);

    REQUIRE(hybrid_handle.get<0>() == char_handle);
    REQUIRE(hybrid_handle.get<1>() == rational_handle);
    REQUIRE(hybrid_handle.get<2>() == double_handle);
}

#include <wmtk/attribute/utils/HybridRationalAccessor.hpp>
TEST_CASE("test_single_hybrid_rational_accessor", "[accessor]")
{
    int64_t size = 20;
    DEBUG_PointMesh m(size);

    auto attr = wmtk::attribute::utils::HybridRationalAttribute<>::register_attribute(
        static_cast<wmtk::PointMesh&>(m),
        "scalar",
        wmtk::PrimitiveType::Vertex);
    auto vertices = m.get_all(wmtk::PrimitiveType::Vertex);


    wmtk::attribute::utils::HybridRationalAccessor acc(static_cast<wmtk::PointMesh&>(m), attr);

    int value = 0;


    value = 0;
    for (const auto& vtup : vertices) {
        double v = value;
        auto [c, r, d] = acc.value(vtup);
        REQUIRE(c.size() == 1);
        REQUIRE(r.size() == 1);
        REQUIRE(d.size() == 1);
        for (auto& cv : c) {
            CHECK(cv == 0);
        }
        for (auto& cd : d) {
            CHECK(cd == 0);
        }
        for (auto& cr : r) {
            CHECK(cr == 0);
        }
        r.setConstant(double(v));
        acc.round(vtup);
        for (auto& cv : c) {
            CHECK(cv == 1);
        }
        for (auto& cd : d) {
            CHECK(cd == v);
        }
        for (auto& cr : r) {
            CHECK(cr == v);
        }
        value++;
    }
    value = 0;
    for (const auto& vtup : vertices) {
        double v = value;
        auto cc = acc.char_const_value(vtup);
        auto cr = acc.rational_const_value(vtup);
        auto cd = acc.double_const_value(vtup);
        auto c = acc.char_value(vtup);
        auto r = acc.rational_value(vtup);
        auto d = acc.double_value(vtup);
        CHECK(c(0) == 1);
        CHECK(d(0) == v);
        CHECK(r(0) == v);
        CHECK(cc(0) == 1);
        CHECK(cd(0) == v);
        CHECK(cr(0) == v);
        for (auto& cv : c) {
            CHECK(cv == 1);
        }
        for (auto& dv : d) {
            CHECK(dv == v);
        }
        for (auto& rv : r) {
            CHECK(rv == v);
        }
        for (const auto& cv : cc) {
            CHECK(cv == 1);
        }
        for (const auto& dv : cd) {
            CHECK(dv == v);
        }
        for (const auto& rv : cr) {
            CHECK(rv == v);
        }
        value++;
    }
    // TypedAttributeHandle<char> c;
    auto vec_attr = wmtk::attribute::utils::HybridRationalAttribute<>::register_attribute(
        static_cast<wmtk::PointMesh&>(m),
        "vec",
        wmtk::PrimitiveType::Vertex,
        3);
    wmtk::attribute::utils::HybridRationalAccessor vec_acc(
        static_cast<wmtk::PointMesh&>(m),
        vec_attr);
}
