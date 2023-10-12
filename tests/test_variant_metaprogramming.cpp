#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/metaprogramming/as_variant.hpp>

namespace {
// Declare a base type that has some sort of ID member and a way of identifying
// an appropriate derived type
struct Input
{
    int type = -1;
    int id;
};

// Some example derived types
struct A : public Input
{
    A(int id_)
        : Input{0, id_}
    {}
};

struct B : public Input
{
    B(int id_)
        : Input{1, id_}
    {}
};

struct C : public Input
{
    C(int id_)
        : Input{2, id_}
    {}
};


using TestRefType =
    wmtk::utils::metaprogramming::DerivedReferenceWrapperVariantTraits<Input, A, B, C>;

} // namespace

namespace wmtk::utils::metaprogramming {
template <>
size_t TestRefType::get_index(const Input& input)
{
    return input.type;
}
} // namespace wmtk::utils::metaprogramming


TEST_CASE("test_variant_multiprogramming", "[metaprogramming]")
{
    static_assert(std::is_same_v<TestRefType::BaseType, Input>);
    static_assert(std::is_same_v<TestRefType::DerivedTypesTuple, std::tuple<A, B, C>>);
    static_assert(std::is_same_v<
                  TestRefType::ReferenceTuple,
                  std::tuple<
                      std::reference_wrapper<A>,
                      std::reference_wrapper<B>,
                      std::reference_wrapper<C>>>);

    A a(0);
    B b(2);
    C c(4);
    CHECK(TestRefType::get_index(a) == 0);
    CHECK(TestRefType::get_index(b) == 1);
    CHECK(TestRefType::get_index(c) == 2);

    auto a_ref = wmtk::utils::metaprogramming::as_variant<TestRefType>(a);
    auto b_ref = wmtk::utils::metaprogramming::as_variant<TestRefType>(b);
    auto c_ref = wmtk::utils::metaprogramming::as_variant<TestRefType>(c);

    // double check that we get the expected indices in the variant
    CHECK(a_ref.index() == 0);
    CHECK(b_ref.index() == 1);
    CHECK(c_ref.index() == 2);

    // try checking the runtime usage of these variants
    std::visit(
        [&]<typename T>(const T&) {
            //
            constexpr bool same = std::is_same_v<T, std::reference_wrapper<A>>;
            CHECK(same);
            //
        },
        a_ref);
    std::visit(
        [&]<typename T>(const T&) {
            //
            constexpr bool same = std::is_same_v<T, std::reference_wrapper<B>>;
            CHECK(same);
            //
        },
        b_ref);
    std::visit(
        [&]<typename T>(const T&) {
            //
            constexpr bool same = std::is_same_v<T, std::reference_wrapper<C>>;
            CHECK(same);
            //
        },
        c_ref);
}
