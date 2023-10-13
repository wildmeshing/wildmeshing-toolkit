#include <spdlog/spdlog.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>
#include <wmtk/utils/metaprogramming/as_variant.hpp>

namespace {
// Declare a base type that has some sort of ID member and a way of identifying
// an appropriate derived type
struct Input
{
    int type = -1;
    int id;
    Input& operator=(const Input& o) = default;
    Input(const Input& o) = default;

    bool operator==(const Input& o) const { return type == o.type && id == o.id; }
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

struct TestFunctor
{
    template <typename T>
    auto operator()(T& input) const
    {
        using TT = wmtk::utils::metaprogramming::unwrap_ref_decay_t<T>;
        return std::tuple<TT, int>(input, input.id);
    };
};

struct TestFunctor2Args
{
    template <typename T>
    auto operator()(T& input, int data) const
    {
        using TT = wmtk::utils::metaprogramming::unwrap_ref_decay_t<T>;
        return std::tuple<TT, int>(input, input.id * data);
    };
};

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

    auto a_cref = wmtk::utils::metaprogramming::as_const_variant<TestRefType>(a);
    auto b_cref = wmtk::utils::metaprogramming::as_const_variant<TestRefType>(b);
    auto c_cref = wmtk::utils::metaprogramming::as_const_variant<TestRefType>(c);

    {
        Input i{3, 3};
        CHECK_THROWS(wmtk::utils::metaprogramming::as_variant<TestRefType>(i));
    }

    // double check that we get the expected indices in the variant
    CHECK(a_ref.index() == 0);
    CHECK(b_ref.index() == 1);
    CHECK(c_ref.index() == 2);
    CHECK(a_cref.index() == 0);
    CHECK(b_cref.index() == 1);
    CHECK(c_cref.index() == 2);

    // try checking the runtime usage of these variants
    std::visit(
        [&](auto&& test, auto&& test2) {
            using T = std::decay_t<decltype(test)>;
            using cT = std::decay_t<decltype(test2)>;
            CHECK(std::is_same_v<T, std::reference_wrapper<A>>);
            CHECK(std::is_same_v<cT, std::reference_wrapper<const A>>);
        },
        a_ref,
        a_cref);
    std::visit(
        [&](auto&& test, auto&& test2) {
            using T = std::decay_t<decltype(test)>;
            using cT = std::decay_t<decltype(test2)>;
            CHECK(std::is_same_v<T, std::reference_wrapper<B>>);
            CHECK(std::is_same_v<cT, std::reference_wrapper<const B>>);
        },
        b_ref,
        b_cref);
    std::visit(
        [&](auto&& test, auto&& test2) {
            using T = std::decay_t<decltype(test)>;
            using cT = std::decay_t<decltype(test2)>;
            CHECK(std::is_same_v<T, std::reference_wrapper<C>>);
            CHECK(std::is_same_v<cT, std::reference_wrapper<const C>>);
        },
        c_ref,
        c_cref);
}

TEST_CASE("test_variant_multiprogramming_cache", "[metaprogramming]")
{
    A a(0);
    B b(2);
    C c(4);

    wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCache<TestFunctor, TestRefType>
        t1cache;


    {
        t1cache.add(TestFunctor{}(a), a);
        t1cache.add(TestFunctor{}(b), b);
        t1cache.add(TestFunctor{}(c), c);


        {
            auto ra = t1cache.get_variant(a);
            auto rb = t1cache.get_variant(b);
            auto rc = t1cache.get_variant(c);
            CHECK(ra.index() == 0);
            CHECK(rb.index() == 1);
            CHECK(rc.index() == 2);
        }
        {
            auto check = [&](const auto& v, const auto& r) {
                using U = std::decay_t<decltype(v)>;
                using V = std::decay_t<decltype(std::get<0>(r))>;
                static_assert(std::is_same_v<U, V>);
                CHECK(std::get<0>(r) == v);
                CHECK(std::get<1>(r) == v.id);
            };

            auto ra = t1cache.get(a);
            auto rb = t1cache.get(b);
            auto rc = t1cache.get(c);

            check(a, ra);
            check(b, rb);
            check(c, rc);
        }
    }

    wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnCache<TestFunctor2Args, TestRefType, int>
            t2cache;
    {
        t2cache.add(TestFunctor2Args{}(a, 3), a, 3);
        t2cache.add(TestFunctor2Args{}(b, 5), b, 5);
        t2cache.add(TestFunctor2Args{}(c, 7), c, 7);


        {
            auto ra = t2cache.get_variant(a, 3);
            auto rb = t2cache.get_variant(b, 5);
            auto rc = t2cache.get_variant(c, 7);
            CHECK(ra.index() == 0);
            CHECK(rb.index() == 1);
            CHECK(rc.index() == 2);
        }
        {
            auto check = [&](const auto& v, int p, const auto& r) {
                CHECK(std::get<0>(r) == v);
                CHECK(std::get<1>(r) == v.id * p);
            };

            auto ra = t2cache.get(a, 3);
            auto rb = t2cache.get(b, 5);
            auto rc = t2cache.get(c, 7);

            check(a, 3, ra);
            check(b, 5, rb);
            check(c, 7, rc);
        }
    }
}
