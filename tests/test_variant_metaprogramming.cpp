#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <type_traits>
#include <typeinfo>
#include <wmtk/utils/metaprogramming/ReferenceWrappedFunctorReturnCache.hpp>
#include <wmtk/utils/metaprogramming/as_variant.hpp>
#ifndef _MSC_VER
#include <cxxabi.h>
#endif
#include <cstdlib>
#include <memory>
#include <string>

template <class T>
std::string type_name()
{
    typedef typename std::remove_reference<T>::type TR;
    std::unique_ptr<char, void (*)(void*)> own(
#ifndef _MSC_VER
        abi::__cxa_demangle(typeid(TR).name(), nullptr, nullptr, nullptr),
#else
        nullptr,
#endif
        std::free);
    std::string r = own != nullptr ? own.get() : typeid(TR).name();
    if (std::is_const<TR>::value) r += " const";
    if (std::is_volatile<TR>::value) r += " volatile";
    if (std::is_lvalue_reference<T>::value)
        r += "&";
    else if (std::is_rvalue_reference<T>::value)
        r += "&&";
    return r;
}

namespace {
// Declare a base type that has some sort of ID member and a way of identifying
// an appropriate derived type
struct Input
{
    int type = -1;
    int id;
    Input(int type,int id): type(type), id(id) {}
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
struct TestFunctor3ArgsOneRetType
{
    template <typename T>
    auto operator()(T& input, int data) const
    {
        return int(input.id * data);
    };

    void operator()(C&, int) const {}
};

} // namespace

namespace wmtk::utils::metaprogramming {
template <>
size_t TestRefType::get_index(const Input& input)
{
    return input.type;
}
} // namespace wmtk::utils::metaprogramming


TEST_CASE("test_variant_metaprogramming", "[metaprogramming]")
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

TEST_CASE("test_variant_metaprogramming_return_types", "[meta]")
{
    using RefTuple = TestRefType::ReferenceTuple;
    std::cout << type_name<RefTuple>() << std::endl;
    static_assert(std::is_same_v<
                  RefTuple,
                  std::tuple<
                      std::reference_wrapper<A>,
                      std::reference_wrapper<B>,
                      std::reference_wrapper<C>>>);
    using ARef = std::tuple_element_t<0, RefTuple>;
    static_assert(std::is_same_v<ARef, std::reference_wrapper<A>>);

    std::cout << type_name<ARef>() << std::endl;
    using F1ARet = wmtk::utils::metaprogramming::detail::
        ReferenceWrappedFunctorReturnType<TestFunctor, TestRefType::ReferenceTuple>::ReturnType<A>;
    static_assert(std::is_same_v<F1ARet, std::tuple<A, int>>);
    using F1ARefRet = wmtk::utils::metaprogramming::detail::ReferenceWrappedFunctorReturnType<
        TestFunctor,
        TestRefType::ReferenceTuple>::ReturnType<ARef>;
    static_assert(std::is_same_v<F1ARefRet, std::tuple<A, int>>);


    using F1TypeDirty = wmtk::utils::metaprogramming::detail::ReferenceWrappedFunctorReturnType<
        TestFunctor,
        TestRefType::ReferenceTuple>::DirtyReturnTypesTuple;
    static_assert(std::is_same_v<
                  F1TypeDirty,
                  std::tuple<std::tuple<A, int>, std::tuple<B, int>, std::tuple<C, int>>>);
    std::cout << "Dirty:" << type_name<F1TypeDirty>() << std::endl;

    using F1TypeTuple = wmtk::utils::metaprogramming::detail::ReferenceWrappedFunctorReturnType<
        TestFunctor,
        TestRefType::ReferenceTuple>::ReturnTypesTuple;
    static_assert(std::is_same_v<
                  F1TypeTuple,
                  std::tuple<std::tuple<A, int>, std::tuple<B, int>, std::tuple<C, int>>>);
    std::cout << "Clean:" << type_name<F1TypeTuple>() << std::endl;
    using F1VarType =
        wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnType<TestFunctor, TestRefType>;

    std::cout << "f1var: " << type_name<F1VarType>() << std::endl;
    static_assert(std::is_same_v<
                  F1VarType,
                  std::variant<std::tuple<A, int>, std::tuple<B, int>, std::tuple<C, int>>>);

    using F2VarType = wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnType<TestFunctor2Args, TestRefType, int>;
    static_assert(std::is_same_v<
                  F2VarType,
                  std::variant<std::tuple<A, int>, std::tuple<B, int>, std::tuple<C, int>>>);

    // std::cout << type_name<F1Type>() << std::endl;
    std::cout << type_name<F2VarType>() << std::endl;

    using F2VarSingleType = wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnType<TestFunctor3ArgsOneRetType, TestRefType, int>;
    static_assert(std::is_same_v<F2VarSingleType, std::variant<int>>);
}


TEST_CASE("test_variant_metaprogramming_cache", "[metaprogramming]")
{
    A a(0);
    B b(2);
    C c(4);

    wmtk::utils::metaprogramming::ReferenceWrappedFunctorReturnCache<TestFunctor, TestRefType>
        t1cache;

    std::cout << type_name<decltype(t1cache)::TypeHelper>() << std::endl;

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
    wmtk::utils::metaprogramming::
        ReferenceWrappedFunctorReturnCache<TestFunctor3ArgsOneRetType, TestRefType, int>
            t3cache;
    {
        t3cache.add(TestFunctor3ArgsOneRetType{}(a, 3), a, 3);
        t3cache.add(TestFunctor3ArgsOneRetType{}(b, 5), b, 5);
        //t2cache.add(TestFunctor2Args{}(c, 7), c, 7); // purposely do not add because it's void


        {
            auto ra = t3cache.get_variant(a, 3);
            auto rb = t3cache.get_variant(b, 5);
            CHECK(ra.index() == 0);
            CHECK(rb.index() == 0);
        }
        {
            auto check = [&](const auto& v, int p, const auto& r) {
                if constexpr (!std::is_same_v<C, std::decay_t<decltype(v)>>) {
                    CHECK(int(r) == v.id * p);
                }
            };

            auto ra = t3cache.get(a, 3);
            auto rb = t3cache.get(b, 5);

            check(a, 3, ra);
            check(b, 5, rb);
        }
    }

    static_assert(!wmtk::utils::metaprogramming::all_return_void_v<TestFunctor, TestRefType>);
    static_assert(
        !wmtk::utils::metaprogramming::all_return_void_v<TestFunctor2Args, TestRefType, int>);
    static_assert(!wmtk::utils::metaprogramming::
                      all_return_void_v<TestFunctor3ArgsOneRetType, TestRefType, int>);

    // just test that a void can be created?
    auto printer = [](const auto&) { std::cout << "ey!" << std::endl; };
    using PrinterType = decltype(printer);
    static_assert(wmtk::utils::metaprogramming::all_return_void_v<PrinterType, TestRefType>);
}
