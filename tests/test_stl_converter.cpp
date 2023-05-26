//#include <wmtk/serialization/Eigen.h>
//#include <fmt/format.h>
//#include <fmt/printf.h>
//#include <fmt/ranges.h>

#include <Eigen/Dense>
#include <array>
#include <catch2/catch.hpp>
#include <iostream>
#include <tuple>
#include <vector>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/serialization/StlHdf5Serialization.hpp>

//#include <wmtk/serialization/StlHdf5Serialization.h>

// using namespace wmtk::serialization;


namespace {

template <typename T>
struct is_supported : public std::false_type
{
};

template <>
struct is_supported<int> : public std::true_type
{
};
template <>
struct is_supported<float> : public std::true_type
{
};
template <>
struct is_supported<char> : public std::true_type
{
};
template <>
struct is_supported<double> : public std::true_type
{
};

template <typename T>
constexpr static bool is_supported_v(is_supported<std::decay_t<T>>{});
} // namespace

TEST_CASE("primitive2stlconversion", "[stl_conversion]")
{
    //test base case types / primitives
    static_assert(is_supported_v<float>);
    static_assert(is_supported_v<double>);
    static_assert(is_supported_v<int>);
    static_assert(is_supported_v<char>);


    // double check that we can do idempotent tests
    auto trivial_convert_check = []<typename T>(T value) {
        using Converter = wmtk::serialization::detail::DefaultSingleConverter<T>;
        static_assert(std::is_same_v<typename Converter::UserType, T>);
        static_assert(std::is_same_v<typename Converter::StlType, T>);
        T s = Converter::to_stl(value);
        T u = Converter::from_stl(value);

        CHECK(value == u);
        CHECK(value == s);
    };

    trivial_convert_check(int(9));
    trivial_convert_check(char(2));
    trivial_convert_check(float(5.f));
    trivial_convert_check(double(9.0));

    // double check that we can internally use double as the serialization representation
    auto internal_double_check = []<typename T>(T value) {
        using Converter = wmtk::serialization::detail::DefaultSingleConverter<T, double>;
        static_assert(std::is_same_v<typename Converter::UserType, T>);
        static_assert(std::is_same_v<typename Converter::StlType, double>);
        double s = Converter::to_stl(value);
        T u = Converter::from_stl(value);

        CHECK(value == u);
        CHECK(value == s);
    };

    internal_double_check(int(9));
    internal_double_check(char(2));
    internal_double_check(float(5.f));
    internal_double_check(double(9.0));
}

namespace {


// template <typename T, typename Allocator>
//     struct is_supported<std::vector<T,Allocator>>: public std::true_type {};

template <typename T, size_t S>
struct is_supported<std::array<T, S>> : public std::true_type
{
};

template <typename... T>
struct is_supported<std::tuple<T...>>
    : public std::bool_constant<(true && ... && is_supported_v<T>)>
{
};

template <typename A, typename B>
struct is_supported<std::pair<A, B>>
    : public std::bool_constant<(is_supported_v<A> && is_supported_v<B>)>
{
};

template <typename T, int R, int C>
struct is_supported<Eigen::Matrix<T, R, C>>
    : public std::bool_constant<is_supported_v<T> && R != Eigen::Dynamic && C != Eigen::Dynamic>
{
};

} // namespace

TEST_CASE("stl2stlconversion", "[stl_conversion]")
{
    std::array<double, 3> test_arr{{10.0, 11.0, 12.0}};
    static_assert(is_supported_v<decltype(test_arr)>);

    std::tuple<int, float> test_tuple{20, 21.0f};
    static_assert(is_supported_v<decltype(test_tuple)>);

    std::pair<int, float> test_pair{20, 21.0f};
    static_assert(is_supported_v<decltype(test_pair)>);

    std::pair<std::tuple<int, double>, std::array<float, 3>> test_combo{
        {20, 2.f},
        {{21.0f, 12.1f, 0.1f}}};
    static_assert(is_supported_v<decltype(test_combo)>);

    // don't support std::vector because its dynamic size
    std::vector<int> test_vec{0, 1, 2, 3, 4};
    static_assert(!is_supported_v<decltype(test_vec)>);


    // call serialization functions
    // double check that we can do idempotent tests
    auto trivial_convert_check = []<typename T>(T value) {
        using Converter = wmtk::serialization::detail::DefaultSingleConverter<T>;
        static_assert(std::is_same_v<typename Converter::UserType, T>);
        static_assert(std::is_same_v<typename Converter::StlType, T>);
        T s = Converter::to_stl(value);
        T u = Converter::from_stl(value);

        CHECK(value == u);
        CHECK(value == s);
    };

    trivial_convert_check(test_arr);
    trivial_convert_check(test_tuple);
    trivial_convert_check(test_pair);
    trivial_convert_check(test_combo);
}


namespace {


// template <typename T, typename Allocator>
//     struct is_supported<std::vector<T,Allocator>>: public std::true_type {};


} // namespace
TEST_CASE("eigen2stlconversion", "[stl_conversion]")
{
    // eigen static should work ,eigen dynamic shouldn't work
    Eigen::Vector3d test_vec3d = {2., 3., 4.};
    std::array<double, 3> test_vec3d_stl = {{2., 3., 4.}};
    static_assert(is_supported_v<decltype(test_vec3d)>);
    Eigen::VectorXd test_vecXd;
    static_assert(!is_supported_v<decltype(test_vecXd)>);
    // UserStlConverter

    // make sure recursive tuple check works
    std::tuple<int, Eigen::Vector3d, float> test_tuple_with_eig{20, {2.0, 3.0, 4.0}, 21.0f};
    std::tuple<int, std::array<double, 3>, float> test_tuple_with_eig_stl{
        20,
        {{2.0, 3.0, 4.0}},
        21.0f};
    static_assert(is_supported_v<decltype(test_tuple_with_eig)>);

    std::pair<int, Eigen::Vector3d> test_pair_with_eig_a{20, {2.0, 3.0, 4.0}};
    std::pair<int, std::array<double, 3>> test_pair_with_eig_a_stl{20, {{2.0, 3.0, 4.0}}};
    static_assert(is_supported_v<decltype(test_pair_with_eig_a)>);

    std::pair<Eigen::Vector2i, float> test_pair_with_eig_b{{2, 3}, 21.0f};
    std::pair<std::array<int, 2>, float> test_pair_with_eig_b_stl{{{2, 3}}, 21.0f};
    static_assert(is_supported_v<decltype(test_pair_with_eig_b)>);


    // call serialization functions
    // double check that we can do idempotent tests
    auto trivial_convert_check = []<typename T, typename S>(T user, S stl) {
        using Converter = wmtk::serialization::detail::DefaultSingleConverter<T>;
        static_assert(std::is_same_v<typename Converter::UserType, T>);
        static_assert(std::is_same_v<typename Converter::StlType, S>);
        S s = Converter::to_stl(user);
        T u = Converter::from_stl(stl);

        CHECK(user == u);
        CHECK(stl == s);
    };

    trivial_convert_check(test_vec3d, test_vec3d_stl);
    trivial_convert_check(test_tuple_with_eig, test_tuple_with_eig_stl);
    trivial_convert_check(test_pair_with_eig_a, test_pair_with_eig_a_stl);
    trivial_convert_check(test_pair_with_eig_b, test_pair_with_eig_b_stl);
}
