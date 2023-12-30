#include <catch2/catch_test_macros.hpp>
#include <wmtk/utils/metaprogramming/tuple/as_variant_type.hpp>
#include <wmtk/utils/metaprogramming/tuple/concatenate_types.hpp>
#include <wmtk/utils/metaprogramming/tuple/get_unique_remove_void_types.hpp>
#include <wmtk/utils/metaprogramming/tuple/get_unique_types.hpp>
#include <wmtk/utils/metaprogramming/tuple/remove_void_types.hpp>

TEST_CASE("test_concatenate_metaprogramming", "[tuple]")
{
    {
        using a = std::tuple<>;
        using b = std::tuple<>;
        using c = wmtk::utils::metaprogramming::tuple::concatenate_types_t<a, b>;
        static_assert(std::is_same_v<c, std::tuple<>>);
    }
    {
        using a = std::tuple<>;
        using b = std::tuple<int, int64_t, float>;
        using c = wmtk::utils::metaprogramming::tuple::concatenate_types_t<a, b>;
        static_assert(std::is_same_v<c, std::tuple<int, int64_t, float>>);
    }
    {
        using a = std::tuple<int, int64_t, float>;
        using b = std::tuple<>;
        using c = wmtk::utils::metaprogramming::tuple::concatenate_types_t<a, b>;
        static_assert(std::is_same_v<c, std::tuple<int, int64_t, float>>);
    }
    {
        using a = std::tuple<int, int64_t, float>;
        using b = std::tuple<uint32_t, int>;
        using c = wmtk::utils::metaprogramming::tuple::concatenate_types_t<a, b>;
        static_assert(std::is_same_v<c, std::tuple<int, int64_t, float, uint32_t, int>>);
    }
}
TEST_CASE("test_remove_void_metaprogramming", "[tuple]")
{
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<>;
        static_assert(std::is_same_v<type, std::tuple<>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<int, int64_t, float>;
        static_assert(std::is_same_v<type, std::tuple<int, int64_t, float>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<void>;
        static_assert(std::is_same_v<type, std::tuple<>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<void, int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<int, void>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::remove_void_types_t<int, void, int64_t>;
        static_assert(std::is_same_v<type, std::tuple<int, int64_t>>);
    }
    {
        using type =
            wmtk::utils::metaprogramming::tuple::remove_void_types_t<void, int, void, int64_t>;
        static_assert(std::is_same_v<type, std::tuple<int, int64_t>>);
    }
}
TEST_CASE("test_get_unique_metaprogramming", "[tuple]")
{
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_types_t<>;
        static_assert(std::is_same_v<type, std::tuple<>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_types_t<int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_types_t<int, int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_types_t<int, int, int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_types_t<int, int64_t, int>;
        static_assert(std::is_same_v<type, std::tuple<int64_t, int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_types_t<int, int64_t, int64_t>;
        static_assert(std::is_same_v<type, std::tuple<int, int64_t>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::
            get_unique_types_t<int64_t, float, int, int64_t, int64_t, int64_t, int>;
        static_assert(std::is_same_v<type, std::tuple<float, int64_t, int>>);
    }
}
TEST_CASE("test_get_unique_remove_void_types_metaprogramming", "[tuple]")
{
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<>;
        static_assert(std::is_same_v<type, std::tuple<>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<void>;
        static_assert(std::is_same_v<type, std::tuple<>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type =
            wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<int, int, void>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type =
            wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<int, void, int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type =
            wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<void, int, int>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::
            get_unique_remove_void_types_t<void, int, void, int, void>;
        static_assert(std::is_same_v<type, std::tuple<int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::
            get_unique_remove_void_types_t<int, int64_t, void, int>;
        static_assert(std::is_same_v<type, std::tuple<int64_t, int>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::
            get_unique_remove_void_types_t<int, void, int64_t, int64_t>;
        static_assert(std::is_same_v<type, std::tuple<int, int64_t>>);
    }
    {
        using type = wmtk::utils::metaprogramming::tuple::get_unique_remove_void_types_t<
            void,
            int64_t,
            void,
            float,
            int,
            int64_t,
            void,
            int64_t,
            int64_t,
            int,
            void,
            void,
            void>;
        static_assert(std::is_same_v<type, std::tuple<float, int64_t, int>>);
    }
}
