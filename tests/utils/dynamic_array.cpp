#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/DynamicArray.hpp>

using namespace wmtk;
using namespace utils;

TEST_CASE("dynamic_array", "[DynamicArray]")
{
    DynamicArray<int64_t, 5> arr;
    constexpr uint64_t ArraySize = decltype(arr)::array_size();
    CHECK_FALSE(arr.uses_vector());
    CHECK(arr.size() == 0);
    CHECK(arr.capacity() == ArraySize);

    for (uint64_t i = 0; i < ArraySize; ++i) {
        arr.emplace_back(i);
    }

    CHECK_FALSE(arr.uses_vector());
    CHECK(arr.size() == ArraySize);
    CHECK(arr.capacity() == ArraySize);

    arr.emplace_back(ArraySize);
    CHECK(arr.uses_vector());
    CHECK(arr.size() == ArraySize + 1);
    CHECK(arr.capacity() > ArraySize);

    for (uint64_t i = 0; i < arr.size(); ++i) {
        CHECK(arr[i] == i);
        arr[i] = 0;
        CHECK(arr[i] == 0);
        arr[i] = i;
    }

    {
        uint64_t i = 0;
        for (const int v : arr) {
            CHECK(v == i++);
        }
    }
}