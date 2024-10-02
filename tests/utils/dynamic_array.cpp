#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/DynamicArray.hpp>

using namespace wmtk;
using namespace utils;

TEST_CASE("dynamic_array", "[DynamicArray]")
{
    DynamicArray arr;
    CHECK_FALSE(arr.uses_vector());
    CHECK(arr.size() == 0);
    CHECK(arr.capacity() == DynamicArraySize);

    for (size_t i = 0; i < DynamicArraySize; ++i) {
        arr.emplace_back(i);
    }

    CHECK_FALSE(arr.uses_vector());
    CHECK(arr.size() == DynamicArraySize);
    CHECK(arr.capacity() == DynamicArraySize);

    arr.emplace_back(DynamicArraySize + 1);
    CHECK(arr.uses_vector());
    CHECK(arr.size() == DynamicArraySize + 1);
    CHECK(arr.capacity() > DynamicArraySize);
}