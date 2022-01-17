#include <remeshing/dummy.h>

#include <catch2/catch.hpp>

TEST_CASE("test remeshing")
{
    REQUIRE(remeshing::add(1, 2) == 3);
}
