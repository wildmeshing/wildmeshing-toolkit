#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/Cache.hpp>

TEST_CASE("cache_init", "[cache]")
{
    std::filesystem::path dir = "cache";
    Cache cache(dir);
}