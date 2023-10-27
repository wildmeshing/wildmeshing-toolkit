#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/Cache.hpp>

namespace fs = std::filesystem;

TEST_CASE("cache_init", "[cache]")
{
    const fs::path dir = std::filesystem::current_path();
    const std::string prefix = "wmtk_cache_";

    fs::path cache_dir;
    {
        Cache cache(prefix, std::filesystem::current_path());
        cache_dir = cache.path();

        CHECK(fs::exists(cache_dir));

        CHECK(dir == cache_dir.parent_path());
        CHECK(cache_dir.stem().string().rfind(prefix, 0) == 0); // cache dir starts with prefix
    }
    CHECK_FALSE(fs::exists(cache_dir));
}