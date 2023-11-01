#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/Cache.hpp>

namespace fs = std::filesystem;

TEST_CASE("cache_init", "[cache]")
{
    const fs::path dir = std::filesystem::current_path();
    const std::string prefix = "wmtk_cache";

    fs::path cache_dir;
    {
        Cache cache(prefix, dir);
        cache_dir = cache.path();

        CHECK(fs::exists(cache_dir));

        CHECK(dir == cache_dir.parent_path());
        CHECK(cache_dir.stem().string().rfind(prefix, 0) == 0); // cache dir starts with prefix
    }
    CHECK_FALSE(fs::exists(cache_dir));
}

TEST_CASE("cache_files", "[cache]")
{
    fs::path filepath;
    {
        Cache cache("wmtk_cache", std::filesystem::current_path());
        std::string name = "my_new_file";

        filepath = cache.create_unique_file(name, ".txt");

        CHECK(fs::exists(filepath));
        CHECK(filepath.stem().string().rfind(name, 0) == 0);
        CHECK(filepath.extension().string() == ".txt");
    }
    CHECK_FALSE(fs::exists(filepath));
}

TEST_CASE("cache_export_import", "[cache][.]")
{
    // TODO write test for exporting and importing
    CHECK(false);

    // create cache
    // add some files
    // export cache
    // delete cache

    // create new cache
    // import the previously exported
    // check if files are there

    // destroy the exported cache
}