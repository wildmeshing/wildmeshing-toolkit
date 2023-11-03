#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/Cache.hpp>

#include <wmtk/TriMesh.hpp>
#include "tools/TriMesh_examples.hpp"

namespace fs = std::filesystem;

using namespace wmtk;
using namespace utils;

TEST_CASE("cache_init", "[cache]")
{
    const fs::path dir = std::filesystem::current_path();
    const std::string prefix = "wmtk_cache";

    fs::path cache_dir;
    {
        Cache cache(prefix, dir);
        cache_dir = cache.get_cache_path();

        CHECK(fs::exists(cache_dir));

        CHECK(dir == cache_dir.parent_path());
        CHECK(cache_dir.stem().string().rfind(prefix, 0) == 0); // cache dir starts with prefix
    }
    CHECK_FALSE(fs::exists(cache_dir));
}

TEST_CASE("cache_files", "[cache]")
{
    fs::path filepath;
    std::string file_name = "my_new_file";
    {
        Cache cache("wmtk_cache", fs::current_path());

        filepath = cache.create_unique_file(file_name, ".txt");

        CHECK(fs::exists(filepath));
        CHECK(filepath.stem().string().rfind(file_name, 0) == 0);
        CHECK(filepath.extension().string() == ".txt");

        const fs::path filepath_from_cache = cache.get_file_path(file_name);

        CHECK(filepath_from_cache == filepath);
    }
    CHECK_FALSE(fs::exists(filepath));
}

TEST_CASE("cache_read_write_mesh", "[cache]")
{
    Cache cache("wmtk_cache", fs::current_path());
    TriMesh mesh = tests::single_triangle();

    const std::string name = "cached_mesh";
    cache.write_mesh(mesh, name);

    auto mesh_from_cache = cache.read_mesh(name);

    CHECK(*mesh_from_cache == mesh);
    CHECK_THROWS(cache.read_mesh("some_file_that_does_not_exist"));
}

TEST_CASE("cache_export_import", "[cache]")
{
    const fs::path export_location =
        Cache::create_unique_directory("wmtk_cache_export", fs::current_path());

    const std::vector<std::string> file_names = {"a", "b", "c"};

    // create cache
    fs::path first_cache_path;
    {
        Cache cache("wmtk_cache", fs::current_path());
        // generate some files
        for (const std::string& name : file_names) {
            const fs::path p = cache.create_unique_file(name, ".txt");
            CHECK(fs::exists(p));
            CHECK(p.stem().string().rfind(name, 0) == 0);
            CHECK(p.extension().string() == ".txt");
        }

        first_cache_path = cache.get_cache_path();

        // delete dummy directory
        fs::remove_all(export_location);
        REQUIRE_FALSE(fs::exists(export_location));
        // export cache to dummy directory
        REQUIRE(cache.export_cache(export_location));
    }
    CHECK_FALSE(fs::exists(first_cache_path));

    // create new cache
    {
        Cache cache("wmtk_cache", fs::current_path());
        // import the previously exported
        CHECK(cache.import_cache(export_location));

        // check if files are there
        for (const std::string& name : file_names) {
            const fs::path p = cache.get_file_path(name);
            CHECK(fs::exists(p));
            CHECK(p.stem().string().rfind(name, 0) == 0);
            CHECK(p.extension().string() == ".txt");
        }
    }

    // try to import even though the cache contains a file
    {
        Cache cache("wmtk_cache", fs::current_path());
        cache.create_unique_file("some_file", "");
        // import should not work if the cache already contains files
        CHECK_FALSE(cache.import_cache(export_location));
    }

    // clean up export
    fs::remove_all(export_location);
}