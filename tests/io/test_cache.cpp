#include <optional>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/Cache.hpp>

#include <wmtk/operations/EdgeSplit.hpp>

#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TriMesh_examples.hpp>

#include <catch2/catch_test_macros.hpp>
using namespace wmtk;
using namespace wmtk::io;
using namespace wmtk::tests;

namespace fs = std::filesystem;

TEST_CASE("cache_init", "[cache][io]")
{
    const fs::path dir = std::filesystem::current_path();
    const std::string prefix = "wmtk_cache";

    fs::path cache_dir;
    {
        io::Cache cache(prefix, dir);
        cache_dir = cache.get_cache_path();

        CHECK(fs::exists(cache_dir));

        CHECK(dir == cache_dir.parent_path());
        CHECK(cache_dir.stem().string().rfind(prefix, 0) == 0); // cache dir starts with prefix
    }
    CHECK_FALSE(fs::exists(cache_dir));
}

TEST_CASE("cache_move_assignment", "[cache][io]")
{
    const fs::path dir = std::filesystem::current_path();
    const std::string prefix = "wmtk_cache";

    std::optional<Cache> cache_opt;
    fs::path cache_dir;
    {
        io::Cache cache(prefix, dir);
        cache_dir = cache.get_cache_path();
        CHECK(fs::exists(cache_dir));
        cache_opt = std::move(cache);
        CHECK(fs::exists(cache_dir));
    }
    cache_opt.reset();
    CHECK_FALSE(fs::exists(cache_dir));
}
TEST_CASE("cache_move_constructor", "[cache][io]")
{
    const fs::path dir = std::filesystem::current_path();
    const std::string prefix = "wmtk_cache";

    std::optional<Cache> cache_opt;
    fs::path cache_dir;
    {
        io::Cache cache(prefix, dir);
        cache_dir = cache.get_cache_path();
        CHECK(fs::exists(cache_dir));
        cache_opt.emplace(std::move(cache));
        CHECK(fs::exists(cache_dir));
    }
    cache_opt.reset();
    CHECK_FALSE(fs::exists(cache_dir));
}

TEST_CASE("cache_files", "[cache][io]")
{
    fs::path filepath;
    std::string file_name = "my_new_file";
    {
        io::Cache cache("wmtk_cache", fs::current_path());

        filepath = cache.create_unique_file(file_name, ".txt");

        CHECK(fs::exists(filepath));
        CHECK(filepath.stem().string().rfind(file_name, 0) == 0);
        CHECK(filepath.extension().string() == ".txt");

        const fs::path filepath_from_cache = cache.get_file_path(file_name);

        CHECK(filepath_from_cache == filepath);
    }
    CHECK_FALSE(fs::exists(filepath));
}

TEST_CASE("cache_export_import", "[cache][io]")
{
    const fs::path export_location =
        io::Cache::create_unique_directory("wmtk_cache_export", fs::current_path());

    const std::vector<std::string> file_names = {"a", "b", "c"};

    // create cache
    fs::path first_cache_path;
    {
        io::Cache cache("wmtk_cache", fs::current_path());
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
        io::Cache cache("wmtk_cache", fs::current_path());
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
        io::Cache cache("wmtk_cache", fs::current_path());
        cache.create_unique_file("some_file", "");
        // import should not work if the cache already contains files
        CHECK_FALSE(cache.import_cache(export_location));
    }

    // clean up export
    fs::remove_all(export_location);
}

TEST_CASE("cache_equals", "[cache][io]")
{
    throw std::runtime_error("Write test without using cache read and write functionality");

    // Cache c1("wmtk_cache", ".");
    // Cache c2("wmtk_cache", ".");
    //
    // CHECK(c1.equals(c2));
    //
    //// add some contents
    //{
    //    std::shared_ptr<TriMesh> mptr =
    //        std::make_shared<TriMesh>(tests::single_equilateral_triangle());
    //    TriMesh& m = *mptr;
    //    auto a1 = m.register_attribute<int64_t>("a1", PrimitiveType::Triangle, 1);
    //    auto a2 = m.register_attribute<double>("a2", PrimitiveType::Triangle, 1);
    //    auto acc1 = m.create_accessor<int64_t>(a1);
    //    auto acc2 = m.create_accessor<double>(a2);
    //    const auto faces = m.get_all(PrimitiveType::Triangle);
    //    for (size_t i = 0; i < faces.size(); ++i) {
    //        acc1.scalar_attribute(faces[i]) = i;
    //        acc2.scalar_attribute(faces[i]) = i * i;
    //    }
    //
    //    c1.write_mesh(m, "equilateral_triangle");
    //    c2.write_mesh(m, "equilateral_triangle");
    //}
    //
    // CHECK(c1.equals(c2));
    //
    // SECTION("more_meshes")
    //{
    //    auto m = c2.read_mesh("equilateral_triangle");
    //    c2.write_mesh(*m, "another_mesh");
    //    CHECK_FALSE(c1.equals(c2));
    //}
    // SECTION("read_and_write_back")
    //{
    //    auto m = c2.read_mesh("equilateral_triangle");
    //    c2.write_mesh(*m, "equilateral_triangle");
    //    CHECK(c1.equals(c2));
    //}
    // SECTION("different_attributes")
    //{
    //    auto m = c2.read_mesh("equilateral_triangle");
    //
    //    auto a1 = m->get_attribute_handle<int64_t>("a1", PrimitiveType::Triangle);
    //    auto acc1 = m->create_accessor<int64_t>(a1);
    //    const auto faces = m->get_all(PrimitiveType::Triangle);
    //    for (size_t i = 0; i < faces.size(); ++i) {
    //        acc1.scalar_attribute(faces[i]) = i + 1;
    //    }
    //
    //    c2.write_mesh(*m, "equilateral_triangle");
    //    CHECK_FALSE(c1.equals(c2));
    //}
}
