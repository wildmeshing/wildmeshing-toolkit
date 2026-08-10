#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/resolve_path.hpp>

namespace fs = std::filesystem;
using wmtk::utils::resolve_path;

TEST_CASE("resolve_path_empty_root", "[utils][resolve_path]")
{
    // `wmtk_app -j config.json` leaves the components with input_dir == "", because
    // app/main.cpp derives it from the config file's parent_path(). Resolving against
    // an empty root has to mean "the current directory" rather than throw: fs::absolute("")
    // throws on libstdc++ and does not on libc++, so this was a Linux-only abort --
    // every component, every model, before any work was done.
    fs::path resolved;
    REQUIRE_NOTHROW(resolved = resolve_path("", "out.log"));
    CHECK(resolved.is_absolute());
    CHECK(resolved == fs::weakly_canonical(fs::current_path() / "out.log"));
}

TEST_CASE("resolve_path_relative_root", "[utils][resolve_path]")
{
    const fs::path resolved = resolve_path("a", "b.txt");
    CHECK(resolved.is_absolute());
    CHECK(resolved == fs::weakly_canonical(fs::current_path() / "a" / "b.txt"));
}

TEST_CASE("resolve_path_absolute_path_ignores_root", "[utils][resolve_path]")
{
    const fs::path absolute_input = fs::current_path() / "c" / "b.txt";

    // An absolute path is returned untouched whatever the root is -- including the
    // empty root, which must not reach fs::absolute() on the way.
    CHECK(resolve_path("a", absolute_input) == absolute_input);
    CHECK(resolve_path("", absolute_input) == absolute_input);
}
