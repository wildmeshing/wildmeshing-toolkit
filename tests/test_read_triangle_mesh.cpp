#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <wmtk/io/read_triangle_mesh.hpp>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

/**
 * Write the exact shape of Thingi10K's 286163.stl and 74463.stl: a binary STL whose
 * 80-byte header is followed by a triangle count of zero. 84 bytes, well-formed, and
 * describing nothing. The file is generated rather than checked in so that what makes
 * it interesting is visible here instead of hidden in a binary blob.
 */
void write_empty_binary_stl(const fs::path& path)
{
    std::array<char, 80> header{};
    const std::string tag = "COLOR="; // what those two Thingi10K files actually carry
    std::copy(tag.begin(), tag.end(), header.begin());

    const std::uint32_t n_triangles = 0;

    std::ofstream f(path, std::ios::binary);
    REQUIRE(f.is_open());
    f.write(header.data(), header.size());
    f.write(reinterpret_cast<const char*>(&n_triangles), sizeof(n_triangles));
    f.close();

    REQUIRE(fs::file_size(path) == 84);
}

/**
 * Write a binary STL that declares two triangles and then holds only one, the shape of
 * Thingi10K's 77942.stl -- which declares 6964 and is 50 bytes, exactly one facet record,
 * short of holding them. libigl parses face 0, then runs out of file on face 1.
 */
void write_truncated_binary_stl(const fs::path& path)
{
    std::array<char, 80> header{};
    const std::uint32_t declared_triangles = 2; // but only one facet follows

    // One complete facet: normal, three vertices, attribute byte count. 50 bytes.
    const std::array<float, 12> facet{
        0.f,
        0.f,
        1.f, // normal
        0.f,
        0.f,
        0.f, // v0
        1.f,
        0.f,
        0.f, // v1
        0.f,
        1.f,
        0.f, // v2
    };
    const std::uint16_t attribute_byte_count = 0;

    std::ofstream f(path, std::ios::binary);
    REQUIRE(f.is_open());
    f.write(header.data(), header.size());
    f.write(reinterpret_cast<const char*>(&declared_triangles), sizeof(declared_triangles));
    f.write(reinterpret_cast<const char*>(facet.data()), facet.size() * sizeof(float));
    f.write(reinterpret_cast<const char*>(&attribute_byte_count), sizeof(attribute_byte_count));
    f.close();

    REQUIRE(fs::file_size(path) == 84 + 50); // an 84-byte preamble and one of two facets
}

} // namespace

TEST_CASE("read_triangle_mesh_rejects_empty_mesh", "[io][read_triangle_mesh]")
{
    // libigl reads this file happily and hands back V and F as 0x0 -- not 0x3, which
    // is what the rest of the reader assumes. In Release that produced a segfault:
    // the bounding-box reductions (V.colwise().minCoeff(), in clean_triangle_mesh and
    // again in the tetwild component) dereference the null data pointer of an empty
    // matrix, before a single line of log was written. In Debug the missing third
    // column tripped the reader's own asserts first. The reader now rejects an empty
    // mesh straight after reading it, ahead of both.
    const fs::path path = fs::temp_directory_path() / "wmtk_test_empty_binary_stl_84_bytes.stl";
    write_empty_binary_stl(path);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    SECTION("single path, default tolerance")
    {
        // The default tol_rel is non-negative, so this takes the bounding-box branch
        // that used to crash.
        CHECK_THROWS(wmtk::io::read_triangle_mesh(path.string(), V, F));
    }

    SECTION("single path, vertex merging disabled")
    {
        // Both tolerances negative skips the bounding box entirely, so this reaches
        // the emptiness check by the other route.
        CHECK_THROWS(wmtk::io::read_triangle_mesh(path.string(), V, F, -1.0, -1.0));
    }

    SECTION("multiple paths")
    {
        const std::vector<std::string> paths{path.string(), path.string()};
        CHECK_THROWS(wmtk::io::read_triangle_mesh(paths, V, F));
    }

    fs::remove(path);
}

TEST_CASE("read_triangle_mesh_names_the_file_it_could_not_parse", "[io][read_triangle_mesh]")
{
    // Thingi10K 77942.stl is truncated one facet short of what its header declares.
    // libigl throws on it rather than returning false, and its message -- "Failed to
    // parse face 6963 from STL file" -- does not say which file. Nothing up the stack
    // caught it, so the exception travelled out of main and killed the process with no
    // filename anywhere in the output. Asserting on the path is the point of the test:
    // it threw before this change too, just uselessly.
    const fs::path path = fs::temp_directory_path() / "wmtk_test_truncated_binary_stl.stl";
    write_truncated_binary_stl(path);

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;

    SECTION("single path")
    {
        CHECK_THROWS_WITH(
            wmtk::io::read_triangle_mesh(path.string(), V, F),
            Catch::Matchers::ContainsSubstring(path.string()));
    }

    SECTION("multiple paths")
    {
        const std::vector<std::string> paths{path.string()};
        CHECK_THROWS_WITH(
            wmtk::io::read_triangle_mesh(paths, V, F),
            Catch::Matchers::ContainsSubstring(path.string()));
    }

    fs::remove(path);
}
