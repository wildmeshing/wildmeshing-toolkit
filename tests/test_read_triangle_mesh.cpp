#include <catch2/catch_test_macros.hpp>

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
