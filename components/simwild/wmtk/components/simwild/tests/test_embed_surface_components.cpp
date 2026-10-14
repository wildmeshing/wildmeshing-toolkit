#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/EmbedSurface.hpp>

#include <filesystem>
#include <fstream>

using namespace wmtk;
using namespace wmtk::components::simwild;

namespace {

/// Two disjoint closed tetrahedra, written as one OBJ. One file, two connected components.
std::filesystem::path write_two_tetrahedra()
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "wmtk_two_tetrahedra.obj";
    std::ofstream out(path);

    // Tetrahedron A, around the origin.
    out << "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\n";
    // Tetrahedron B, well clear of A so the two cannot touch.
    out << "v 10 10 10\nv 11 10 10\nv 10 11 10\nv 10 10 11\n";

    // Each closed, with outward orientation.
    out << "f 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n";
    out << "f 5 7 6\nf 5 6 8\nf 5 8 7\nf 6 7 8\n";
    return path;
}

/// Number of inputs the embedding ended up with -- one tag column per input.
int input_count(const std::filesystem::path& path, const bool split)
{
    EmbedSurface surf({path.string()}, {Matrix4d::Identity()}, -1, -1, split);
    surf.embed_surface();
    return int(surf.T_tags().cols());
}

} // namespace

TEST_CASE("EmbedSurface can treat each connected component as its own input", "[simwild][embed]")
{
    const std::filesystem::path path = write_two_tetrahedra();

    // Off (the default): the file is one input, so the two tetrahedra share a tag and the
    // region between them cannot be distinguished from the region inside either.
    REQUIRE(input_count(path, false) == 1);

    // On: each component is an input in its own right, and gets its own tag column.
    REQUIRE(input_count(path, true) == 2);

    std::filesystem::remove(path);
}
