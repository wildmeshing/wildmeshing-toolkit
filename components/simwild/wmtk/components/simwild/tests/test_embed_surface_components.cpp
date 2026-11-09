#include <catch2/catch_test_macros.hpp>

#include <jse/jse.h>
#include <spdlog/sinks/ostream_sink.h>
#include <simwild_spec.hpp>
#include <wmtk/components/simwild/EmbedSurface.hpp>
#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/utils/Logger.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <random>
#include <sstream>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

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

/// A file of its own in the temp directory, so concurrent runs do not collide, holding `obj`.
std::filesystem::path write_obj(const std::string& name, const std::string& obj)
{
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() /
        ("wmtk_" + name + "_" + std::to_string(std::random_device{}()) + ".obj");
    std::ofstream(path) << obj;
    return path;
}

/// The warnings the reader logs on a closed input and an open one of input_type `open_type`
/// ("" for no input_type: both "volume"), 3D (read_mesh) or 2D (read_curves).
std::string reader_warnings(const int dim, const std::string& open_type)
{
    // 3D: tetrahedron A and an open square beside it; 2D: a triangle and a straight open
    // polyline. Flat, either open input has a winding number below 0.5 everywhere (a bent
    // polyline exceeds 0.5 inside its bend, and tags faces there).
    const std::filesystem::path closed = write_obj(
        "closed",
        dim == 3 ? "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nf 1 3 2\nf 1 2 4\nf 1 4 3\nf 2 3 4\n"
                 : "v 0 0 0\nv 1 0 0\nv 0 1 0\nl 1 2 3 1\n");
    const std::filesystem::path open = write_obj(
        "open",
        dim == 3 ? "v 2 0 0\nv 3 0 0\nv 3 1 0\nv 2 1 0\nf 1 2 3\nf 1 3 4\n"
                 : "v 2 0 0\nv 3 0 0\nv 4 0 0\nl 1 2 3\n");
    nlohmann::json j;
    j["application"] = "simwild";
    j["input"] = {closed.string(), open.string()};
    if (!open_type.empty()) j["input_type"] = {"volume", open_type};
    const auto spec = jse::embed::wmtk_simwild_spec::simwild_spec::spec();
    jse::JSE engine;
    REQUIRE(engine.verify_json(j, spec));
    j = engine.inject_defaults(j, spec);

    // The logger's own level too: another case of the suite turns it off
    // (test_simwild_energies.cpp), which would hide every warning.
    std::ostringstream text;
    const auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(text);
    const auto level = logger().level();
    logger().set_level(spdlog::level::warn);
    logger().sinks().push_back(sink);
    const std::vector<std::string> paths = {closed.string(), open.string()};
    if (dim == 3) {
        read_mesh(paths, "", j);
    } else {
        read_curves(paths, "", j);
    }
    auto& sinks = logger().sinks();
    sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
    logger().set_level(level);
    std::filesystem::remove(closed);
    std::filesystem::remove(open);
    return text.str();
}

} // namespace

TEST_CASE("simwild-warns-only-for-an-input-meant-to-enclose", "[simwild][embed]")
{
    // An input whose winding number is above 0.5 nowhere tags no cell. For a "volume" input,
    // declared or by default, that is a defect, and the reader warns. A "surface" input in 3D, a
    // "curve" in 2D, encloses nothing by its type, so tagging nothing is expected and reports
    // none. Until 2026-11 the winding-number tagging warned for both (prism_simple_shapes.py's
    // square, prism_demo4.py's S1). The closed input tags its inside: no warning either way.
    const std::string warning = "No winding number above 0.5 for input";
    for (const int dim : {3, 2}) {
        CAPTURE(dim);
        CHECK(
            reader_warnings(dim, dim == 3 ? "surface" : "curve").find(warning) ==
            std::string::npos);
        for (const std::string type : {"volume", ""}) {
            CAPTURE(type);
            const std::string w = reader_warnings(dim, type);
            CHECK(w.find(warning + " 1") != std::string::npos);
            CHECK(w.find(warning + " 0") == std::string::npos);
        }
    }
}

TEST_CASE(
    "EmbedSurface can treat each connected component as its own input",
    "[simwild][embed]" + release_only)
{
    const std::filesystem::path path = write_two_tetrahedra();

    // Off (the default): the file is one input, so the two tetrahedra share a tag and the
    // region between them cannot be distinguished from the region inside either.
    REQUIRE(input_count(path, false) == 1);

    // On: each component is an input in its own right, and gets its own tag column.
    REQUIRE(input_count(path, true) == 2);

    std::filesystem::remove(path);
}
