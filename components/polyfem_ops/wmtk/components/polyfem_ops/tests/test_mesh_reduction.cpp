#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/MeshReduction.hpp>

#include <set>
#include <string>
#include <vector>

using wmtk::components::polyfem_ops::classify_reduced_cell;
using wmtk::components::polyfem_ops::ReducedBody;
using wmtk::components::polyfem_ops::TagNames;

namespace {

/// The default `ambient_like_tags`: the option is empty, so only "ambient" itself is ambient-like.
const std::set<std::string> plain_ambient = {"ambient"};

const std::vector<int64_t> vertices = {1, 2, 3, 4};

} // namespace

TEST_CASE("polyfem_ops reduction classifies cells", "[components][polyfem_ops]")
{
    // The whole point of the reduction: polyfem gets exactly two materials, so each cell must
    // land in exactly one of them. These are the rules of _write_polyfem_reduced_msh.
    SECTION("ambient only")
    {
        REQUIRE(
            classify_reduced_cell(TagNames{"ambient"}, plain_ambient, vertices) ==
            ReducedBody::ambient);
    }
    SECTION("a body tag")
    {
        REQUIRE(
            classify_reduced_cell(TagNames{"tag_0"}, plain_ambient, vertices) == ReducedBody::body);
    }
    SECTION("several body tags: an overlap cell is still one body cell")
    {
        REQUIRE(
            classify_reduced_cell(TagNames{"tag_0", "tag_1"}, plain_ambient, vertices) ==
            ReducedBody::body);
    }
    SECTION("tagless cells are skipped, not written as ambient")
    {
        REQUIRE(classify_reduced_cell(TagNames{}, plain_ambient, vertices) == ReducedBody::skip);
    }
    SECTION("ambient together with a body tag is refused")
    {
        // One element cannot carry two materials; the Python raises ValueError here and the
        // message names the cell so the offending element can be found in the mesh.
        REQUIRE_THROWS(classify_reduced_cell(TagNames{"ambient", "tag_0"}, plain_ambient, vertices));
    }
}

TEST_CASE("polyfem_ops reduction honours ambient_like_tags", "[components][polyfem_ops]")
{
    // `ambient_like_tags` is for user primitives (box_0 and friends) that are ambient as far as
    // the solve is concerned: they get the ambient AMIPS weight and volume normalization.
    const std::set<std::string> ambient_like = {"ambient", "box_0"};

    SECTION("an ambient-like tag alone is ambient, without carrying the name 'ambient'")
    {
        REQUIRE(
            classify_reduced_cell(TagNames{"box_0"}, ambient_like, vertices) ==
            ReducedBody::ambient);
    }
    SECTION("ambient plus an ambient-like tag is ambient, not a refusal")
    {
        REQUIRE(
            classify_reduced_cell(TagNames{"ambient", "box_0"}, ambient_like, vertices) ==
            ReducedBody::ambient);
    }
    SECTION("a cell that also carries a body tag stays body")
    {
        REQUIRE(
            classify_reduced_cell(TagNames{"box_0", "tag_0"}, ambient_like, vertices) ==
            ReducedBody::body);
    }
    SECTION("ambient plus a body tag is still refused when an ambient-like tag is in the set")
    {
        REQUIRE_THROWS(
            classify_reduced_cell(TagNames{"ambient", "box_0", "tag_0"}, ambient_like, vertices));
    }
}
