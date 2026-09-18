#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/ConstraintMatrices.hpp>
#include <wmtk/components/polyfem_ops/NumpyCompat.hpp>

#include <cstdint>
#include <vector>

using wmtk::components::polyfem_ops::pairwise_sum;
using wmtk::components::polyfem_ops::parse_axes;

namespace {

/// A reproducible value sequence: a 64-bit linear congruential generator (the constants are
/// Knuth's MMIX ones), the top 53 bits turned into a fraction, centred and scaled by a power of
/// ten from 1e-3 to 1e3. Every step is exact in both languages, so the same k gives the same
/// double here and in the numpy run that produced the sums below.
std::vector<double> lcg_values(int64_t n)
{
    static const double scales[7] = {1e-3, 1e-2, 1e-1, 1.0, 1e1, 1e2, 1e3};
    uint64_t x = 0x853c49e6748fea9bULL;
    std::vector<double> out;
    out.reserve(static_cast<size_t>(n));
    for (int64_t k = 0; k < n; ++k) {
        x = 6364136223846793005ULL * x + 1442695040888963407ULL;
        const double u = static_cast<double>(x >> 11) * 0x1p-53;
        out.push_back((u - 0.5) * scales[k % 7]);
    }
    return out;
}

} // namespace

// The two constraint normalizations divide every written value by a np.sum over the matrix
// entries, so the sum has to agree to the last bit or the whole file moves. numpy's reduction is
// pairwise, not a running total. Each expected value below is `float(np.sum(a)).hex()` from
// numpy 2.4.6 on the array lcg_values(n) builds; the lengths straddle numpy's two thresholds
// (the 8-element unrolled block and the 128-element split). For n = 100, 129 and 1000 a plain
// left-to-right accumulation gives a DIFFERENT double, which is what makes this a real test.
TEST_CASE("pairwise_sum matches numpy's np.sum", "[polyfem_ops][numpy]")
{
    const std::vector<std::pair<int64_t, double>> expected = {
        {1, -0x1.021e05240379ap-11},
        {7, -0x1.9a71ab2b5b1b4p+8},
        {8, -0x1.9a71b4f4d186fp+8},
        {9, -0x1.9a71ce36d0eabp+8},
        {100, -0x1.0b981e27106b3p+9},
        {128, 0x1.7e382a2711d98p+9},
        {129, 0x1.7e372be63290cp+9},
        {1000, 0x1.3d37f2fd6fedcp+11},
    };
    for (const auto& [n, sum] : expected) {
        const std::vector<double> values = lcg_values(n);
        REQUIRE(pairwise_sum(values) == sum);
    }
}

TEST_CASE("pairwise_sum of the empty array is zero", "[polyfem_ops][numpy]")
{
    // np.sum([]) is 0.0, and a mass matrix over an empty node set would take this path.
    REQUIRE(pairwise_sum(std::vector<double>{}) == 0.0);
}

// parse_axes turns the `axes` of a protected_regions entry into the component indices that are
// held. The letters are lower-cased, de-duplicated and sorted, and each must exist in the mesh's
// dimension -- 'z' is a valid axis of a 3D mesh and an error on a 2D one.
TEST_CASE("parse_axes mirrors constraints.parse_axes", "[polyfem_ops][axes]")
{
    using nlohmann::json;

    REQUIRE_FALSE(parse_axes(json(), 3).has_value()); // absent: hold every component
    REQUIRE(parse_axes(json("x"), 3).value() == std::vector<int>{0});
    REQUIRE(parse_axes(json("z"), 3).value() == std::vector<int>{2});
    REQUIRE(parse_axes(json("xy"), 3).value() == std::vector<int>{0, 1});
    REQUIRE(parse_axes(json("yx"), 3).value() == std::vector<int>{0, 1}); // sorted
    REQUIRE(parse_axes(json("ZX"), 3).value() == std::vector<int>{0, 2}); // lower-cased
    REQUIRE(parse_axes(json("xx"), 3).value() == std::vector<int>{0}); // de-duplicated
    REQUIRE(parse_axes(json("xy"), 2).value() == std::vector<int>{0, 1});
    // An empty string parses to an empty axis set; the pin writer is what rejects it, exactly as
    // in the Python.
    REQUIRE(parse_axes(json(""), 3).value().empty());
    // The integer-list form, which only a direct engine call can produce.
    REQUIRE(parse_axes(json::array({2}), 3).value() == std::vector<int>{2});
    REQUIRE(parse_axes(json::array({2, 0, 2}), 3).value() == std::vector<int>{0, 2});

    REQUIRE_THROWS(parse_axes(json("z"), 2)); // no z on a 2D mesh
    REQUIRE_THROWS(parse_axes(json("w"), 3)); // not an axis at all
}
