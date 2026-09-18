#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/polyfem_ops/PythonFormat.hpp>

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

using wmtk::components::polyfem_ops::python_repr;

// The collision proxy OBJ is compared byte for byte against the one pysimwild writes, and its
// vertex lines are Python f-strings over floats, i.e. repr(). Every expected string below was
// produced by CPython 3.11's repr() on the value in the same row; the interesting rows are the
// ones that exercise the two places Python differs from every C++ default: the fixed/exponential
// switch (decpt <= -4 or decpt > 16, so 0.0001 but 1e-05, and 1000000000000000.0 but 1e+16) and
// the ".0" that keeps an integral value looking like a float.
TEST_CASE("python_repr matches CPython repr on a value table", "[polyfem_ops][repr]")
{
    const std::vector<std::pair<double, std::string>> table = {
        {0.0, "0.0"},
        {-0.0, "-0.0"},
        {1.0, "1.0"},
        {-1.0, "-1.0"},
        {0.5, "0.5"},
        {-0.5, "-0.5"},
        {0.1, "0.1"},
        {-0.1, "-0.1"},
        {0.2, "0.2"},
        {0.3, "0.3"},
        {1.0 / 3.0, "0.3333333333333333"},
        {-1.0 / 3.0, "-0.3333333333333333"},
        {2.0 / 3.0, "0.6666666666666666"},
        {2.5e-7, "2.5e-07"},
        {1e-4, "0.0001"},
        {1e-5, "1e-05"},
        {-1e-5, "-1e-05"},
        {9.999e-5, "9.999e-05"},
        {1e-3, "0.001"},
        {1e15, "1000000000000000.0"},
        {1e16, "1e+16"},
        {-1e16, "-1e+16"},
        {1e17, "1e+17"},
        {2e16 + 8, "2.000000000000001e+16"},
        {1e22, "1e+22"},
        {1e23, "1e+23"},
        {123456789.123, "123456789.123"},
        {6.02e23, "6.02e+23"},
        {1e-300, "1e-300"},
        {1e300, "1e+300"},
        {5e-324, "5e-324"}, // smallest positive denormal
        {-5e-324, "-5e-324"},
        {2.2250738585072014e-308, "2.2250738585072014e-308"}, // smallest positive normal
        {1.7976931348623157e308, "1.7976931348623157e+308"},
        {1234567890123456.0, "1234567890123456.0"},
        {12345678901234567.0, "1.2345678901234568e+16"},
        {3.14159265358979, "3.14159265358979"},
        {-2.718281828459045, "-2.718281828459045"},
        {100.0, "100.0"},
        {1e100, "1e+100"},
        {1e-100, "1e-100"},
        {7.0, "7.0"},
        {1e1, "10.0"},
        {1e-1, "0.1"},
        {0.0001220703125, "0.0001220703125"},
        {1e-6, "1e-06"},
    };

    for (const auto& [value, expected] : table) {
        CAPTURE(expected);
        CHECK(python_repr(value) == expected);
    }
}

// The table above is a fixed list, and the first defect this formatter had -- reading the
// exponent past the end of the un-terminated std::to_chars buffer -- only showed up in it when
// the byte after the output happened to be a digit, which was about one run in ten. A repr that
// does not parse back to the same double is wrong whatever the value, and checking that over a
// deterministic sweep of bit patterns turns that class of defect from a flake into a certainty.
TEST_CASE("python_repr round-trips every double it is given", "[polyfem_ops][repr]")
{
    // A fixed-seed 64-bit LCG over raw bit patterns: normals, denormals and both signs, the same
    // sequence on every machine and every run.
    uint64_t state = 0x853c49e6748fea9bULL;
    size_t checked = 0;
    for (int i = 0; i < 200000; ++i) {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        double value = 0;
        std::memcpy(&value, &state, sizeof(value));
        if (!std::isfinite(value)) {
            continue; // repr() of inf/nan is covered separately and does not round-trip
        }
        const std::string text = python_repr(value);
        char* parsed_end = nullptr;
        const double back = std::strtod(text.c_str(), &parsed_end);
        INFO("value bits " << state << " printed as " << text);
        REQUIRE(parsed_end == text.c_str() + text.size()); // the whole string is the number
        REQUIRE(std::memcmp(&back, &value, sizeof(value)) == 0);
        ++checked;
    }
    CHECK(checked > 190000);
}

TEST_CASE("python_repr matches CPython repr on non-finite values", "[polyfem_ops][repr]")
{
    CHECK(python_repr(std::numeric_limits<double>::infinity()) == "inf");
    CHECK(python_repr(-std::numeric_limits<double>::infinity()) == "-inf");
    CHECK(python_repr(std::numeric_limits<double>::quiet_NaN()) == "nan");
}
