#include "PythonFormat.hpp"

#include <wmtk/utils/Logger.hpp>

#include <array>
#include <charconv>
#include <cmath>
#include <string>

namespace wmtk::components::polyfem_ops {

namespace {

/// The shortest round-tripping decimal of `value`, split the way David Gay's dtoa (the routine
/// CPython's repr uses) reports it: `digits` are the significant digits with no leading or
/// trailing zero, `decpt` the decimal exponent, so |value| = 0.<digits> * 10^decpt.
///
/// std::to_chars with chars_format::scientific is the same shortest-round-trip digit string, only
/// already rendered as d[.ddd]e[+-]dd; we parse it back rather than re-deriving the digits. Going
/// through the scientific form (instead of the default overload) is deliberate: the default picks
/// fixed or scientific by whichever is shorter, which is NOT CPython's rule, and its exponent
/// padding is unspecified -- parsing removes both dependencies.
void shortest_digits(double value, std::string& digits, int& decpt)
{
    std::array<char, 64> buf;
    const auto res =
        std::to_chars(buf.data(), buf.data() + buf.size(), value, std::chars_format::scientific);
    if (res.ec != std::errc()) {
        log_and_throw_error("python_repr: std::to_chars failed on a finite double");
    }

    const char* p = buf.data();
    const char* const end = res.ptr;
    if (p != end && *p == '-') {
        ++p; // the caller prints the sign; -0.0 has no digits to tell it from 0.0
    }
    digits.clear();
    for (; p != end && *p != 'e'; ++p) {
        if (*p != '.') {
            digits.push_back(*p);
        }
    }
    // std::to_chars does NOT null-terminate, so the exponent has to be read within [p, end).
    // Reading it with atoi() instead passed most values and then printed 6.02e+23 as
    // "6.02e+230" roughly one run in ten, whenever the uninitialised byte after the output
    // happened to be a digit.
    int exponent = 0;
    if (p != end) {
        ++p; // skip the 'e'
        bool negative_exponent = false;
        if (p != end && (*p == '+' || *p == '-')) {
            negative_exponent = *p == '-';
            ++p;
        }
        unsigned int magnitude = 0;
        if (std::from_chars(p, end, magnitude).ec != std::errc()) {
            log_and_throw_error("python_repr: unparsable exponent in the std::to_chars output");
        }
        exponent = negative_exponent ? -static_cast<int>(magnitude)
                                     : static_cast<int>(magnitude);
    }

    // dtoa reports zero as digits "0", decpt 1, which the fixed-point branch below turns into
    // "0" and then "0.0"; to_chars writes it as 0e+00, i.e. the same digits and exponent 0.
    decpt = exponent + 1;

    // Trailing zeros never appear in a shortest round-trip digit string except for zero itself.
    while (digits.size() > 1 && digits.back() == '0') {
        digits.pop_back();
    }
}

} // namespace

std::string python_list(const std::set<std::string>& values)
{
    std::string out = "[";
    bool first = true;
    for (const auto& s : values) {
        if (!first) out += ", ";
        first = false;
        out += "'" + s + "'";
    }
    return out + "]";
}

std::string python_list(const std::vector<int64_t>& values)
{
    std::string out = "[";
    for (size_t i = 0; i < values.size(); ++i) {
        if (i != 0) out += ", ";
        out += std::to_string(values[i]);
    }
    return out + "]";
}

std::string python_repr(double value)
{
    // repr() of a non-finite float; these cannot reach an OBJ from a valid mesh, but the
    // formatter is a mirror of repr() and silently emitting "inf" the C++ way would be a lie.
    if (std::isnan(value)) {
        return "nan";
    }
    if (std::isinf(value)) {
        return value < 0 ? "-inf" : "inf";
    }

    std::string digits;
    int decpt = 0;
    shortest_digits(value, digits, decpt);

    const bool negative = std::signbit(value);
    const int n = static_cast<int>(digits.size());

    std::string out;
    if (negative) {
        out.push_back('-');
    }

    if (decpt <= -4 || decpt > 16) {
        // Exponential form: d[.ddd]e<sign><at least two digits>.
        out.push_back(digits[0]);
        if (n > 1) {
            out.push_back('.');
            out.append(digits, 1, std::string::npos);
        }
        out.push_back('e');
        int exp = decpt - 1;
        out.push_back(exp < 0 ? '-' : '+');
        exp = std::abs(exp);
        const std::string exp_digits = std::to_string(exp);
        if (exp_digits.size() < 2) {
            out.push_back('0');
        }
        out += exp_digits;
        return out;
    }

    if (decpt <= 0) {
        out += "0.";
        out.append(static_cast<size_t>(-decpt), '0');
        out += digits;
    } else if (decpt < n) {
        out.append(digits, 0, static_cast<size_t>(decpt));
        out.push_back('.');
        out.append(digits, static_cast<size_t>(decpt), std::string::npos);
    } else {
        out += digits;
        out.append(static_cast<size_t>(decpt - n), '0');
        out += ".0"; // Py_DTSF_ADD_DOT_0: an integral repr still reads as a float
    }
    return out;
}

} // namespace wmtk::components::polyfem_ops
