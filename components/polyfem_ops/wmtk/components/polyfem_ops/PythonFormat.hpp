#pragma once

#include <string>

namespace wmtk::components::polyfem_ops {

/**
 * @brief Format a double exactly as CPython's `repr()` does.
 *
 * The collision proxy OBJ must be byte-identical to the one pysimwild writes, and
 * `constraints.write_collision_mesh_obj` builds its vertex lines with an f-string (`f"v {x} {y}
 * {z}\n"`), which calls `str()` on a float, which for Python 3 is `repr()`. Mirroring that
 * formatter is therefore part of the file format, not a cosmetic choice.
 *
 * CPython's rule (Objects/stringlib, pystrtod.c `format_float_short`, format code 'r'):
 *   - the digit string is the shortest one that round-trips to the same double;
 *   - with `decpt` the decimal exponent (value = 0.d1d2... * 10^decpt), the result is written in
 *     exponential form iff `decpt <= -4 || decpt > 16`, e.g. `1e-05` but `0.0001`, and `1e+16`
 *     but `1000000000000000.0`. The 16 (rather than 17) is CPython's: a 16-digit shortest repr
 *     padded with bogus zeros looks wrong (repr(2e16+8) would read 20000000000000010.0);
 *   - the exponent always carries a sign and at least two digits (`1e-05`, `1e+16`, `5e-324`);
 *   - a result with neither '.' nor 'e' gets ".0" appended (Py_DTSF_ADD_DOT_0), so `1.0`, not `1`;
 *   - the sign of -0.0 is kept: `-0.0`.
 */
std::string python_repr(double value);

} // namespace wmtk::components::polyfem_ops
