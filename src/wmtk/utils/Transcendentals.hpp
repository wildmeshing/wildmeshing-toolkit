#pragma once

// Namespace-level wrappers for the transcendental functions whose libm
// implementations are not correctly rounded and therefore differ by a few ULP
// across platforms (macOS libm vs glibc, x86 SSE/AVX vs ARM NEON). Feeding such a
// result into a discrete decision (mesh-operation ordering, energy comparisons,
// winding-number signs) makes the whole pipeline diverge across OSes/CPUs.
//
// By default these forward to the fast, architecture-specific std:: versions --
// there is no reason to pay for determinism in a normal build.
//
// When the CMake option WMTK_FP_STRICT is ON it defines the WMTK_FP_STRICT macro
// globally (see the top-level CMakeLists.txt), and these wrappers switch to the
// hand-rolled deterministic implementations, which use only the IEEE-754
// correctly-rounded operations (+ - * / sqrt) and fixed polynomials so they
// produce the same bits on every platform. That is what makes the golden-hash
// integration tests reproducible across compilers/OSes/CPUs.
//
// Use wmtk::cbrt / wmtk::atan2 / wmtk::atan anywhere a transcendental feeds into a
// discrete decision; plain std:: is fine everywhere else.

#include <cmath>

#ifdef WMTK_FP_STRICT
#include <wmtk/utils/deterministic_atan2.hpp>
#include <wmtk/utils/deterministic_cbrt.hpp>
#endif

namespace wmtk {

inline double cbrt(double x)
{
#ifdef WMTK_FP_STRICT
    return utils::deterministic_cbrt(x);
#else
    return std::cbrt(x);
#endif
}

inline double atan(double x)
{
#ifdef WMTK_FP_STRICT
    return utils::deterministic_atan(x);
#else
    return std::atan(x);
#endif
}

inline double atan2(double y, double x)
{
#ifdef WMTK_FP_STRICT
    return utils::deterministic_atan2(y, x);
#else
    return std::atan2(y, x);
#endif
}

} // namespace wmtk
