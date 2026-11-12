# tight (https://gitlab.com/minimize-solve/tight), the interval arithmetic of polyfem's Jacobian
# check (polyfem -> hocgv-miso -> misolib -> tight); only a WMTK_WITH_POLYFEM build includes it.
# License: GPL-3.0
#
# misolib's pin, 7af3bd8a, plus patches/tight_nfg_namespace.patch, so that the build has one NFG.
# tight 7af3bd8a takes NFG at f1df171, from before NFG moved its names into `namespace NFG`;
# VolumeRemesher and fast-envelope take aeec975, a descendant of it (recipes/volumeremesher.cmake).
# The patch writes NFG:: where tight names NFG's types and functions; gives a zero rational sign 0
# in Interval::fromRational, since NFG's (num, den, sign) constructor no longer canonicalizes and
# bigrational::get_d() crashes on a zero of sign 1; and moves tight's own NFG pin to aeec975. With
# the two pins equal, tight gets the same NFG whichever declaration wins (it has varied with the
# CMake version and with what the CPM cache already holds). Measured: every
# tight::Interval operation, on 6000 interval pairs plus rationals and strings (504,033 results),
# gives the same bits before and after, on arm64 (macOS) and x86-64 with AVX2 (Linux, GCC 13).
#
# misolib loads tight with include(tight) after appending its own recipe directory to
# CMAKE_MODULE_PATH, so this recipe, earlier on the path, is the one found (as polyfem's
# include()s find wmtk's recipes, recipes/polyfem.cmake). Delete it, and the patch, once
# polyfem's pins reach a tight that takes NFG aeec975 or later.

if(TARGET tight::tight)
    return()
endif()

message(STATUS "Third-party: creating target 'tight::tight'")

# tight compiles itself, and through its nfg target what links it (hocgv-miso), for the CPU of the
# machine that runs cmake: TIGHT_SIMD defaults to AUTO, which on an x86 machine with AVX2 adds
# -mavx2 -mfma (83 files on Kirby). This repo builds its dependencies for the compiler's default
# instruction set (cmake/FindAVX.cmake), so TIGHT_SIMD is the baseline instead: SSE2 on x86-64
# (tight adds -msse2, which x86-64 already has), ARMNEON on arm64 (what AUTO picks there: tight
# adds -march=armv8-a+simd and simde, whose headers NFG's numerics.h includes under __ARM_NEON).
# Measured on Kirby (x86-64, GCC 13), patched tight at SSE2 against AVX2: of the 504,033 results
# above, 388 differ, all products and quotients with a zero or infinite endpoint: the sign of a
# zero, NaN against a bound, and 15 products the AVX2 path does not enclose ([-0, 0.0107] times
# [-1.66e-9, inf] gives [-1.78e-11, 0]); SSE2 encloses every one. NFG takes that AVX2 path on
# arm64 too, through simde. A cache entry, not a CPM option: tight declares it with set(CACHE)
# under CMP0126's OLD behaviour, which drops a normal variable of the same name.
string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" _wmtk_tight_proc)
if(_wmtk_tight_proc MATCHES "^(aarch64|arm)")
    set(_wmtk_tight_simd ARMNEON)
elseif(_wmtk_tight_proc MATCHES "^(x86_64|amd64)")
    set(_wmtk_tight_simd SSE2)
else()
    set(_wmtk_tight_simd NONE)
endif()
set(TIGHT_SIMD ${_wmtk_tight_simd} CACHE STRING
    "SIMD instruction set for tight (AUTO, AVX2, SSE2, ARMNEON, NONE)")

include(CPM)
CPMAddPackage(
    NAME tight
    GITLAB_REPOSITORY minimize-solve/tight
    GIT_TAG 7af3bd8a594309c9dc278f9ed6e3807318e3c85f
    PATCHES "${CMAKE_CURRENT_LIST_DIR}/../patches/tight_nfg_namespace.patch"
)
