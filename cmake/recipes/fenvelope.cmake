# Fast envelope
# License: MIT

if(TARGET FastEnvelope::FastEnvelope)
    return()
endif()

message(STATUS "Third-party: creating target 'FastEnvelope::FastEnvelope'")

include(CPM)
CPMAddPackage(
    NAME FastEnvelope
    # wildmeshing fork, forked from daniel-zint/fast-envelope @ 0a7a6c8.
    # Carries PR #1 (perf: hoist per-node box-cut work) and PR #2, which drops the
    # vendored expansion arithmetic for MarcoAttene's NFG + Indirect_Predicates -- the
    # same pair VolumeRemesher uses. Two copies of those cannot share a binary: both put
    # `expansionObject` in the global namespace and disagree on whether its members are
    # static, which the Itanium ABI mangles identically, so calls through the wrong
    # convention have their arguments shifted by a register.
    #
    # Plus PR #3, which is required with #2: `genericPoint::orient3D` returns the opposite
    # sign from the predicates #2 replaced, and FastEnvelope reads those predicates as the
    # height of the implicit point over an oriented plane -- a point is in a prism when it
    # is below every outward face, i.e. every face NEGATIVE. Forwarding the raw sign makes
    # containment fail everywhere, so the envelope silently grows far more conservative
    # than the requested epsilon rather than erroring out.
    #
    # PR #3 also puts back the graded cascade that #2 removed along with the arithmetic it
    # fronted: the vendored semi-static double filter, then Indirect_Predicates' interval
    # stage, then its exact one. Indirect_Predicates has no double-precision tier of its
    # own, so without the first every query went straight to interval arithmetic, which on
    # arm64 switches the FPU rounding mode twice per predicate -- 48% of samples in
    # `fesetround` and 2.5-3.4x the runtime here. The filter is self-contained double code
    # and reintroduces none of the duplicate expansion definitions that #2 existed to
    # remove.
    #
    # Then PR #5, which drops geogram, libigl and TBB entirely (the three predicate backends
    # collapse to one over Indirect_Predicates, so `FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES`,
    # `..._PSM_PREDICATES` and `FAST_ENVELOPE_ENABLE_TBB` no longer exist and neither does the
    # igl::predicates link this file used to add), moves the sources under src/fastenvelope/
    # leaving <fastenvelope/FastEnvelope.h> resolving as before, and switches the tests to
    # Catch2. And PR #6, which adds envelopes built from edges rather than triangles -- both
    # `FastEnvelope::init(V, edges, eps)` in 3D and the new `FastEnvelope2D` -- which is what
    # lets SampleEnvelope answer segment and 2D queries exactly instead of throwing.
    #
    # And PR #8, which points fast-envelope's own Indirect_Predicates declaration
    # (cmake/recipes/ipred.cmake) at the same commit VolumeRemesher uses. FetchContent keeps
    # the first declaration and silently ignores later ones, so while the two disagreed the
    # include order here decided the version for both -- volumeremesher comes first in the
    # top-level CMakeLists, so fast-envelope ran on a different Indirect_Predicates than the
    # one it was built and tested against upstream. Now that both pin the same commit that
    # include order no longer matters; keep them moving together.
    GITHUB_REPOSITORY wildmeshing/fast-envelope
    # main. A commit rather than the branch name, so the build stays reproducible.
    GIT_TAG b116e1511e2130885ef3cc288c1253fc079b6989
    OPTIONS
    "FAST_ENVELOPE_WITH_UNIT_TESTS OFF"
)

# The FastEnvelope::FastEnvelope alias, and the cli11/libigl this file used to include for it,
# are gone: since #5 fast-envelope declares the alias itself and needs neither library.
set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)