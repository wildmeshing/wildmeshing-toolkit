# Fast envelope
# License: MIT

if(TARGET FastEnvelope::FastEnvelope)
    return()
endif()

include(cli11)
include(libigl)

message(STATUS "Third-party: creating target 'FastEnvelope::FastEnvelope'")

# fast-envelope uses exact orientation predicates in exactly three places
# (src/external/Predicates.cpp: orient_3d, orient_3d_tolerance, orient_2d) and offers three
# backends for them. The default is the whole of geogram, which is a lot of library -- plus,
# on Linux, geogram's TBB dependency -- for two predicates. Turning both geogram options off
# selects its third backend, igl::predicates (Shewchuk), which this repository already links
# and already uses for its own orientation tests. That takes geogram out of the build.
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
    GITHUB_REPOSITORY wildmeshing/fast-envelope
    GIT_TAG 7d615a9c578087dab899cd370cb225464908dcf4
    OPTIONS
    "FAST_ENVELOPE_WITH_UNIT_TESTS OFF"
    "FAST_ENVELOPE_ENABLE_TBB OFF"
    "FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES OFF"
    "FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES OFF"
)

# Its own CMakeLists links a predicate library only in the two geogram branches -- it predates
# the current libigl cmake and never links libigl -- so the fallback needs the target here,
# both for <igl/predicates/predicates.h> and for the symbols behind it.
target_link_libraries(FastEnvelope PUBLIC igl::predicates)

set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)
add_library(FastEnvelope::FastEnvelope ALIAS FastEnvelope)