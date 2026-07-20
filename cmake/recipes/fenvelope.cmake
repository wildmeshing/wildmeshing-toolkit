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
    GITHUB_REPOSITORY daniel-zint/fast-envelope
    GIT_TAG 0a7a6c8f5ada9bbdd66da0b861680453f91e3846
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