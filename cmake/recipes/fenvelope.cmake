# Fast envelope
# License: MIT

if(TARGET FastEnvelope::FastEnvelope)
    return()
endif()

include(cli11)

message(STATUS "Third-party: creating target 'FastEnvelope::FastEnvelope'")

include(CPM)
CPMAddPackage(
    NAME FastEnvelope
    # wildmeshing fork tracking PR #1 (perf: hoist per-node box-cut work).
    # Forked from daniel-zint/fast-envelope @ 0a7a6c8; GIT_TAG is the PR branch tip.
    GITHUB_REPOSITORY wildmeshing/fast-envelope
    GIT_TAG 86d41345aa80179790a9142bbafaaf2de21d9054
    OPTIONS
    "FAST_ENVELOPE_WITH_UNIT_TESTS OFF"
    "FAST_ENVELOPE_ENABLE_TBB OFF"
)

set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)
add_library(FastEnvelope::FastEnvelope ALIAS FastEnvelope)