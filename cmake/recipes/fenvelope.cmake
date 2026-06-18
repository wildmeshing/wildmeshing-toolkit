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
    GITHUB_REPOSITORY daniel-zint/fast-envelope
    GIT_TAG 0a7a6c8f5ada9bbdd66da0b861680453f91e3846
    OPTIONS
    "FAST_ENVELOPE_WITH_UNIT_TESTS OFF"
    "FAST_ENVELOPE_ENABLE_TBB OFF"
)

set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)
add_library(FastEnvelope::FastEnvelope ALIAS FastEnvelope)