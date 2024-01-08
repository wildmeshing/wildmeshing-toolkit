# Fast envelope
# License: MIT

if(TARGET FastEnvelope::FastEnvelope)
    return()
endif()

set(FAST_ENVELOPE_WITH_UNIT_TESTS OFF)
set(FAST_ENVELOPE_ENABLE_TBB OFF)
set(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES OFF)
set(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES ON)


message(STATUS "Third-party: creating target 'FastEnvelope::FastEnvelope'")


include(CPM)
CPMAddPackage("gh:wangbolun300/fast-envelope#e2c5e076f12e1684f0aff26f257799b85052a5a7")

set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)
add_library(FastEnvelope::FastEnvelope ALIAS FastEnvelope)
