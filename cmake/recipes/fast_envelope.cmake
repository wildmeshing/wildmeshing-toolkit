# Fast envelope
# License: MIT

if(TARGET FastEnvelope::FastEnvelope)
    return()
endif()

set(FAST_ENVELOPE_WITH_UNIT_TESTS OFF)
set(FAST_ENVELOPE_ENABLE_TBB OFF)

# HACK because there is a linker error on Windows otherwise
if(WIN32)
    set(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES ON)
    set(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES OFF)
else()
    set(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES OFF)
    set(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES ON)
endif()


message(STATUS "Third-party: creating target 'FastEnvelope::FastEnvelope'")


include(CPM)
CPMAddPackage("gh:wangbolun300/fast-envelope#38b2cfbbf0f78ced8b4c5981168e8ded809ac654")

set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)
add_library(FastEnvelope::FastEnvelope ALIAS FastEnvelope)
