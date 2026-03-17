# Fast envelope
# License: MIT

if(TARGET FastEnvelope::FastEnvelope)
    return()
endif()

set(FAST_ENVELOPE_WITH_UNIT_TESTS OFF)
set(FAST_ENVELOPE_ENABLE_TBB OFF)

include(cli11)

# # HACK because there is a linker error on Windows otherwise
# if(WIN32)
#     set(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES ON)
#     set(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES OFF)
# else()
#     set(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES OFF)
#     set(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES ON)
# endif()


message(STATUS "Third-party: creating target 'FastEnvelope::FastEnvelope'")


include(CPM)
CPMAddPackage("gh:daniel-zint/fast-envelope#5a826b77cdcf099dc693eac04612af04d32c8c03")

set_target_properties(FastEnvelope PROPERTIES FOLDER third_party)
add_library(FastEnvelope::FastEnvelope ALIAS FastEnvelope)