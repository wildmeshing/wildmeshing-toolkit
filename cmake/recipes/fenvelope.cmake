if(TARGET FastEnvelope)
    return()
endif()

message(STATUS "Third-party: creating target 'FastEnvelope'")

# Options for envelope module
option(FAST_ENVELOPE_WITH_UNIT_TESTS             "Unit test project"                   OFF)
option(FAST_ENVELOPE_ENABLE_TBB                  "Enable TBB"                          OFF)
option(FAST_ENVELOPE_WITH_GEOGRAM_PREDICATES     "Use whole Geogram for predicates"    OFF)
option(FAST_ENVELOPE_WITH_GEOGRAM_PSM_PREDICATES "Use Geogram predicates only"         OFF)
option(FAST_ENVELOPE_WITH_GMP                    "Use gmp to have rational predicates" OFF)
option(FAST_ENVELOPE_WITH_TIMER                  "Use timer in the library for debug"  OFF)

include(FetchContent)
FetchContent_Declare(
    fenvelope
    GIT_REPOSITORY https://github.com/wangbolun300/fast-envelope.git
    GIT_TAG 5d5d5ac99b14400b2757e043fbc1bd9eacd0cced
)
FetchContent_MakeAvailable(fenvelope)
