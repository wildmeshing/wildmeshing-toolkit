# delaunay_psm


if(TARGET geogram::delauanay_psm)
    return()
endif()

message(STATUS "Third-party: creating target 'geogram::delauanay_psm'")

include(FetchContent)
FetchContent_Declare(
    delauanay_psm
    GIT_REPOSITORY https://github.com/wildmeshing/Delaunay_psm.git
    GIT_TAG 3ecb4d6a1ccdcf5f3325a67470f18ae5b4074343
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(delauanay_psm)

add_library(delaunay_psm STATIC ${delauanay_psm_SOURCE_DIR}/Delaunay_psm.cpp)
target_compile_definitions(delaunay_psm PUBLIC GEO_STATIC_LIBS)
target_compile_features(delaunay_psm PRIVATE cxx_std_11)
target_include_directories(delaunay_psm PUBLIC ${delauanay_psm_SOURCE_DIR})
add_library(geogram::delaunay_psm ALIAS delaunay_psm)