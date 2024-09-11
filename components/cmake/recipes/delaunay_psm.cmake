# delaunay_psm

# optional

if(TARGET geogram::delaunay_psm)
    return()
endif()

message(STATUS "Third-party: creating target 'geogram::delauanay_psm'")

include(CPM)
CPMAddPackage(
    NAME delauanay_psm
    GITHUB_REPOSITORY wildmeshing/Delaunay_psm
    GIT_TAG 0da678f007015e2c94b06d1450bfea4c46680cef
)

add_library(delaunay_psm STATIC ${delauanay_psm_SOURCE_DIR}/Delaunay_psm.cpp)
target_compile_definitions(delaunay_psm PUBLIC GEO_STATIC_LIBS)
target_compile_features(delaunay_psm PRIVATE cxx_std_11)
target_include_directories(delaunay_psm PUBLIC ${delauanay_psm_SOURCE_DIR})
add_library(geogram::delaunay_psm ALIAS delaunay_psm)

set_target_properties(delaunay_psm PROPERTIES FOLDER third_party)
