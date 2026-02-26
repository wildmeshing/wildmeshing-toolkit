# delaunay_psm


if(TARGET geogram::delauanay_psm)
    return()
endif()

message(STATUS "Third-party: creating target 'geogram::delauanay_psm'")

include(FetchContent)
FetchContent_Declare(
    delauanay_psm
    GIT_REPOSITORY https://github.com/wildmeshing/Delaunay_psm
    GIT_TAG 85225e697ce496e744589b1576718754499e03a4
)
FetchContent_MakeAvailable(delauanay_psm)

add_library(delaunay_psm STATIC ${delauanay_psm_SOURCE_DIR}/Delaunay_psm.h ${delauanay_psm_SOURCE_DIR}/Delaunay_psm.cpp)
target_compile_definitions(delaunay_psm PUBLIC GEO_STATIC_LIBS)
target_compile_features(delaunay_psm PRIVATE cxx_std_17)
target_include_directories(delaunay_psm PUBLIC ${delauanay_psm_SOURCE_DIR})
add_library(geogram::delaunay_psm ALIAS delaunay_psm)

set_target_properties(delaunay_psm PROPERTIES FOLDER third_party)
set_target_properties(delaunay_psm PROPERTIES POSITION_INDEPENDENT_CODE ON)