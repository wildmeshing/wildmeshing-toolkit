if(TARGET metis::metis)
    return()
endif()

message(STATUS "Third-party: creating target 'metis::metis'")

include(FetchContent)
FetchContent_Declare(
    metis
    GIT_REPOSITORY https://github.com/jdumas/METIS.git
    GIT_TAG d29126a68e34252b977d03bd2d89eb765f148a19
)

FetchContent_GetProperties(metis)
if(NOT metis_POPULATED)
    FetchContent_Populate(metis)
endif()

# Create metis target
file(GLOB INC_FILES
    "${metis_SOURCE_DIR}/GKlib/*.h"
    "${metis_SOURCE_DIR}/libmetis/*.h"
)
file(GLOB SRC_FILES
    "${metis_SOURCE_DIR}/GKlib/*.c"
    "${metis_SOURCE_DIR}/libmetis/*.c"
)
list(REMOVE_ITEM SRC_FILES "${metis_SOURCE_DIR}/GKlib/gkregex.c")

add_library(metis STATIC ${INC_FILES} ${SRC_FILES})
add_library(metis::metis ALIAS metis)

if(MSVC)
    target_compile_definitions(metis PUBLIC USE_GKREGEX)
    target_compile_definitions(metis PUBLIC "__thread=__declspec(thread)")
endif()

target_include_directories(metis PRIVATE "${metis_SOURCE_DIR}/GKlib")
target_include_directories(metis PRIVATE "${metis_SOURCE_DIR}/libmetis")

include(GNUInstallDirs)
target_include_directories(metis SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${metis_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

set_target_properties(metis PROPERTIES FOLDER third_party)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    target_compile_options(metis PRIVATE
        "-Wno-unused-variable"
        "-Wno-sometimes-uninitialized"
        "-Wno-absolute-value"
        "-Wno-shadow"
    )
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    target_compile_options(metis PRIVATE
        "-w" # Disallow all warnings from metis.
    )
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    target_compile_options(metis PRIVATE
        "/w" # Disable all warnings from metis!
    )
endif()

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME metis)
install(DIRECTORY ${metis_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS metis EXPORT Metis_Targets)
install(EXPORT Metis_Targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/metis NAMESPACE metis::)
