if(TARGET parmetis::parmetis)
    return()
endif()

message(STATUS "Third-party: creating target 'parmetis::parmetis'")

include(FetchContent)
FetchContent_Declare(
    parmetis
    GIT_REPOSITORY https://github.com/wildmeshing/parMETIS.git
    GIT_TAG 01bd73f42a4d6654ce59df7b63561515fedf1375
)

FetchContent_GetProperties(parmetis)
if(NOT parmetis_POPULATED)
    FetchContent_Populate(parmetis)
endif()

# Create metis target
file(GLOB INC_FILES
    "${parmetis_SOURCE_DIR}/libparmetis/*.h"
)
file(GLOB SRC_FILES
    "${parmetis_SOURCE_DIR}/libparmetis/*.c"
)
list(REMOVE_ITEM SRC_FILES "${parmetis_SOURCE_DIR}/GKlib/gkregex.c")

add_library(parmetis STATIC ${INC_FILES} ${SRC_FILES})
add_library(parmetis::parmetis ALIAS parmetis)

if(MSVC)
    target_compile_definitions(parmetis PUBLIC USE_GKREGEX)
    target_compile_definitions(parmetis PUBLIC "__thread=__declspec(thread)")
endif()

target_include_directories(parmetis PRIVATE "${parmetis_SOURCE_DIR}/libparmetis")
target_include_directories(parmetis PRIVATE "${metis_SOURCE_DIR}/GKlib")

include(GNUInstallDirs)
target_include_directories(parmetis SYSTEM PUBLIC
    "$<BUILD_INTERFACE:${parmetis_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

set_target_properties(parmetis PROPERTIES FOLDER third_party)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "AppleClang" OR
   "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    target_compile_options(parmetis PRIVATE
        "-Wno-unused-variable"
        "-Wno-sometimes-uninitialized"
        "-Wno-absolute-value"
        "-Wno-shadow"
    )
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
    target_compile_options(parmetis PRIVATE
        "-w" # Disallow all warnings from metis.
    )
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
    target_compile_options(parmetis PRIVATE
        "/w" # Disable all warnings from metis!
    )
endif()

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME parmetis)
install(DIRECTORY ${parmetis_SOURCE_DIR}/include DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS parmetis EXPORT parmetis_Targets)
install(EXPORT parmetis_Targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/parmetis NAMESPACE parmetis::)

include(FindMPI)
if(NOT MPI_FOUND)
  message(FATAL_ERROR "mpi is not found")
endif()

target_link_libraries(parmetis PUBLIC metis::metis MPI::MPI_CXX)