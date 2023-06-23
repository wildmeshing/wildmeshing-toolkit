
if(TARGET paraviewo::paraviewo)
    return()
endif()

message(STATUS "Third-party (external): creating target 'paraviewo::paraviewo'")
option(PARAVIEWO_BUILD_TESTS "Build Tests" OFF)
set(PARAVIEWO_BUILD_TESTS OFF CACHE BOOL "" FORCE)
include(FetchContent)
FetchContent_Declare(
    paraviewo
    GIT_REPOSITORY https://github.com/polyfem/paraviewo.git
    GIT_TAG a373a0edb2ef41795b8426cb329d2d2eebe132a1
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(paraviewo)

set_target_properties(paraviewo PROPERTIES FOLDER third_party)