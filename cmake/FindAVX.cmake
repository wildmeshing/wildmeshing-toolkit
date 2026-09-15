# FindAVX -- deliberately empty, and it shadows a dependency's module. Do not "finish" it.
#
# Tight-Inclusion (pulled in by ipc-toolkit) calls find_package(AVX) and ships its own
# cmake/find/FindAVX.cmake, which on GCC settles on -march=native and then does
#
#   set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} ${AVX_FLAGS}")
#   set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "...")
#   set(CMAKE_CXX_FLAGS_RELEASE        "...")
#
# -- so the flag reaches its whole directory scope, not just the tight_inclusion target, and
# editing the target's COMPILE_OPTIONS afterwards would not remove it. The module itself has to
# not run, and there is no variable to pre-set: it opens with an unconditional set(AVX_FLAGS).
#
# An object compiled for the host CPU is only valid on that CPU. CI moves objects between runners
# through ccache, whose hash covers the command line and the sources but not the machine, so one
# built where AVX-512 exists and restored where it does not dies on its first EVEX instruction.
# This repo therefore builds its dependencies for the compiler's default instruction set, so that
# a build is valid wherever it is run. recipes/ipc_toolkit.cmake does the same for ipc's own
# FindSIMD (by a different route, for a reason documented there).
#
# This file wins because the top-level CMakeLists.txt puts ${CMAKE_SOURCE_DIR}/cmake on
# CMAKE_MODULE_PATH before any dependency is added, and both ipc-toolkit and Tight-Inclusion
# APPEND their own cmake/find/ directories -- so ours is searched first. cmake/FindGMP.cmake
# shadows the same way.
#
# Reporting no AVX is a path the real module already takes on a machine without it: it sets
# AVX_FLAGS to "" and its caller feeds that straight into target_compile_options(... PRIVATE ...).
# This also does not disturb ipc's FindSIMD, which probes for AVX through its own internal
# test_avx_availability() macro rather than through find_package(AVX).

message(STATUS "Third-party: FindAVX shadowed -- dependencies build for the default instruction set, not the host CPU's")

set(AVX_FOUND 0)
set(AVX_VERSION "")
set(AVX_FLAGS "")
