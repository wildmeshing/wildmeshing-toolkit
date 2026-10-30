# Copyright 2020 Adobe. All rights reserved.
# This file is licensed to you under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License. You may obtain a copy
# of the License at http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software distributed under
# the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR REPRESENTATIONS
# OF ANY KIND, either express or implied. See the License for the specific language
# governing permissions and limitations under the License.
#

# Eigen (https://gitlab.com/libeigen/eigen)
# License: MPL2

if(TARGET Eigen3::Eigen)
    return()
endif()

option(EIGEN_WITH_MKL "Use Eigen with MKL" OFF)
option(EIGEN_DONT_VECTORIZE "Disable Eigen vectorization" OFF)
option(EIGEN_MPL2_ONLY "Enable Eigen MPL2 license only" OFF)

message(STATUS "Third-party: creating target 'Eigen3::Eigen'")

include(CPM)
# From Google's mirror of the Eigen repository, not gitlab.com: GitLab refuses anonymous clones of
# libeigen/eigen when it is under load ("GitLab is currently unable to handle this request due to
# load"), which failed CI's configure step on every platform in October 2026. The mirror's 3.4.0 tag
# is the same commit, 3147391d946bb4b6c68edd901f2add6ac1f31f8c.
CPMAddPackage(
    NAME eigen
    GIT_REPOSITORY https://eigen.googlesource.com/mirror
    GIT_TAG 3.4.0
    DOWNLOAD_ONLY TRUE
)

add_library(Eigen3_Eigen INTERFACE)
add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)

include(GNUInstallDirs)
target_include_directories(Eigen3_Eigen SYSTEM INTERFACE
    $<BUILD_INTERFACE:${eigen_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

if(EIGEN_MPL2_ONLY)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_MPL2_ONLY)
endif()

if(EIGEN_DONT_VECTORIZE)
  target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_DONT_VECTORIZE)
endif()

# PIN EIGEN'S ABI FOR THE WHOLE BUILD. Not a tuning knob -- a correctness requirement.
#
# Eigen's alignment is normally inferred per translation unit, and EIGEN_MAX_STATIC_ALIGN_BYTES
# is part of the ABI: it sets alignof() for every fixed-size type whose byte size is a multiple
# of 16 (Vector2d, Vector4d, Matrix2d, Matrix4d, and wmtk's own Vector2r/Vector3r, since an
# mpq_t is 32 bytes). Two translation units that infer it differently disagree about the layout
# of those types while agreeing on their mangled names, so the linker merges inline definitions
# from both worlds arbitrarily and a caller built for 8-byte alignment hands its stack slot to a
# callee built for 16.
#
# That is not hypothetical here. ipc-toolkit exports EIGEN_DONT_VECTORIZE PUBLIC (see
# recipes/ipc_toolkit.cmake), which zeroes the inferred alignment -- but only for the targets
# that link it. Core wildmeshing_toolkit does not link it and kept 16. The result was Eigen's
# unaligned-array assert aborting wmtk_test_manifold_extraction and wmtk_test_topological_offset
# on every platform in both Debug and Release, inside to_rational(), a function neither the
# offset nor ipc has anything to do with.
#
# Setting the two macros explicitly wins over the inference (ConfigureVectorization.h guards its
# default with #ifndef), so every TU gets the same layout no matter what else it defines. 16 is
# the value every wmtk platform already inferred, so this pins the status quo rather than
# changing it: alignment only, never arithmetic, so no result moves. It also decouples the
# layouts from -mavx, which would otherwise raise a subset of targets to 32 and split the build
# the same way.
#
target_compile_definitions(Eigen3_Eigen INTERFACE
    EIGEN_MAX_ALIGN_BYTES=16
    EIGEN_MAX_STATIC_ALIGN_BYTES=16
)

# THE HEAP TOO, WHERE THE COMPILER TARGETS AVX. How Eigen allocates a dynamic matrix is a third
# per-unit setting, which the two above do not fix: aligned_malloc() takes plain malloc() when
# EIGEN_MALLOC_ALREADY_ALIGNED, and otherwise its own over-allocating handmade_aligned_malloc(),
# which returns a pointer up to one alignment inside the block and frees it with
# handmade_aligned_free(). Memory.h takes malloc() where EIGEN_DEFAULT_ALIGN_BYTES == 16 (on glibc,
# macOS and Win64, not under ASan), and ConfigureVectorization.h sets that to the larger of
# EIGEN_MAX_ALIGN_BYTES and an inferred ideal: 16, 32 or 64 where the unit sees AVX or AVX-512, 0
# under EIGEN_DONT_VECTORIZE. Without AVX every unit gets 16 and malloc(). With it -- Ubuntu
# 26.04's amd64v3 GCC targets x86-64-v3 by default, and that is GitHub's ubuntu-latest -- the
# targets that link ipc-toolkit get 16 and malloc(), and the others 32 and
# handmade_aligned_malloc(): a matrix isotropic_remeshing returns on the memory route, built in its
# own unit, was freed by the Python module's with free(), 16 or 32 bytes into the block ("double
# free or corruption"), and 21 of the unit tests, the memory-route ones and
# wmtk_test_topological_offset, aborted the same way. An ASan build hides it, as Memory.h skips
# malloc() under __SANITIZE_ADDRESS__ in every unit.
#
# So where the compiler targets AVX, EIGEN_MALLOC_ALREADY_ALIGNED=0 sends every unit to
# handmade_aligned_malloc(), which stores each block's start: a block allocated in one unit is
# freed in any other. Elsewhere it is left unset, since there it would only cost: on Linux x86-64,
# where every unit had used malloc(), it made the two tetwild tests 5% and 8% slower. One gap
# remains, in Eigen: handmade_aligned_realloc() re-aligns by the calling unit's
# EIGEN_DEFAULT_ALIGN_BYTES, so a conservativeResize() in an ipc-linked unit of a matrix allocated
# in another can lose its last 16 bytes. Closing that takes EIGEN_MAX_ALIGN_BYTES=32, which changes
# what every unit assumes of every dynamic matrix.
#
# The check sees what reaches every unit: the compiler's default and each configuration's
# CMAKE_CXX_FLAGS. No dependency adds -mavx to a unit that includes Eigen: ipc-toolkit's SIMD flags
# are stripped (recipes/ipc_toolkit.cmake), Tight-Inclusion's FindAVX is shadowed
# (cmake/FindAVX.cmake), and VolumeRemesher's -mavx2 stays in its own units, which do not. With
# this file's EIGEN_DONT_VECTORIZE option on, no unit infers more than 16, so it is skipped.
if(NOT EIGEN_DONT_VECTORIZE)
    include(CheckCXXSourceCompiles)
    set(_wmtk_eigen_avx OFF)
    block(PROPAGATE _wmtk_eigen_avx)
        if(CMAKE_CONFIGURATION_TYPES)
            set(_configs ${CMAKE_CONFIGURATION_TYPES})
        else()
            set(_configs "${CMAKE_BUILD_TYPE}")
        endif()
        foreach(_config IN LISTS _configs)
            string(TOUPPER "${_config}" _CONFIG)
            set(CMAKE_TRY_COMPILE_CONFIGURATION "${_config}")
            check_cxx_source_compiles("
                #ifndef __AVX__
                #error the compiler does not target AVX
                #endif
                int main() { return 0; }" WMTK_EIGEN_TARGETS_AVX_${_CONFIG})
            if(WMTK_EIGEN_TARGETS_AVX_${_CONFIG})
                set(_wmtk_eigen_avx ON)
            endif()
        endforeach()
    endblock()
    if(_wmtk_eigen_avx)
        message(STATUS "Third-party: AVX target -- Eigen allocates by handmade_aligned_malloc()")
        target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_MALLOC_ALREADY_ALIGNED=0)
    endif()
endif()

if(EIGEN_WITH_MKL)
    # TODO: Checks that, on 64bits systems, `mkl::mkl` is using the LP64 interface
    # (by looking at the compile definition of the target)
    include(mkl)
    target_link_libraries(Eigen3_Eigen INTERFACE mkl::mkl)
    target_compile_definitions(Eigen3_Eigen INTERFACE
        EIGEN_USE_MKL_ALL
        EIGEN_USE_LAPACKE_STRICT
    )
endif()

# On Windows, enable natvis files to improve debugging experience
if(WIN32 AND eigen_SOURCE_DIR)
    target_sources(Eigen3_Eigen INTERFACE $<BUILD_INTERFACE:${eigen_SOURCE_DIR}/debug/msvc/eigen.natvis>)
endif()

# Install rules
set(CMAKE_INSTALL_DEFAULT_COMPONENT_NAME eigen)
set_target_properties(Eigen3_Eigen PROPERTIES EXPORT_NAME Eigen)
install(DIRECTORY ${eigen_SOURCE_DIR} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS Eigen3_Eigen EXPORT Eigen_Targets)
install(EXPORT Eigen_Targets DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/eigen NAMESPACE Eigen3::)