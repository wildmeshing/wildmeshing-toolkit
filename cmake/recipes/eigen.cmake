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
# the same way. EIGEN_MAX_ALIGN_BYTES, the heap's alignment, is set below, the same in every TU too.
#
target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_MAX_STATIC_ALIGN_BYTES=16)

# THE HEAP TOO, ON X86-64. How Eigen allocates a dynamic matrix is a third per-unit setting:
# aligned_malloc() takes plain malloc() when EIGEN_MALLOC_ALREADY_ALIGNED, and otherwise its own
# over-allocating handmade_aligned_malloc(), which returns a pointer up to one alignment inside the
# block and frees it with handmade_aligned_free(). Memory.h takes malloc() where
# EIGEN_DEFAULT_ALIGN_BYTES == 16 (on glibc, macOS and Win64; not under ASan on glibc), and
# ConfigureVectorization.h sets that to the larger of EIGEN_MAX_ALIGN_BYTES and an inferred ideal:
# 16, 32 or 64 where the unit sees AVX or AVX-512, 0 under EIGEN_DONT_VECTORIZE. Without AVX every
# unit gets 16 and malloc(). With it -- Ubuntu 26.04's amd64v3 GCC targets x86-64-v3 by default,
# and that is GitHub's ubuntu-latest -- the targets that link ipc-toolkit got 16 and malloc(), and
# the others 32 and handmade_aligned_malloc(): a matrix isotropic_remeshing returns on the memory
# route, built in its own unit, was freed by the Python module's with free(), 16 or 32 bytes into
# the block ("double free or corruption"), and 21 of the unit tests, the memory-route ones and
# wmtk_test_topological_offset, aborted the same way. Eigen 3.4.1 and 5.0.1 choose the allocator
# the same way, so a newer Eigen does not fix it.
#
# Setting only EIGEN_MALLOC_ALREADY_ALIGNED=0 sends every unit to handmade_aligned_malloc(), which
# fixes the free but not the alignment: handmade_aligned_malloc() and handmade_aligned_realloc()
# align by the calling unit's EIGEN_DEFAULT_ALIGN_BYTES, and where they are not inlined the linker
# keeps one copy for the whole binary. So code built for AVX could be handed a 16-byte block it
# reads with aligned loads, and a conservativeResize() could drop the last 16 bytes of a matrix.
#
# So on x86-64 EIGEN_MAX_ALIGN_BYTES is 32, what a unit that sees AVX infers, and 64 where the
# compiler targets AVX-512. EIGEN_DEFAULT_ALIGN_BYTES, the larger of it and the ideal, is then the
# same in every unit -- ipc-linked, plain or AVX -- and never 16, so every unit allocates, frees and
# reallocates with handmade_aligned_malloc() at the same alignment. It is set on every x86-64 build
# rather than where a check sees the compiler target AVX, because no check sees everything that
# reaches a unit (a parent project's add_compile_options, a consumer's target_compile_options, one
# slice of a universal build), and a single unit it missed would split the heap again. The cost
# of handmade_aligned_malloc() over malloc() in every unit, measured by forcing it on macOS arm64:
# over the 48 serial integration configs every output is byte-identical, the summed time 0.998x,
# each config within its own run-to-run spread, and peak memory within 10 MB either way. Elsewhere
# no unit infers more than 16, so it stays 16 and malloc(). With this file's EIGEN_DONT_VECTORIZE
# option on, no unit infers more than 0, so it stays 16 too.
#
# AVX-512 is seen only through the compiler's default and CMAKE_CXX_FLAGS[_<CONFIG>]. A unit built
# for it some other way still frees and reallocates like the rest, all of them being on
# handmade_aligned_malloc(), but may be handed 32-byte blocks where it assumes 64.
set(_wmtk_eigen_heap_align 16)
if(NOT EIGEN_DONT_VECTORIZE)
    include(CheckCXXSourceCompiles)
    block(PROPAGATE _wmtk_eigen_heap_align)
        # Compile only: a toolchain that cannot link a test executable says nothing about its
        # target. Each check is written to fail where any slice of the build has the property, so a
        # universal build counts as x86-64, and is re-run on every configure, as the flags may have
        # changed since the last.
        set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
        unset(WMTK_EIGEN_NOT_X86 CACHE)
        check_cxx_source_compiles("
            #if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
            #error the build targets x86
            #endif
            int f() { return 0; }" WMTK_EIGEN_NOT_X86)
        if(NOT WMTK_EIGEN_NOT_X86)
            set(_wmtk_eigen_heap_align 32)
            get_property(_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
            if(_multi_config)
                set(_configs ${CMAKE_CONFIGURATION_TYPES})
            elseif(CMAKE_BUILD_TYPE)
                set(_configs ${CMAKE_BUILD_TYPE})
            else()
                # No build type: CMAKE_CXX_FLAGS alone, as no CMAKE_CXX_FLAGS_NONE is set.
                set(_configs None)
            endif()
            foreach(_config IN LISTS _configs)
                string(TOUPPER "${_config}" _CONFIG)
                set(CMAKE_TRY_COMPILE_CONFIGURATION "${_config}")
                unset(WMTK_EIGEN_NOT_AVX512_${_CONFIG} CACHE)
                check_cxx_source_compiles("
                    #ifdef __AVX512F__
                    #error the build targets AVX-512
                    #endif
                    int f() { return 0; }" WMTK_EIGEN_NOT_AVX512_${_CONFIG})
                if(NOT WMTK_EIGEN_NOT_AVX512_${_CONFIG})
                    set(_wmtk_eigen_heap_align 64)
                endif()
            endforeach()
        endif()
    endblock()
endif()
if(NOT _wmtk_eigen_heap_align EQUAL 16)
    message(STATUS "Third-party: x86 target -- Eigen allocates by handmade_aligned_malloc() at "
        "${_wmtk_eigen_heap_align} bytes")
endif()
target_compile_definitions(Eigen3_Eigen INTERFACE EIGEN_MAX_ALIGN_BYTES=${_wmtk_eigen_heap_align})

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