# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

# PR #15 (branch `2d`), a descendant of the previous pin, which adds the 2D pipeline:
# vol_rem::embed_seg_in_tri_mesh in VolumeRemesher/2d/embed2d.h, used by triwild to
# insert its input segments. The 3D entry point is unchanged.
#
# Exact-arithmetic backend: VOLUMEREMESHER_WITH_GMP defaults to OFF, so
# `vol_rem::bigrational` is upstream's built-in bignum rather than mpq_class and
# USE_GNU_GMP_CLASSES is not defined. That selects the `init_from_bin(get_str())`
# branch of the arrangement-vertex conversions in tetwild, simwild and triwild, which
# is exact: the built-in bigrational::get_str() emits the fraction in base 2, the base
# init_from_bin parses.
include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 8077213e53022549ac66f3d9befdf6f0bd6c404c
    OPTIONS
    "VOLUMEREMESHER_BUILD_TESTS OFF"
    )

set_target_properties(mesh_generator_lib PROPERTIES FOLDER third-party)

# VolumeRemesher marks its SIMD flags (-mavx2/-mfma on GCC/Clang, /arch:AVX2 on
# MSVC) as PUBLIC, so they propagate to every target that links it. Mixing AVX and
# non-AVX translation units gives Eigen inconsistent vector alignment (32 vs 16
# bytes) across the binary -- an ODR violation that leads to a misaligned AVX access
# and a crash in unrelated Eigen code (e.g. polysolve's dense LDLT during simwild
# smoothing). Keep the flags for VR's own compilation, but stop propagating them.
get_target_property(_vr_iface_opts mesh_generator_lib INTERFACE_COMPILE_OPTIONS)
if(_vr_iface_opts)
    list(REMOVE_ITEM _vr_iface_opts "-mavx2" "-mfma" "-msse2" "/arch:AVX2")
    set_target_properties(mesh_generator_lib PROPERTIES INTERFACE_COMPILE_OPTIONS "${_vr_iface_opts}")
endif()