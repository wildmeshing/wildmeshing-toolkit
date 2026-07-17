# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeMesher'")

include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 9e32fe8329890d564e179ed3176f266d8f519f12
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