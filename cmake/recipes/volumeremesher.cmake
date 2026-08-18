# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeRemesher::VolumeRemesher'")

# wildmeshing/VolumeRemesher main. Carries the 2D pipeline
# (vol_rem::embed_seg_in_tri_mesh in VolumeRemesher/2d/embed2d.h, used by triwild to
# insert its input segments) plus, since PR #19, an externalized exact-arithmetic
# kernel: numerics.h and the predicate headers are now thin shims over NFG and
# Indirect_Predicates, fetched and pinned by VolumeRemesher itself. Those upstream
# headers live in the global namespace; the shims re-export the public types into
# `vol_rem` with using-declarations, so the include paths and the API this project
# depends on are unchanged. Note the build now fetches two more repositories.
#
# Exact-arithmetic backend: VOLUMEREMESHER_WITH_GMP defaults to OFF, so
# `vol_rem::bigrational` is upstream's built-in bignum rather than mpq_class and
# USE_GNU_GMP_CLASSES is not defined. That selects the `init_from_bin(get_str())`
# branch of the arrangement-vertex conversions in tetwild, simwild and triwild, which
# is exact: the built-in bigrational::get_str() emits the fraction in base 2, the base
# init_from_bin parses.
#
# Pinned at main. The previous pin (64c52aa5) is this one's first parent; the only change
# between them is VolumeRemesher PR #25, which stores cached orient3D results in a
# `signed char` rather than a `char`. That is a correctness fix for Linux on arm64, where
# `char` is unsigned (AAPCS64) and a cached -1 read back as 255: every constraint was then
# judged not to split its cell and the input surface was silently never embedded. It has no
# effect on x86-64 or on macOS, where `char` is already signed.
#
# Before that (ba8a7329 -> 64c52aa5) came PR #24, which added an output to
# embed_tri_in_poly_mesh -- out_triangle_group, mapping each input triangle to its coplanar
# group, i.e. to its index into out_triangle_provenance. Nothing existing changed behaviour,
# but the signature grew, so every caller of that function had to be updated in the same
# commit.
include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 609e32c43a52336f087c608ce5f1bd73b41e5845
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