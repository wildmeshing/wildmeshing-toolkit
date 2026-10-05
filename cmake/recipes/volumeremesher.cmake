# VolumeMesher (From Marco Attene)

if(TARGET VolumeRemesher::VolumeRemesher)
    return()
endif()

message(STATUS "Third-party: creating target 'VolumeRemesher::VolumeRemesher'")

# wildmeshing/VolumeRemesher main. Carries the 2D pipeline
# (vol_rem::embed_seg_in_tri_mesh in VolumeRemesher/2d/embed2d.h, used by triwild to
# insert its input segments) and fetches and pins its exact-arithmetic and Delaunay
# dependencies itself: NFG (number types), Indirect_Predicates (predicates) and, since
# PR #26, Delaunay3D (the 3D tetrahedrization, which used to be built in).
#
# All three define everything inside their own namespaces -- NFG, IPs and Del3D -- and
# VolumeRemesher includes them unwrapped, so the number type its API returns is
# `NFG::bigrational` and its Delaunay mesh is `vol_rem::TetMesh`, a typedef for
# `Del3D::TetMesh_t<basicVec3d>` (VolumeRemesher/delaunay3d_wrapper.h). fast-envelope
# fetches NFG and Indirect_Predicates under the same names; FetchContent keeps the first
# declaration, which is VolumeRemesher's, so the two pins must stay identical (see
# fenvelope.cmake).
#
# Exact-arithmetic backend: VOLUMEREMESHER_WITH_GMP defaults to OFF, so
# `NFG::bigrational` is upstream's built-in bignum rather than mpq_class and
# USE_GNU_GMP_CLASSES is not defined. That selects the `init_from_bin(get_str())`
# branch of the arrangement-vertex conversions in tetwild, simwild and triwild, which
# is exact: the built-in bigrational::get_str() emits the fraction in base 2, the base
# init_from_bin parses.
#
# Pinned at main, 75a70dc. Since the previous pin (609e32c4):
#
#   - PR #26 replaced the built-in 3D Delaunay with Delaunay3D. It produces a different,
#     equally valid tetrahedrization, so every 3D output moves -- tetwild and simwild here,
#     and all 28 of VolumeRemesher's own 3D reference hashes. The 2D pipeline has its own
#     kernel and is unaffected. `vertex_t::original_index` is gone: Delaunay3D does not
#     permute the vertices it is given (src/wmtk/utils/Delaunay.cpp).
#   - NFG, Indirect_Predicates and Delaunay3D moved into namespaces, and
#     include/VolumeRemesher/{numerics,implicit_point,indirect_predicates}.h -- the shims
#     that wrapped them in vol_rem -- were deleted.
#   - Indirect_Predicates is back on MarcoAttene upstream: the fix the wildmeshing fork
#     carried is upstream #15, merged.
#
# Before that (64c52aa5 -> 609e32c4) came VolumeRemesher PR #25, which stores cached
# orient3D results in a `signed char` rather than a `char`. That is a correctness fix for Linux on arm64, where
# `char` is unsigned (AAPCS64) and a cached -1 read back as 255: every constraint was then
# judged not to split its cell and the input surface was silently never embedded. It has no
# effect on x86-64 or on macOS, where `char` is already signed.
#
# Before that (ba8a7329 -> 64c52aa5) came PR #24, which added an output to
# embed_tri_in_poly_mesh -- out_triangle_group, mapping each input triangle to its coplanar
# group, i.e. to its index into out_triangle_provenance. Nothing existing changed behaviour,
# but the signature grew, so every caller of that function had to be updated in the same
# commit.
#
# On MSVC, VolumeRemesher is built WITHOUT /arch:AVX2. NFG picks the storage of
# `NFG::interval_number` by preprocessor: `__m128d interval` (high, then min_low) when
# __SSE2__ or __AVX2__ is defined, `double min_low, high` otherwise -- the same two bounds
# in the opposite order. GCC and Clang define __SSE2__ on every x86-64 target, so all
# translation units agree there. MSVC never defines __SSE2__, only __AVX2__ under
# /arch:AVX2, which VolumeRemesher sets and this file strips from its consumers (below). So
# VolumeRemesher's translation units got the SIMD layout while fast-envelope's and the
# toolkit's got the scalar one, under one mangled name now that NFG is a namespace of its
# own rather than wrapped in vol_rem. The linker keeps one copy of each inline member, so
# intervals built by one side are read with their bounds swapped by the other: the
# interval filters report wrong signs as certain. Symptom on Windows Release only: tetwild's
# splits rejected as "produced a surface segment outside the envelope" in an endless retry
# loop, and split max energy around 1e102. Without /arch:AVX2 every copy is scalar.
# VolumeRemesher documents its scalar path as byte-identical to the AVX2 one.
set(WMTK_VOLREM_OPTIONS "VOLUMEREMESHER_BUILD_TESTS OFF")
if(MSVC)
    list(APPEND WMTK_VOLREM_OPTIONS "VOLREM_ENABLE_AVX2 OFF")
endif()
include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 75a70dc79a13a5de41369ce899134974665cd4b5
    OPTIONS
    ${WMTK_VOLREM_OPTIONS}
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