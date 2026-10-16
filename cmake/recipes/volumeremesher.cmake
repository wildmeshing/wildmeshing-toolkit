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
# init_from_bin parses. init_from_bin is mpq_set_str with no mpq_canonicalize, so it relies
# on VolumeRemesher returning every coordinate in lowest terms (GMP requires canonical
# operands; see PR #27 below).
#
# Pinned at main, 846eaa0. Since the previous pin (fc72cc0) came PRs #29 and #30:
#
#   - PR #29: embed_tri_in_poly_mesh frees the BSPcomplex it builds. It used to leak it, so
#     the whole arrangement stayed allocated for the rest of the run: about 1 GB of tetwild's
#     3.1 GB peak on Thingi10K 46024, 400 MB on 1017020.
#   - PR #30 pins NFG to wildmeshing/NFG#1 (05f99ea), which is 9b7635a plus
#     bignatural::trimMemoryPool(). NFG's thread-local bignatural pool only grows, so the
#     arrangement's high-water mark stayed allocated too (171 MB on 46024, 52 MB of a 245 MB
#     2D peak on 193153); embed_triangles_in_tets and embed_segments now return it once the
#     coordinates are converted. fast-envelope #11 pins the same NFG commit.
#
# Neither changes any output.
#
# Before that (48b200f -> fc72cc0) came PR #28, which pins NFG to
# upstream 9b7635a -- MarcoAttene/NFG#4 merged, see below -- so NFG and Indirect_Predicates
# are upstream's latest, and VolumeRemesher and fast-envelope declare identical commits of
# both. Nothing changes on GCC or Clang.
#
# Before that (75a70dc -> 48b200f) came PR #27: the exact
# coordinates returned by embed_tri_in_poly_mesh and embed_seg_in_tri_mesh are in lowest
# terms again, and are computed in parallel. NFG ecd60a8 (pinned since 75a70dc) stopped
# reducing bigrational products, so at 75a70dc those functions returned x = lx / d
# unreduced (3/6, not 1/2), and Rational's operator== -- mpq_equal, which compares
# numerator and denominator directly -- said false for equal values. That changed tetwild's
# output on some inputs: 2 of the 11 registered tetwild configs (thingi_1344050,
# thingi_229953) and several of the challenging ones. Wherever the output moved with #27, it
# is byte-identical to what canonicalizing every coordinate on this side gives, and every
# other output we measured is unchanged.
#
# Before that (609e32c4 -> 75a70dc):
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
# NFG#4 is why the NFG pin matters on Windows. NFG picks how `NFG::interval_number` stores
# its bounds by preprocessor: as `__m128d interval` (high, then min_low) when it detects SSE2
# or AVX2, as `double min_low, high` otherwise -- the same two values in the opposite order.
# Before #4 it detected SSE2 with __SSE2__ alone, which MSVC never defines, so on Windows
# VolumeRemesher's translation units (built with /arch:AVX2, which defines __AVX2__) got the
# SIMD layout while fast-envelope's and the toolkit's (built without it, since this file
# strips it from consumers) got the scalar one, and since all three share NFG:: the linker
# kept one copy of each inline member: intervals built on one side were read with swapped
# bounds on the other. Symptom, Windows Release only: tetwild splits rejected as "produced a
# surface segment outside the envelope" in an endless retry loop, split max energy around
# 1e102. #4 also accepts _M_X64 as SSE2, so every MSVC x64 translation unit gets the SIMD
# layout. Until it merged, this file declared `nfg` from a wildmeshing fork carrying it, ahead
# of VolumeRemesher, to override VolumeRemesher's and fast-envelope's pins.
#
# #31 (merged as 9e74ca7) fixes forced points and edges (tetwild's input_edges / input_points): a
# forced vertex at an input vertex's position was mapped to the background's coincident copy,
# which insertion evicts from every tet (stack-buffer overflow in
# intersections_TetVrtOnConstraintSide), and forcing-triangle corners could land outside the
# background mesh. It also pins NFG a35198d and Delaunay3D c9b3eec (wildmeshing forks).
# FetchContent keeps VolumeRemesher's NFG declaration, so fast-envelope builds against a35198d
# too; its own recipe still names the older commit until fast-envelope follows.
#
# Then #32 and #33. Neither changes any output here: all 50 registered configs are
# byte-identical across them.
#
#   - #32 (c8329b3) adds the Python bindings: the CLI's writers are split into in-memory
#     snapshots (BSPcomplex::extractSurface/extractVolume), parallel_blocks rethrows a worker's
#     exception on the calling thread, and NFG moves to 2330267 -- upstream main
#     (MarcoAttene/NFG#6 merged) plus MarcoAttene/NFG#7, under which ip_error throws when
#     NFG_IP_ERROR_THROWS is defined. Nothing here defines it, and embed_tri_in_poly_mesh /
#     embed_seg_in_tri_mesh call none of the rewritten writers. fast-envelope builds against
#     2330267 too (VolumeRemesher's declaration wins; its own recipe still names 05f99ea).
#   - #33 (57e7753) moves Delaunay3D to upstream main e18e96a, which inserts the vertices of
#     tetrahedrize() in Morton order above 1150 of them, plus MarcoAttene/Delaunay3D#2 and #3
#     (wildmeshing/Delaunay3D f1c7c19); #3 breaks ties between points in the same Morton cell
#     by index, so the insertion order does not depend on the standard library's sort. It
#     changes the order the tets are created in, not which tets are created: symbolic
#     perturbation makes the set order-independent. VolumeRemesher's embedding never calls
#     tetrahedrize() (it inserts into our background mesh), and our own delaunay3D -- the
#     tetwild/simwild background grid, src/wmtk/utils/Delaunay.cpp -- sorts the tets it
#     returns, so neither sees the new order. Measured on crown, octocat and thingi 36075, the
#     background-grid Delaunay is 10-30 ms faster.
include(CPM)
CPMAddPackage(
    NAME VolumeRemesher
    GITHUB_REPOSITORY wildmeshing/VolumeRemesher
    GIT_TAG 57e7753521c98554129fd08545a5b3458228e5ba
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