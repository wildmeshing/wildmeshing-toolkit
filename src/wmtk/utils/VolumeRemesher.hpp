#pragma once

// VolumeRemesher's 3D API (embed_tri_in_poly_mesh, and vol_rem::TetMesh from
// delaunay3d_wrapper.h), minus the macros it leaks. Include this instead of
// <VolumeRemesher/embed.h>.
//
// Delaunay3D, which VolumeRemesher includes from embed.h on, marks its tetrahedra with
// `#define DT_UNKNOWN 0`, `DT_OUT 1` and `DT_IN 2`. glibc's <dirent.h> declares DT_UNKNOWN as
// an enumerator, so on Linux any translation unit that reaches <dirent.h> afterwards -- Eigen's
// SparseExtra does, through libigl -- expands it to `0 = 0` and fails to compile. macOS defines
// DT_UNKNOWN as the same macro, and Windows has no <dirent.h>, so only Linux breaks.
//
// Dropping the macros here is safe: the preprocessor has already expanded them in Delaunay3D's
// templates by the time the header ends, and nothing in the toolkit uses them.
#include <VolumeRemesher/embed.h>

#undef DT_UNKNOWN
#undef DT_OUT
#undef DT_IN
