#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

#include <cstddef>
#include <functional>

namespace wmtk::utils {

/**
 * @file
 * Whether a cell can join a region of cells without changing the region's topology. The region X
 * is the closed union of the cells for which `in_region` holds, asked of cell ids in the mesh as
 * it is; the cell t is not one of them (`in_region` is never asked about t itself).
 *
 * The test is that X meets t in a disk of whole facets of t. X + t is then X with a ball glued on
 * along a disk of its boundary, which keeps X's homotopy type (components, tunnels, voids), and X
 * itself up to homeomorphism if it is a manifold with boundary.
 *
 * In 2D, X meets t in a disk exactly when
 *  - two corners of t are in X and the edge between them is, or
 *  - all three corners are and two edges are (one edge leaves the third corner touching X
 *    alone, which t would turn into a tunnel or a pinch; three edges fill a hole of X).
 * In 3D, exactly when
 *  - three corners are in X and one face of t is (the fourth corner is off X), or
 *  - all four corners are and three faces are, or
 *  - all four corners are, two faces are, and the edge on neither face is not.
 * Anything less (no shared facet: t would start a new piece or touch X at an edge or a corner) or
 * more (all facets: t fills a void) is refused. The domain boundary plays no part: it changes
 * nothing about the X that t meets.
 *
 * The test is sufficient, not necessary: X can meet t in something contractible that is no disk,
 * such as a facet with an edge hanging off it (3D) or a lone corner. Adding t would then keep X's
 * homotopy type, but pinch X at that edge or corner, so those are refused too.
 *
 * Only cells are read, so X's membership of a vertex or an edge is whether a cell of X holds it.
 */

/// Whether adding face fid to the region keeps the region's topology.
bool tri_consistent_topology(
    const TriMesh& m,
    size_t fid,
    const std::function<bool(size_t fid)>& in_region);

/// Whether adding tet tid to the region keeps the region's topology.
bool tet_consistent_topology(
    const TetMesh& m,
    size_t tid,
    const std::function<bool(size_t tid)>& in_region);

} // namespace wmtk::utils
