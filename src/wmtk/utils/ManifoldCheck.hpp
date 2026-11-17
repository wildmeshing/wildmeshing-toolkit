#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

#include <cstddef>
#include <functional>
#include <vector>

namespace wmtk::utils {

/**
 * @file
 * Manifold checks of a pure subcomplex K of a triangle or tet mesh: the cells (faces in 2D, tets
 * in 3D) for which a test holds, by id, with all their simplices. A vertex or an edge is manifold
 * in K if K is a manifold with boundary near it: near a vertex, K is a disk (2D) or a ball (3D),
 * whole or cut by K's boundary; near an edge (3D), K's tets around it form a single wedge. A
 * vertex or an edge on no cell of K is manifold. The space outside the mesh counts as outside K.
 *
 * The tests are combinatorial: K's cells around the simplex are joined through the facets they
 * share there, and K is manifold at it if they form one group (3D vertices: and so do the cells
 * outside K, the space outside the mesh counted as one of them).
 *
 * manifold_extraction labels the non-manifold vertices and edges of the selected tets'
 * boundary; tight_seal_geom compares each tag's non-manifold vertices before and after its fill.
 */

/// Whether the cell with this id is in K.
using CellTest = std::function<bool(size_t)>;

/// 2D: whether vertex v is manifold in K: K's faces around v form one fan, joined through the
/// edges at v.
bool vertex_is_manifold(const TriMesh& m, const TriMesh::Tuple& v, const CellTest& in_K);

/// 3D: whether edge e is manifold in K: K's tets around e form one wedge, joined through the
/// faces holding e.
bool edge_is_manifold(const TetMesh& m, const TetMesh::Tuple& e, const CellTest& in_K);

/// 3D: whether vertex v is manifold in K: K's tets around v are joined through the faces at v,
/// and so are the tets around v outside K, together with the space outside the mesh, which lies
/// beyond each of v's faces on the mesh's boundary. Equivalently, K's link of v (the faces
/// opposite v in K's tets) is a disk or the whole sphere around v. A vertex of an edge that is
/// not manifold is not manifold either.
bool vertex_is_manifold(const TetMesh& m, const TetMesh::Tuple& v, const CellTest& in_K);

/// The vertices of `cells` (ids) that are not manifold in K, ascending; in 3D with both ends of
/// each edge of `cells` that is not. Every non-manifold vertex and edge of K lies on a cell of K,
/// so with K's cells as `cells` this gives all of K's non-manifold vertices; any other cells give
/// those among their own vertices.
std::vector<size_t>
nonmanifold_vertices(const TriMesh& m, const std::vector<size_t>& cells, const CellTest& in_K);
std::vector<size_t>
nonmanifold_vertices(const TetMesh& m, const std::vector<size_t>& cells, const CellTest& in_K);

} // namespace wmtk::utils
