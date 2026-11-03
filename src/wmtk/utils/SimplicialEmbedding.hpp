#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

#include <cstddef>
#include <functional>

namespace wmtk::utils {

/**
 * @file
 * Simplicial embedding of a subcomplex K of a triangle or tet mesh. K is simplicially embedded if
 * every simplex of the mesh whose vertices all lie in K belongs to K (K is the full subcomplex on
 * its vertices). Then every simplex outside K has a vertex off K: no cell outside K has all its
 * vertices in K, and no edge outside K joins two vertices of K.
 *
 * K is given by membership tests, asked about the mesh as it is at the time. simplicial_embedding
 * splits, in this order, every cell outside K whose facets all lie in K, then (3D) every face
 * outside K whose edges all do, then every edge outside K whose two vertices do; each pass lists
 * its simplices first, then splits them. The mesh's split hooks place the new vertex and set the
 * new simplices' attributes, and the tests must count none of the new simplices in K (the new
 * vertex is inside a simplex outside K); the three passes then leave K embedded.
 *
 * topological_offset embeds its input complex (its labels), tight_seal_geom the union of its
 * regions (by tag).
 */

/// K in a triangle mesh: whether a vertex (by id), an edge or a face (by any tuple of it) is in K.
struct TriComplex
{
    std::function<bool(size_t vid)> vertex;
    std::function<bool(const TriMesh::Tuple& edge)> edge;
    std::function<bool(const TriMesh::Tuple& face)> face;
};

/// K in a tet mesh.
struct TetComplex
{
    std::function<bool(size_t vid)> vertex;
    std::function<bool(const TetMesh::Tuple& edge)> edge;
    std::function<bool(const TetMesh::Tuple& face)> face;
    std::function<bool(const TetMesh::Tuple& tet)> tet;
};

struct SimplicialEmbeddingStats
{
    size_t tets_split = 0; ///< 3D only
    size_t faces_split = 0;
    size_t edges_split = 0;
    size_t refused = 0; ///< splits the mesh refused (a split hook); K may then not be embedded
};

/// Whether a cell keeps K embedded: it lies in K, or its vertices in K are at most one, or they
/// span a simplex of K (an edge; in 3D, an edge or a face). A cell outside K with all its vertices
/// in K breaks it.
bool cell_keeps_embedding(const TriMesh& m, const TriComplex& K, const TriMesh::Tuple& face);
bool cell_keeps_embedding(const TetMesh& m, const TetComplex& K, const TetMesh::Tuple& tet);

/// The number of cells that break the embedding: 0 if K is simplicially embedded.
size_t count_embedding_violations(const TriMesh& m, const TriComplex& K);
size_t count_embedding_violations(const TetMesh& m, const TetComplex& K);

/// Makes K simplicially embedded, as above: faces outside K whose edges all lie in K are split
/// (split_face), then edges outside K whose vertices do (split_edge). Serial: the storage grows as
/// the splits need.
SimplicialEmbeddingStats simplicial_embedding(TriMesh& m, const TriComplex& K);
/// The same in 3D: tets (split_tet), then faces (split_face), then edges (split_edge).
SimplicialEmbeddingStats simplicial_embedding(TetMesh& m, const TetComplex& K);

} // namespace wmtk::utils
