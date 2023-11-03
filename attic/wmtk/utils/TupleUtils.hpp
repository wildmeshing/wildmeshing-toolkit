#pragma once

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>

namespace wmtk {
void unique_vertex_tuples(const TriMesh& m, std::vector<TriMesh::Tuple>& vertices);
void unique_face_tuples(const TriMesh& m, std::vector<TriMesh::Tuple>& vertices);
void unique_edge_tuples(const TriMesh& m, std::vector<TriMesh::Tuple>& edges);
void unique_edge_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& edges);
void unique_directed_edge_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& edges);
void unique_face_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& faces);

} // namespace wmtk
