#pragma once

#include <wmtk/TetMesh.h>

namespace wmtk {
void unique_edge_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& edges);
void unique_directed_edge_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& edges);
void unique_face_tuples(const TetMesh& m, std::vector<TetMesh::Tuple>& faces);

} // namespace wmtk
