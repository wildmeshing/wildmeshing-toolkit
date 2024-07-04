#pragma once
namespace wmtk {
class Mesh;
class EdgeMesh;
class TriMesh;
class TetMesh;

} // namespace wmtk

namespace wmtk::tests {
bool is_free(const Mesh& m);
bool is_free(const EdgeMesh& m);
bool is_free(const TriMesh& m);
bool is_free(const TetMesh& m);
} // namespace wmtk::tests
