#pragma once
#include <memory>
#include <wmtk/PrimitiveType.hpp>
namespace wmtk {
class Mesh;
class EdgeMesh;
class TriMesh;
class TetMesh;

} // namespace wmtk

namespace wmtk::tests {
    std::shared_ptr<Mesh> add_free_child_mesh(Mesh& m, PrimitiveType pt);
} // namespace wmtk::tests
