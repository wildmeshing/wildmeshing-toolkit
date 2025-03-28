#pragma once

#include <memory>

namespace wmtk {
    class TriMesh;
    class Mesh;
    class EdgeMesh;
namespace components {

struct EdgeInsertionMeshes
{
    std::shared_ptr<wmtk::Mesh> tri_mesh;
    std::shared_ptr<wmtk::Mesh> inserted_edge_mesh;
    std::shared_ptr<wmtk::Mesh> bbox_mesh;
};

EdgeInsertionMeshes edge_insertion(EdgeMesh& input_mesh, TriMesh& bg_mesh);

} // namespace components
} // namespace wmtk
