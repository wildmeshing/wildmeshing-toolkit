#pragma once

#include <string>
#include <tuple>
#include <memory>
#include <vector>

namespace wmtk {
    class Mesh;
    class TriMesh;
    class TetMesh;
    namespace attribute {
        class MeshAttributeHandle;
    }
}

namespace wmtk::components::triangle_insertion {

class ChildMeshes
{
public:
    std::shared_ptr<Mesh> surface_mesh = nullptr;
    std::shared_ptr<Mesh> open_boundary_mesh = nullptr;
    std::shared_ptr<Mesh> nonmanifold_edge_mesh = nullptr;
    std::shared_ptr<Mesh> bbox_mesh = nullptr;
};

std::tuple<std::shared_ptr<wmtk::TetMesh>, ChildMeshes> triangle_insertion(
    const TetMesh& bg_mesh,
    const std::string& bg_position,
    const TriMesh& mesh_in,
    const std::string& in_position,
    std::vector<attribute::MeshAttributeHandle>& pass_through,
    bool round = true,
    bool track_submeshes = true,
    bool make_child_free = false);

} // namespace wmtk::components::triangle_insertion
