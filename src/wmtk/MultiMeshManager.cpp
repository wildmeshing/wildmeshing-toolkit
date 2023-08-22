#include "MultiMeshManager.hpp"

namespace wmtk
{
    MultiMeshManager::MultiMeshManager()
        : is_parent_mesh(false)
        , child_id(-1)
    {}

    MultiMeshManager::~MultiMeshManager() = default;
    MultiMeshManager::MultiMeshManager(const MultiMeshManager& o) = default;
    MultiMeshManager::MultiMeshManager(MultiMeshManager&& o) = default;
    MultiMeshManager& MultiMeshManager::operator=(const MultiMeshManager& o) = default;
    MultiMeshManager& MultiMeshManager::operator=(MultiMeshManager&& o) = default;

    void MultiMeshManager::register_child_mesh(Mesh&parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<std::array<Tuple,2>>&child_mesh_simplex_map)
    {
        // TODO: implement this

        // register child_mesh
        child_meshes.push_back(child_mesh);
        // register map_to_child

        // register map_to_parent

        // set map_to_child

        // set map_to_parent
       
        // update child_id
    }

    bool MultiMeshManager::is_child_mesh_valid(const Mesh& parent_mesh) const
    {
        // TODO: implement this
        return true;
    }
}