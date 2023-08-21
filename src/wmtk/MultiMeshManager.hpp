#pragma once
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributes.hpp"

namespace wmtk
{
class MultiMeshManager
{
    public:
    
    MultiMeshManager();
    ~MultiMeshManager();
    MultiMeshManager(const MultiMeshManager& o);
    MultiMeshManager(MultiMeshManager&& o);
    MultiMeshManager& operator=(const MultiMeshManager& o);
    MultiMeshManager& operator=(MultiMeshManager&& o);

    //=========================================================
    // Storage of MultiMesh
    //=========================================================

    // Child Meshes
    std::vector<std::shared_ptr<Mesh>> child_meshes;

    // size of child_meshes.size(), store the map to the base_tuple of the child_meshes (on the top level simplex of each child_mesh)
    // a map is a pair of two tuples, from a tuple in parent_mesh to a tuple in child_mesh
    // i.e. Dim(map_to_child[i]) == Dim(child_meshes[i])
    std::vector<MeshAttributeHandle<long>> map_to_child_handles;
    // store the map to the base_tuple of the parent_mesh
    MeshAttributeHandle<long> map_to_parent_handle;
    
    // register child_meshes and the map from child_meshes to this mesh, child_mesh_simplex 
    void register_child_mesh(Mesh&parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<std::array<Tuple,2>>&child_mesh_simplex_map);

    // helper function to check if this mesh is a valid child_mesh of parent_mesh
    // i.e. the connectivity of this mesh is a subset of this in parent_mesh
    bool is_child_mesh_valid(const Mesh& parent_mesh) const;

    // Map source_tuple from source_mesh to target_mesh
    Tuple map_tuple_between_meshes(const Mesh& source_mesh, const Mesh& target_mesh, const Tuple& source_tuple) const;
}

}