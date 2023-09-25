#pragma once
#include <spdlog/spdlog.h>
#include <tuple>
#include "Simplex.hpp"
#include "Tuple.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributes.hpp"

namespace wmtk {
class Mesh;
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

    bool is_root() const;
    long child_id() const;
    //
    std::vector<long> absolute_id() const;


    // register child_meshes and the map from child_meshes to this mesh, child_mesh_simplex
    void register_child_mesh(
        Mesh& my_mesh,
        const std::shared_ptr<Mesh>& child_mesh,
        const std::vector<std::array<Tuple, 2>>& child_mesh_simplex_map);


    bool are_maps_valid(const Mesh& my_mesh) const;

    std::vector<Tuple> find_all_simplices_in_child_mesh(
        const Mesh& my_mesh,
        const Mesh& child_mesh,
        const Simplex& simplex_parent);

    std::vector<Tuple> find_all_simplices_in_child_mesh(
        const Mesh& my_mesh,
        const Mesh& child_mesh,
        long child_id,
        const Simplex& simplex_parent);


    // Utility function to map a edge tuple to all its children, used in operations
    std::vector<Tuple> map_edge_tuple_to_all_children(const Mesh& my_mesh, const Simplex& tuple)
        const;

private:
    MeshAttributeHandle<long> std::vector<Simplex> find_all_simplices_in_child_mesh(
        const Mesh& my_mesh,
        const ChildData& child_data,
        const Simplex& simplex) const;

    // helper function to check if this mesh is a valid child_mesh of my_mesh
    // i.e. the connectivity of this mesh is a subset of this in my_mesh
    bool is_child_mesh_valid(const Mesh& my_mesh, const Mesh& child_mesh) const;

    // checks that the map is consistent
    bool is_child_map_valid(const Mesh& my_mesh, const ChildData& child) const;

private:
    Mesh* m_parent = nullptr;
    // only valid if this is teh child of some other mesh
    // store the map to the base_tuple of the my_mesh
    MeshAttributeHandle<long> map_to_parent_handle;
    long m_child_id = -1;

    struct ChildData
    {
        std::shared_ptr<Mesh> mesh;
        // store the map from the manager's mesh to the child mesh (on the top
        // level simplex of the mesh)
        // encoded by a pair of two tuples, from a tuple in current mesh to a tuple in
        // child_mesh
        MeshAttributeHandle<long> map_handle;
    };

    // Child Meshes
    std::vector<ChildData> m_children;
};

} // namespace wmtk
