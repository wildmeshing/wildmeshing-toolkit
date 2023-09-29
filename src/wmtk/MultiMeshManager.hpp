#pragma once
#include <tuple>
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributes.hpp"
#include "Simplex.hpp"
#include "Tuple.hpp"
#include <spdlog/spdlog.h>

namespace wmtk
{
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

    bool is_parent_mesh() const;
    long child_id() const;
    // helper function to read/write the map from source_tuple to target_tuple to the attribute
    static void write_tuple_map_attribute(MeshAttributeHandle<long> map_handle, Mesh& source_mesh, const Tuple& source_tuple, const Tuple& target_tuple);
    static std::tuple<Tuple, Tuple> read_tuple_map_attribute(MeshAttributeHandle<long> map_handle, const Mesh& source_mesh, const Tuple& source_tuple);

    // Child Meshes
    std::vector<std::shared_ptr<Mesh>> child_meshes;

    // Only valid if this is_parent_mesh == true
    // size of child_meshes.size(), store the map to the base_tuple of the child_meshes (on the top level simplex of each child_mesh)
    // a map is a pair of two tuples, from a tuple in parent_mesh to a tuple in child_mesh
    // i.e. Dim(map_to_child[i]) == Dim(child_meshes[i])
    std::vector<MeshAttributeHandle<long>> map_to_child_handles;
    
    // Only valid if this is is_parent_mesh == false
    // store the map to the base_tuple of the parent_mesh
    MeshAttributeHandle<long> map_to_parent_handle;
    
    // register child_meshes and the map from child_meshes to this mesh, child_mesh_simplex 
    static void register_child_mesh(Mesh&parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<std::array<Tuple,2>>&child_mesh_simplex_map);
    static void register_child_mesh(Mesh&parent_mesh, std::shared_ptr<Mesh> child_mesh, const std::vector<long>& child_mesh_simplex_id_map);
    
    // helper function to check if this mesh is a valid child_mesh of parent_mesh
    // i.e. the connectivity of this mesh is a subset of this in parent_mesh
    static bool is_child_mesh_valid(const Mesh& parent_mesh, const Mesh& child_mesh);

    bool is_map_valid(const Mesh& parent_mesh) const;

    // TODO: make it a free function? static function?
    // Map source_tuple from source_mesh to target_mesh
    static Tuple map_tuple_between_meshes(const Mesh& source_mesh, const Mesh& target_mesh, MeshAttributeHandle<long> source_map_handle, const Tuple& source_tuple);

    static std::vector<Simplex> find_all_simplices_in_child_mesh(const Mesh& parent_mesh, long child_id, const Simplex& simplex_parent);


    // Utility function to map a edge tuple to all its children, used in operations
    static std::vector<Tuple> map_edge_tuple_to_all_children(const Mesh& parent_mesh,const Tuple& edge_tuple);

    private:
        // flag indicating if this mesh is a parent mesh
        bool m_is_parent_mesh;
        long m_child_id;
};

}