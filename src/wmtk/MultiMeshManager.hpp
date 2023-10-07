#pragma once
#include <spdlog/spdlog.h>
#include <tuple>
#include "Accessor.hpp"
#include "Simplex.hpp"
#include "Tuple.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributes.hpp"
// included to make a friend as this requires IDs
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>

namespace wmtk {

class Mesh;
class MultiMeshManager
{
public:
    friend std::vector<std::array<Tuple, 2>> multimesh::same_simplex_dimension_surjection(
        const Mesh& parent,
        const Mesh& child,
        const std::vector<long>& parent_simplices);

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
    // an id computed using the path to the root. NOTE: traversing from the root requies iterating
    // these indices backwards
    std::vector<long> absolute_id() const;


    // register child_meshes and the map from child_meshes to this mesh, child_mesh_simplex
    //
    void register_child_mesh(
        Mesh& my_mesh,
        const std::shared_ptr<Mesh>& child_mesh,
        const std::vector<std::array<Tuple, 2>>& child_tuple_my_tuple_map);


    // bool are_maps_valid(const Mesh& my_mesh) const;

    //===========
    //===========
    // Map functions
    //===========
    //===========
    // Note that when we map a M-tuplefrom a K-complex to a J-complex there are different
    // relationships necessary if K == J
    //    if M == K then this is unique
    //    if M < K then this is many to many
    // if K < J
    //    if M == K then it is one to many
    //    if M < K then it is many to many
    //
    //    Note also that functions that end with _tuple or _tuples willl return tuples rather than simplices

    //===========
    // Simplex maps
    //===========
    // generic mapping function that maps a tuple from "this" mesh to the other mesh
    std::vector<Simplex> map(const Mesh& my_mesh, const Mesh& other_mesh, const Simplex& my_simplex)
        const;
    // generic mapping function that maps a tuple from "this" mesh to the other mesh
    std::vector<Tuple>
    map_tuples(const Mesh& my_mesh, const Mesh& other_mesh, const Simplex& my_simplex) const;


    Simplex map_to_parent(const Mesh& my_mesh, const Simplex& my_simplex) const;
    Tuple map_to_parent_tuple(const Mesh& my_mesh, const Simplex& my_simplex) const;


    // generic mapping function that maps a tuple from "this" mesh to one of its children
    std::vector<Simplex>
    map_to_child(const Mesh& my_mesh, const Mesh& child_mesh, const Simplex& my_simplex) const;
    std::vector<Tuple> map_to_child_tuples(
        const Mesh& my_mesh,
        const Mesh& child_mesh,
        const Simplex& my_simplex) const;


    // Utility function to map a edge tuple to all its children, used in operations
    std::vector<Tuple> map_edge_tuple_to_all_children(const Mesh& my_mesh, const Simplex& tuple)
        const;

    const Mesh& get_root_mesh(const Mesh& my_mesh) const;
    Mesh& get_root_mesh(Mesh& my_mesh);

protected:
    struct ChildData
    {
        std::shared_ptr<Mesh> mesh;
        // store the map from the manager's mesh to the child mesh (on the top
        // level simplex of the mesh)
        // encoded by a pair of two tuples, from a tuple in current mesh to a tuple in
        // child_mesh
        MeshAttributeHandle<long> map_handle;
    };

private:
    Mesh* m_parent = nullptr;
    // only valid if this is teh child of some other mesh
    // store the map to the base_tuple of the my_mesh
    MeshAttributeHandle<long> map_to_parent_handle;
    long m_child_id = -1;


    // Child Meshes
    std::vector<ChildData> m_children;

protected: // protected to enable unit testing
    //===========
    // Tuple maps
    //===========

    // generic mapping function that maps a tuple from "this" mesh to its parent. We don't actually
    // need the simplex parent of the tuple being mapped up so we can throw away the simplex-nes
    Tuple map_tuple_to_parent_tuple(const Mesh& my_mesh, const Tuple& my_tuple) const;


    // wrapper for implementing converting tuple to a child using the internal map data
    std::vector<Tuple> map_to_child_tuples(
        const Mesh& my_mesh,
        const ChildData& child_data,
        const Simplex& simplex) const;

    // wrapper for implementing converting tuple to a child using the internal map data
    std::vector<Tuple>
    map_to_child_tuples(const Mesh& my_mesh, long child_id, const Simplex& simplex) const;




    static Tuple map_tuple_between_meshes(
        const Mesh& source_mesh,
        const Mesh& target_mesh,
        const ConstAccessor<long>& source_to_target_map_accessor,
        const Tuple& source_tuple);

    const std::vector<ChildData>& children() const { return m_children; }

    static std::string parent_to_child_map_attribute_name(long index);
    static std::string child_to_parent_map_attribute_name();

private:
    // this is defined internally but is preferablly invoked through the multimesh free function
    static std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
        const Mesh& parent,
        const Mesh& child,
        const std::vector<long>& parent_simplices);
};

} // namespace wmtk
