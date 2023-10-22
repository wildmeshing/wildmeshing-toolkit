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

namespace operations::utils {
class UpdateEdgeOperationMultiMeshMapFunctor;
}
namespace multimesh {
template <long cell_dimension, typename NodeFunctor, typename EdgeFunctor>
class MultiMeshVisitor;
template <typename Visitor>
class MultiMeshVisitorExecutor;
} // namespace multimesh
class Mesh;
class MultiMeshManager
{
public:
    friend std::vector<std::array<Tuple, 2>> multimesh::same_simplex_dimension_surjection(
        const Mesh& parent,
        const Mesh& child,
        const std::vector<long>& parent_simplices);
    template <long cell_dimension, typename NodeFunctor, typename EdgeFunctor>
    friend class multimesh::MultiMeshVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshVisitorExecutor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;


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
    // Note that when we map a M-tuple from a K-complex to a J-complex there are different
    // relationships necessary if K == J
    //    if M == K then this is unique
    //    if M < K then this is many to many
    // if K < J
    //    if M == K then it is one to many
    //    if M < K then it is many to many
    //
    //    Note also that functions that end with _tuple or _tuples willl return tuples rather than
    //    simplices

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


    Simplex map_to_root(const Mesh& my_mesh, const Simplex& my_simplex) const;
    Tuple map_to_root_tuple(const Mesh& my_mesh, const Simplex& my_simplex) const;

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

    Tuple map_tuple_to_root_tuple(const Mesh& my_mesh, const Tuple& my_tuple) const;

    // wrapper for implementing converting tuple to a child using the internal map data
    std::vector<Tuple> map_to_child_tuples(
        const Mesh& my_mesh,
        const ChildData& child_data,
        const Simplex& simplex) const;

    // wrapper for implementing converting tuple to a child using the internal map data
    std::vector<Tuple>
    map_to_child_tuples(const Mesh& my_mesh, long child_id, const Simplex& simplex) const;


    // updates the map tuples to children for a particular dimension.
    // for eeach simplex we store its global index and all variations of that face using a
    // consistent set of subsimplices wrt the tuple representation If the new tuple has a
    // representation
    //
    // it cannot handle map updates of its faces?
    void update_map_tuple_hashes(
        Mesh& my_mesh,
        PrimitiveType primitive_type,
        const std::vector<std::tuple<long, std::vector<Tuple>>>& simplices_to_update,
        const std::vector<std::tuple<long, std::array<long, 2>>>& split_cell_maps = {});


    // uses the available parameters to find a tuple that is equivalent to old_smiplex but using
    // still-existing top level simplices. by equivalent each sub-simplex of old_simplex's tuple
    // maps to the same thing as the returned tuple
    std::optional<Tuple> find_valid_tuple(
        Mesh& my_mesh,
        const Simplex& old_simplex,
        const long old_gid,
        const std::vector<Tuple>& tuple_alternatives,
        const std::vector<std::tuple<long, std::array<long, 2>>>& split_cell_maps = {}) const;

    // returns a tuple such that every subsmipelx in old_simplex's tuple maps to the same smiplex as
    std::optional<Tuple> find_valid_tuple_from_alternatives(
        Mesh& my_mesh,
        PrimitiveType primitive_type,
        const std::vector<Tuple>& tuple_alternatives) const;

    // returns a tuple such that every subsmipelx in old_simplex's tuple maps to the same smiplex as
    // before
    std::optional<Tuple> find_valid_tuple_from_split(
        Mesh& my_mesh,
        const Simplex& old_simplex,
        const long old_gid,
        const std::vector<Tuple>& tuple_alternatives,
        const std::vector<std::tuple<long, std::array<long, 2>>>& split_cell_maps) const;

    std::optional<Tuple> try_updating_map_tuple_from_split(
        Mesh& my_mesh,
        const Simplex& old_simplex, // map tuple is contained in this
        const long old_gid,
        const std::vector<Tuple>& tuple_alternatives,
        const std::tuple<long, std::array<long, 2>>& split_cell_maps) const;

    void transport_to_new_tuple(
        Mesh& my_mesh,
        const Simplex& old_simplex,
        const long old_gid,
        const std::vector<Tuple>& tuple_alternatives,
        const std::vector<std::tuple<long, std::array<long, 2>>>& split_cell_maps = {});

    static std::optional<Tuple> find_tuple_from_gid(
        const Mesh& my_mesh,
        PrimitiveType primitive_type,
        const std::vector<Tuple>& tuples,
        long gid);

    static Tuple map_tuple_between_meshes(
        const Mesh& source_mesh,
        const Mesh& target_mesh,
        const ConstAccessor<long>& source_to_target_map_accessor,
        const Tuple& source_tuple);

    const std::vector<ChildData>& children() const { return m_children; }
    std::vector<ChildData>& children() { return m_children; }

    static std::string parent_to_child_map_attribute_name(long index);
    static std::string child_to_parent_map_attribute_name();

    // returns {parent_to_child, child_to_parent}
    std::array<attribute::MutableAccessor<long>, 2> get_map_accessors(Mesh& my_mesh, ChildData& c);
    // returns {parent_to_child, child_to_parent}
    std::array<attribute::ConstAccessor<long>, 2> get_map_const_accessors(
        const Mesh& my_mesh,
        const ChildData& c) const;


    // helper for updating multimap
    static long child_global_cid(
        const attribute::ConstAccessor<long>& parent_to_child,
        long parent_gid);
    static long parent_global_cid(
        const attribute::ConstAccessor<long>& child_to_parent,
        long child_gid);

private:
    // this is defined internally but is preferablly invoked through the multimesh free function
    static std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
        const Mesh& parent,
        const Mesh& child,
        const std::vector<long>& parent_simplices);
};

} // namespace wmtk
