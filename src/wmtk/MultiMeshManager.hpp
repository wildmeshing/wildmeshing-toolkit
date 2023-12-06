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
#include <wmtk/operations/utils/UpdateVertexMultiMeshMapHash.hpp>


namespace wmtk {

namespace operations::utils {
class UpdateEdgeOperationMultiMeshMapFunctor;
}
namespace multimesh {
template <long cell_dimension, typename NodeFunctor>
class MultiMeshSimplexVisitor;
template <typename Visitor>
class MultiMeshSimplexVisitorExecutor;

template <typename NodeFunctor>
class MultiMeshVisitor;
template <typename Visitor>
class MultiMeshVisitorExecutor;
} // namespace multimesh
class Mesh;
class SimplicialComplex;
/**
 * @brief Implementation details for how the Mesh class implements multiple meshes
 */
class MultiMeshManager
{
public:
    // utility function for mapping the same set of simplices (or a subset of equivalent simplices)
    friend std::vector<std::array<Tuple, 2>> multimesh::same_simplex_dimension_surjection(
        const Mesh& parent,
        const Mesh& child,
        const std::vector<long>& parent_simplices);

    // let the visitor object access the internal details
    template <long cell_dimension, typename NodeFunctor>
    friend class multimesh::MultiMeshSimplexVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshSimplexVisitorExecutor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    friend void operations::utils::update_vertex_operation_multimesh_map_hash(
        Mesh& m,
        const SimplicialComplex& vertex_closed_star,
        Accessor<long>& parent_hash_accessor);
    template <typename NodeFunctor>
    friend class multimesh::MultiMeshVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshVisitorExecutor;


    MultiMeshManager();
    ~MultiMeshManager();
    MultiMeshManager(const MultiMeshManager& o);
    MultiMeshManager(MultiMeshManager&& o);
    MultiMeshManager& operator=(const MultiMeshManager& o);
    MultiMeshManager& operator=(MultiMeshManager&& o);

    //=========================================================
    // Storage of MultiMesh
    //=========================================================

    /**
     * @brief Specifies whether this structure is the root of a multi-mesh tree
     * @returns true if this is the root, false otherwise
     */
    bool is_root() const;
    /**
     * @brief Specifies the child id of this mesh if it a child mesh in a mult-mesh tree
     *
     * throws if it not a child
     *
     * @returns the local index of this manager's mesh in its parent's array
     */
    long child_id() const;
    // @brief a unique id for this mesh with respect to its multi-mesh tree
    //
    // This is guaranteed to be the sequence of mesh indices used to traverse from the root of the
    // structure to this mesh (backwards)
    std::vector<long> absolute_id() const;


    /**
     * @brief register a another mesh as a child of this mesh.
     * @param my_mesh the mesh that this structure is owned by
     * @param child_mesh the mesh that will be added a child of this mesh
     * @param the set of tuples {A,B} where A is from "this mesh" and B represents a top-dimension
     *simplex in the child_mesh. Every top dimension tuple of the child mesh must be represented
     *here
     **/
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
    /**
     * @brief maps a simplex from this mesh to any other mesh
     *
     *
     * Generic interface for mapping between two arbitrary meshes in a multi-mesh structure
     * Note that this finds ALL versions of a simplex, potentially crossing over topological
     * features above the pairs of simplices being mapped. For instance, if we map a trimesh seam
     * edge to itself using this interface it will find the edge on the other side of the seam. If
     * more granular mappings are required consider manually navigating the tree with map_to_parent
     * and map_to_child, which of course require a more particular understanding on how a simplex is
     * mapped. throws if two meshes are not part of the same multi-mesh structure
     *
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param the mesh a simplex should be mapped to
     * @param the simplex being mapped to the child mesh
     * @returns every simplex that could correspond to this simplex
     * */
    std::vector<Simplex> map(const Mesh& my_mesh, const Mesh& other_mesh, const Simplex& my_simplex)
        const;
    /**
     * @brief maps a simplex from this mesh to any other mesh
     *
     *
     * Generic interface for mapping between two arbitrary meshes in a multi-mesh structure
     * Note that this finds ALL versions of a simplex, potentially crossing over topological
     * features above the pairs of simplices being mapped. For instance, if we map a trimesh seam
     * edge to itself using this interface it will find the edge on the other side of the seam. If
     * more granular mappings are required consider manually navigating the tree with map_to_parent
     * and map_to_child, which of course require a more particular understanding on how a simplex is
     * mapped. throws if two meshes are not part of the same multi-mesh structure
     *
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param the mesh a simplex should be mapped to
     * @param the simplex being mapped to the child mesh
     * @returns every simplex that could correspond to this simplex, without the dimension encoded
     * */
    std::vector<Tuple>
    map_tuples(const Mesh& my_mesh, const Mesh& other_mesh, const Simplex& my_simplex) const;


    /**
     * @brief optimized map from a simplex from this mesh to its direct parent
     *
     *
     * Maps a simplex to its direct parent in the multi-mesh structure.
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * throws if this is the root
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param the simplex being mapped to the parent mesh
     * @return the unique parent mesh's simplex that is parent to the input one
     * */
    Simplex map_to_parent(const Mesh& my_mesh, const Simplex& my_simplex) const;
    /**
     * @brief optimized map from a simplex from this mesh to its direct parent
     *
     *
     * Maps a simplex to its direct parent in the multi-mesh structure.
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * throws if this is the root
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param the simplex being mapped to the parent mesh
     * @return the unique parent mesh's simplex that is parent to the input one, without the
     * dimension encoded
     * */
    Tuple map_to_parent_tuple(const Mesh& my_mesh, const Simplex& my_simplex) const;


    /**
     * @brief maps a simplex from this mesh to the root mesh
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param the simplex being mapped to the parent mesh
     * @return the unique root mesh's simplex that is the root to the input one
     * */
    Simplex map_to_root(const Mesh& my_mesh, const Simplex& my_simplex) const;
    /**
     * @brief maps a simplex from this mesh to the root mesh
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param the simplex being mapped to the parent mesh
     * @return the unique root mesh's simplex that is the root to the input one, without the
     * dimension encoded
     * */
    Tuple map_to_root_tuple(const Mesh& my_mesh, const Simplex& my_simplex) const;

    /**
     * @brief optimized map fromsimplex from this mesh to one of its direct children
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_mesh the mesh that this structure is owned by
     * @param child mesh the simplex shoudl be mapped to
     * @param the simplex being mapped to the child mesh
     * @param the set of child mesh's simplices that are equivalent to the input simplex
     * */
    std::vector<Simplex>
    map_to_child(const Mesh& my_mesh, const Mesh& child_mesh, const Simplex& my_simplex) const;
    std::vector<Tuple> map_to_child_tuples(
        const Mesh& my_mesh,
        const Mesh& child_mesh,
        const Simplex& my_simplex) const;


    /* @brief obtains the root mesh of this multi-mesh tree
     *
     * @param my_mesh the mesh that this structure is owned by
     */
    const Mesh& get_root_mesh(const Mesh& my_mesh) const;
    /* @brief obtains the root mesh of this multi-mesh tree
     *
     * @param my_mesh the mesh that this structure is owned by
     */
    Mesh& get_root_mesh(Mesh& my_mesh);
    std::vector<std::shared_ptr<Mesh>> get_child_meshes() const;

protected:
    // Storage of a child mesh (a pointer from the mesh + the map from this mesh -> the child)
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
    // only valid if this is the child of some other mesh
    // store the map to the base_tuple of the my_mesh
    MeshAttributeHandle<long> map_to_parent_handle;

    // the index of this mesh with respect to its parent's m_children
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


    // utility static function for mapping a tuple between the source and target given a specified
    // map accessor
    static Tuple map_tuple_between_meshes(
        const Mesh& source_mesh,
        const Mesh& target_mesh,
        const ConstAccessor<long>& source_to_target_map_accessor,
        const Tuple& source_tuple);

    const std::vector<ChildData>& children() const { return m_children; }
    std::vector<ChildData>& children() { return m_children; }

    // uility for consistently specifying the name of the attribute used to map this mesh to its
    // parent
    static std::string parent_to_child_map_attribute_name(long index);
    // uility for consistently specifying the name of the attribute used to map this mesh to its
    // parent
    static std::string child_to_parent_map_attribute_name();

    // returns {parent_to_child, child_to_parent} accessors
    std::array<attribute::MutableAccessor<long>, 2> get_map_accessors(Mesh& my_mesh, ChildData& c);
    // returns {parent_to_child, child_to_parent} accessors
    std::array<attribute::ConstAccessor<long>, 2> get_map_const_accessors(
        const Mesh& my_mesh,
        const ChildData& c) const;


    //===========
    // Utilities for updating maps after operations
    //===========
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


    static std::optional<Tuple> find_tuple_from_gid(
        const Mesh& my_mesh,
        PrimitiveType primitive_type,
        const std::vector<Tuple>& tuples,
        long gid);

    // helper for updating multimap used in the update multimesh edge functor
    static long child_global_cid(
        const attribute::ConstAccessor<long>& parent_to_child,
        long parent_gid);
    // helper for updating multimap used in the update multimesh edge functor
    static long parent_global_cid(
        const attribute::ConstAccessor<long>& child_to_parent,
        long child_gid);

private:
    // this is defined internally but is preferablly invoked through the multimesh free function
    static std::vector<std::array<Tuple, 2>> same_simplex_dimension_surjection(
        const Mesh& parent,
        const Mesh& child,
        const std::vector<long>& parent_simplices);

public:
    /**
     * @brief update all the hashes of the top-simplces of the parent mesh around a vertex
     * hashes of the parent tuples in the maps for all child meshes
     *
     * @param m mesh the tuple belongs to
     * @param vertex operating vertex tuple
     * @param hash_accessor hash accessor of m
     */
    static void update_vertex_operation_hashes_internal(
        Mesh& m,
        const Tuple& vertex,
        Accessor<long>& hash_accessor);
    static void update_vertex_operation_multimesh_map_hash_internal(
        Mesh& m,
        const SimplicialComplex& vertex_closed_star,
        Accessor<long>& parent_hash_accessor);

public:
    // remove after bug fix
    void check_map_valid(const Mesh& my_mesh) const;

    void check_child_map_valid(const Mesh& my_mesh, const ChildData& child_data) const;
};
} // namespace wmtk
