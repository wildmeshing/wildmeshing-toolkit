#pragma once

#define MTAO_PUBLICIZING_ID
#include <Eigen/Core>

#include <initializer_list>

#include <memory>
#include <tuple>
// just includes function prorotypes to befriend
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>


// need to return this header
#include "Accessor.hpp"

// is a member of the Mesh class
#include "MultiMeshManager.hpp"

// basic data for the class
#include "Simplex.hpp"
#include "Tuple.hpp"
#include "Types.hpp"
#include "attribute/Attribute.hpp" // Why do we need to include this now?
#include "attribute/AttributeInitializationHandle.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributeHandle.hpp"
#include "attribute/MeshAttributes.hpp"
#include "multimesh/attribute/AttributeScopeHandle.hpp"


#include "simplex/Simplex.hpp"


// if we have concepts then switch_tuples uses forward_iterator concept
#if defined(__cpp_concepts)
#include <iterator>
#endif


namespace wmtk {
// thread management tool that we will PImpl
namespace attribute {
class AttributeManager;
template <typename T>
class TupleAccessor;

} // namespace attribute
namespace operations {
class CollapseNewAttributeStrategy;
class AttributeTransferStrategyBase;
class SplitNewAttributeStrategy;
class Operation;
class EdgeCollapse;
class EdgeSplit;
class EdgeOperationData;
namespace utils {
class UpdateEdgeOperationMultiMeshMapFunctor;
}
} // namespace operations

namespace simplex {
class RawSimplex;
namespace utils {
class SimplexComparisons;
}
} // namespace simplex

namespace io {
class ParaviewWriter;
}
namespace multimesh {
template <long cell_dimension, typename NodeFunctor>
class MultiMeshSimplexVisitor;
template <typename NodeFunctor>
class MultiMeshVisitor;
template <typename Visitor>
class MultiMeshSimplexVisitorExecutor;
template <typename Visitor>
class MultiMeshVisitorExecutor;

namespace utils::internal {
class TupleTag;
}
} // namespace multimesh

class SimplicialComplex;


// NOTE: the implementation of this class is split into several files to improve clang-format
// performance
// * Mesh.cpp
// * Mesh_attributes.cpp
// * Mesh_construction.cpp
class Mesh : public std::enable_shared_from_this<Mesh>, public wmtk::utils::MerkleTreeInteriorNode
{
public:
    template <typename T>
    friend class attribute::AccessorBase;
    template <typename T>
    friend class attribute::TupleAccessor;
    friend class io::ParaviewWriter;
    friend class HDF5Reader;
    friend class MultiMeshManager;
    friend class attribute::AttributeManager;
    template <long cell_dimension, typename NodeFunctor>
    friend class multimesh::MultiMeshSimplexVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshSimplexVisitorExecutor;
    template <typename NodeFunctor>
    friend class multimesh::MultiMeshVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshVisitorExecutor;
    friend class multimesh::attribute::AttributeScopeHandle;
    friend class multimesh::utils::internal::TupleTag;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    friend class simplex::RawSimplex;
    friend class simplex::utils::SimplexComparisons;
    friend class operations::Operation;
    friend class operations::EdgeCollapse;
    friend class operations::EdgeSplit;
    friend class operations::EdgeOperationData;

    friend void operations::utils::update_vertex_operation_multimesh_map_hash(
        Mesh& m,
        const SimplicialComplex& vertex_closed_star,
        Accessor<long>& parent_hash_accessor);

    friend void operations::utils::update_vertex_operation_hashes(
        Mesh& m,
        const Tuple& vertex,
        Accessor<long>& hash_accessor);

    friend std::shared_ptr<Mesh> multimesh::utils::extract_and_register_child_mesh_from_tag_handle(
        Mesh& m,
        const MeshAttributeHandle<long>& tag_handle,
        const long tag_value);

    virtual long top_cell_dimension() const = 0;
    PrimitiveType top_simplex_type() const;

    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    // dimension is the dimension of the top level simplex in this mesh
    // That is, a TriMesh is a 2, a TetMesh is a 3
    Mesh(const long& dimension);
    // maximum primitive type id for supported attribute primitive locations
    Mesh(const long& dimension, const long& max_primitive_type_id, PrimitiveType hash_type);
    Mesh(Mesh&& other);
    Mesh(const Mesh& other);
    Mesh& operator=(const Mesh& other);
    Mesh& operator=(Mesh&& other);
    virtual ~Mesh();

    void serialize(MeshWriter& writer);

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(PrimitiveType type) const;

    /**
     * Consolidate the attributes, moving all valid simplexes at the beginning of the corresponding
     * vector
     */
    virtual std::tuple<std::vector<std::vector<long>>, std::vector<std::vector<long>>>
    consolidate();

    /**
     * Returns a vector of vectors of attribute handles. The first index denotes the type of simplex
     * pointed by the attribute (i.e. the index type). As an example, the FV relationship points to
     * vertices so it should be returned in the slot [0].
     */
    virtual std::vector<std::vector<TypedAttributeHandle<long>>> connectivity_attributes()
        const = 0;

    template <typename T>
    [[nodiscard]] attribute::AttributeInitializationHandle<T> register_attribute(
        const std::string& name,
        PrimitiveType type,
        long size,
        bool replace = false,
        T default_value = T(0));

    /* @brief registers an attribute without assuming the mesh exists */
    template <typename T>
    [[nodiscard]] TypedAttributeHandle<T> register_attribute_nomesh(
        const std::string& name,
        PrimitiveType type,
        long size,
        bool replace = false,
        T default_value = T(0));


    template <typename T>
    bool has_attribute(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools

    template <typename T>
    MeshAttributeHandle<T> get_attribute_handle(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools

    // appends a new attribute strategy to the system and appends a handle to it
    template <typename T>
    [[nodiscard]] attribute::AttributeInitializationHandle<T> add_new_attribute_strategy(
        const MeshAttributeHandle<T>& handle);

    void clear_new_attribute_strategies();

    template <typename T>
    Accessor<T> create_accessor(const TypedAttributeHandle<T>& handle);

    template <typename T>
    ConstAccessor<T> create_const_accessor(const TypedAttributeHandle<T>& handle) const;
    template <typename T>
    ConstAccessor<T> create_accessor(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    long get_attribute_dimension(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    std::string get_attribute_name(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    void clear_attributes(
        PrimitiveType ptype,
        const std::vector<AttributeHandle> keep_attributes = {});


    // creates a scope as long as the AttributeScopeHandle exists
    [[nodiscard]] multimesh::attribute::AttributeScopeHandle create_scope();


    /**
     * @brief Evaluate the passed in function inside the parent scope.
     * The parent_scope function can be nested to reach deeper levels in the scope stack.
     *
     * @param f The function that is evaluated within the parent scope.
     * @param args... The other arguments to this function
     * @returns The return value of f.
     */
    template <typename Functor, typename... Args>
    decltype(auto) parent_scope(Functor&& f, Args&&... args) const;


    ConstAccessor<char> get_flag_accessor(PrimitiveType type) const;
    ConstAccessor<long> get_cell_hash_accessor() const;
    ConstAccessor<char> get_const_flag_accessor(PrimitiveType type) const;
    ConstAccessor<long> get_const_cell_hash_accessor() const;


    long get_cell_hash(long cell_index, const ConstAccessor<long>& hash_accessor) const;
    // utility function for getting a cell's hash - slow because it creates a new accessor
    long get_cell_hash_slow(long cell_index) const;


    bool operator==(const Mesh& other) const;

    virtual bool is_connectivity_valid() const = 0;

protected: // member functions
    Accessor<char> get_flag_accessor(PrimitiveType type);
    Accessor<long> get_cell_hash_accessor();

    /**
     * @brief update hash in given cell
     *
     * @param cell tuple in which the hash should be updated
     * @param hash_accessor hash accessor
     */
    void update_cell_hash(const Tuple& cell, Accessor<long>& hash_accessor);

    /**
     * @brief update hashes in given cells
     *
     * @param cells vector of tuples in which the hash should be updated
     * @param hash_accessor hash accessor
     */
    void update_cell_hashes(const std::vector<Tuple>& cells, Accessor<long>& hash_accessor);
    /**
     * @brief same as `update_cell_hashes` but slow because it creates a new accessor
     */
    /**
     * @brief update hash in given cell
     *
     * @param cell tuple in which the hash should be updated
     * @param hash_accessor hash accessor
     */
    void update_cell_hash(const long cell_index, Accessor<long>& hash_accessor);

    /**
     * @brief update hashes in given cells
     *
     * @param cells vector of tuples in which the hash should be updated
     * @param hash_accessor hash accessor
     */
    void update_cell_hashes(const std::vector<long>& cell_indices, Accessor<long>& hash_accessor);

    void update_cell_hashes_slow(const std::vector<Tuple>& cells);

    /**
     * @brief return the same tuple but with updated hash
     *
     * This function should only be used in operations to create a valid return tuple in a known
     * position.
     *
     * @param tuple tuple with potentially outdated hash
     * @param hash_accessor hash accessor
     * @return tuple with updated hash
     */
    Tuple resurrect_tuple(const Tuple& tuple, const ConstAccessor<long>& hash_accessor) const;

    /**
     * @brief same as `resurrect_tuple` but slow because it creates a new accessor
     */
    Tuple resurrect_tuple_slow(const Tuple& tuple);

    // provides new simplices - should ONLY be called in our atomic topological operations
    // all returned simplices are active (i.e their flags say they exist)
    [[nodiscard]] std::vector<long> request_simplex_indices(PrimitiveType type, long count);


protected:
    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    virtual Tuple tuple_from_id(const PrimitiveType type, const long gid) const = 0;
    std::vector<std::vector<long>> simplices_to_gids(
        const std::vector<std::vector<Simplex>>& simplices) const;
    /**
     * @brief reserve space for all attributes data types for all dimensional simplices
     *
     * @param top_d the top dimensional simplex
     */
    void reserve_attributes_to_fit();
    void reserve_attributes(PrimitiveType type, long size);
    void reserve_attributes(long dimension, long size);


public:
    /**
     * @brief switch the orientation of the Tuple of the given dimension
     * @note this is not doen in place. Return a new Tuple of the switched state
     *
     * @param m
     * @param type  d-0 -> switch vertex
                    d-1 -> switch edge
                    d-2 -> switch face
                    d-3 -> switch tetrahedron
    */
    virtual Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const = 0;

    Tuple switch_vertex(const Tuple& tuple) const;
    Tuple switch_edge(const Tuple& tuple) const;
    Tuple switch_face(const Tuple& tuple) const;
    Tuple switch_tetrahedron(const Tuple& tuple) const;


    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
    // in debug mode this will assert a failure, in release this will return a null tuple
#if defined(__cpp_concepts)
    template <std::forward_iterator ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples(const Tuple& tuple, const ContainerType& op_sequence) const;
    // annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples(const Tuple& tuple, const std::initializer_list<PrimitiveType>& op_sequence)
        const;

    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
#if defined(__cpp_concepts)
    template <std::forward_iterator ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples_unsafe(const Tuple& tuple, const ContainerType& op_sequence) const;
    // annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples_unsafe(
        const Tuple& tuple,
        const std::initializer_list<PrimitiveType>& op_sequence) const;

    void set_capacities_from_flags();
    /**
     * @brief read in the m_capacities return the upper bound for the number of entities of the
     * given dimension
     *
     * @param type
     * @return int
     */
    long capacity(PrimitiveType type) const;

    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true if the Tuple is oriented counter-clockwise
     * @return false
     */
    virtual bool is_ccw(const Tuple& tuple) const = 0;
    /**
     * @brief check if a simplex of codimension 1 is a boundary simplex
     *
     * @param tuple
     * @return true if all tuple simplices besides the cell are on the boundary
     * @return false otherwise
     */
    [[deprecated("use is_boundary(Tuple,PrimitiveType) instead")]] bool is_boundary(
        const Tuple& codim_1_simplex) const;

    /**
     * @brief check if a simplex lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    bool is_boundary(const Simplex& tuple) const;
    /**
     * @brief check if a simplex (encoded as a tuple/primitive pair) lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    virtual bool is_boundary(const Tuple& tuple, PrimitiveType pt) const = 0;
    virtual bool is_boundary_vertex(const Tuple& tuple) const = 0;
    virtual bool is_boundary_edge(const Tuple& tuple) const
    {
        throw std::runtime_error("is_boundary_edge dosent make sense for this mesh");
    }


    bool is_hash_valid(const Tuple& tuple, const ConstAccessor<long>& hash_accessor) const;

    /**
     * @brief check validity of tuple including its hash
     *
     * @param tuple the tuple to be checked
     * @param type only the top cell dimension, other validity follows with assumption of
     * manifoldness. 2->triangle, 3->tetrahedron
     * @return true if is valid
     * @return false
     */
    virtual bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const = 0;
    bool is_valid_slow(const Tuple& tuple) const;


    //============================
    // MultiMesh interface
    //============================
    //
    /**
     * @brief return true if this mesh is the root of a multimesh tree
     */
    bool is_multi_mesh_root() const;
    /**
     * @brief returns a reference to the root of a multimesh tree
     */
    Mesh& get_multi_mesh_root();
    /**
     * @brief returns a const reference to the root of a multimesh tree
     */
    const Mesh& get_multi_mesh_root() const;

    /**
     * @brief returns the direct multimesh child meshes for the current mesh
     */
    std::vector<std::shared_ptr<Mesh>> get_child_meshes() const;

    /**
     * @brief returns a unique identifier for this mesh within a single multimesh structure
     *
     * Typically the root node should have an empty vector as the id {}
     * Its first child will have an id of {0}
     * Its second child will have an id of {1}
     * Its first child's second child will have an id of {0,1}
     */
    std::vector<long> absolute_multi_mesh_id() const;


    /**
     * @brief register a mesh as the child of this mesh
     *
     *  The parameter map_tuples is a sequence of {A,B} where A is a tuple of
     *  this mesh and B is a tuple of the child mesh.
     *  The tuple B is assumed to encode a top dimension simplex in the child
     *  mesh, and A is any tuple that corresponds to that simplex in the parent
     *  mesh
     *
     * @param child_mesh the mesh that will become a child of this mesh
     * @param mesh_tuples a sequence of corresponding tuples between meshes
     */
    void register_child_mesh(
        const std::shared_ptr<Mesh>& child_mesh,
        const std::vector<std::array<Tuple, 2>>& map_tuples);

    /**
     * @brief maps a simplex from this mesh to any other mesh
     *
     *
     * Generic interface for mapping between two arbitrary meshes in a multi-mesh structure.
     * Note that this finds ALL versions of a simplex, potentially crossing over topological
     * features above the pairs of simplices being mapped. For instance, if we map a trimesh seam
     * edge to itself using this interface it will find the edge on the other side of the seam. If
     * more granular mappings are required consider manually navigating the tree with map_to_parent
     * and map_to_child, which of course require a more particular understanding on how a simplex is
     * mapped. Throws if two meshes are not part of the same multi-mesh structure
     *
     *
     * @param mesh the mesh a simplex should be mapped to
     * @param simplex the simplex being mapped to the child mesh
     * @returns every simplex that corresponds to this simplex
     * */
    std::vector<Simplex> map(const Mesh& other_mesh, const Simplex& my_simplex) const;


    /*
     * @brief map a collection of simplices to another mesh
     *
     * @param mesh the mesh the simplices should be mapped to
     * @param simplices the simplices being mapped to the child mesh
     * @returns every simplex that corresponds to the passed simplices
     * */
    std::vector<Simplex> map(const Mesh& other_mesh, const std::vector<Simplex>& my_simplices)
        const;

    /**
     * @brief maps a simplex from this mesh to any other mesh using LUB mesh as root
     *
     *
     * Satisfies the same properties of standard map, but uses a the LUB as the root
     *
     *
     * @param mesh the mesh a simplex should be mapped to
     * @param simplex the simplex being mapped to the child mesh
     * @returns every simplex that corresponds to this simplex
     * */
    std::vector<Simplex> lub_map(const Mesh& other_mesh, const Simplex& my_simplex) const;


    /*
     * @brief maps a collection of simplices from this mesh to any other mesh using LUB mesh as root
     *
     * Satisfies the same properties of standard map, but uses a the LUB as the root
     *
     * @param mesh the mesh the simplices should be mapped to
     * @param simplices the simplices being mapped to the child mesh
     * @returns every simplex that corresponds to the passed simplices
     * */
    std::vector<Simplex> lub_map(const Mesh& other_mesh, const std::vector<Simplex>& my_simplices)
        const;

    /**
     * @brief optimized map from a simplex from this mesh to its direct parent
     *
     *
     * Maps a simplex to its direct parent in the multi-mesh structure.
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * throws if this is the root
     *
     * @param my_simplex the simplex being mapped to the parent mesh
     * @return the unique parent mesh's simplex that is parent to the input one
     * */
    Simplex map_to_parent(const Simplex& my_simplex) const;

    /**
     * @brief maps a simplex from this mesh to the root mesh
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_simplex the simplex being mapped to the parent mesh
     * @return the unique root mesh's simplex that is the root to the input one
     * */
    Simplex map_to_root(const Simplex& my_simplex) const;

    /**
     * @brief optimized map fromsimplex from this mesh to one of its direct children
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param child_mesh the simplex shoudl be mapped to
     * @param my_simplex the simplex being mapped to the child mesh
     * @param the set of child mesh's simplices that are equivalent to the input simplex
     * */
    std::vector<Simplex> map_to_child(const Mesh& child_mesh, const Simplex& my_simplex) const;


    /**
     * @brief maps a simplex from this mesh to any other mesh
     *
     *
     * Generic interface for mapping between two arbitrary meshes in a multi-mesh structure
     * Note that this finds ALL versions of a simplex, potentially crossing over topological
     * features above the pairs of simplices being mapped. For instance, if we map a trimesh seam
     * edge to itself using this interface it will find the edge on the other side of the seam. If
     * more granular mappings are required, consider manually navigating the tree with map_to_parent
     * and map_to_child, which of course requires a more particular understanding on how a simplex
     * is mapped. throws if two meshes are not part of the same multi-mesh structure
     *
     *
     * @param other_mesh the mesh a simplex should be mapped to
     * @param my_simplex the simplex being mapped to the child mesh
     * @returns every simplex that corresponds to this simplex, without the dimension encoded
     * */
    std::vector<Tuple> map_tuples(const Mesh& other_mesh, const Simplex& my_simplex) const;

    /*
     * @brief map a collection of homogeneous simplices to another mesh
     *
     * @param mesh the mesh the simplices should be mapped to
     * @param primitive_type the type of primitive the simplices are
     * @param tuples the tuples used to represent the simplices
     * @returns every simplex that corresponds to the passed simplices
     * */
    std::vector<Tuple> map_tuples(
        const Mesh& other_mesh,
        PrimitiveType pt,
        const std::vector<Tuple>& my_simplices) const;

    /**
     * @brief maps a simplex from this mesh to any other mesh using LUB mesh as root
     *
     *
     * Satisfies the same properties of standard map_tuples, but uses a the LUB as the root
     *
     *
     * @param mesh the mesh a simplex should be mapped to
     * @param simplex the simplex being mapped to the child mesh
     * @returns every simplex that corresponds to this simplex
     * */
    std::vector<Tuple> lub_map_tuples(const Mesh& other_mesh, const Simplex& my_simplex) const;


    /*
     * @brief maps a collection of simplices from this mesh to any other mesh using LUB mesh as root
     *
     * Satisfies the same properties of standard map_tuples, but uses a the LUB as the root
     *
     * @param mesh the mesh the simplices should be mapped to
     * @param simplices the simplices being mapped to the child mesh
     * @returns every simplex that corresponds to the passed simplices
     * */
    std::vector<Tuple> lub_map_tuples(
        const Mesh& other_mesh,
        PrimitiveType pt,
        const std::vector<Tuple>& my_simplices) const;

    /**
     * @brief optimized map from a simplex from this mesh to its direct parent
     *
     *
     * Maps a simplex to its direct parent in the multi-mesh structure.
     * Can only be used in applications with guaranteed multi-mesh structures
     *
     * throws if this is the root
     *
     * @param my_simplex the simplex being mapped to the parent mesh
     * @return the unique parent mesh's simplex that is parent to the input one, without the
     * dimension encoded
     * */
    Tuple map_to_parent_tuple(const Simplex& my_simplex) const;

    /**
     * @brief maps a simplex from this mesh to the root mesh
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_simplex the simplex being mapped to the parent mesh
     * @return the unique root mesh's simplex that is the root to the input one, without the
     * dimension encoded
     * */
    Tuple map_to_root_tuple(const Simplex& my_simplex) const;

    /**
     * @brief optimized map fromsimplex from this mesh to one of its direct children
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param child mesh the simplex shoudl be mapped to
     * @param child_mesh the simplex being mapped to the child mesh
     * @param my_simplex the set of child mesh's simplices that are equivalent to the input simplex,
     * without the dimension encoded
     * */
    std::vector<Tuple> map_to_child_tuples(const Mesh& child_mesh, const Simplex& my_simplex) const;


    /**
     * @brief wrapper function to update hashes (for parent mesh *this and its child meshes) after
     * vertex operations
     *
     * @param vertex operating vertex tuple
     * @param hash_accessor hash accesor of the parent mesh (*this)
     */
    void update_vertex_operation_hashes(const Tuple& vertex, Accessor<long>& hash_accessor);

private:
    /*
     * @brief returns if the other mesh is part of the same multi-mesh structure
     * @param other the other being mesh being checked
     * @returns true if they are part of the same structure
     **/
    bool is_from_same_multi_mesh_structure(const Mesh& other) const;

protected:
    // creates a scope as long as the AttributeScopeHandle exists
    [[nodiscard]] attribute::AttributeScopeHandle create_single_mesh_scope();

protected:
    /**
     * @brief return the global id of the Tuple of the given dimension
     *
     * @param m
     * @param type  d-0 -> vertex
                    d-1 -> edge
                    d-2 -> face
                    d-3 -> tetrahedron
        * @return long id of the entity
    */
#if defined(MTAO_PUBLICIZING_ID)
public: // TODO remove
#else
protected:
#endif
    virtual long id(const Tuple& tuple, PrimitiveType type) const = 0;
    long id(const Simplex& s) const { return id(s.tuple(), s.primitive_type()); }


    template <typename T>
    static auto& get_index_access(attribute::MutableAccessor<T>& attr)
    {
        return attr.index_access();
    }
    template <typename T>
    static auto& get_index_access(const attribute::ConstAccessor<T>& attr)
    {
        return attr.index_access();
    }

    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<long> capacities);

    // reserves extra attributes than necessary right now
    void reserve_more_attributes(PrimitiveType type, long size);
    // reserves extra attributes than necessary right now
    void reserve_more_attributes(const std::vector<long>& sizes);

    // std::shared_ptr<AccessorCache> request_accesor_cache();
    //[[nodiscard]] AccessorScopeHandle push_accesor_scope();

protected: // THese are protected so unit tests can access - do not use manually in other derived
           // classes?
    attribute::AttributeManager m_attribute_manager;

    MultiMeshManager m_multi_mesh_manager;

    std::vector<attribute::MeshAttributeHandleVariant> m_attributes;

public:
    // TODO: these are hacky locations for the deadline - we will eventually move strategies away
    // from here
    // TODO 2: users will get to externally access a list - just keeping for this merge
    std::vector<std::shared_ptr<operations::AttributeTransferStrategyBase>> m_transfer_strategies;

private:
    // PImpl'd manager of per-thread update stacks
    // Every time a new access scope is requested the manager creates another level of indirection
    // for updates
    // std::unique_ptr<AttributeScopeManager> m_attribute_scope_manager;

    //=========================================================
    // Simplex Attribute
    //=========================================================


    /**
     * @brief   0x1 == true = simplex is active (simplex exists)
     *          all flag default to 0
     *
     */
    std::vector<TypedAttributeHandle<char>> m_flag_handles;

    // hashes for top level simplices (i.e cells) to identify whether tuples
    // are invalid or not
    TypedAttributeHandle<long> m_cell_hash_handle;


    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @param include_deleted if true returns also the deleted tuples (default false)
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(PrimitiveType type, const bool include_deleted) const;
};


template <typename T>
Accessor<T> Mesh::create_accessor(const TypedAttributeHandle<T>& handle)
{
    return Accessor<T>(*this, handle);
}
template <typename T>
ConstAccessor<T> Mesh::create_const_accessor(const TypedAttributeHandle<T>& handle) const
{
    return ConstAccessor<T>(*this, handle);
}
template <typename T>
ConstAccessor<T> Mesh::create_accessor(const TypedAttributeHandle<T>& handle) const
{
    return create_const_accessor(handle);
}

template <typename T>
MeshAttributeHandle<T> Mesh::get_attribute_handle(
    const std::string& name,
    const PrimitiveType ptype) const
{
    MeshAttributeHandle<T> r;
    r.m_base_handle = m_attribute_manager.get<T>(ptype).attribute_handle(name);
    r.m_primitive_type = ptype;
    r.m_mesh = const_cast<Mesh*>(this);

    return r;
}

template <typename T>
bool Mesh::has_attribute(const std::string& name, const PrimitiveType ptype) const
{
    return m_attribute_manager.get<T>(ptype).has_attribute(name);
}

template <typename T>
long Mesh::get_attribute_dimension(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_attribute_dimension(handle);
}

template <typename T>
std::string Mesh::get_attribute_name(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_name(handle);
}

template <typename T>
void Mesh::clear_attributes(PrimitiveType ptype, const std::vector<AttributeHandle> keep_attributes)
{
    m_attribute_manager.clear_attributes<T>(ptype, keep_attributes);
}

template <typename Functor, typename... Args>
inline decltype(auto) Mesh::parent_scope(Functor&& f, Args&&... args) const
{
    return m_attribute_manager.parent_scope(std::forward<Functor>(f), std::forward<Args>(args)...);
}

inline Tuple Mesh::switch_vertex(const Tuple& tuple) const
{
    return switch_tuple(tuple, PrimitiveType::Vertex);
}
inline Tuple Mesh::switch_edge(const Tuple& tuple) const
{
    return switch_tuple(tuple, PrimitiveType::Edge);
}
inline Tuple Mesh::switch_face(const Tuple& tuple) const
{
    return switch_tuple(tuple, PrimitiveType::Face);
}
inline Tuple Mesh::switch_tetrahedron(const Tuple& tuple) const
{
    return switch_tuple(tuple, PrimitiveType::Tetrahedron);
}
#if defined(__cpp_concepts)
template <std::forward_iterator ContainerType>
#else
template <typename ContainerType>
#endif
Tuple Mesh::switch_tuples(const Tuple& tuple, const ContainerType& sequence) const
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    const PrimitiveType top_type = top_simplex_type();
    for (const PrimitiveType primitive : sequence) {
        // for top level simplices we cannot navigate across boundaries
        if (primitive == top_type && is_boundary(r)) {
            assert(!is_boundary(r));
            r = {};
            return r;
        }
        r = switch_tuple(r, primitive);
    }
    return r;
}

#if defined(__cpp_concepts)
template <std::forward_iterator ContainerType>
#else
template <typename ContainerType>
#endif
Tuple Mesh::switch_tuples_unsafe(const Tuple& tuple, const ContainerType& sequence) const
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    for (const PrimitiveType primitive : sequence) {
        r = switch_tuple(r, primitive);
    }
    return r;
}

} // namespace wmtk
