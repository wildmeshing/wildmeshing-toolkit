#pragma once

#include <Eigen/Core>

#include <initializer_list>

#include <memory>
#include <tuple>
// just includes function prorotypes to befriend
//#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>


// need to return this header
#include "attribute/Accessor.hpp"

// is a member of the Mesh class
#include "MultiMeshManager.hpp"

// basic data for the class
#include <wmtk/simplex/Simplex.hpp>
#include "MeshBase.hpp"
#include "Tuple.hpp"
#include "Types.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/FlagAccessor.hpp"
#include "attribute/MeshAttributeHandle.hpp"
#include "attribute/MeshAttributes.hpp"
#include "multimesh/attribute/AttributeScopeHandle.hpp"

#include "multimesh/attribute/UseParentScopeRAII.hpp"

#include "simplex/IdSimplex.hpp"
#include "simplex/NavigatableSimplex.hpp"
#include "simplex/Simplex.hpp"


// if we have concepts then switch_tuples uses forward_iterator concept
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
#include <ranges>
#endif


namespace wmtk {
namespace tests {
class DEBUG_Mesh;
} // namespace tests
// thread management tool that we will PImpl
namespace attribute {
class AttributeManager;
template <typename T, typename MeshType, int Dim>
class Accessor;

} // namespace attribute
namespace operations {
class Operation;
class EdgeCollapse;
class EdgeSplit;
class EdgeOperationData;
namespace internal {
class CollapseAlternateFacetData;
}
namespace utils {
class UpdateEdgeOperationMultiMeshMapFunctor;
}
} // namespace operations

namespace io {
class ParaviewWriter;
}
namespace multimesh {
template <int64_t cell_dimension, typename NodeFunctor>
class MultiMeshSimplexVisitor;
template <typename NodeFunctor>
class MultiMeshVisitor;
template <typename Visitor>
class MultiMeshSimplexVisitorExecutor;
template <typename Visitor>
class MultiMeshVisitorExecutor;

namespace utils {
class MapValidator;
namespace internal {
class TupleTag;
}
} // namespace utils
} // namespace multimesh


// NOTE: the implementation of this class is split into several files to improve clang-format
// performance
// * Mesh.cpp
// * Mesh_attributes.cpp
// * Mesh_construction.cpp
class Mesh : public std::enable_shared_from_this<Mesh>,
             public MeshBase,
             public wmtk::utils::MerkleTreeInteriorNode
{
public:
    friend class tests::DEBUG_Mesh;
    template <typename T, int Dim>
    friend class attribute::AccessorBase;
    template <typename T, typename MeshType, int Dim>
    friend class attribute::Accessor;
    friend class io::ParaviewWriter;
    friend class HDF5Reader;
    friend class multimesh::attribute::UseParentScopeRAII;
    friend class multimesh::MultiMeshManager;
    friend class attribute::AttributeManager;
    template <int64_t cell_dimension, typename NodeFunctor>
    friend class multimesh::MultiMeshSimplexVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshSimplexVisitorExecutor;
    template <typename NodeFunctor>
    friend class multimesh::MultiMeshVisitor;
    friend class multimesh::utils::MapValidator;
    template <typename Visitor>
    friend class multimesh::MultiMeshVisitorExecutor;
    friend class multimesh::attribute::AttributeScopeHandle;
    friend class multimesh::utils::internal::TupleTag;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    friend class operations::Operation;
    friend class operations::EdgeCollapse;
    friend class operations::EdgeSplit;
    friend class operations::EdgeOperationData;
    friend class operations::internal::CollapseAlternateFacetData;


    int64_t top_cell_dimension() const override;
    PrimitiveType top_simplex_type() const;
    bool is_free() const;

    // attribute directly hashes its "children" components so it overrides "child_hashes"
    std::map<std::string, const wmtk::utils::Hashable*> child_hashables() const override;
    std::map<std::string, std::size_t> child_hashes() const override;

    // dimension is the dimension of the top level simplex in this mesh
    // That is, a TriMesh is a 2, a TetMesh is a 3
    Mesh(const int64_t& dimension);
    // maximum primitive type id for supported attribute primitive locations
    Mesh(const int64_t& dimension, const int64_t& max_primitive_type_id, PrimitiveType hash_type);
    Mesh(Mesh&& other);
    Mesh(const Mesh& other) = delete;
    Mesh& operator=(const Mesh& other) = delete;
    Mesh& operator=(Mesh&& other);
    virtual ~Mesh();

    void serialize(MeshWriter& writer, const Mesh* local_root = nullptr) const;

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(PrimitiveType type) const override;

    std::vector<simplex::IdSimplex> get_all_id_simplex(PrimitiveType type) const override;
    /**
     * @brief Retrieve the IdSimplex that is represented by the tuple and primitive type.
     */
    simplex::IdSimplex get_id_simplex(const Tuple& tuple, PrimitiveType pt) const;

    simplex::IdSimplex get_id_simplex(const simplex::Simplex& s) const;

    /**
     * @brief Convert an IdSimplex into a Simplex.
     */
    simplex::Simplex get_simplex(const simplex::IdSimplex& s) const;

    Tuple get_tuple_from_id_simplex(const simplex::IdSimplex& s) const;

    /**
     * Consolidate the attributes, moving all valid simplexes at the beginning of the corresponding
     * vector
     */
    virtual std::tuple<std::vector<std::vector<int64_t>>, std::vector<std::vector<int64_t>>>
    consolidate();

    /**
     * Returns a vector of vectors of attribute handles. The first index denotes the type of simplex
     * pointed by the attribute (i.e. the index type). As an example, the FV relationship points to
     * vertices so it should be returned in the slot [0].
     */
    virtual std::vector<std::vector<TypedAttributeHandle<int64_t>>> connectivity_attributes()
        const = 0;


    std::vector<attribute::MeshAttributeHandle::HandleVariant> builtin_attributes() const;
    std::vector<attribute::MeshAttributeHandle::HandleVariant> custom_attributes() const;


    /* @brief registers an attribute without assuming the mesh exists */
    template <typename T>
    [[nodiscard]] attribute::MeshAttributeHandle register_attribute(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace = false,
        T default_value = T(0));

    /* @brief registers an attribute without assuming the mesh exists, returns a typed attribute */
    template <typename T>
    [[nodiscard]] attribute::TypedAttributeHandle<T> register_attribute_typed(
        const std::string& name,
        PrimitiveType type,
        int64_t size,
        bool replace = false,
        T default_value = T(0));


public:
    template <typename T>
    bool has_attribute(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools

    template <typename T>
    attribute::MeshAttributeHandle get_attribute_handle(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools

    template <typename T>
    attribute::TypedAttributeHandle<T> get_attribute_handle_typed(
        const std::string& name,
        const PrimitiveType ptype) const; // block standard topology tools


    template <typename T, int D = Eigen::Dynamic>
    attribute::Accessor<T, Mesh, D> create_accessor(const attribute::MeshAttributeHandle& handle);


    template <typename T, int D = Eigen::Dynamic>
    const attribute::Accessor<T, Mesh, D> create_const_accessor(
        const attribute::MeshAttributeHandle& handle) const;

    template <typename T, int D = Eigen::Dynamic>
    attribute::Accessor<T, Mesh, D> create_accessor(const TypedAttributeHandle<T>& handle);

    template <typename T, int D = Eigen::Dynamic>
    const attribute::Accessor<T, Mesh, D> create_const_accessor(
        const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    int64_t get_attribute_dimension(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    const T& get_attribute_default_value(const TypedAttributeHandle<T>& handle) const;

    template <typename T>
    std::string get_attribute_name(const TypedAttributeHandle<T>& handle) const;

    std::string get_attribute_name(
        const attribute::MeshAttributeHandle::HandleVariant& handle) const;

    /**
     * @brief Remove all custom attributes besides the one passed in.
     *
     * @param custom_attributes Vector of attributes that should be kept
     */
    void clear_attributes(
        const std::vector<attribute::MeshAttributeHandle::HandleVariant>& keep_attributes);
    void clear_attributes();
    void clear_attributes(const std::vector<attribute::MeshAttributeHandle>& keep_attributes);
    void delete_attribute(const attribute::MeshAttributeHandle& to_delete);
    void delete_attribute(const attribute::MeshAttributeHandle::HandleVariant& to_delete);


    // creates a scope as int64_t as the AttributeScopeHandle exists
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


    const attribute::FlagAccessor<Mesh> get_flag_accessor(PrimitiveType type) const;
    const attribute::FlagAccessor<Mesh> get_const_flag_accessor(PrimitiveType type) const;


    bool operator==(const Mesh& other) const;

    void assert_capacity_valid() const;
    virtual bool is_connectivity_valid() const = 0;

    virtual std::vector<Tuple> orient_vertices(const Tuple& t) const = 0;

protected: // member functions
    attribute::FlagAccessor<> get_flag_accessor(PrimitiveType type);


protected:
    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    virtual Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const = 0;
    simplex::NavigatableSimplex simplex_from_id(const PrimitiveType type, const int64_t gid) const;
    std::vector<std::vector<int64_t>> simplices_to_gids(
        const std::vector<std::vector<simplex::Simplex>>& simplices) const;
    /**
     * @brief reserve space for all attributes data types for all dimensional simplices
     *
     * @param top_d the top dimensional simplex
     */
    void reserve_attributes_to_fit();
    void reserve_attributes(PrimitiveType type, int64_t size);
    void reserve_attributes(int64_t dimension, int64_t size);


    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<int64_t> capacities);

    // reserves extra attributes than necessary right now
    void reserve_more_attributes(PrimitiveType type, int64_t size);
    // reserves extra attributes than necessary right now, does not pay attention
    void reserve_more_attributes(const std::vector<int64_t>& sizes);

    // makes sure that there are at least `size` simples of type `type` avialable
    void guarantee_more_attributes(PrimitiveType type, int64_t size);
    // makes sure that there are at least `size` simplices avialable at every dimension
    void guarantee_more_attributes(const std::vector<int64_t>& sizes);

    // makes sure that there are at least `size` simples of type `type` avialable
    void guarantee_at_least_attributes(PrimitiveType type, int64_t size);
    // makes sure that there are at least `size` simplices avialable at every dimension
    void guarantee_at_least_attributes(const std::vector<int64_t>& sizes);

    // provides new simplices - should ONLY be called in our atomic topological operations
    // all returned simplices are active (i.e their flags say they exist)
    [[nodiscard]] std::vector<int64_t> request_simplex_indices(PrimitiveType type, int64_t count);

public:
    /**
     * @brief switch the orientation of the Tuple of the given dimension
     * @note this is not done in place. Return a new Tuple of the switched state
     *
     * @param m
     * @param type  d-0 -> switch vertex
                    d-1 -> switch edge
                    d-2 -> switch face
                    d-3 -> switch tetrahedron
    */
    virtual Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override = 0;

    // NOTE: adding per-simplex utility functions here is _wrong_ and will be removed


    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
    // in debug mode this will assert a failure, in release this will return a null tuple
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
    template <std::ranges::forward_range ContainerType>
#else
    template <typename ContainerType>
#endif
    Tuple switch_tuples(const Tuple& tuple, const ContainerType& op_sequence) const;
    // annoying initializer list prototype to catch switch_tuples(t, {PV,PE})
    Tuple switch_tuples(const Tuple& tuple, const std::initializer_list<PrimitiveType>& op_sequence)
        const;

    // Performs a sequence of switch_tuple operations in the order specified in op_sequence.
#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
    template <std::ranges::forward_range ContainerType>
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
    int64_t capacity(PrimitiveType type) const;

    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true if the Tuple is oriented counter-clockwise
     * @return false
     */
    virtual bool is_ccw(const Tuple& tuple) const = 0;

    /**
     * @brief check if a simplex lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    bool is_boundary(const simplex::Simplex& tuple) const;
    /**
     * @brief check if a simplex (encoded as a tuple/primitive pair) lies on a boundary or not
     *
     * @param simplex
     * @return true if this simplex lies on the boundary of the mesh
     * @return false otherwise
     */
    virtual bool is_boundary(PrimitiveType, const Tuple& tuple) const override = 0;


    bool is_hash_valid(const Tuple& tuple, const attribute::Accessor<int64_t>& hash_accessor) const;
    bool is_hash_valid(const Tuple& tuple) const;

    /**
     * @brief check validity of tuple including its hash
     *
     * @param tuple the tuple to be checked
     * @param type only the top cell dimension, other validity follows with assumption of
     * manifoldness. 2->triangle, 3->tetrahedron
     * @return true if is valid
     * @return false
     */
    virtual bool is_valid(const Tuple& tuple) const;

    // whether the tuple refers to a removed / invalid dart in the mesh
    bool is_removed(const Tuple& tuple) const;
    // whether the tuple refers to a removed / invalid simplex in the mesh
    bool is_removed(const Tuple& tuple, PrimitiveType pt) const;

protected:
    // whether the tuple refers to a removed / invalid facet
    bool is_removed(int64_t index) const;
    // whether the tuple refers to a removed / invalid simplex
    bool is_removed(int64_t index, PrimitiveType pt) const;

public:
    /**
     * @brief Check if the cached id in a simplex is up-to-date.
     */
    bool is_valid(const simplex::Simplex& s) const;


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

    Mesh& get_multi_mesh_mesh(const std::vector<int64_t>& absolute_id);
    const Mesh& get_multi_mesh_mesh(const std::vector<int64_t>& absolute_id) const;


    Mesh& get_multi_mesh_child_mesh(const std::vector<int64_t>& relative_id);
    const Mesh& get_multi_mesh_child_mesh(const std::vector<int64_t>& relative_id) const;

    /**
     * @brief returns the direct multimesh child meshes for the current mesh
     */
    std::vector<std::shared_ptr<Mesh>> get_child_meshes() const;

    /**
     * @brief returns all multimesh child meshes
     */
    std::vector<std::shared_ptr<Mesh>> get_all_child_meshes() const;

    /**
     * @brief returns all meshes in multimesh
     */
    std::vector<std::shared_ptr<const Mesh>> get_all_meshes() const;


    /**
     * @brief returns a unique identifier for this mesh within a single multimesh structure
     *
     * Typically the root node should have an empty vector as the id {}
     * Its first child will have an id of {0}
     * Its second child will have an id of {1}
     * Its first child's second child will have an id of {0,1}
     */
    std::vector<int64_t> absolute_multi_mesh_id() const;


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
        const std::shared_ptr<Mesh>& child_mesh_ptr,
        const std::vector<std::array<Tuple, 2>>& map_tuples);

    /**
     * @brief Deregister a child mesh.
     *
     * The child mesh is not deleted. It is only detached from the multi-mesh structure. The child
     * mesh becomes the new root for its own children. Attribute handles for the child and parent
     * mesh will be invalidated by deregistration.
     */
    void deregister_child_mesh(const std::shared_ptr<Mesh>& child_mesh_ptr);


private:
    /**
     * @brief Update the child handles after clearing attributes.
     */
    void update_child_handles();

public:
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
    std::vector<simplex::Simplex> map(const Mesh& other_mesh, const simplex::Simplex& my_simplex)
        const;


    /*
     * @brief map a collection of simplices to another mesh
     *
     * @param mesh the mesh the simplices should be mapped to
     * @param simplices the simplices being mapped to the child mesh
     * @returns every simplex that corresponds to the passed simplices
     * */
    std::vector<simplex::Simplex> map(
        const Mesh& other_mesh,
        const std::vector<simplex::Simplex>& my_simplices) const;

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
    std::vector<simplex::Simplex> lub_map(
        const Mesh& other_mesh,
        const simplex::Simplex& my_simplex) const;


    /*
     * @brief maps a collection of simplices from this mesh to any other mesh using LUB mesh as root
     *
     * Satisfies the same properties of standard map, but uses a the LUB as the root
     *
     * @param mesh the mesh the simplices should be mapped to
     * @param simplices the simplices being mapped to the child mesh
     * @returns every simplex that corresponds to the passed simplices
     * */
    std::vector<simplex::Simplex> lub_map(
        const Mesh& other_mesh,
        const std::vector<simplex::Simplex>& my_simplices) const;

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
    simplex::Simplex map_to_parent(const simplex::Simplex& my_simplex) const;

    /**
     * @brief maps a simplex from this mesh to the root mesh
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_simplex the simplex being mapped to the parent mesh
     * @return the unique root mesh's simplex that is the root to the input one
     * */
    simplex::Simplex map_to_root(const simplex::Simplex& my_simplex) const;

    /**
     * @brief optimized map fromsimplex from this mesh to one of its direct children
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param child_mesh the simplex shoudl be mapped to
     * @param my_simplex the simplex being mapped to the child mesh
     * @param the set of child mesh's simplices that are equivalent to the input simplex
     * */
    std::vector<simplex::Simplex> map_to_child(
        const Mesh& child_mesh,
        const simplex::Simplex& my_simplex) const;


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
    std::vector<Tuple> map_tuples(const Mesh& other_mesh, const simplex::Simplex& my_simplex) const;

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
    std::vector<Tuple> lub_map_tuples(const Mesh& other_mesh, const simplex::Simplex& my_simplex)
        const;


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
    Tuple map_to_parent_tuple(const simplex::Simplex& my_simplex) const;

    /**
     * @brief maps a simplex from this mesh to the root mesh
     *
     * Cannot be used outside of applications with guaranteed multi-mesh structures
     *
     * @param my_simplex the simplex being mapped to the parent mesh
     * @return the unique root mesh's simplex that is the root to the input one, without the
     * dimension encoded
     * */
    Tuple map_to_root_tuple(const simplex::Simplex& my_simplex) const;

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
    std::vector<Tuple> map_to_child_tuples(
        const Mesh& child_mesh,
        const simplex::Simplex& my_simplex) const;


    /*
     * @brief identifies if this simplex can be mapped to another mesh
     *
     * This tries to map a simplex to another mesh and wil return true if a simplex is found.
     * Note that the cost of this is almost hte same as mapping, so it is more efficient to map and
     * check if the returned vector is empty rather than call this function.
     */
    bool can_map(const Mesh& other_mesh, const simplex::Simplex& my_simplex) const;

    /**
     * @brief wrapper function to update hashes (for parent mesh *this and its child meshes) after
     * vertex operations
     *
     * @param vertex operating vertex tuple
     * @param hash_accessor hash accesor of the parent mesh (*this)
     */
    void update_vertex_operation_hashes(
        const Tuple& vertex,
        attribute::Accessor<int64_t>& hash_accessor);


    /**
     * @brief returns if the mesh has a child mesh in the given dimension
     *
     * @param dimension
     * @return true
     * @return false
     */
    bool has_child_mesh_in_dimension(int64_t dimension) const
    {
        return m_multi_mesh_manager.has_child_mesh_in_dimension(dimension);
    }

    bool has_child_mesh() const { return m_multi_mesh_manager.has_child_mesh(); }

    /*
     * @brief returns if the other mesh is part of the same multi-mesh structure
     * @param other the other being mesh being checked
     * @returns true if they are part of the same structure
     **/
    bool is_from_same_multi_mesh_structure(const Mesh& other) const;

protected:
    // creates a scope as int64_t as the AttributeScopeHandle exists
    [[nodiscard]] attribute::AttributeScopeHandle create_single_mesh_scope();

public:
    /**
     * @brief return the global id of the Tuple of the given dimension
     *
     * This function uses the implementation defined in a derived class (like a Tri/TetMesh) using a virtual function, but to enable more opportunities teh actual virtual function is id_virtual
     *
     * @param m
     * @param type  d-0 -> vertex
                    d-1 -> edge
                    d-2 -> face
                    d-3 -> tetrahedron
        * @return int64_t id of the entity
    */
    int64_t id(const Tuple& tuple, PrimitiveType type) const override;

    int64_t id(const simplex::NavigatableSimplex& s) const { return s.index(); }
    int64_t id(const simplex::IdSimplex& s) const { return s.index(); }

    /// Forwarding version of id on simplices that does id caching
    virtual int64_t id(const simplex::Simplex& s) const override = 0;

protected:
    /// Internal utility to allow id to be virtual with a non-virtual overload in derived -Mesh classes.
    /// Mesh::id invokes Mesh::id_virtual which is final overriden by MeshCRTP<TriMesh>::id_virtual, which in turn invokes MeshCRTP<TriMesh>::id, and then TriMesh::id.
    /// This circuitous mechanism makes MeshCRTP<TriMesh>::id and TriMesh::id fully inlineable, so code that wants to take in any derived class can get optimized results with MeshCRTP, or for cases where classes want to utilize just TriMesh they can get inline/accelerated usage as well.
    virtual int64_t id_virtual(const Tuple& tuple, PrimitiveType type) const = 0;


    template <typename T, typename MeshType>
    static auto& get_index_access(attribute::Accessor<T, MeshType>& attr)
    {
        return attr.index_access();
    }
    template <typename T, typename MeshType>
    static auto& get_index_access(const attribute::Accessor<T, MeshType>& attr)
    {
        return attr.index_access();
    }


    // std::shared_ptr<AccessorCache> request_accesor_cache();
    //[[nodiscard]] AccessorScopeHandle push_accesor_scope();

protected: // THese are protected so unit tests can access - do not use manually in other derived
           // classes?
    attribute::AttributeManager m_attribute_manager;

    multimesh::MultiMeshManager m_multi_mesh_manager;

    int64_t m_top_cell_dimension = -1;

    // assumes no adjacency data exists
    bool m_is_free = false;

private:
    // PImpl'd manager of per-thread update stacks
    // Every time a new access scope is requested the manager creates another level of indirection
    // for updates
    // std::unique_ptr<AttributeScopeManager> m_attribute_scope_manager;

    //=========================================================
    // simplex::Simplex Attribute
    //=========================================================


    /**
     * @brief   0x1 == true = simplex is active (simplex exists)
     *          all flag default to 0
     *
     */
    std::vector<TypedAttributeHandle<char>> m_flag_handles;

    // hashes for top level simplices (i.e cells) to identify whether tuples
    // are invalid or not
    TypedAttributeHandle<int64_t> m_cell_hash_handle;


    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @param include_deleted if true returns also the deleted tuples (default false)
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(PrimitiveType type, const bool include_deleted) const;
    std::vector<simplex::IdSimplex> get_all_id_simplex(
        PrimitiveType type,
        const bool include_deleted) const;
};


template <typename T, int D>
inline auto Mesh::create_accessor(const TypedAttributeHandle<T>& handle)
    -> attribute::Accessor<T, Mesh, D>
{
    return attribute::Accessor<T, Mesh, D>(*this, handle);
}
template <typename T, int D>
inline auto Mesh::create_const_accessor(const TypedAttributeHandle<T>& handle) const
    -> const attribute::Accessor<T, Mesh, D>
{
    return attribute::Accessor<T, Mesh, D>(*this, handle);
}

template <typename T, int D>
inline auto Mesh::create_accessor(const attribute::MeshAttributeHandle& handle)
    -> attribute::Accessor<T, Mesh, D>
{
    assert(&handle.mesh() == this);
    assert(handle.holds<T>());
    return create_accessor<T, D>(handle.as<T>());
}


template <typename T, int D>
inline auto Mesh::create_const_accessor(const attribute::MeshAttributeHandle& handle) const
    -> const attribute::Accessor<T, Mesh, D>
{
    assert(&handle.mesh() == this);
    assert(handle.holds<T>());
    return create_const_accessor<T, D>(handle.as<T>());
}

template <typename T>
inline attribute::MeshAttributeHandle Mesh::get_attribute_handle(
    const std::string& name,
    const PrimitiveType ptype) const
{
    return wmtk::attribute::MeshAttributeHandle(
        *const_cast<Mesh*>(this),
        get_attribute_handle_typed<T>(name, ptype));
}

template <typename T>
inline attribute::TypedAttributeHandle<T> Mesh::get_attribute_handle_typed(
    const std::string& name,
    const PrimitiveType ptype) const
{
    wmtk::attribute::TypedAttributeHandle<T> h;
    h.m_base_handle = m_attribute_manager.get<T>(ptype).attribute_handle(name);
    h.m_primitive_type = ptype;
    return h;
}
template <typename T>
inline bool Mesh::has_attribute(const std::string& name, const PrimitiveType ptype) const
{
    return m_attribute_manager.get<T>(ptype).has_attribute(name);
}

template <typename T>
inline int64_t Mesh::get_attribute_dimension(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_attribute_dimension(handle);
}

template <typename T>
inline std::string Mesh::get_attribute_name(const TypedAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_name(handle);
}

template <typename Functor, typename... Args>
inline decltype(auto) Mesh::parent_scope(Functor&& f, Args&&... args) const
{
    multimesh::attribute::UseParentScopeRAII raii(const_cast<Mesh&>(*this));

    return std::invoke(std::forward<Functor>(f), std::forward<Args>(args)...);
}

#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
template <std::ranges::forward_range ContainerType>
#else
template <typename ContainerType>
#endif
inline Tuple Mesh::switch_tuples(const Tuple& tuple, const ContainerType& sequence) const
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    const PrimitiveType top_type = top_simplex_type();

    const int64_t boundary_dim = top_cell_dimension() - 1;
    const PrimitiveType boundary_pt = static_cast<PrimitiveType>(boundary_dim);

    for (const PrimitiveType primitive : sequence) {
        // for top level simplices we cannot navigate across boundaries
        if (primitive == top_type && is_boundary(boundary_pt, r)) {
            assert(!is_boundary(boundary_pt, r));
            r = {};
            return r;
        }
        r = switch_tuple(r, primitive);
    }
    return r;
}
inline bool Mesh::is_free() const
{
    return m_is_free;
}

inline int64_t Mesh::top_cell_dimension() const
{
    return m_top_cell_dimension;
}
inline PrimitiveType Mesh::top_simplex_type() const
{
    int64_t dimension = top_cell_dimension();
    assert(dimension >= 0);
    assert(dimension < 4);
    return static_cast<PrimitiveType>(dimension);
}


#if defined(__cpp_concepts) && defined(__cpp_lib_ranges)
template <std::ranges::forward_range ContainerType>
#else
template <typename ContainerType>
#endif
inline Tuple Mesh::switch_tuples_unsafe(const Tuple& tuple, const ContainerType& sequence) const
{
    static_assert(std::is_same_v<typename ContainerType::value_type, PrimitiveType>);
    Tuple r = tuple;
    for (const PrimitiveType primitive : sequence) {
        r = switch_tuple(r, primitive);
    }
    return r;
}

inline int64_t Mesh::id(const Tuple& tuple, PrimitiveType type) const
{
    return id_virtual(tuple, type);
}
} // namespace wmtk
