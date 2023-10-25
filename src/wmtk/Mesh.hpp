#pragma once

#include <Eigen/Core>

#include <initializer_list>

#include <memory>
#include <wmtk/io/ParaviewWriter.hpp>
#include "Accessor.hpp"
#include "MultiMeshManager.hpp"
#include "Primitive.hpp"
#include "Simplex.hpp"
#include "Tuple.hpp"
#include "Types.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributes.hpp"

#include "simplex/Simplex.hpp"


// if we have concepts then switch_tuples uses forward_iterator concept
#if defined(__cpp_concepts)
#include <iterator>
#endif


namespace wmtk {
// thread management tool that we will PImpl
namespace attribute {
class AttributeScopeManager;
template <typename T>
class TupleAccessor;

} // namespace attribute
namespace operations {
class Operation;
namespace utils {
struct UpdateEdgeOperationMultiMeshMapFunctor;
}
} // namespace operations
namespace multimesh {
template <long cell_dimension, typename NodeFunctor, typename EdgeFunctor>
class MultiMeshVisitor;
template <typename Visitor>
class MultiMeshVisitorExecutor;
} // namespace multimesh

class Mesh : public std::enable_shared_from_this<Mesh>
{
public:
    template <typename T>
    friend class attribute::AccessorBase;
    template <typename T>
    friend class attribute::TupleAccessor;
    friend class ParaviewWriter;
    friend class MeshReader;
    friend class MultiMeshManager;
    template <long cell_dimension, typename NodeFunctor, typename EdgeFunctor>
    friend class multimesh::MultiMeshVisitor;
    template <typename Visitor>
    friend class multimesh::MultiMeshVisitorExecutor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;

    virtual long top_cell_dimension() const = 0;
    PrimitiveType top_simplex_type() const;

    friend class operations::Operation;

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
     * Removes all unset space
     */
    void clean();


    template <typename T>
    MeshAttributeHandle<T> register_attribute(
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

    template <typename T>
    Accessor<T> create_accessor(const MeshAttributeHandle<T>& handle);

    template <typename T>
    ConstAccessor<T> create_const_accessor(const MeshAttributeHandle<T>& handle) const;
    template <typename T>
    ConstAccessor<T> create_accessor(const MeshAttributeHandle<T>& handle) const;

    template <typename T>
    long get_attribute_dimension(const MeshAttributeHandle<T>& handle) const;


    // creates a scope as long as the AttributeScopeHandle exists
    [[nodiscard]] attribute::AttributeScopeHandle create_scope();


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
    // std::vector<MeshAttributes<Rational>> m_rational_attributes;

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
     * @brief check if all tuple simplices besides the cell are on the boundary
     *
     * @param tuple
     * @return true if all tuple simplices besides the cell are on the boundary
     * @return false otherwise
     */
    virtual bool is_boundary(const Tuple& tuple) const = 0;

    virtual bool is_boundary_vertex(const Tuple& vertex) const = 0;
    virtual bool is_boundary_edge(const Tuple& vertex) const = 0;

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


    bool simplices_are_equal(const Simplex& s0, const Simplex& s1) const;

    bool simplex_is_less(const Simplex& s0, const Simplex& s1) const;


    //============================
    // MultiMesh interface
    //============================
    bool is_multi_mesh_root() const;
    Mesh& get_multi_mesh_root();
    const Mesh& get_multi_mesh_root() const;
    std::vector<long> absolute_multi_mesh_id() const;
    void register_child_mesh(
        const std::shared_ptr<Mesh>& child_mesh,
        const std::vector<std::array<Tuple, 2>>& map_tuples);

    // a generic map interface between pairs of mesh in a single multi-mesh structure
    std::vector<Simplex> map(const Mesh& other_mesh, const Simplex& my_simplex) const;
    // map to just the parent
    Simplex map_to_parent(const Simplex& my_simplex) const;

    Simplex map_to_root(const Simplex& my_simplex) const;
    // map to just a child
    std::vector<Simplex> map_to_child(const Mesh& child_mesh, const Simplex& my_simplex) const;

    // a generic map interface between pairs of mesh in a single multi-mesh structure but returns
    // tuples Each tuple partial encodes a Simplex, whose dimension is the same as my_simplex
    std::vector<Tuple> map_tuples(const Mesh& other_mesh, const Simplex& my_simplex) const;
    // map to just the parent
    Tuple map_to_parent_tuple(const Simplex& my_simplex) const;
    Tuple map_to_root_tuple(const Simplex& my_simplex) const;
    // map to just a child
    std::vector<Tuple> map_to_child_tuples(const Mesh& child_mesh, const Simplex& my_simplex) const;


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
    std::vector<MeshAttributeHandle<char>> m_flag_handles;

    // hashes for top level simplices (i.e cells) to identify whether tuples
    // are invalid or not
    MeshAttributeHandle<long> m_cell_hash_handle;
};


template <typename T>
Accessor<T> Mesh::create_accessor(const MeshAttributeHandle<T>& handle)
{
    return Accessor<T>(*this, handle);
}
template <typename T>
ConstAccessor<T> Mesh::create_const_accessor(const MeshAttributeHandle<T>& handle) const
{
    return ConstAccessor<T>(*this, handle);
}
template <typename T>
ConstAccessor<T> Mesh::create_accessor(const MeshAttributeHandle<T>& handle) const
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
    return r;
}

template <typename T>
bool Mesh::has_attribute(const std::string& name, const PrimitiveType ptype) const
{
    return m_attribute_manager.get<T>(ptype).has_attribute(name);
}

template <typename T>
long Mesh::get_attribute_dimension(const MeshAttributeHandle<T>& handle) const
{
    return m_attribute_manager.get_attribute_dimension(handle);
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
