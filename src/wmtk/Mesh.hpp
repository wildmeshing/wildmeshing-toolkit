#pragma once

#include "Accessor.hpp"
#include "Primitive.hpp"
#include "Simplex.hpp"
#include "Tuple.hpp"
#include "Types.hpp"
#include "attribute/AttributeManager.hpp"
#include "attribute/AttributeScopeHandle.hpp"
#include "attribute/MeshAttributes.hpp"

#include <wmtk/io/ParaviewWriter.hpp>

#include <Eigen/Core>

namespace wmtk {
// thread management tool that we will PImpl
class AttributeScopeManager;
namespace operations {
class Operation;
}

class Mesh
{
public:
    template <typename T>
    friend class AccessorBase;
    friend class ParaviewWriter;
    friend class MeshReader;
    friend class operations::Operation;

    // dimension is the dimension of the top level simplex in this mesh
    // That is, a TriMesh is a 2, a TetMesh is a 3
    Mesh(const long& dimension);
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


    // Split and collapse are the two atomic operations we want to support for each type of mesh.
    // These functions are intended to be called within an single Operation and
    // not on their own and the semantics between each derived Mesh class and
    // its SplitEdge and CollapseEdge operations should be treated as internal
    // implementation deatils.
    //
    // As such, the split_edge and collapse_edge functions JUST implement the
    // updates to topological updates and any precondition / postcondition checks
    // should be implemented by the user.
    //
    // These functions take in a single tuple, referring to the edge being
    // operated on, and return a single tuple that refers to the new topology.
    // This returned tuple has specific meaning for each derived Mesh class

    virtual Tuple split_edge(const Tuple& t) = 0;
    virtual Tuple collapse_edge(const Tuple& t) = 0;

    template <typename T>
    MeshAttributeHandle<T> register_attribute(
        const std::string& name,
        PrimitiveType type,
        long size,
        bool replace = false);

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


    // creates a scope as long as the AttributeScopeHandle exists
    [[nodiscard]] AttributeScopeHandle create_scope();


    ConstAccessor<char> get_flag_accessor(PrimitiveType type) const;
    ConstAccessor<long> get_cell_hash_accessor() const;
    ConstAccessor<char> get_const_flag_accessor(PrimitiveType type) const;
    ConstAccessor<long> get_const_cell_hash_accessor() const;


    // utility function for getting a cell's hash - slow because it creates a new accessor
    long get_cell_hash_slow(long cell_index) const;

    bool operator==(const Mesh& other) const;

    virtual bool is_connectivity_valid() const = 0;

protected: // member functions
    Accessor<char> get_flag_accessor(PrimitiveType type);
    Accessor<long> get_cell_hash_accessor();

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

    Tuple switch_vertex(const Tuple& tuple) const
    {
        return switch_tuple(tuple, PrimitiveType::Vertex);
    }
    Tuple switch_edge(const Tuple& tuple) const { return switch_tuple(tuple, PrimitiveType::Edge); }
    Tuple switch_face(const Tuple& tuple) const { return switch_tuple(tuple, PrimitiveType::Face); }


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

    /**
     * @brief
     *
     * @param tuple the tuple to be checked
     * @param type only the top cell dimension, other validity follows with assumption of
     * manifoldness. 2->triangle, 3->tetrahedron
     * @return true if is valid
     * @return false
     */
    virtual bool is_valid(const Tuple& tuple) const = 0;
    /**
     * @brief check if tuple was involved in a topological operation and is therefore invalid
     *
     * @param tuple the tuple to be checked
     * @return true if tuple is outdated
     * @return false otherwise
     */
    virtual bool is_outdated(const Tuple& tuple) const = 0;


    bool simplex_is_equal(const Simplex& s0, const Simplex& s1) const;

    bool simplex_is_less(const Simplex& s0, const Simplex& s1) const;

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

    // specifies the number of simplices of each type and resizes attributes appropritely
    void set_capacities(std::vector<long> capacities);


    // std::shared_ptr<AccessorCache> request_accesor_cache();
    //[[nodiscard]] AccessorScopeHandle push_accesor_scope();

private: // members
    AttributeManager m_attribute_manager;

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

} // namespace wmtk
