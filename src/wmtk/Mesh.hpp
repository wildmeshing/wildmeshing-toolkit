#pragma once

#include "Accessor.hpp"
#include "MeshAttributes.hpp"
#include "Primitive.hpp"
#include "Simplex.hpp"
#include "Tuple.hpp"
#include "Types.hpp"

#include <wmtk/io/MeshWriter.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <Eigen/Core>

namespace wmtk {
class Mesh
{
public:
    template <typename T, bool isConst>
    friend class Accessor;
    friend class ParaviewWriter;

    Mesh(const long& dimension);
    virtual ~Mesh();

    void serialize(MeshWriter& writer);

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all(const PrimitiveType& type) const;

    /**
     * Removes all unset space
     */
    void clean();

    virtual void split_edge(const Tuple& t) = 0;
    virtual void collapse_edge(const Tuple& t) = 0;

    template <typename T>
    MeshAttributeHandle<T> register_attribute(
        const std::string& name,
        PrimitiveType type,
        long size,
        bool replace = false);

    template <typename T>
    MeshAttributeHandle<T> get_attribute_handle(
        const std::string& name); // block standard topology tools

    template <typename T>
    Accessor<T> create_accessor(const MeshAttributeHandle<T>& handle);

    template <typename T>
    ConstAccessor<T> create_accessor(const MeshAttributeHandle<T>& handle) const;

    ConstAccessor<char> get_flag_accessor(PrimitiveType type) const;
    ConstAccessor<long> get_cell_hash_accessor() const;

    bool operator==(const Mesh& other) const;

    virtual bool is_connectivity_valid() const;

protected: // member functions
    Accessor<char> get_flag_accessor(PrimitiveType type);
    Accessor<long> get_cell_hash_accessor();

    // provides new simplices - should ONLY be called in our atomic topological operations
    // all returned simplices are active (i.e their flags say they exist)
    [[nodiscard]] std::vector<long> request_simplex_indices(PrimitiveType type, long count);

protected: // members
    std::vector<MeshAttributes<char>> m_char_attributes;
    std::vector<MeshAttributes<long>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;

    std::vector<std::vector<long>> simplices_to_gids(
        const std::vector<std::vector<Simplex>>& simplices) const;

    void delete_simplices(
        const std::vector<std::vector<Simplex>>& simplices,
        OperationState& state);

private: // members
    std::vector<long> m_capacities;

    /**
     * @brief   0x1 == true = simplex is active (simplex exists)
     *          all flag defaul to 0
     *
     */
    std::vector<MeshAttributeHandle<char>> m_flag_handles;
    MeshAttributeHandle<long> m_cell_hash_handle;

protected:
    // std::vector<MeshAttributes<Rational>> m_rational_attributes;
    template <typename T>
    MeshAttributes<T>& get_mesh_attributes(PrimitiveType ptype);

    template <typename T>
    MeshAttributes<T>& get_mesh_attributes(const MeshAttributeHandle<T>& handle);

    template <typename T>
    const MeshAttributes<T>& get_mesh_attributes(PrimitiveType ptype) const;

    template <typename T>
    const MeshAttributes<T>& get_mesh_attributes(const MeshAttributeHandle<T>& handle) const;

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    virtual Tuple tuple_from_id(const PrimitiveType type, const long gid) const = 0;

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
    virtual Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const = 0;
    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true if the Tuple is oriented counter-clockwise
     * @return false
     */
    virtual bool is_ccw(const Tuple& tuple) const = 0;
    /**
     * @param tuple
     * @return true if the edge tuple is a obundary one
     * @return false
     */
    virtual bool is_boundary(const Tuple& edge) const = 0;
    /**
     * @brief read in the m_capacities return the upper bound for the number of entities of the
     * given dimension
     *
     * @param type
     * @return int
     */
    long capacity(PrimitiveType type) const;
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

    void set_capacities_from_flags();

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
    virtual long id(const Tuple& tuple, const PrimitiveType& type) const = 0;
    long id(const Simplex& s) const { return id(s.tuple(), s.primitive_type()); }

    void set_capacities(std::vector<long> capacities);
};


template <typename T>
Accessor<T> Mesh::create_accessor(const MeshAttributeHandle<T>& handle)
{
    return Accessor<T>(*this, handle);
}
template <typename T>
ConstAccessor<T> Mesh::create_accessor(const MeshAttributeHandle<T>& handle) const
{
    return ConstAccessor<T>(*this, handle);
}

template <typename T>
const MeshAttributes<T>& Mesh::get_mesh_attributes(PrimitiveType ptype) const
{
    size_t index = get_simplex_dimension(ptype);
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes[index];
    }
    if constexpr (std::is_same_v<T, long>) {
        return m_long_attributes[index];
    }
    if constexpr (std::is_same_v<T, double>) {
        return m_double_attributes[index];
    }
    // if constexpr(std::is_same_v<T,Rational>) {
    //     return m_rational_attributes;
    // }
}
template <typename T>
const MeshAttributes<T>& Mesh::get_mesh_attributes(const MeshAttributeHandle<T>& handle) const
{
    return get_mesh_attributes<T>(handle.m_primitive_type);
}

template <typename T>
MeshAttributes<T>& Mesh::get_mesh_attributes(PrimitiveType ptype)
{
    size_t index = get_simplex_dimension(ptype);
    if constexpr (std::is_same_v<T, char>) {
        return m_char_attributes[index];
    }
    if constexpr (std::is_same_v<T, long>) {
        return m_long_attributes[index];
    }
    if constexpr (std::is_same_v<T, double>) {
        return m_double_attributes[index];
    }
    // if constexpr(std::is_same_v<T,Rational>) {
    //     return m_rational_attributes;
    // }
}

template <typename T>
MeshAttributes<T>& Mesh::get_mesh_attributes(const MeshAttributeHandle<T>& handle)
{
    return get_mesh_attributes<T>(handle.m_primitive_type);
}
} // namespace wmtk
