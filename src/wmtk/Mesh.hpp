#pragma once

#include "Accessor.hpp"
#include "MeshAttributes.hpp"
#include "Primitive.hpp"
#include "Tuple.hpp"
#include "Types.hpp"

#include <Eigen/Core>

namespace wmtk {
class Mesh
{
public:
    template <typename T>
    friend class Accessor;

    Mesh();
    virtual ~Mesh();

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples referring to each type
     */
    virtual std::vector<Tuple> get_all(const PrimitiveType& type) const = 0;

    /**
     * Removes all unset space
     */
    void clean();

    virtual void split_edge(const Tuple& t) = 0;
    virtual void collapse_edge(const Tuple& t) = 0;

    AttributeHandle
    register_attribute(const std::string& name, const PrimitiveType& type, long size);

    template <typename T>
    MeshAttributeHandle<T>
    register_attribute(const std::string& name, PrimitiveType type, long size);

    template <typename T>
    MeshAttributeHandle<T> get_attribute_handle(
        const std::string& name); // block standard topology tools

    template <typename T>
    Accessor<T> create_accessor(const MeshAttributeHandle<T>& handle);

    template <typename T>
    const Accessor<T> create_accessor(const MeshAttributeHandle<T>& handle) const;

    long capacity(const PrimitiveType& type) const;

protected:
    std::vector<MeshAttributes<char>> m_char_attributes;
    std::vector<MeshAttributes<long>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;
    // std::vector<MeshAttributes<Rational>> m_rational_attributes;
    template <typename T>
    MeshAttributes<T>& get_mesh_attributes(PrimitiveType ptype);

    template <typename T>
    MeshAttributes<T>& get_mesh_attributes(const MeshAttributeHandle<T>& handle);

    template <typename T>
    const MeshAttributes<T>& get_mesh_attributes(PrimitiveType ptype) const;

    template <typename T>
    const MeshAttributes<T>& get_mesh_attributes(const MeshAttributeHandle<T>& handle) const;

    Tuple tuple_from_cell(long cid) const;


    /**
     * @brief reserve space for all attributes data types for all dimensional simplices
     *
     * @param top_d the top dimensional simplex
     */
    void mesh_attributes_reserve(const PrimitiveType& top_d);

public:
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
    bool is_valid(const Tuple& tuple) const;

private:
    std::vector<long> m_capacities;
    // 0x1 == true = is active
    std::vector<MeshAttributeHandle<char>> m_flags;
};


template <typename T>
Accessor<T> Mesh::create_accessor(const MeshAttributeHandle<T>& handle)
{
    return Accessor(*this, handle);
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
