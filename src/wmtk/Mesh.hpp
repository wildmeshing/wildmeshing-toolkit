#pragma once
#include "Accessor.hpp"
#include "MeshAttributes.hpp"


namespace wmtk {

enum class PrimitiveType { Vertex, Edge, Face, Tetrahedron };

constexpr size_t get_simplex_dimension(PrimitiveType t)
{
    switch (t) {
    case PrimitiveType::Vertex: return 0;
    case PrimitiveType::Edge: return 1;
    case PrimitiveType::Face: return 2;
    case PrimitiveType::Tetrahedron: return 3;
    }
}

class Accessor;

class Mesh
{
public:
    friend class Accessor;
    Mesh();
    virtual ~Mesh();

    /**
     * Generate the connectivity of the mesh
     * @param n_vertices Input number of vertices
     * @param cells tris/tets connectivity
     */
    void initialize(long n_vertices, const std::vector<std::vector<long>>& cells);

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples referring to each type
     */
    std::vector<Tuple> get_all_of(const PrimitiveType& type) const;

    /**
     * Removes all unset space
     */
    void clean();

    virtual void split_edge(const Tuple& t) = 0;
    virtual void collapse_edge(const Tuple& t) = 0;

    /**
     * @brief a duplicate of Tuple::switch_tuple function
     */
    Tuple switch_tuple(const PrimitiveType& type, const Tuple& t) const
    {
        return t.switch_tuple(*this, type);
    }

    /**
     * @brief verify the connectivity validity of the mesh
     * @note a valid mesh can have cells that are is_removed == true
     */
    // bool check_mesh_connectivity_validity() const;


    ///**
    // * @brief Get the incident vertices for a triangle
    // *
    // * @param t tuple pointing to an face
    // * @return tuples of incident vertices
    // */
    // std::array<Tuple, 3> oriented_tri_vertices(const Tuple& t) const;

    ///**
    // * @brief Get the incident vertices for a triangle
    // *
    // * @param t tuple pointing to an face
    // * @return global vids of incident vertices
    // */
    // std::array<size_t, 3> oriented_tri_vids(const Tuple& t) const;

    ///**
    // * Generate a face Tuple using global cell id
    // * @param cid global cell for the triangle/tetrahedron
    // * @return a Tuple
    // */
    // Tuple tuple_from_cell(size_t cid) const = 0;

    ///**
    // * Generate a vertex Tuple using local vid
    // * @param vid global vid
    // * @note tuple refers to vid
    // */
    // Tuple tuple_from_vertex(size_t vid) const;


    ///**
    // * @brief perform the given function for each face
    // *
    // */
    // void for_each_face(const std::function<void(const Tuple&)>&);

    ///**
    // * @brief perform the given function for each edge
    // *
    // */
    // void for_each_edge(const std::function<void(const Tuple&)>&);

    ///**
    // * @brief perform the given function for each vertex
    // *
    // */
    // void for_each_vertex(const std::function<void(const Tuple&)>&);

    AttributeHandle
    register_attribute(const std::string& name, const PrimitiveType& type, long size);

    template <typename T>
    T scalar_attribute(const AttributeHandle& handle, const PrimitiveType& type, const Tuble& tuble)
        const;

    long gid(const PrimitiveType& type);

    template <typename T>
    void register_attribute(const std::string& name, PrimitiveType ptype, long size);

    template <typename T>
    Accessor<T>
    register_attribute_with_accessor(const std::string& name, PrimitiveType ptype, long size);

protected:
    std::vector<MeshAttributes<bool>> m_bool_attributes;
    std::vector<MeshAttributes<long>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;
    // std::vector<MeshAttributes<Rational>> m_rational_attributes;
    template <typename T>
    MeshAttributes<T>& get_mesh_attributes(PrimitiveType ptype)
    {
        size_t index = get_simplex_dimension(ptype);
        if constexpr (std::is_same_v<T, bool>) {
            return m_bool_attributes[index];
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


    /**
     * Generate the vertex connectivity of the mesh using the existing triangle structure
     * @param n_vertices Input number of vertices
     */
    virtual void build_vertex_connectivity(size_t n_vertices) = 0;
};


class TriMesh : public Mesh
{
    TriMesh();

private:
    Accessor<long> m_vf_accessor;
    Accessor<long> m_ef_accessor;

    Accessor<long> m_fv_accessor;
    Accessor<long> m_fe_accessor;
    Accessor<long> m_ff_accessor;
};

class TetMesh : public Mesh
{
    TetMesh();

private:
    Accessor<long> m_vt_accessor;
    Accessor<long> m_et_accessor;
    Accessor<long> m_ft_accessor;

    Accessor<long> m_tv_accessor;
    Accessor<long> m_te_accessor;
    Accessor<long> m_tf_accessor;
    Accessor<long> m_tt_accessor;
};

template <typename T>
void Mesh::register_attribute(const std::string& name, PrimitiveType ptype, long size)
{
    template get_mesh_attributes<T>(ptype).register_attribute(name, size);
}

template <typename T>
Accessor<T>
Mesh::register_attribute_with_accessor(const std::string& name, PrimitiveType ptype, long size)
{
    return get_mesh_attributes<T>(ptype).register_attribute_with_accessor(name, size);
}
} // namespace wmtk
