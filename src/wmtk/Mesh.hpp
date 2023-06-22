#pragma once
#include "MeshAttributes.hpp"


namespace wmtk {

    enum class PrimitiveType {
        Vertex,
        Edge,
        Face,
        Tetrahedron
    };

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
    bool check_mesh_connectivity_validity() const;


    /**
     * @brief Get the incident vertices for a triangle
     *
     * @param t tuple pointing to an face
     * @return tuples of incident vertices
     */
    std::array<Tuple, 3> oriented_tri_vertices(const Tuple& t) const;

    /**
     * @brief Get the incident vertices for a triangle
     *
     * @param t tuple pointing to an face
     * @return global vids of incident vertices
     */
    std::array<size_t, 3> oriented_tri_vids(const Tuple& t) const;

    /**
     * Generate a face Tuple using global cell id
     * @param cid global cell for the triangle/tetrahedron
     * @return a Tuple
     */
    Tuple tuple_from_cell(size_t cid) const = 0;

    /**
     * Generate a vertex Tuple using local vid
     * @param vid global vid
     * @note tuple refers to vid
     */
    Tuple tuple_from_vertex(size_t vid) const;


    /**
     * @brief perform the given function for each face
     *
     */
    void for_each_face(const std::function<void(const Tuple&)>&);

    /**
     * @brief perform the given function for each edge
     *
     */
    void for_each_edge(const std::function<void(const Tuple&)>&);

    /**
     * @brief perform the given function for each vertex
     *
     */
    void for_each_vertex(const std::function<void(const Tuple&)>&);

    AttributeHandle
    register_attribute(const std::string& name, const PrimitiveType& type, long size);

    template <typename T>
    T scalar_attribute(const AttributeHandle& handle, const PrimitiveType& type, const Tuble& tuble)
        const;

    long gid(const PrimitiveType& type);

protected:
    std::vector<MeshAttributes<bool>> m_bool_attributes;
    std::vector<MeshAttributes<long>> m_long_attributes;
    std::vector<MeshAttributes<double>> m_double_attributes;
    // std::vector<MeshAttributes<Rational>> m_rational_attributes;
    template <typename T>
    MeshAttributes<T>& get_mesh_attributes()
    {
        if constexpr (std::is_same_as_v<T, bool>) {
            return m_bool_attributes;
        }
        if constexpr (std::is_same_as_v<T, long>) {
            return m_long_attributes;
        }
        if constexpr (std::is_same_as_v<T, double>) {
            return m_double_attributes;
        }
        // if constexpr(std::is_same_as_v<T,Rational>) {
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
    TriMesh()
    {
        m_vf_accessor =
            m_long_attributes.register_attribute_with_accessor("m_vf", PrimitiveType::Vertex, 1);
        m_ef_accessor =
            m_long_attributes.register_attribute_with_accessor("m_ef", PrimitiveType::Edge, 1);

        m_fv_accessor =
            m_long_attributes.register_attribute_with_accessor("m_fv", PrimitiveType::Face, 3);
        m_fe_accessor =
            m_long_attributes.register_attribute_with_accessor("m_fe", PrimitiveType::Face, 3);
        m_ff_accessor =
            m_long_attributes.register_attribute_with_accessor("m_ff", PrimitiveType::Face, 3);
    }

private:
    AttributeAccessor m_vf_accessor;
    AttributeAccessor m_ef_accessor;

    AttributeAccessor m_fv_accessor;
    AttributeAccessor m_fe_accessor;
    AttributeAccessor m_ff_accessor;
};

class TetMesh : public Mesh
{
    TetMesh()
    {
        m_vt_handle =
            m_long_attributes.register_attribute_with_accessor("m_vt", PrimitiveType::Vertex, 1);
        m_et_handle =
            m_long_attributes.register_attribute_with_accessor("m_et", PrimitiveType::Edge, 1);
        m_ft_handle =
            m_long_attributes.register_attribute_with_accessor("m_ft", PrimitiveType::Face, 1);

        m_tv_handle = m_long_attributes.register_attribute_with_accessor(
            "m_tv",
            PrimitiveType::Tetrahedron,
            4);
        m_te_handle = m_long_attributes.register_attribute_with_accessor(
            "m_te",
            PrimitiveType::Tetrahedron,
            6);
        m_tf_handle = m_long_attributes.register_attribute_with_accessor(
            "m_tf",
            PrimitiveType::Tetrahedron,
            4);
        m_tt_handle = m_long_attributes.register_attribute_with_accessor(
            "m_tt",
            PrimitiveType::Tetrahedron,
            4);
    }

private:
    AttributeAccessor m_vt_accessor;
    AttributeAccessor m_et_accessor;
    AttributeAccessor m_ft_accessor;

    AttributeAccessor m_tv_accessor;
    AttributeAccessor m_te_accessor;
    AttributeAccessor m_tf_accessor;
    AttributeAccessor m_tt_accessor;
};
} // namespace wmtk
