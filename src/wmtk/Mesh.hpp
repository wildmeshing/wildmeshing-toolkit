#include <Tuple.h>
#pragma once


namespace wmtk {

class Mesh
{
public:
    Mesh();
    virtual ~Mesh();

    /**
     * Generate the connectivity of the mesh
     * @param n_vertices Input number of vertices
     * @param cells tris/tets connectivity
     */
    void initialiaze(long n_vertices, const std::vector<std::vector<long>>& cells);

    /**
     * Generate a vector of Tuples from global vertex/edge/triangle/tetrahedron index
     * @param type the type of tuple, can be vertex/edge/triangle/tetrahedron
     * @return vector of Tuples refering to each type
     */
    std::vector<Tuple> get_all_of(const PrimitiveType& type) const;

    /**
     * Removes all unsed space
     */
    void clean();

    virtual void split_edge(const Tuple& t) = 0;
    virtual void collapse_edge(const Tuple& t) = 0;

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
    std::array<long, 3> oriented_tri_vids(const Tuple& t) const;

    /**
     * Generate a face Tuple using global cell id
     * @param cid global cell for the triangle/tetrahedron
     * @return a Tuple
     */
    Tuple tuple_from_cell(long cid) const = 0;

    /**
     * Generate avertex Tuple using local vid
     * @param vid global vid
     * @note tuple refers to vid
     */
    Tuple tuple_from_vertex(long vid) const;


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
    std::vector<MeshAttributes<Rational>> m_rational_attributes;


    /**
     * Generate the vertex connectivity of the mesh using the existing triangle structure
     * @param n_vertices Input number of vertices
     */
    virtual void build_vertex_connectivity(long n_vertices) = 0;

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
    virtual long id(const Tuple& tuple, const PrimitiveType& type) const;
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
    virtual Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const;
    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true if the Tuple is oriented counter-clockwise
     * @return false
     */
    virtual bool is_ccw(const Tuple& tuple) const;

    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true
     * @return false
     */
    bool is_valid(const Tuple& tuple) const;
};


class TriMesh : public Mesh
{
    TriMesh()
    {
        m_vf_handle = m_long_attributes.register_attribute("m_vf", PrimitiveType::Vertex, 1);
        m_ef_handle = m_long_attributes.register_attribute("m_ef", PrimitiveType::Edge, 1);

        m_fv_handle = m_long_attributes.register_attribute("m_fv", PrimitiveType::Triangle, 3);
        m_fe_handle = m_long_attributes.register_attribute("m_fe", PrimitiveType::Triangle, 3);
        m_ff_handle = m_long_attributes.register_attribute("m_ff", PrimitiveType::Triangle, 3);
    }

private:
    AttributeHandle m_vf_handle;
    AttributeHandle m_ef_handle;

    AttributeHandle m_fv_handle;
    AttributeHandle m_fe_handle;
    AttributeHandle m_ff_handle;

public:
    void split_edge(const Tuple& t) override;
    void collapse_edge(const Tuple& t) override;

    void build_vertex_connectivity(long n_vertices) override;

    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
};

class TetMesh : public Mesh
{
    TetMesh()
    {
        m_vt_handle = m_long_attributes.register_attribute("m_vt", PrimitiveType::Vertex, 1);
        m_et_handle = m_long_attributes.register_attribute("m_et", PrimitiveType::Edge, 1);
        m_ft_handle = m_long_attributes.register_attribute("m_ft", PrimitiveType::Triangle, 1);

        m_tv_handle = m_long_attributes.register_attribute("m_tv", PrimitiveType::Tetrahedron, 4);
        m_te_handle = m_long_attributes.register_attribute("m_te", PrimitiveType::Tetrahedron, 6);
        m_tf_handle = m_long_attributes.register_attribute("m_tf", PrimitiveType::Tetrahedron, 4);
        m_tt_handle = m_long_attributes.register_attribute("m_tt", PrimitiveType::Tetrahedron, 4);
    }

private:
    AttributeHandle m_vt_handle;
    AttributeHandle m_et_handle;
    AttributeHandle m_ft_handle;

    AttributeHandle m_tv_handle;
    AttributeHandle m_te_handle;
    AttributeHandle m_tf_handle;
    AttributeHandle m_tt_handle;

public:
    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
};
} // namespace wmtk
