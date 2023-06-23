#pragma once
#include "Accessor.hpp"
#include "MeshAttributes.hpp"
#include "Primitive.hpp"
#include "Tuple.h"

using namespace Eigen;
namespace wmtk {



class Mesh
{
public:
    using RowVectors3l = Eigen::Matrix<long, Eigen::Dynamic, 3>;
    using VectorXl = Eigen::Matrix<long, Eigen::Dynamic, 1>;
    using RowVectors4l = Eigen::Matrix<long, Eigen::Dynamic, 4>;
    using RowVectors3d = Eigen::Matrix<double, Eigen::Dynamic, 3>;
    template <typename T>
    friend class Accessor;
    Mesh();
    virtual ~Mesh();

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

    /**
     * @brief Get the incident vertices for a triangle
     *
     * @param t tuple pointing to an face
     * @return global vids of incident vertices
     */
    std::array<long, 3> oriented_tri_vids(const Tuple& t) const;

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

    long gid(const PrimitiveType& type);


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
     * @brief give the upper bound for the number of entities of the given dimension
     *
     * @param type
     * @return int
     */
    int capacity(const PrimitiveType& type) const;
    /**
     * @brief TODO this needs dimension?
     *
     * @param m
     * @return true
     * @return false
     */
    bool is_valid(const Tuple& tuple) const;
private:
    std::vector<long> m_capacities;
    // 0x1 == true = is active
    std::vector<MeshAttributeHandle<char>> m_flags;
};


class TriMesh : public Mesh
{
private:
    MeshAttributeHandle<long> m_vf_handle;
    MeshAttributeHandle<long> m_ef_handle;

    MeshAttributeHandle<long> m_fv_handle;
    MeshAttributeHandle<long> m_fe_handle;
    MeshAttributeHandle<long> m_ff_handle;

public:
    TriMesh();

    void split_edge(const Tuple& t) override;
    void collapse_edge(const Tuple& t) override;

    void build_vertex_connectivity(long n_vertices) override;

    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    void initialize(
        Eigen::Ref<const RowVectors3l>& FV,
        Eigen::Ref<const RowVectors3l>& FE,
        Eigen::Ref<const RowVectors3l>& FF,
        Eigen::Ref<const VectorXl>& VF,
        Eigen::Ref<const VectorXl>& EF,
        Eigen::Ref<const RowVectors4l>& seam) const;
};

class TetMesh : public Mesh
{
private:
    MeshAttributeHandle<long> m_vt_handle;
    MeshAttributeHandle<long> m_et_handle;
    MeshAttributeHandle<long> m_ft_handle;

    MeshAttributeHandle<long> m_tv_handle;
    MeshAttributeHandle<long> m_te_handle;
    MeshAttributeHandle<long> m_tf_handle;
    MeshAttributeHandle<long> m_tt_handle;

public:
    TetMesh();

    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    void initialize(
        Eigen::Ref<const RowVectors4l>& TV,
        Eigen::Ref<const RowVectors4l>& TE,
        Eigen::Ref<const RowVectors4l>& TF,
        Eigen::Ref<const RowVectors4l>& TT,
        Eigen::Ref<const VectorXl>& VT,
        Eigen::Ref<const VectorXl>& ET,
        Eigen::Ref<const VectorXl>& FT) const;
};

template <typename T>
MeshAttributeHandle<T>
Mesh::register_attribute(const std::string& name, PrimitiveType ptype, long size)
{
    //return MeshAttributeHandle<T>{
    //    .m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
    //    .m_primitive_type = ptype};

     MeshAttributeHandle<T> r;
     r.m_base_handle = get_mesh_attributes<T>(ptype).register_attribute(name, size),
     r.m_primitive_type = ptype;
     return r;
}
/**
 * @brief given the mesh connectivity in matrix format, initialize the topology data used for Mesh
 * @param F input connectivity in (N x 3) matrix format (igl convention)
 * @param FV output connectivity in (N x 3) matrix format, same as F
 * @param FE three edges of every triangle in (N x 3) matrix format
 * @param FF three edge-adjacent faces of every triangle in (N x 3) matrix format
 * @param VF one adjacent triangle (arbitrarily chosen) of every vertex in (N x 1) matrix format
 * @param EF one adjacent triangle (arbitrarily chosen) of every edge in (N x 1) matrix format
 */

void trimesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3l> F,
    Eigen::Ref<Mesh::RowVectors3l> FV,
    Eigen::Ref<Mesh::RowVectors3l> FE,
    Eigen::Ref<Mesh::RowVectors3l> FF,
    Eigen::Ref<Mesh::VectorXl> VF,
    Eigen::Ref<Mesh::VectorXl> EF);

void tetmesh_topology_initialization(
    Eigen::Ref<const Mesh::RowVectors3d> V,
    Eigen::Ref<const Mesh::RowVectors4l> F,
    TetMesh& mesh);

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
