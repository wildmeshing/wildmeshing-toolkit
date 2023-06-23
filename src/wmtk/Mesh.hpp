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
    std::vector<Tuple> get_all(const PrimitiveType& type) const;

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

    virtual std::vector<Tuple> get_vertices() const = 0;
    virtual std::vector<Tuple> get_edges() const = 0;
    virtual std::vector<Tuple> get_faces() const = 0;
    virtual std::vector<Tuple> get_tetrahedrons() const = 0;

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

    std::vector<Tuple> get_vertices() const override { throw "not implemented"; }
    std::vector<Tuple> get_edges() const override { throw "not implemented"; }
    std::vector<Tuple> get_faces() const override { throw "not implemented"; }
    std::vector<Tuple> get_tetrahedrons() const override { throw "not implemented"; }
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
