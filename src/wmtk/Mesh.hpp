#pragma once
#include "Accessor.hpp"
#include "MeshAttributes.hpp"
#include "Primitive.hpp"
#include "Tuple.h"


namespace wmtk {
using Matl3 = Eigen::Matrix<long, Eigen::Dynamic, 3>;
using Matl1 = Eigen::Matrix<long, Eigen::Dynamic, 1>;
using Matl4 = Eigen::Matrix<long, Eigen::Dynamic, 4>;


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
    virtual Tuple switch_tuple(const PrimitiveType& type, const Tuple& t) const = 0;

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

    /**
     * Generate a face Tuple using global cell id
     * @param cid global cell for the triangle/tetrahedron
     * @return a Tuple
     */
    Tuple tuple_from_cell(long cid) const;

    /**
     * Generate a vertex Tuple using local vid
     * @param vid global vid
     * @note tuple refers to vid
     */
    virtual Tuple tuple_from_vertex(long vid) const = 0;


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
    T scalar_attribute(const AttributeHandle& handle, const PrimitiveType& type, const Tuple& tuble)
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

public:
    void split_edge(const Tuple& t) override;
    void collapse_edge(const Tuple& t) override;

    void build_vertex_connectivity(long n_vertices) override;

    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    void initialize(
        Eigen::Ref<const Matl3>& FV,
        Eigen::Ref<const Matl3>& FE,
        Eigen::Ref<const Matl3>& FF,
        Eigen::Ref<const Matl1>& VF,
        Eigen::Ref<const Matl1>& EF,
        Eigen::Ref<const Matl4>& seam) const;
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

public:
    long id(const Tuple& tuple, const PrimitiveType& type) const override;
    Tuple switch_tuple(const Tuple& tuple, const PrimitiveType& type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    void initialize(
        Eigen::Ref<const Matl4>& TV,
        Eigen::Ref<const Matl4>& TE,
        Eigen::Ref<const Matl4>& TF,
        Eigen::Ref<const Matl4>& TT,
        Eigen::Ref<const Matl1>& VT,
        Eigen::Ref<const Matl1>& ET,
        Eigen::Ref<const Matl1>& FT) const;
};

template <typename T>
void Mesh::register_attribute(const std::string& name, PrimitiveType ptype, long size)
{
    get_mesh_attributes<T>(ptype).register_attribute(name, size);
}
template <typename T>
Accessor<T>
Mesh::register_attribute_with_accessor(const std::string& name, PrimitiveType ptype, long size)
{
    return get_mesh_attributes<T>(ptype).register_attribute_with_accessor(name, size);
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
    Eigen::Ref<const MaxtrixXi>& F,
    Eigen::Ref<const Matl3>& FE,
    Eigen::Ref<const Matl3>& FF,
    Eigen::Ref<const Matl1>& VF,
    Eigen::Ref<const Matl1>& EF)
{
    std::vector<std::vector<long>> TTT;
    FV.resize(F.rows(), F.cols());
    FE.resize(F.rows(), F.cols());
    FF.resize(F.rows(), F.cols());
    VF.resize(F.rows(), 1);
    EF.resize(F.rows(), 1);
    TTT.resize(F.rows(), std::vector<long>(4));
    for (int f = 0; f < F.rows(); ++f) {
        for (int i = 0; i < F.cols(); ++i) {
            // v1 v2 f ei
            long v1 = std::static_cast<long>(F(f, i));
            long v2 = std::static_cast<long>(F(f, (i + 1) % F.cols()));
            if (v1 > v2) std::swap(v1, v2);
            std::vector<long> r(4);
            r[0] = v1;
            r[1] = v2;
            r[2] = f;
            r[3] = i;
            TTT[f] = r;
            FV(f, i) = v1;
        }
    }
    std::sort(TTT.begin(), TTT.end());

    // iterate over TTT to initialize topology
    // assumption is the same edge is always next to each other in the sorted TTT
    int unique_edges = 0;
    long v01 = TTT[0][0];
    long v02 = TTT[0][1];
    long f0 = TTT[0][2];
    long e0 = TTT[0][3];
    FE(f0, e0) = unique_edges;
    VF(v01, 0) = f0;
    VF(v02, 0) = f0;
    EF(unique_edges, 0) = f0;

    for (int i = 1; i < TTT.size(); ++i) {
        int va1 = TTT[i][0];
        int va2 = TTT[i][1];
        int fa = TTT[i][2];
        int eia = TTT[i][3];

        int vb1 = TTT[i - 1][0];
        int vb2 = TTT[i - 1][1];
        int fb = TTT[i - 1][2];
        int eib = TTT[i - 1][3];
        if (va1 == vb1 & va2 == vb2) {
            // same edge
            FF(fa, eia) = fb;
            FF(fb, eib) = fa;
            continue;
        } else {
            unique_edges++;
            FE(fa, eia) = unique_edges;
            VF(va1, 0) = fa;
            VF(va2, 0) = fa;
            EF(unique_edges, 0) = fa;
            FF(fa, eia) = -1;
        }
    }
}

void tetmesh_topology_initialization(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const TetMesh& mesh)
{}
} // namespace wmtk
