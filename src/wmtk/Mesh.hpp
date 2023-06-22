#pragma once
#include <Tuple.h>
#include "Accessor.hpp"
#include "MeshAttributes.hpp"
#include "Accessor.hpp"
#include "Primitive.h"


namespace wmtk {
using Matl3 = Eigen::Matrix<long, Eigen::Dynamic, 3>;
using Matl1 = Eigen::Matrix<long, Eigen::Dynamic, 1>;
using Matl4 = Eigen::Matrix<long, Eigen::Dynamic, 4>;

class Accessor;

class Mesh
{
public:
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
        Eigen::Ref<const Matl1>& EF);
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

void trimesh_topology_initialization(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const TriMesh& mesh)
{
    // Accessor<>
    
    // std::vector<std::vector<long>> VF(vertex_capacity());
    // for(int j = 0; j < triangle_capacity(); j) 
    // {
    //     auto f = _topology_accessor.get_attribute<long>(m_fv_handle, j); // Eigen::Map // Eigen::Vector3i
    //     for(long vidx : f) { 
    //         VF[vidx].emplace_back(j);
    //         long& vf = _topology_accessor.get_attribute_single<long>(m_vf_handle, vidx); // Eigen::Map // Eigen::Vector3i
    //         v = j;
    //     }
    // std::vector<std::array<long,2>> e_array;
    // for(int j = 0; j < triangle_capacity(); ++j) 
    // {
    //     auto f = _topology_accessor.get_attribute<long>(m_fv_handle, j); // Eigen::Map // Eigen::Vector3i
    //     for(int k = 0; k < 3; ++ k) 
    //     {
    //         int kp1 = (k+1)%3;
    //         int a = f(k);
    //         int b = f(kp1);
    //         if(a > b) 
    //         {
    //             std::swap(a,b);
    //             e_array.emplace_back(std::array<long,2>{{a,b}});
    //         }
    //         std::sort(e_array.begin(),e_array.end());
    //         e_array.erase(std::unique(e_array.begin(),e_array.end()), e_array.end());
    //     }
    // }
}

void tetmesh_topology_initialization(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const TetMesh& mesh)
{}
} // namespace wmtk
