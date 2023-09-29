#pragma once

#include "Mesh.hpp"
#include "Tuple.hpp"

#include <Eigen/Core>

namespace wmtk {
// Simple mesh without topology. Mainly useful for testing attributes without having to construct
// topologies
class EdgeMesh : public Mesh
{
public:
    EdgeMesh();
    // EdgeMesh(const EdgeMesh&& o);
    EdgeMesh(EdgeMesh&& o);
    EdgeMesh& operator=(const EdgeMesh& o);
    EdgeMesh& operator=(EdgeMesh&& o);

    void initialize(Eigen::Ref<const RowVectors2l> E);

    void initialize(
        Eigen::Ref<const RowVectors2l> EV,
        Eigen::Ref<const RowVectors2l> EE,
        Eigen::Ref<const RowVectors2l> VE);

    PrimitiveType top_simplex_type() const override { return PrimitiveType::Edge; }
    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;
    bool is_ccw(const Tuple& tuple) const override;
    bool is_boundary(const Tuple& tuple) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;
    // TODO: should just write is_boundary(PrimitiveType)
    bool is_boundary_edge(const Tuple& tuple) const override { return true; }


    bool is_valid(const Tuple& tuple, ConstAccessor<long>& hash_accessor) const override;

    Tuple edge_tuple_from_id(long id) const;
    Tuple vertex_tuple_from_id(long id) const;

    Tuple split_edge(const Tuple&, Accessor<long>&) override { return {}; }
    Tuple collapse_edge(const Tuple&, Accessor<long>&) override { return {}; }
    bool is_connectivity_valid() const override { return true; }

protected:
    long id(const Tuple& tuple, PrimitiveType type) const override;
    long id(const Simplex& simplex) const { return id(simplex.tuple(), simplex.primitive_type()); }
    long id_vertex(const Tuple& tuple) const { return id(tuple, PrimitiveType::Vertex); }
    long id_edge(const Tuple& tuple) const { return id(tuple, PrimitiveType::Edge); }

    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const long gid) const override;

    // internal structure that encapsulations the actual execution of split and collapse
    class EdgeMeshOperationExecutor;
    static Tuple with_different_cid(const Tuple& t, long cid);


    // not sure if it is needed
    attribute::MeshAttributeHandle<long> m_ve_handle;
    attribute::MeshAttributeHandle<long> m_ev_handle;
    attribute::MeshAttributeHandle<long> m_ee_handle; // record relationship between edges
};

} // namespace wmtk