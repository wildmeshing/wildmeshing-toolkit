#pragma once

#include <Eigen/Core>
#include <wmtk/operations/edge_mesh/EdgeOperationData.hpp>
#include "MeshCRTP.hpp"
#include "Tuple.hpp"

namespace wmtk {

namespace operations::utils {
class MultiMeshEdgeSplitFunctor;
class MultiMeshEdgeCollapseFunctor;
class UpdateEdgeOperationMultiMeshMapFunctor;
} // namespace operations::utils
class EdgeMesh : public MeshCRTP<EdgeMesh>
{
public:
    friend class MeshCRTP<EdgeMesh>;
    template <typename U, typename MeshType, typename AT, int Dim>
    friend class attribute::Accessor;
    friend class operations::utils::MultiMeshEdgeSplitFunctor;
    friend class operations::utils::MultiMeshEdgeCollapseFunctor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    EdgeMesh();
    ~EdgeMesh() override;
    EdgeMesh(const EdgeMesh& o) = delete;
    EdgeMesh(EdgeMesh&& o) = default;
    EdgeMesh& operator=(const EdgeMesh& o) = delete;
    EdgeMesh& operator=(EdgeMesh&& o) = default;

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;

    bool is_ccw(const Tuple& tuple) const override;
    using Mesh::is_boundary;
    bool is_boundary(PrimitiveType, const Tuple& tuple) const override;
    bool is_boundary_vertex(const Tuple& tuple) const;


    void initialize(Eigen::Ref<const RowVectors2l> E, bool is_free = false);
    void initialize_free(int64_t count);

    void initialize(
        Eigen::Ref<const RowVectors2l> EV,
        Eigen::Ref<const RowVectors2l> EE,
        Eigen::Ref<const VectorXl> VE);

    bool is_valid(const Tuple& tuple) const final override;

    bool is_connectivity_valid() const override;

    std::vector<std::vector<TypedAttributeHandle<int64_t>>> connectivity_attributes()
        const override;


    Tuple switch_vertex(const Tuple& tuple) const;
    Tuple switch_edge(const Tuple& tuple) const;

    std::vector<Tuple> orient_vertices(const Tuple& tuple) const override;
    int64_t id(const Tuple& tuple, PrimitiveType type) const;
    using MeshCRTP<EdgeMesh>::id; // getting the (simplex) prototype

    int64_t id_vertex(const Tuple& tuple) const { return id(tuple, PrimitiveType::Vertex); }
    int64_t id_edge(const Tuple& tuple) const { return id(tuple, PrimitiveType::Edge); }

protected:
    /**
     * @brief internal function that returns the tuple of requested type, and has the global index
     * cid
     *
     * @param gid
     * @return Tuple
     */
    Tuple tuple_from_id(const PrimitiveType type, const int64_t gid) const override;

    Tuple tuple_from_global_ids(int64_t eid, int64_t vid) const;

protected:
    attribute::TypedAttributeHandle<int64_t> m_ve_handle;

    attribute::TypedAttributeHandle<int64_t> m_ev_handle;
    attribute::TypedAttributeHandle<int64_t> m_ee_handle;

    Tuple vertex_tuple_from_id(int64_t id) const;
    Tuple edge_tuple_from_id(int64_t id) const;

    // internal structure that encapsulations the actual execution of split and collapse
    class EdgeMeshOperationExecutor;
};

inline Tuple EdgeMesh::switch_vertex(const Tuple& tuple) const
{
    return switch_tuple(tuple, PrimitiveType::Vertex);
}
inline Tuple EdgeMesh::switch_edge(const Tuple& tuple) const
{
    return switch_tuple(tuple, PrimitiveType::Edge);
}
inline int64_t EdgeMesh::id(const Tuple& tuple, PrimitiveType type) const
{
    switch (type) {
    case PrimitiveType::Vertex: {
        const attribute::Accessor<int64_t, EdgeMesh> ev_accessor =
            create_const_accessor<int64_t>(m_ev_handle);
        auto ev = ev_accessor.const_vector_attribute<2>(tuple);
        return ev(tuple.local_vid());
    }
    case PrimitiveType::Edge: {
        return tuple.global_cid();
    }
    case PrimitiveType::Triangle:
    case PrimitiveType::Tetrahedron:
    default: assert(false); // "Tuple id: Invalid primitive type")
    }

    return -1;
}
} // namespace wmtk
