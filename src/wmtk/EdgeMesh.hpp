#pragma once

#include <Eigen/Core>
#include <wmtk/operations/edge_mesh/EdgeOperationData.hpp>
#include "Mesh.hpp"
#include "Tuple.hpp"

namespace wmtk {

namespace operations::utils {
class MultiMeshEdgeSplitFunctor;
class MultiMeshEdgeCollapseFunctor;
class UpdateEdgeOperationMultiMeshMapFunctor;
} // namespace operations::utils
class EdgeMesh : public Mesh
{
public:
    friend class operations::utils::MultiMeshEdgeSplitFunctor;
    friend class operations::utils::MultiMeshEdgeCollapseFunctor;
    friend class operations::utils::UpdateEdgeOperationMultiMeshMapFunctor;
    EdgeMesh();
    EdgeMesh(const EdgeMesh& o);
    EdgeMesh(EdgeMesh&& o);
    EdgeMesh& operator=(const EdgeMesh& o);
    EdgeMesh& operator=(EdgeMesh&& o);

    int64_t top_cell_dimension() const override { return 1; }

    Tuple switch_tuple(const Tuple& tuple, PrimitiveType type) const override;

    bool is_ccw(const Tuple& tuple) const override;
    using Mesh::is_boundary;
    bool is_boundary(const Tuple& tuple, PrimitiveType) const override;
    bool is_boundary_vertex(const Tuple& tuple) const override;


    void initialize(Eigen::Ref<const RowVectors2l> E);

    void initialize(
        Eigen::Ref<const RowVectors2l> EV,
        Eigen::Ref<const RowVectors2l> EE,
        Eigen::Ref<const VectorXl> VE);

    bool is_valid(const Tuple& tuple, ConstAccessor<int64_t>& hash_accessor) const override;

    bool is_connectivity_valid() const override;

    std::vector<std::vector<TypedAttributeHandle<int64_t>>> connectivity_attributes()
        const override;

protected:
    int64_t id(const Tuple& tuple, PrimitiveType type) const override;
    int64_t id(const simplex::Simplex& simplex) const
    {
        return id(simplex.tuple(), simplex.primitive_type());
    }

    int64_t id_vertex(const Tuple& tuple) const { return id(tuple, PrimitiveType::Vertex); }
    int64_t id_edge(const Tuple& tuple) const { return id(tuple, PrimitiveType::Edge); }

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

} // namespace wmtk
