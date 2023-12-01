#include <wmtk/components/adaptive_tessellation/operations/internal/ATOperation.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>

namespace AT_op = wmtk::components::adaptive_tessellation::operations;
namespace wmtk::components::adaptive_tessellation::operations {
class ATInteriorSplitAtMidpoint;
}
template <>
struct wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint>
{
    AT_op::internal::ATOperation m_AT_op;
    wmtk::operations::OperationSettings<wmtk::operations::tri_mesh::EdgeSplitAtMidpoint>
        edge_split_midpoint_settings;

    MeshAttributeHandle<double> m_pos_handle;
    wmtk::InvariantCollection AT_interior_split_invariants;
    bool split_boundary_edges = false;
    void initialize_invariants(const TriMesh& uv_mesh);
    OperationSettings(AT_op::internal::ATOperation AT_op);
};
namespace wmtk::components::adaptive_tessellation::operations {
class ATInteriorSplitAtMidpoint : public wmtk::operations::tri_mesh::EdgeSplitAtMidpoint
{
public:
    ATInteriorSplitAtMidpoint(
        const Tuple& t,
        const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& settings);

protected:
    bool execute() override;
    Mesh& uv_mesh();
    Mesh& position_mesh();
    const Mesh& const_uv_mesh() const;
    const Mesh& const_position_mesh() const;

private:
    Accessor<double> m_pos_accessor;
    const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& m_settings;
    Eigen::Vector3d pos0, pos1;
};
} // namespace wmtk::components::adaptive_tessellation::operations