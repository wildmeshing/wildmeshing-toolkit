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
    TriMesh& uv_mesh();
    TriMesh& position_mesh();
    const TriMesh& const_uv_mesh() const;
    const TriMesh& const_position_mesh() const;

private:
    Accessor<double> m_pos_accessor;
    const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& m_settings;
    Eigen::Vector3d pos0, pos1;
};
} // namespace wmtk::components::adaptive_tessellation::operations