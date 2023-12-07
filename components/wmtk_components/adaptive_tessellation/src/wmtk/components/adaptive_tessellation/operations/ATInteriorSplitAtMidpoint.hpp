#include <wmtk/components/adaptive_tessellation/operations/internal/ATData.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>

namespace AT_op = wmtk::components::adaptive_tessellation::operations;
namespace wmtk::components::adaptive_tessellation::operations {
class ATInteriorSplitAtMidpoint;
}
template <>
struct wmtk::operations::OperationSettings<AT_op::ATInteriorSplitAtMidpoint>
{
    AT_op::internal::ATData m_AT_data;
    wmtk::operations::OperationSettings<wmtk::operations::tri_mesh::EdgeSplitAtMidpoint>
        edge_split_midpoint_settings;

    bool split_boundary_edges = false;

    void initialize_invariants(const TriMesh& uv_mesh);
    OperationSettings(AT_op::internal::ATData AT_data);
    bool are_invariants_initialized() const;
};
namespace wmtk::components::adaptive_tessellation::operations {
class ATInteriorSplitAtMidpoint : public wmtk::operations::tri_mesh::EdgeSplitAtMidpoint
{
public:
    ATInteriorSplitAtMidpoint(
        Mesh& uv_mesh,
        const Tuple& t,
        const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& settings);
    ATInteriorSplitAtMidpoint(
        TriMesh& uv_mesh,
        const Tuple& t,
        const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& settings);

protected:
    bool execute() override;
    TriMesh& uv_mesh();
    TriMesh& position_mesh();
    const TriMesh& const_uv_mesh() const;
    const TriMesh& const_position_mesh() const;

private:
    const wmtk::operations::OperationSettings<ATInteriorSplitAtMidpoint>& m_settings;
    Accessor<double> m_pos_accessor;
};
} // namespace wmtk::components::adaptive_tessellation::operations
