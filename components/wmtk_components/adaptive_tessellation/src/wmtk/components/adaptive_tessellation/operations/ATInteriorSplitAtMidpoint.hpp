#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include "ATOperationBase.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class ATInteriorSplitAtMidpoint;
}
template <>
struct OperationSettings<tri_mesh::ATInteriorSplitAtMidpoint>
{
    OperationSettings<ATOperationBase> base_settings;
    OperationSettigns<tri_mesh::EdgeSplitAtMidpoint> edge_split_midpoint_settings;

    bool split_boundary_edges = false;
    void initialize_invariants(const TriMesh& mesh);
    invariants.add(std::make_shared<InteriorEdgeInvariant>(m));
};
class ATInteriorSplitAtMidpoint : public EdgeSplitAtMidpoint
{
public:
    ATInteriorSplitAtMidpoint(
        const TriMesh& position_mesh,
        const Tuple& t,
        const OperationSettings<ATInteriorSplitAtMidpoint>& settings);

protected:
    bool execute() override;
    const TriMesh& uv_mesh() const;
    const TriMesh& position_mesh() const;

private:
    const OperationSettings<ATInteriorSplitAtMidpoint>& m_settings;

    Eigen::Vector2d uv0, uv1;
    Accessor<double> m_uv_accessor;
}
} // namespace wmtk::operations