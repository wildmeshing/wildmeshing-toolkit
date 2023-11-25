#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include "AT.hpp"

namespace wmtk::operations {
namespace tri_mesh {
class ATSplitAtMidpoint;
}
template <>
struct OperationSettings<tri_mesh::ATSplitAtMidpoint>
{
    OperationSettings<ATOperationBase> base_settings;
    OperationSettigns<tri_mesh::EdgeSplitAtMidpoint> edge_split_midpoint_settings;
    InvarientsCollection invarients;
    void initialize_invariants(const TriMesh& mesh);
};
class ATSplitAtMidpoint : public ATOperationBase, public EdgeSplitAtMidpoint
{
public:
    ATSplitAtMidpoint(
        const TriMesh& uv_mesh,
        const TriMesh& position_mesh,
        const std::vector<EdgeMesh>& edge_meshes,
        const std::map<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>>& edge_meshes_map,
        const Tuple& t,
        const OperationSettings<ATSplitAtMidpoint>& settings);

protected:
    bool execute() override;
}
} // namespace wmtk::operations