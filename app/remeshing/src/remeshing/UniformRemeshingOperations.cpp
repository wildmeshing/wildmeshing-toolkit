#include "UniformRemeshingOperations.h"
namespace app::remeshing {

std::map<std::string, std::shared_ptr<wmtk::TriMeshOperation>> UniformRemeshing::get_operations() const
{
    std::map<std::string, std::shared_ptr<wmtk::TriMeshOperation>> r;
    auto add_operation = [&](auto&& op) { r[op->name()] = op; };
    add_operation(std::make_shared<UniformRemeshingEdgeCollapseOperation>());
    add_operation(std::make_shared<UniformRemeshingEdgeSwapOperation>());
    add_operation(std::make_shared<UniformRemeshingEdgeSplitOperation>());
    add_operation(std::make_shared<UniformRemeshingVertexSmoothOperation>());
    return r;
}

} // namespace app::remeshing
