#pragma once
#include <memory>

namespace wmtk {
class Mesh;
namespace attribute {
    class MeshAttributeHandle;
}
namespace operations {
class Operation;
namespace composite {
class EdgeSwap;
}
} // namespace operations
} // namespace wmtk
namespace wmtk::components::isotropic_remeshing {

struct IsotropicRemeshingOptions;

namespace internal {
std::shared_ptr<wmtk::operations::Operation> configure_swap(
    Mesh& m,
    const IsotropicRemeshingOptions& opts);

void configure_swap_transfer(
   operations::composite::EdgeSwap& swap,
    const attribute::MeshAttributeHandle& vertex_handle);
}
} // namespace wmtk::components::isotropic_remeshing
