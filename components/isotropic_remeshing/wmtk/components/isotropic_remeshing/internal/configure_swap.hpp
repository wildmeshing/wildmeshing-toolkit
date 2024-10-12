#pragma once
#include <memory>

namespace wmtk {
class Mesh;
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
}
} // namespace wmtk::components::isotropic_remeshing
