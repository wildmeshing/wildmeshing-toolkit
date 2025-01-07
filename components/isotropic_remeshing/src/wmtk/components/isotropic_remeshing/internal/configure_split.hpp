
#pragma once
#include <memory>

namespace wmtk {
namespace invariants {
class InvariantCollection;
}

namespace operations {
class EdgeSplit;
}
class Mesh;
} // namespace wmtk

namespace wmtk::components::isotropic_remeshing {

struct IsotropicRemeshingOptions;

namespace internal {


std::shared_ptr<invariants::InvariantCollection> split_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions&);
void configure_split(operations::EdgeSplit& ec, Mesh& m, const IsotropicRemeshingOptions&);
} // namespace internal
} // namespace wmtk::components::isotropic_remeshing
