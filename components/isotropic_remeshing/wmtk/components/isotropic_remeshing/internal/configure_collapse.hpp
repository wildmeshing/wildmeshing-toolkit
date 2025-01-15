#pragma once
#include <memory>

namespace wmtk {
namespace invariants {
class InvariantCollection;
}

namespace operations {
class EdgeCollapse;
}
class Mesh;
} // namespace wmtk

namespace wmtk::components::isotropic_remeshing {

struct IsotropicRemeshingOptions;

namespace internal {

std::shared_ptr<wmtk::invariants::InvariantCollection> collapse_core_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions&);

std::shared_ptr<wmtk::invariants::InvariantCollection> collapse_invariants(
    Mesh& m,
    const IsotropicRemeshingOptions&);
void configure_collapse(operations::EdgeCollapse& ec, Mesh& m, const IsotropicRemeshingOptions&);
} // namespace internal
} // namespace wmtk::components::isotropic_remeshing
