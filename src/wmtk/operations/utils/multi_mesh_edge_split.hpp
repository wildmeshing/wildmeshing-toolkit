#pragma once
#include <memory>

namespace wmtk {

namespace invariants {
class InvariantCollection;
}
namespace operations::utils {


// Initializes any invariants for splitting (which is None by default, but enabling a pattern
// with other operations)
std::shared_ptr<InvariantCollection> multimesh_split_edge_invariants(const Mesh& m);
void multi_mesh_split_edge(Mesh& mesh, const Tuple& t);


} // namespace operations::utils
} // namespace wmtk
